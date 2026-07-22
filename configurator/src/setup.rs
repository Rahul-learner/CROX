use eframe::egui;
use std::f32::consts::PI;
use std::io::{Read, Write};
use std::time::{Duration, Instant};
use egui::{Color32, Pos2, Stroke, Vec2, RichText, Margin};

pub struct SetupState {
    // Raw telemetry values (degrees)
    raw_roll: f32,
    raw_pitch: f32,
    raw_yaw: f32,
    // Smoothed values used for rendering
    roll: f32, // degrees
    pitch: f32,
    yaw: f32,
    // Camera offset from user mouse drag (radians)
    cam_yaw_offset: f32,
    cam_pitch_offset: f32,
    zoom: f32,
    last_poll_time: Instant,
}

impl Default for SetupState {
    fn default() -> Self {
        Self {
            raw_roll: 0.0,
            raw_pitch: 0.0,
            raw_yaw: 0.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            cam_yaw_offset: 0.0,
            cam_pitch_offset: 0.0,
            zoom: 1.0,
            last_poll_time: Instant::now(),
        }
    }
}

#[derive(Clone, Copy)]
struct Point3D {
    x: f32,
    y: f32,
    z: f32,
}

impl Point3D {
    fn rotate(&self, roll: f32, pitch: f32, yaw: f32) -> Self {
        let (sr, cr) = roll.sin_cos();
        let (sp, cp) = pitch.sin_cos();
        let (sy, cy) = yaw.sin_cos();

        // Yaw
        let xy = self.x * cy - self.y * sy;
        let yy = self.x * sy + self.y * cy;
        let zy = self.z;

        // Pitch
        let xp = xy * cp + zy * sp;
        let yp = yy;
        let zp = -xy * sp + zy * cp;

        // Roll
        let xr = xp;
        let yr = yp * cr - zp * sr;
        let zr = yp * sr + zp * cr;

        Self { x: xr, y: yr, z: zr }
    }
}

impl SetupState {
    pub fn ui(&mut self, ui: &mut egui::Ui, port_handle: &mut Option<Box<dyn serialport::SerialPort>>) {
        // Poll telemetry if connected
        if port_handle.is_some() && self.last_poll_time.elapsed() > Duration::from_millis(20) {
            self.last_poll_time = Instant::now();
            self.poll_attitude(port_handle);
        }

        // ── 3D Canvas fills the ENTIRE available area ──
        let available = ui.available_size();
        let canvas_size = Vec2::new(available.x.max(300.0), available.y.max(300.0));

        let (response, painter) = ui.allocate_painter(canvas_size, egui::Sense::drag());
        let rect = response.rect;

        // Premium dark-mode background
        painter.rect_filled(rect, 0.0, Color32::from_rgb(22, 24, 32));

        // Handle mouse drag for camera rotation
        if response.dragged() {
            let delta = ui.input(|i| i.pointer.delta());
            self.cam_yaw_offset -= delta.x * 0.005;
            self.cam_pitch_offset += delta.y * 0.005;
        }
        // Handle mouse wheel for zooming
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                self.zoom += scroll * 0.005;
                self.zoom = self.zoom.clamp(0.3, 5.0);
            }
        }

        // ── Overlaid toolbar buttons (top-left) ──
        let overlay_rect = rect.shrink(15.0);
        {
            let btn_y = overlay_rect.top();
            let mut btn_x = overlay_rect.left();

            let btn_labels = ["Calibrate Accelerometer", "Calibrate Magnetometer", "Reset / Reboot"];
            let btn_widths = [180.0, 180.0, 120.0];
            for (i, (label, w)) in btn_labels.iter().zip(btn_widths.iter()).enumerate() {
                let btn_rect = egui::Rect::from_min_size(
                    Pos2::new(btn_x, btn_y),
                    Vec2::new(*w, 28.0),
                );
                let hovered = response.hover_pos().map_or(false, |pos| btn_rect.contains(pos));
                let fill = if hovered {
                    Color32::from_rgb(241, 196, 15)
                } else {
                    Color32::from_rgb(243, 156, 18)
                };
                painter.rect_filled(btn_rect, 6.0, fill);
                painter.text(
                    btn_rect.center(),
                    egui::Align2::CENTER_CENTER,
                    *label,
                    egui::FontId::proportional(13.0),
                    Color32::BLACK,
                );
                if hovered && response.clicked() {
                    match i {
                        0 => {
                            if let Some(port) = port_handle.as_mut() {
                                let _ = port.write(b"CALIBRATE_ACCEL\n");
                            }
                        }
                        2 => {
                            if let Some(port) = port_handle.as_mut() {
                                let _ = port.write(b"REBOOT\n");
                            }
                        }
                        _ => {}
                    }
                }
                btn_x += w + 10.0;
            }
        }

        // ── Angle overlays (below toolbar) ──
        let info_top = overlay_rect.left_top() + Vec2::new(0.0, 45.0);
        painter.text(info_top, egui::Align2::LEFT_TOP,
            format!("Heading: {:>6.1}°", self.yaw),
            egui::FontId::monospace(18.0), Color32::from_rgb(200, 200, 200));
        painter.text(info_top + Vec2::new(0.0, 25.0), egui::Align2::LEFT_TOP,
            format!("Pitch:   {:>6.1}°", self.pitch),
            egui::FontId::monospace(18.0), Color32::from_rgb(200, 200, 200));
        painter.text(info_top + Vec2::new(0.0, 50.0), egui::Align2::LEFT_TOP,
            format!("Roll:    {:>6.1}°", self.roll),
            egui::FontId::monospace(18.0), Color32::from_rgb(200, 200, 200));

        // ── 3D Rendering Logic ──
        let center = rect.center();
        let base_scale = canvas_size.y.min(canvas_size.x) * 0.35;
        let scale = base_scale * self.zoom;

        // Low-pass filter for smooth animation
        const FILTER_ALPHA: f32 = 0.15;
        self.roll = self.roll * (1.0 - FILTER_ALPHA) + self.raw_roll * FILTER_ALPHA;
        self.pitch = self.pitch * (1.0 - FILTER_ALPHA) + self.raw_pitch * FILTER_ALPHA;
        self.yaw = self.yaw * (1.0 - FILTER_ALPHA) + self.raw_yaw * FILTER_ALPHA;
        let r = self.roll * PI / 180.0;
        let p = self.pitch * PI / 180.0;
        let y = self.yaw * PI / 180.0;

        // Camera offsets (no base tilt – lay flat from behind)
        let cam_pitch = self.cam_pitch_offset;
        let cam_yaw = self.cam_yaw_offset;

        // Project 3D → 2D  (returns screen pos + depth for sorting)
        let project = |pt: Point3D| -> (Pos2, f32) {
            let rotated = pt.rotate(r, p, y);
            let viewed = rotated.rotate(0.0, cam_pitch, cam_yaw);
            // X = forward → depth, Y = right → screen X, Z = down → screen Y
            (Pos2::new(
                center.x + viewed.y * scale,
                center.y + viewed.z * scale,
            ), viewed.x)
        };

        // ── Build mesh faces ──
        // Each face stores its 4 world-space vertices, colour, and a depth bias
        // so the painter's algorithm can break ties deterministically.
        let mut faces: Vec<(Vec<Point3D>, Color32, f32)> = Vec::new();
        let mut box_index: f32 = 0.0;

        let mut add_box = |cx: f32, cy: f32, cz: f32, l: f32, w: f32, h: f32, col: Color32, yaw_deg: f32, faces: &mut Vec<(Vec<Point3D>, Color32, f32)>, bias: f32| {
            let rad = yaw_deg * PI / 180.0;
            let cos_y = rad.cos();
            let sin_y = rad.sin();

            let pts = [
                (-l, -w, -h), ( l, -w, -h), ( l,  w, -h), (-l,  w, -h),
                (-l, -w,  h), ( l, -w,  h), ( l,  w,  h), (-l,  w,  h),
            ];

            let mut v = Vec::new();
            for (px, py, pz) in pts {
                let rx = px * cos_y - py * sin_y;
                let ry = px * sin_y + py * cos_y;
                v.push(Point3D { x: rx + cx, y: ry + cy, z: pz + cz });
            }

            let darken = |c: Color32, factor: f32| -> Color32 {
                Color32::from_rgb(
                    (c.r() as f32 * factor) as u8,
                    (c.g() as f32 * factor) as u8,
                    (c.b() as f32 * factor) as u8,
                )
            };

            faces.push((vec![v[0], v[1], v[2], v[3]], darken(col, 0.4), bias)); // bottom
            faces.push((vec![v[4], v[5], v[6], v[7]], col, bias));              // top
            faces.push((vec![v[1], v[5], v[6], v[2]], darken(col, 0.7), bias)); // right
            faces.push((vec![v[0], v[4], v[7], v[3]], darken(col, 0.7), bias)); // left
            faces.push((vec![v[2], v[6], v[7], v[3]], darken(col, 0.85), bias));// front
            faces.push((vec![v[0], v[1], v[5], v[4]], darken(col, 0.6), bias)); // back
        };

        // Drone parts (Aerospace: X=forward, Y=right, Z=down)
        let c_gray       = Color32::from_rgb(178, 178, 178);
        let c_red        = Color32::from_rgb(230, 51, 51);
        let c_dark_gray  = Color32::from_rgb(102, 102, 102);
        let c_darker_gray= Color32::from_rgb(76, 76, 76);
        let sm = 0.25_f32;

        // Fuselage (center)
        add_box(0.0,       0.0, 0.0,          1.5*sm, 0.3*sm,  0.25*sm, c_gray,        0.0, &mut faces, { box_index += 1.0; box_index });
        // Nose / Indicator (front)
        add_box(1.8*sm,    0.0, 0.0,          0.5*sm, 0.2*sm,  0.15*sm, c_red,         0.0, &mut faces, { box_index += 1.0; box_index });
        // Main Wing (mounted on top of fuselage: Z = -0.28)
        add_box(-0.2*sm,   0.0, -0.28*sm,     0.6*sm, 1.75*sm, 0.04*sm, c_dark_gray,   0.0, &mut faces, { box_index += 1.0; box_index });
        // Vertical Tail Fin (mounted on rear top: Z = -0.55)
        add_box(-1.1*sm,   0.0, -0.55*sm,     0.4*sm, 0.04*sm, 0.3*sm,  c_darker_gray, 0.0, &mut faces, { box_index += 1.0; box_index });

        // ── Project and sort faces (Painter's Algorithm) ──
        let mut projected_faces: Vec<(Vec<Pos2>, Color32, f32)> = Vec::new();
        for (pts, col, bias) in &faces {
            let mut proj_pts = Vec::new();
            let mut avg_z = 0.0;
            for pt in pts {
                let (p2, z) = project(*pt);
                proj_pts.push(p2);
                avg_z += z;
            }
            avg_z /= proj_pts.len() as f32;
            // Add tiny bias based on box index so parts sort deterministically
            avg_z += bias * 0.001;
            projected_faces.push((proj_pts, *col, avg_z));
        }

        // Sort back-to-front (most positive depth = furthest away = drawn first)
        projected_faces.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap_or(std::cmp::Ordering::Equal));

        // Draw faces
        for (pts, col, _) in &projected_faces {
            painter.add(egui::Shape::convex_polygon(pts.clone(), *col, Stroke::new(0.5, Color32::from_rgb(20, 20, 20))));
        }

        // ── Axis lines ──
        let origin = project(Point3D { x: 0.0, y: 0.0, z: 0.0 }).0;
        let axis_len = 1.5;
        let px = project(Point3D { x: axis_len, y: 0.0, z: 0.0 }).0;
        let py = project(Point3D { x: 0.0, y: axis_len, z: 0.0 }).0;
        let pz = project(Point3D { x: 0.0, y: 0.0, z: axis_len }).0;
        painter.line_segment([origin, px], Stroke::new(2.0, Color32::from_rgb(255, 100, 100)));
        painter.line_segment([origin, py], Stroke::new(2.0, Color32::from_rgb(100, 255, 100)));
        painter.line_segment([origin, pz], Stroke::new(2.0, Color32::from_rgb(100, 100, 255)));
        painter.text(px, egui::Align2::LEFT_BOTTOM, "Roll",  egui::FontId::proportional(12.0), Color32::from_rgb(255, 120, 120));
        painter.text(py, egui::Align2::LEFT_BOTTOM, "Pitch", egui::FontId::proportional(12.0), Color32::from_rgb(120, 255, 120));
        painter.text(pz, egui::Align2::LEFT_BOTTOM, "Yaw",   egui::FontId::proportional(12.0), Color32::from_rgb(120, 120, 255));

        // ── Reset Z button (top-right) ──
        let reset_btn_rect = egui::Rect::from_min_size(
            overlay_rect.right_top() - Vec2::new(150.0, 0.0),
            Vec2::new(150.0, 30.0),
        );
        let hovered = response.hover_pos().map_or(false, |pos| reset_btn_rect.contains(pos));
        let btn_fill = if hovered { Color32::from_rgb(60, 65, 80) } else { Color32::from_rgb(40, 45, 55) };
        painter.rect_filled(reset_btn_rect, 6.0, btn_fill);
        painter.rect_stroke(reset_btn_rect, 6.0, Stroke::new(1.0, Color32::from_rgb(80, 85, 95)));
        painter.text(reset_btn_rect.center(), egui::Align2::CENTER_CENTER,
            "Reset Z axis", egui::FontId::proportional(14.0), Color32::WHITE);
        if hovered && response.clicked() {
            self.yaw = 0.0;
        }

        // ── Overlaid Info / GPS panels (bottom-right) ──
        {
            let panel_w = 280.0;
            let panel_margin = 15.0;
            let panel_x = overlay_rect.right() - panel_w;
            let mut panel_y = overlay_rect.bottom() - 10.0; // start from bottom

            // GPS panel
            let gps_entries = [
                ("3D Fix:", "False", Color32::from_rgb(231, 76, 60)),
                ("Sats:", "0", Color32::WHITE),
                ("Latitude:", "0.0000", Color32::WHITE),
                ("Longitude:", "0.0000", Color32::WHITE),
            ];
            let gps_h = 30.0 + gps_entries.len() as f32 * 22.0 + 15.0;
            panel_y -= gps_h;
            let gps_rect = egui::Rect::from_min_size(Pos2::new(panel_x, panel_y), Vec2::new(panel_w, gps_h));
            painter.rect_filled(gps_rect, 10.0, Color32::from_rgba_premultiplied(30, 35, 45, 200));
            painter.rect_stroke(gps_rect, 10.0, Stroke::new(1.0, Color32::from_rgb(50, 55, 65)));
            painter.text(gps_rect.left_top() + Vec2::new(panel_margin, 8.0), egui::Align2::LEFT_TOP,
                "GPS", egui::FontId::proportional(16.0), Color32::WHITE);
            for (i, (label, value, col)) in gps_entries.iter().enumerate() {
                let row_y = gps_rect.top() + 32.0 + i as f32 * 22.0;
                painter.text(Pos2::new(gps_rect.left() + panel_margin, row_y), egui::Align2::LEFT_TOP,
                    *label, egui::FontId::proportional(13.0), Color32::from_rgb(180, 180, 180));
                painter.text(Pos2::new(gps_rect.left() + panel_margin + 140.0, row_y), egui::Align2::LEFT_TOP,
                    *value, egui::FontId::proportional(13.0), *col);
            }

            // Info panel
            panel_y -= 10.0; // gap
            let info_entries = [
                ("Arming Disable Flags:", "0"),
                ("Battery voltage:", "0.0 V"),
                ("Capacity drawn:", "0 mAh"),
                ("Current draw:", "0.00 A"),
                ("RSSI:", "0 %"),
            ];
            let info_h = 30.0 + info_entries.len() as f32 * 22.0 + 15.0;
            panel_y -= info_h;
            let info_rect = egui::Rect::from_min_size(Pos2::new(panel_x, panel_y), Vec2::new(panel_w, info_h));
            painter.rect_filled(info_rect, 10.0, Color32::from_rgba_premultiplied(30, 35, 45, 200));
            painter.rect_stroke(info_rect, 10.0, Stroke::new(1.0, Color32::from_rgb(50, 55, 65)));
            painter.text(info_rect.left_top() + Vec2::new(panel_margin, 8.0), egui::Align2::LEFT_TOP,
                "Info", egui::FontId::proportional(16.0), Color32::WHITE);
            for (i, (label, value)) in info_entries.iter().enumerate() {
                let row_y = info_rect.top() + 32.0 + i as f32 * 22.0;
                painter.text(Pos2::new(info_rect.left() + panel_margin, row_y), egui::Align2::LEFT_TOP,
                    *label, egui::FontId::proportional(13.0), Color32::from_rgb(180, 180, 180));
                painter.text(Pos2::new(info_rect.left() + panel_margin + 160.0, row_y), egui::Align2::LEFT_TOP,
                    *value, egui::FontId::proportional(13.0), Color32::WHITE);
            }
        }
    }
    
    fn poll_attitude(&mut self, port_handle: &mut Option<Box<dyn serialport::SerialPort>>) {
        if let Some(port) = port_handle {
            let _ = port.write(b"GET_ATTITUDE\n");
            let _ = port.flush();
            
            let mut buf = vec![0; 256];
            port.set_timeout(Duration::from_millis(10)).unwrap();
            if let Ok(n) = port.read(&mut buf) {
                if let Ok(s) = String::from_utf8(buf[..n].to_vec()) {
                    for line in s.lines() {
                        if line.starts_with("ATTITUDE,") {
                            let parts: Vec<&str> = line.split(',').collect();
                            if parts.len() == 4 {
                                if let (Ok(r), Ok(p), Ok(y)) = (parts[1].parse::<f32>(), parts[2].parse::<f32>(), parts[3].parse::<f32>()) {
                                     self.raw_roll = r;
                                     self.raw_pitch = p;
                                     self.raw_yaw = y;
                                 }
                            }
                        } else {
                            let raw_line = line.trim();
                            if !raw_line.is_empty() && !raw_line.starts_with("DEBUG") {
                                crate::log_event(&format!("RX: {}", raw_line), crate::LogLevel::Rx);
                            }
                        }
                    }
                }
            }
        }
    }
}
