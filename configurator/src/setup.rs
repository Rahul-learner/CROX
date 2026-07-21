use eframe::egui;
use std::f32::consts::PI;
use std::io::{Read, Write};
use std::time::{Duration, Instant};
use egui::{Color32, Pos2, Stroke, Vec2, RichText, Margin};

pub struct SetupState {
    pub roll: f32, // degrees
    pub pitch: f32,
    pub yaw: f32,
    last_poll_time: Instant,
}

impl Default for SetupState {
    fn default() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
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

        ui.vertical(|ui| {
            ui.add_space(10.0);
            
            // Top Toolbar for Buttons
            ui.horizontal(|ui| {
                // Style Betaflight yellow buttons
                ui.style_mut().visuals.widgets.inactive.bg_fill = Color32::from_rgb(243, 156, 18);
                ui.style_mut().visuals.widgets.inactive.fg_stroke = Stroke::new(1.0, Color32::BLACK);
                ui.style_mut().visuals.widgets.hovered.bg_fill = Color32::from_rgb(241, 196, 15);
                ui.style_mut().visuals.widgets.hovered.fg_stroke = Stroke::new(1.0, Color32::BLACK);
                
                if ui.add_sized([180.0, 30.0], egui::Button::new(RichText::new("Calibrate Accelerometer").strong().color(Color32::BLACK))).clicked() {
                    if let Some(port) = port_handle.as_mut() {
                        let _ = port.write(b"CALIBRATE_ACCEL\n");
                    }
                }
                ui.add_space(10.0);
                
                ui.style_mut().visuals.widgets.inactive.bg_fill = Color32::from_rgb(100, 100, 100); // disabled look
                if ui.add_sized([180.0, 30.0], egui::Button::new(RichText::new("Calibrate Magnetometer").color(Color32::from_rgb(200,200,200)))).clicked() { }
                ui.add_space(10.0);
                
                ui.style_mut().visuals.widgets.inactive.bg_fill = Color32::from_rgb(243, 156, 18);
                if ui.add_sized([120.0, 30.0], egui::Button::new(RichText::new("Reset / Reboot").strong().color(Color32::BLACK))).clicked() {
                    if let Some(port) = port_handle.as_mut() {
                        let _ = port.write(b"REBOOT\n");
                    }
                }
                
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.add_sized([80.0, 30.0], egui::Button::new(RichText::new("Restore").strong().color(Color32::BLACK))).clicked() { }
                    ui.add_space(5.0);
                    if ui.add_sized([80.0, 30.0], egui::Button::new(RichText::new("Backup").strong().color(Color32::BLACK))).clicked() { }
                });
            });
            
            ui.add_space(15.0);

            ui.horizontal(|ui| {
                // Center column: 3D Drone & Info
                ui.vertical(|ui| {
                    ui.heading(RichText::new("Setup").size(24.0).strong().color(Color32::WHITE));
                    ui.label(RichText::new("Place board or frame on leveled surface, proceed with calibration.").color(Color32::from_rgb(180, 180, 180)));
                    ui.add_space(10.0);
                    
                    // 3D Canvas
                    let available = ui.available_size();
                    let canvas_size = Vec2::new(available.x.max(400.0), available.y.max(400.0));
                    
                    let (response, painter) = ui.allocate_painter(canvas_size, egui::Sense::hover());
                    let rect = response.rect;
                    // Draw a nice subtle background
                    painter.rect_filled(rect, 12.0, Color32::from_rgb(25, 28, 35));
                    painter.rect_stroke(rect, 12.0, Stroke::new(1.0, Color32::from_rgb(50, 55, 65)));
                    
                    // Angle overlays
                    let overlay_rect = rect.shrink(15.0);
                    painter.text(
                        overlay_rect.left_top(),
                        egui::Align2::LEFT_TOP,
                        format!("Heading: {:>6.1}°", self.yaw),
                        egui::FontId::monospace(18.0),
                        Color32::from_rgb(200, 200, 200),
                    );
                    painter.text(
                        overlay_rect.left_top() + Vec2::new(0.0, 25.0),
                        egui::Align2::LEFT_TOP,
                        format!("Pitch:   {:>6.1}°", self.pitch),
                        egui::FontId::monospace(18.0),
                        Color32::from_rgb(200, 200, 200),
                    );
                    painter.text(
                        overlay_rect.left_top() + Vec2::new(0.0, 50.0),
                        egui::Align2::LEFT_TOP,
                        format!("Roll:    {:>6.1}°", self.roll),
                        egui::FontId::monospace(18.0),
                        Color32::from_rgb(200, 200, 200),
                    );
                    
                    // 3D Rendering Logic
                    let center = rect.center();
                    let scale = (canvas_size.y.min(canvas_size.x) * 0.35).clamp(80.0, 250.0);
                    
                    // Convert angles to radians. Note: Standard aerospace is X forward, Y right, Z down.
                    let r = self.roll * PI / 180.0;
                    let p = self.pitch * PI / 180.0;
                    let y = 0.0; // Yaw disabled in 3D visualizer since it often shows rate or drifts without mag
                
                // Drone geometry (X=forward, Y=right, Z=up)
                // To display nicely in isometric 3D, we add a base camera tilt
                let cam_pitch = 25.0 * PI / 180.0; // Positive pitch to look from above
                let cam_yaw = 180.0 * PI / 180.0;  // 180 yaw to face the drone's front towards the bottom of the screen
                
                // Function to project 3D to 2D
                let project = |pt: Point3D| -> (Pos2, f32) {
                    let rotated = pt.rotate(r, p, y);
                    let viewed = rotated.rotate(0.0, cam_pitch, cam_yaw);
                    (Pos2::new(
                        center.x + viewed.y * scale,
                        center.y - viewed.x * scale
                    ), viewed.z)
                };
                
                let mut faces: Vec<(Vec<Point3D>, Color32)> = Vec::new();

                let mut add_box = |cx: f32, cy: f32, cz: f32, l: f32, w: f32, h: f32, col: Color32, yaw_deg: f32| {
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
                        v.push(Point3D {
                            x: rx + cx,
                            y: ry + cy,
                            z: pz + cz
                        });
                    }

                    // Faces using right hand rule (outward normals)
                    // Added a slight shading based on the face direction for pseudo-lighting
                    let darken = |c: Color32, factor: f32| -> Color32 {
                        Color32::from_rgb(
                            (c.r() as f32 * factor) as u8,
                            (c.g() as f32 * factor) as u8,
                            (c.b() as f32 * factor) as u8,
                        )
                    };
                    
                    faces.push((vec![v[0], v[1], v[2], v[3]], darken(col, 0.4))); // bottom
                    faces.push((vec![v[4], v[5], v[6], v[7]], col));              // top
                    faces.push((vec![v[1], v[5], v[6], v[2]], darken(col, 0.7))); // right
                    faces.push((vec![v[0], v[4], v[7], v[3]], darken(col, 0.7))); // left
                    faces.push((vec![v[2], v[6], v[7], v[3]], darken(col, 0.85)));// front
                    faces.push((vec![v[0], v[1], v[5], v[4]], darken(col, 0.6))); // back
                };

                let c_body = Color32::from_rgb(34, 34, 34);
                let c_arm = Color32::from_rgb(136, 136, 136);
                let c_red = Color32::from_rgb(255, 51, 51);
                let c_blue = Color32::from_rgb(51, 51, 255);
                let c_green = Color32::from_rgb(0, 255, 0);

                // Central body
                add_box(0.0, 0.0, 0.0, 0.25, 0.25, 0.1, c_body, 0.0);
                
                // Arms
                let arm_l = 0.6;
                add_box(arm_l/2.0, arm_l/2.0, 0.0, arm_l/2.0, 0.04, 0.04, c_arm, 45.0);
                add_box(arm_l/2.0, -arm_l/2.0, 0.0, arm_l/2.0, 0.04, 0.04, c_arm, -45.0);
                add_box(-arm_l/2.0, arm_l/2.0, 0.0, arm_l/2.0, 0.04, 0.04, c_arm, 135.0);
                add_box(-arm_l/2.0, -arm_l/2.0, 0.0, arm_l/2.0, 0.04, 0.04, c_arm, -135.0);
                
                // Rotors
                add_box(arm_l, arm_l, 0.1, 0.2, 0.2, 0.02, c_red, 0.0); // M2 (Front Right)
                add_box(arm_l, -arm_l, 0.1, 0.2, 0.2, 0.02, c_blue, 0.0); // M4 (Front Left)
                add_box(-arm_l, arm_l, 0.1, 0.2, 0.2, 0.02, c_blue, 0.0); // M1 (Back Right)
                add_box(-arm_l, -arm_l, 0.1, 0.2, 0.2, 0.02, c_red, 0.0); // M3 (Back Left)

                // Indicator Arrow
                add_box(0.35, 0.0, 0.05, 0.1, 0.02, 0.02, c_green, 0.0);
                
                // Project and sort faces (Painter's Algorithm)
                let mut projected_faces = Vec::new();
                for (pts, col) in faces {
                    let mut proj_pts = Vec::new();
                    let mut avg_z = 0.0;
                    for pt in pts {
                        let (p2, z) = project(pt);
                        proj_pts.push(p2);
                        avg_z += z;
                    }
                    avg_z /= proj_pts.len() as f32;
                    projected_faces.push((proj_pts, col, avg_z));
                }

                // Sort by Z descending (largest Z is furthest away, drawn first)
                projected_faces.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap());

                // Draw faces
                for (pts, col, _) in projected_faces {
                    painter.add(egui::Shape::convex_polygon(pts, col, Stroke::new(0.5, Color32::from_rgb(20, 20, 20))));
                }
                
                // Add axis lines
                let origin = project(Point3D{x:0.0, y:0.0, z:0.0}).0;
                let axis_len = 1.5;
                let px = project(Point3D{x:axis_len, y:0.0, z:0.0}).0;
                let py = project(Point3D{x:0.0, y:axis_len, z:0.0}).0;
                let pz = project(Point3D{x:0.0, y:0.0, z:axis_len}).0;
                painter.line_segment([origin, px], Stroke::new(2.0, Color32::from_rgb(255, 50, 50))); // X=Red
                painter.line_segment([origin, py], Stroke::new(2.0, Color32::from_rgb(50, 255, 50))); // Y=Green
                painter.line_segment([origin, pz], Stroke::new(2.0, Color32::from_rgb(50, 50, 255))); // Z=Blue
                
                
                let btn_rect = overlay_rect.shrink2(Vec2::new(0.0, 10.0));
                
                // Hacky manual hit detection for a virtual button drawn on the canvas
                let reset_btn_rect = egui::Rect::from_min_size(
                    btn_rect.right_top() - Vec2::new(150.0, 0.0), 
                    Vec2::new(150.0, 30.0)
                );
                
                let hovered = response.hover_pos().map_or(false, |pos| reset_btn_rect.contains(pos));
                let btn_fill = if hovered { Color32::from_rgb(60, 65, 80) } else { Color32::from_rgb(40, 45, 55) };
                
                painter.rect_filled(reset_btn_rect, 6.0, btn_fill);
                painter.rect_stroke(reset_btn_rect, 6.0, Stroke::new(1.0, Color32::from_rgb(80, 85, 95)));
                painter.text(
                    reset_btn_rect.center(), 
                    egui::Align2::CENTER_CENTER, 
                    "Reset Z axis", 
                    egui::FontId::proportional(14.0), 
                    Color32::WHITE
                );
                
                if hovered && response.clicked() {
                    self.yaw = 0.0;
                }
                
            }); // End Center column
            
            ui.add_space(20.0);
            
            // Right column: Info Panels
            ui.vertical(|ui| {
                ui.set_width(320.0);
                
                let panel_frame = egui::Frame::none()
                    .fill(Color32::from_rgb(30, 35, 45))
                    .rounding(10.0)
                    .inner_margin(Margin::same(15.0))
                    .stroke(Stroke::new(1.0, Color32::from_rgb(50, 55, 65)));
                
                panel_frame.show(ui, |ui| {
                    ui.heading(RichText::new("Info").color(Color32::WHITE).strong());
                    ui.add_space(5.0);
                    ui.separator();
                    ui.add_space(5.0);
                    egui::Grid::new("info_grid").num_columns(2).spacing([40.0, 8.0]).show(ui, |ui| {
                        ui.label(RichText::new("Arming Disable Flags:").color(Color32::from_rgb(180, 180, 180))); 
                        ui.label(RichText::new("0").color(Color32::WHITE).strong()); ui.end_row();
                        
                        ui.label(RichText::new("Battery voltage:").color(Color32::from_rgb(180, 180, 180))); 
                        ui.label(RichText::new("0.0 V").color(Color32::WHITE).strong()); ui.end_row();
                        
                        ui.label(RichText::new("Capacity drawn:").color(Color32::from_rgb(180, 180, 180))); 
                        ui.label(RichText::new("0 mAh").color(Color32::WHITE).strong()); ui.end_row();
                        
                        ui.label(RichText::new("Current draw:").color(Color32::from_rgb(180, 180, 180))); 
                        ui.label(RichText::new("0.00 A").color(Color32::WHITE).strong()); ui.end_row();
                        
                        ui.label(RichText::new("RSSI:").color(Color32::from_rgb(180, 180, 180))); 
                        ui.label(RichText::new("0 %").color(Color32::WHITE).strong()); ui.end_row();
                    });
                });
                
                ui.add_space(15.0);
                
                panel_frame.show(ui, |ui| {
                    ui.heading(RichText::new("GPS").color(Color32::WHITE).strong());
                    ui.add_space(5.0);
                    ui.separator();
                    ui.add_space(5.0);
                    egui::Grid::new("gps_grid").num_columns(2).spacing([40.0, 8.0]).show(ui, |ui| {
                        ui.label(RichText::new("3D Fix:").color(Color32::from_rgb(180, 180, 180))); 
                        ui.label(RichText::new("False").color(Color32::from_rgb(231, 76, 60)).strong()); ui.end_row();
                        
                        ui.label(RichText::new("Sats:").color(Color32::from_rgb(180, 180, 180))); 
                        ui.label(RichText::new("0").color(Color32::WHITE).strong()); ui.end_row();
                        
                        ui.label(RichText::new("Latitude:").color(Color32::from_rgb(180, 180, 180))); 
                        ui.label(RichText::new("0.0000").color(Color32::WHITE).strong()); ui.end_row();
                        
                        ui.label(RichText::new("Longitude:").color(Color32::from_rgb(180, 180, 180))); 
                        ui.label(RichText::new("0.0000").color(Color32::WHITE).strong()); ui.end_row();
                    });
                });
                });
            });
        });
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
                                    self.roll = r;
                                    self.pitch = p;
                                    self.yaw = y;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
