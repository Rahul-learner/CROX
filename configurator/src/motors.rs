use eframe::egui;
use std::io::{Read, Write};
use std::time::{Duration, Instant};
use egui::{Color32, RichText, Vec2, Stroke};

pub struct MotorsState {
    pub motor1: u32,
    pub motor2: u32,
    pub motor3: u32,
    pub motor4: u32,
    
    // Test mode
    pub test_enabled: bool,
    pub test_m1: u32,
    pub test_m2: u32,
    pub test_m3: u32,
    pub test_m4: u32,
    pub test_confirmed: bool,
    
    // ESC Calibration
    pub esc_cal_active: bool,
    pub esc_msg: String,
    
    last_poll: Instant,
}

impl Default for MotorsState {
    fn default() -> Self {
        Self {
            motor1: 1000, motor2: 1000, motor3: 1000, motor4: 1000,
            test_enabled: false,
            test_m1: 1000, test_m2: 1000, test_m3: 1000, test_m4: 1000,
            test_confirmed: false,
            esc_cal_active: false,
            esc_msg: "".to_string(),
            last_poll: Instant::now(),
        }
    }
}

impl MotorsState {
    pub fn ui(&mut self, ui: &mut egui::Ui, port_handle: &mut Option<Box<dyn serialport::SerialPort>>) {
        // Poll at 20Hz when not in test mode
        if !self.test_enabled && port_handle.is_some() && self.last_poll.elapsed() > Duration::from_millis(50) {
            self.last_poll = Instant::now();
            self.poll_motors(port_handle);
        }

        ui.heading(RichText::new("Motors").size(24.0).strong().color(Color32::WHITE));
        ui.add_space(10.0);
        
        // Draw the quad layout
        let available = ui.available_size();
        let quad_size = available.x.min(available.y - 200.0).min(500.0);
        
        ui.horizontal(|ui| {
            // Left side: quad motor layout
            ui.vertical(|ui| {
                let (rect, _) = ui.allocate_exact_size(Vec2::new(quad_size, quad_size), egui::Sense::hover());
                let painter = ui.painter();
                
                // Background
                painter.rect_filled(rect, 12.0, Color32::from_rgb(25, 28, 35));
                painter.rect_stroke(rect, 12.0, Stroke::new(1.0, Color32::from_rgb(50, 55, 65)));
                
                let center = rect.center();
                let motor_offset = quad_size * 0.3;
                
                // Draw X-frame arms
                let fl = egui::Pos2::new(center.x - motor_offset, center.y - motor_offset);
                let fr = egui::Pos2::new(center.x + motor_offset, center.y - motor_offset);
                let bl = egui::Pos2::new(center.x - motor_offset, center.y + motor_offset);
                let br = egui::Pos2::new(center.x + motor_offset, center.y + motor_offset);
                
                painter.line_segment([center, fl], Stroke::new(6.0, Color32::from_rgb(60, 60, 60)));
                painter.line_segment([center, fr], Stroke::new(6.0, Color32::from_rgb(60, 60, 60)));
                painter.line_segment([center, bl], Stroke::new(6.0, Color32::from_rgb(60, 60, 60)));
                painter.line_segment([center, br], Stroke::new(6.0, Color32::from_rgb(60, 60, 60)));
                
                // Draw center body
                painter.circle_filled(center, 20.0, Color32::from_rgb(40, 40, 40));
                
                // Draw "FRONT" label
                painter.text(
                    egui::Pos2::new(center.x, rect.min.y + 15.0),
                    egui::Align2::CENTER_CENTER,
                    "FRONT",
                    egui::FontId::proportional(14.0),
                    Color32::from_rgb(243, 156, 18),
                );
                
                // Draw motor circles with value
                let motors = [
                    (fl, "M1 (FL)", self.motor1, Color32::from_rgb(46, 204, 113)),
                    (fr, "M2 (FR)", self.motor2, Color32::from_rgb(46, 204, 113)),
                    (bl, "M3 (BL)", self.motor3, Color32::from_rgb(231, 76, 60)),
                    (br, "M4 (BR)", self.motor4, Color32::from_rgb(231, 76, 60)),
                ];
                
                for (pos, label, value, color) in motors {
                    let radius = 40.0;
                    let fill_fraction = ((value as f32 - 1000.0) / 1000.0).clamp(0.0, 1.0);
                    
                    // Background circle
                    painter.circle_filled(pos, radius, Color32::from_rgb(220, 220, 220));
                    
                    // Filled arc (approximated as filled circle with alpha)
                    let fill_color = Color32::from_rgba_unmultiplied(color.r(), color.g(), color.b(), (fill_fraction * 200.0 + 55.0) as u8);
                    painter.circle_filled(pos, radius * fill_fraction.max(0.1), fill_color);
                    
                    // Border
                    painter.circle_stroke(pos, radius, Stroke::new(2.0, color));
                    
                    // Value text
                    painter.text(pos, egui::Align2::CENTER_CENTER,
                        format!("{}", value),
                        egui::FontId::proportional(14.0),
                        Color32::BLACK,
                    );
                    // Label
                    painter.text(
                        egui::Pos2::new(pos.x, pos.y + radius + 12.0),
                        egui::Align2::CENTER_CENTER,
                        label,
                        egui::FontId::proportional(12.0),
                        Color32::from_rgb(100, 100, 100),
                    );
                }
            });
            
            ui.add_space(30.0);
            
            // Right side: Motor test controls
            ui.vertical(|ui| {
                ui.set_width(280.0);
                
                egui::Frame::group(ui.style()).fill(Color32::from_rgb(255, 245, 238)).show(ui, |ui| {
                    ui.heading("Motor Test Mode");
                    ui.separator();
                    
                    if !self.test_confirmed && !self.test_enabled {
                        ui.label(RichText::new("⚠ WARNING: Motor test will spin motors!").color(Color32::RED).strong());
                        ui.label("Remove all propellers before enabling.");
                        ui.add_space(10.0);
                        
                        if ui.button(RichText::new("I understand, enable motor test").color(Color32::RED)).clicked() {
                            self.test_confirmed = true;
                        }
                    }
                    
                    if self.test_confirmed {
                        ui.checkbox(&mut self.test_enabled, "Enable Motor Test");
                        
                        if self.test_enabled {
                            ui.add_space(10.0);
                            ui.label("Drag sliders to set motor speed (1000-1300 µs):");
                            
                            let mut m1 = self.test_m1 as f32;
                            let mut m2 = self.test_m2 as f32;
                            let mut m3 = self.test_m3 as f32;
                            let mut m4 = self.test_m4 as f32;
                            
                            ui.add(egui::Slider::new(&mut m1, 1000.0..=1300.0).text("M1 (FL)"));
                            ui.add(egui::Slider::new(&mut m2, 1000.0..=1300.0).text("M2 (FR)"));
                            ui.add(egui::Slider::new(&mut m3, 1000.0..=1300.0).text("M3 (BL)"));
                            ui.add(egui::Slider::new(&mut m4, 1000.0..=1300.0).text("M4 (BR)"));
                            
                            self.test_m1 = m1 as u32;
                            self.test_m2 = m2 as u32;
                            self.test_m3 = m3 as u32;
                            self.test_m4 = m4 as u32;
                            
                            ui.add_space(10.0);
                            
                            ui.horizontal(|ui| {
                                let send_btn = ui.add_sized([120.0, 30.0],
                                    egui::Button::new(RichText::new("Send to FC").color(Color32::WHITE))
                                        .fill(Color32::from_rgb(243, 156, 18))
                                );
                                if send_btn.clicked() {
                                    self.send_motor_test(port_handle);
                                }
                                
                                let stop_btn = ui.add_sized([120.0, 30.0],
                                    egui::Button::new(RichText::new("STOP ALL").color(Color32::WHITE))
                                        .fill(Color32::from_rgb(231, 76, 60))
                                );
                                if stop_btn.clicked() {
                                    self.test_m1 = 1000;
                                    self.test_m2 = 1000;
                                    self.test_m3 = 1000;
                                    self.test_m4 = 1000;
                                    self.send_motor_test(port_handle);
                                }
                            });
                        }
                    }
                });
                
                ui.add_space(20.0);
                
                // ESC Calibration Wizard
                egui::Frame::group(ui.style()).fill(Color32::from_rgb(40, 20, 20)).show(ui, |ui| {
                    ui.heading(RichText::new("ESC Calibration").color(Color32::from_rgb(255, 100, 100)));
                    ui.separator();
                    ui.label(RichText::new("⚠ WARNING: PROPELLERS MUST BE OFF").strong().color(Color32::RED));
                    ui.add_space(5.0);
                    ui.label("1. Disconnect battery (keep USB connected).");
                    ui.label("2. Click 'Start Calibration' (Motors -> 2000µs).");
                    ui.label("3. Plug in battery, wait for ESC beeps.");
                    ui.label("4. Click 'Finish Calibration' (Motors -> 1000µs).");
                    ui.label("5. Wait for final ESC confirmation beeps.");
                    
                    ui.add_space(10.0);
                    ui.horizontal(|ui| {
                        if ui.add_sized([120.0, 30.0], egui::Button::new("Start (MAX)")).clicked() {
                            if let Some(port) = port_handle {
                                let _ = port.write(b"ESC_CAL_START\n");
                                self.esc_cal_active = true;
                                self.esc_msg = "Motors at MAX. Plug in battery!".to_string();
                            }
                        }
                        
                        if ui.add_sized([120.0, 30.0], egui::Button::new("Finish (MIN)")).clicked() {
                            if let Some(port) = port_handle {
                                let _ = port.write(b"ESC_CAL_END\n");
                                self.esc_cal_active = false;
                                self.esc_msg = "Calibration finished. Disconnect battery.".to_string();
                            }
                        }
                    });
                    
                    if !self.esc_msg.is_empty() {
                        ui.add_space(5.0);
                        ui.label(RichText::new(&self.esc_msg).color(Color32::from_rgb(243, 156, 18)));
                    }
                });
                
                if !self.test_enabled {
                    self.test_confirmed = false;
                }
            });
        });
    }
    
    fn poll_motors(&mut self, port_handle: &mut Option<Box<dyn serialport::SerialPort>>) {
        if let Some(port) = port_handle {
            let _ = port.write(b"GET_MOTORS\n");
            let _ = port.flush();
            
            let mut buf = vec![0; 256];
            let _ = port.set_timeout(Duration::from_millis(10));
            if let Ok(n) = port.read(&mut buf) {
                if let Ok(s) = String::from_utf8(buf[..n].to_vec()) {
                    for line in s.lines() {
                        if line.starts_with("MOTORS,") {
                            let parts: Vec<&str> = line.split(',').collect();
                            if parts.len() == 5 {
                                if let (Ok(m1), Ok(m2), Ok(m3), Ok(m4)) = (
                                    parts[1].parse::<u32>(), parts[2].parse::<u32>(),
                                    parts[3].parse::<u32>(), parts[4].parse::<u32>(),
                                ) {
                                    self.motor1 = m1;
                                    self.motor2 = m2;
                                    self.motor3 = m3;
                                    self.motor4 = m4;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    fn send_motor_test(&self, port_handle: &mut Option<Box<dyn serialport::SerialPort>>) {
        if let Some(port) = port_handle {
            let cmd = format!("SET_MOTOR_TEST,{},{},{},{}\n", self.test_m1, self.test_m2, self.test_m3, self.test_m4);
            let _ = port.write(cmd.as_bytes());
            let _ = port.flush();
        }
    }
}
