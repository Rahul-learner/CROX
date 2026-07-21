use eframe::egui;
use std::io::{Read, Write};
use std::time::{Duration, Instant};
use egui::{Color32, RichText, Vec2};
use crate::config_writer;

pub struct ReceiverState {
    pub ch_roll: f32,
    pub ch_pitch: f32,
    pub ch_throttle: f32,
    pub ch_yaw: f32,
    
    pub cal_roll: f32,
    pub cal_pitch: f32,
    pub cal_throttle: f32,
    pub cal_yaw: f32,
    
    // Tuning parameters
    pub expo: f32,
    pub deadband: f32,
    pub yaw_deadband: f32,
    pub roll_center: f32,
    pub pitch_center: f32,
    pub yaw_center: f32,
    pub roll_reverse: bool,
    pub pitch_reverse: bool,
    pub yaw_reverse: bool,
    
    pub status_msg: String,
    
    last_poll: Instant,
}

impl Default for ReceiverState {
    fn default() -> Self {
        Self {
            ch_roll: 1500.0, ch_pitch: 1500.0, ch_throttle: 1000.0, ch_yaw: 1500.0,
            cal_roll: 0.0, cal_pitch: 0.0, cal_throttle: 1000.0, cal_yaw: 0.0,
            
            expo: 0.0, deadband: 2.0, yaw_deadband: 2.0,
            roll_center: 1500.0, pitch_center: 1500.0, yaw_center: 1500.0,
            roll_reverse: false, pitch_reverse: false, yaw_reverse: false,
            
            status_msg: "Disconnected".to_string(),
            last_poll: Instant::now(),
        }
    }
}

impl ReceiverState {
    pub fn ui(&mut self, ui: &mut egui::Ui, port_handle: &mut Option<Box<dyn serialport::SerialPort>>) {
        // Poll at 20Hz
        if port_handle.is_some() && self.last_poll.elapsed() > Duration::from_millis(50) {
            self.last_poll = Instant::now();
            self.poll_rc(port_handle);
        }

        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.heading(egui::RichText::new("Receiver & Stick Configuration").size(24.0).strong().color(egui::Color32::WHITE));
                ui.add_space(20.0);
                if ui.add_sized([150.0, 30.0], egui::Button::new(egui::RichText::new("Read from FC").strong())).clicked() {
                    self.sync_from_fc(port_handle);
                }
                ui.add_space(5.0);
                if ui.add_sized([150.0, 30.0], egui::Button::new(egui::RichText::new("Save and Apply").strong().color(egui::Color32::BLACK)).fill(egui::Color32::from_rgb(243, 156, 18))).clicked() {
                    self.send_to_fc(port_handle);
                }
            });
            ui.add_space(15.0);
            
            let panel_frame = egui::Frame::none()
                .fill(egui::Color32::from_rgb(30, 35, 45))
                .rounding(10.0)
                .inner_margin(egui::Margin::same(15.0))
                .stroke(egui::Stroke::new(1.0, egui::Color32::from_rgb(50, 55, 65)));
                
            ui.horizontal(|ui| {
                // Left Column: Live Channels
                ui.vertical(|ui| {
                    ui.set_width(350.0);
                    panel_frame.show(ui, |ui| {
                        ui.heading(egui::RichText::new("Live Channel Monitor (Raw)").strong().color(egui::Color32::WHITE));
                        ui.add_space(10.0);

                        let raw_channels = [
                            ("CH1 — Roll",     self.ch_roll),
                            ("CH2 — Pitch",    self.ch_pitch),
                            ("CH3 — Throttle", self.ch_throttle),
                            ("CH4 — Yaw",      self.ch_yaw),
                        ];

                        let bar_width = 320.0;

                        for (name, raw) in raw_channels {
                            ui.horizontal(|ui| {
                                ui.label(RichText::new(name).strong().size(14.0));
                                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                    ui.label(RichText::new(format!("{:>4.0} µs", raw)).monospace().size(14.0).color(Color32::from_rgb(200, 200, 200)));
                                });
                            });

                            let (rect, _response) = ui.allocate_exact_size(Vec2::new(bar_width, 24.0), egui::Sense::hover());
                            ui.painter().rect_filled(rect, 4.0, Color32::from_rgb(50, 50, 50));
                            let raw_fraction = ((raw - 1000.0) / 1000.0).clamp(0.0, 1.0);
                            let raw_filled = egui::Rect::from_min_max(rect.min, egui::Pos2::new(rect.min.x + rect.width() * raw_fraction, rect.max.y));
                            let raw_center_dist = ((raw - 1500.0).abs() / 500.0).clamp(0.0, 1.0);
                            let raw_color = if name.contains("Throttle") {
                                Color32::from_rgb((raw_fraction * 255.0) as u8, ((1.0 - raw_fraction * 0.5) * 200.0) as u8, 50)
                            } else {
                                Color32::from_rgb((raw_center_dist * 243.0) as u8, ((1.0 - raw_center_dist * 0.3) * 200.0) as u8, 50)
                            };
                            ui.painter().rect_filled(raw_filled, 4.0, raw_color);

                            if !name.contains("Throttle") {
                                let center_x = rect.min.x + rect.width() * 0.5;
                                ui.painter().line_segment(
                                    [egui::Pos2::new(center_x, rect.min.y), egui::Pos2::new(center_x, rect.max.y)],
                                    egui::Stroke::new(2.0, Color32::from_rgb(255, 255, 255)),
                                );
                            }

                            ui.add_space(8.0);
                        }
                    });

                    ui.add_space(15.0);

                    panel_frame.show(ui, |ui| {
                        ui.heading(egui::RichText::new("Calibrated Output").strong().color(egui::Color32::WHITE));
                        ui.add_space(10.0);

                        let cal_channels = [
                            ("Roll Target",     self.cal_roll),
                            ("Pitch Target",    self.cal_pitch),
                            ("Throttle Input",  self.cal_throttle),
                            ("Yaw Target",      self.cal_yaw),
                        ];

                        let bar_width = 320.0;

                        for (name, cal) in cal_channels {
                            ui.horizontal(|ui| {
                                ui.label(RichText::new(name).strong().size(14.0));
                                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                    if name.contains("Throttle") {
                                        ui.label(RichText::new(format!("{:>4.0} µs", cal)).monospace().size(14.0).color(Color32::from_rgb(180, 200, 255)));
                                    } else {
                                        ui.label(RichText::new(format!("{:>5.1} deg/s", cal)).monospace().size(14.0).color(Color32::from_rgb(180, 200, 255)));
                                    }
                                });
                            });

                            let (rect, _response) = ui.allocate_exact_size(Vec2::new(bar_width, 24.0), egui::Sense::hover());
                            ui.painter().rect_filled(rect, 4.0, Color32::from_rgb(50, 50, 50));
                            
                            let cal_fraction = if name.contains("Throttle") {
                                ((cal - 1000.0) / 1000.0).clamp(0.0, 1.0)
                            } else {
                                ((cal + 30.0) / 60.0).clamp(0.0, 1.0)
                            };
                            let cal_filled = egui::Rect::from_min_max(rect.min, egui::Pos2::new(rect.min.x + rect.width() * cal_fraction, rect.max.y));
                            let cal_center_dist = if name.contains("Throttle") { 0.0 } else { (cal.abs() / 30.0).clamp(0.0, 1.0) };
                            
                            let cal_color = if name.contains("Throttle") {
                                Color32::from_rgb((cal_fraction * 200.0) as u8, 150, 255)
                            } else {
                                Color32::from_rgb(100, ((1.0 - cal_center_dist * 0.3) * 200.0) as u8, (cal_center_dist * 255.0) as u8)
                            };
                            ui.painter().rect_filled(cal_filled, 4.0, cal_color);

                            if !name.contains("Throttle") {
                                let center_x = rect.min.x + rect.width() * 0.5;
                                ui.painter().line_segment(
                                    [egui::Pos2::new(center_x, rect.min.y), egui::Pos2::new(center_x, rect.max.y)],
                                    egui::Stroke::new(2.0, Color32::from_rgb(255, 255, 255)),
                                );
                            }

                            ui.add_space(8.0);
                        }
                    });
                });
                
                ui.add_space(15.0);
                
                // Right Column: Tuning
                ui.vertical(|ui| {
                    ui.set_width(450.0);
                    
                    panel_frame.show(ui, |ui| {
                        ui.heading(egui::RichText::new("Stick Adjustments").strong().color(egui::Color32::WHITE));
                        ui.label(egui::RichText::new("Expo & Deadband").color(egui::Color32::from_rgb(150, 150, 150)));
                        ui.separator();
                        ui.add_space(5.0);
                        ui.add(egui::Slider::new(&mut self.expo, 0.0..=1.0).text("RC Expo (Softer mid-stick)"));
                        ui.add(egui::Slider::new(&mut self.deadband, 0.0..=20.0).text("Roll/Pitch Deadband (µs)"));
                        ui.add(egui::Slider::new(&mut self.yaw_deadband, 0.0..=20.0).text("Yaw Deadband (µs)"));
                        ui.add_space(15.0);
                        
                        ui.label(egui::RichText::new("Channel Trims (Centering)").color(egui::Color32::from_rgb(150, 150, 150)));
                        ui.separator();
                        ui.add_space(5.0);
                        ui.add(egui::Slider::new(&mut self.roll_center, 1400.0..=1600.0).text("Roll Center"));
                        ui.add(egui::Slider::new(&mut self.pitch_center, 1400.0..=1600.0).text("Pitch Center"));
                        ui.add(egui::Slider::new(&mut self.yaw_center, 1400.0..=1600.0).text("Yaw Center"));
                        ui.add_space(15.0);
                        
                        ui.label(egui::RichText::new("Channel Reversing").color(egui::Color32::from_rgb(150, 150, 150)));
                        ui.separator();
                        ui.add_space(5.0);
                        ui.horizontal(|ui| {
                            ui.checkbox(&mut self.roll_reverse, "Reverse Roll");
                            ui.add_space(10.0);
                            ui.checkbox(&mut self.pitch_reverse, "Reverse Pitch");
                            ui.add_space(10.0);
                            ui.checkbox(&mut self.yaw_reverse, "Reverse Yaw");
                        });
                    });
                });
            });
            ui.add_space(10.0);
            ui.label(egui::RichText::new(&self.status_msg).color(egui::Color32::from_rgb(46, 204, 113)));
        });
    }

    fn poll_rc(&mut self, port_handle: &mut Option<Box<dyn serialport::SerialPort>>) {
        if let Some(port) = port_handle {
            let _ = port.write(b"GET_RC\n");
            let mut buf = vec![0; 512];
            let _ = port.set_timeout(Duration::from_millis(10));
            if let Ok(n) = port.read(&mut buf) {
                if let Ok(s) = String::from_utf8(buf[..n].to_vec()) {
                    for line in s.lines() {
                        if line.starts_with("RC,") {
                            let parts: Vec<&str> = line.split(',').collect();
                            if parts.len() == 9 {
                                if let (Ok(r), Ok(p), Ok(t), Ok(y), Ok(cr), Ok(cp), Ok(ct), Ok(cy)) = (
                                    parts[1].parse::<f32>(), parts[2].parse::<f32>(),
                                    parts[3].parse::<f32>(), parts[4].parse::<f32>(),
                                    parts[5].parse::<f32>(), parts[6].parse::<f32>(),
                                    parts[7].parse::<f32>(), parts[8].parse::<f32>(),
                                ) {
                                    self.ch_roll = r; self.ch_pitch = p;
                                    self.ch_throttle = t; self.ch_yaw = y;
                                    self.cal_roll = cr; self.cal_pitch = cp;
                                    self.cal_throttle = ct; self.cal_yaw = cy;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    pub fn sync_from_fc(&mut self, port_handle: &mut Option<Box<dyn serialport::SerialPort>>) {
        if let Some(port) = port_handle {
            let _ = port.write(b"GET_RC_TUNE\n");
            std::thread::sleep(Duration::from_millis(50));
            
            let mut buf = vec![0; 1024];
            if let Ok(n) = port.read(&mut buf) {
                if let Ok(s) = String::from_utf8(buf[..n].to_vec()) {
                    for line in s.lines() {
                        if line.starts_with("RC_TUNE,") {
                            let parts: Vec<&str> = line.split(',').collect();
                            if parts.len() == 10 {
                                if let (Ok(ex), Ok(db), Ok(ydb), Ok(rc), Ok(pc), Ok(yc), Ok(rr), Ok(pr), Ok(yr)) = (
                                    parts[1].parse::<f32>(), parts[2].parse::<f32>(), parts[3].parse::<f32>(),
                                    parts[4].parse::<f32>(), parts[5].parse::<f32>(), parts[6].parse::<f32>(),
                                    parts[7].parse::<i32>(), parts[8].parse::<i32>(), parts[9].parse::<i32>()
                                ) {
                                    self.expo = ex; self.deadband = db; self.yaw_deadband = ydb;
                                    self.roll_center = rc; self.pitch_center = pc; self.yaw_center = yc;
                                    self.roll_reverse = rr > 0; self.pitch_reverse = pr > 0; self.yaw_reverse = yr > 0;
                                    self.status_msg = "Synced RC configuration from FC.".to_string();
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    fn send_to_fc(&mut self, port_handle: &mut Option<Box<dyn serialport::SerialPort>>) {
        if let Some(port) = port_handle {
            let cmd = format!("SET_RC_TUNE,{:.4},{:.4},{:.4},{:.1},{:.1},{:.1},{},{},{}\n",
                self.expo, self.deadband, self.yaw_deadband,
                self.roll_center, self.pitch_center, self.yaw_center,
                if self.roll_reverse { 1 } else { 0 },
                if self.pitch_reverse { 1 } else { 0 },
                if self.yaw_reverse { 1 } else { 0 });
            let _ = port.write(cmd.as_bytes());
            
            // Write to config.h
            config_writer::update_config_define("DEFAULT_RC_EXPO", self.expo);
            config_writer::update_config_define("DEFAULT_RC_DEADBAND", self.deadband);
            config_writer::update_config_define("DEFAULT_RC_YAW_DEADBAND", self.yaw_deadband);
            
            config_writer::update_config_define("DEFAULT_RC_ROLL_CENTER", self.roll_center);
            config_writer::update_config_define("DEFAULT_RC_PITCH_CENTER", self.pitch_center);
            config_writer::update_config_define("DEFAULT_RC_YAW_CENTER", self.yaw_center);
            
            config_writer::update_config_define_bool("DEFAULT_RC_ROLL_REVERSE", self.roll_reverse);
            config_writer::update_config_define_bool("DEFAULT_RC_PITCH_REVERSE", self.pitch_reverse);
            config_writer::update_config_define_bool("DEFAULT_RC_YAW_REVERSE", self.yaw_reverse);
            
            self.status_msg = "Sent RC configuration to FC and saved to config.h!".to_string();
        } else {
            self.status_msg = "Cannot send, port not connected.".to_string();
        }
    }
}
