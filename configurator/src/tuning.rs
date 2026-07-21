use eframe::egui;
use serialport::SerialPort;
use std::time::Duration;
use std::io::{Read, Write};
use crate::config_writer;

pub struct TuningState {
    pub master_multiplier: f32,
    
    // PIDs (Base values)
    pub acro_p: f32, pub acro_i: f32, pub acro_d: f32,
    pub angle_p: f32, pub angle_i: f32, pub angle_d: f32,
    pub yaw_p: f32, pub yaw_i: f32, pub yaw_d: f32,
    
    // Feed Forward
    pub ff_roll: f32, pub ff_pitch: f32, pub ff_yaw: f32,
    
    // TPA & Constraints
    pub tpa_breakpoint: f32,
    pub tpa_factor: f32,
    pub d_cutoff_hz: f32,
    pub integral_limit: f32,
    
    pub status_msg: String,
}

impl Default for TuningState {
    fn default() -> Self {
        Self {
            master_multiplier: 1.0,
            acro_p: 1.5, acro_i: 0.5, acro_d: 0.05,
            angle_p: 2.0, angle_i: 0.5, angle_d: 0.05,
            yaw_p: 3.0, yaw_i: 1.0, yaw_d: 0.0,
            ff_roll: 0.0, ff_pitch: 0.0, ff_yaw: 0.0,
            tpa_breakpoint: 1350.0, tpa_factor: 0.3,
            d_cutoff_hz: 25.0, integral_limit: 400.0,
            status_msg: "Disconnected".to_string(),
        }
    }
}

impl TuningState {
    pub fn ui(&mut self, ui: &mut egui::Ui, _ctx: &egui::Context, port_handle: &mut Option<Box<dyn SerialPort>>) {
        
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.heading(egui::RichText::new("PID Tuning").size(24.0).strong().color(egui::Color32::WHITE));
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
            
            panel_frame.show(ui, |ui| {
                ui.heading(egui::RichText::new("PID Profiles & Multipliers").strong().color(egui::Color32::WHITE));
                ui.add_space(10.0);
                ui.horizontal(|ui| {
                    ui.label("Master Multiplier:");
                    ui.add(egui::Slider::new(&mut self.master_multiplier, 0.5..=2.0));
                    ui.label(egui::RichText::new("(Scales all PIDs globally)").color(egui::Color32::from_rgb(150, 150, 150)));
                });
            });
            
            ui.add_space(15.0);

            ui.horizontal(|ui| {
                // Left Column: The PIDs
                ui.vertical(|ui| {
                    ui.set_width(350.0);
                    panel_frame.show(ui, |ui| {
                        ui.heading(egui::RichText::new("Acro Flight Mode").strong().color(egui::Color32::WHITE));
                        ui.separator();
                        ui.add_space(10.0);
                        ui.add(egui::Slider::new(&mut self.acro_p, 0.0..=15.0).text("Proportional"));
                        ui.add(egui::Slider::new(&mut self.acro_i, 0.0..=10.0).text("Integral"));
                        ui.add(egui::Slider::new(&mut self.acro_d, 0.0..=2.0).text("Derivative"));
                        ui.add_space(15.0);
                        
                        ui.heading(egui::RichText::new("Angle Flight Mode").strong().color(egui::Color32::WHITE));
                        ui.separator();
                        ui.add_space(10.0);
                        ui.add(egui::Slider::new(&mut self.angle_p, 0.0..=15.0).text("Proportional"));
                        ui.add(egui::Slider::new(&mut self.angle_i, 0.0..=10.0).text("Integral"));
                        ui.add(egui::Slider::new(&mut self.angle_d, 0.0..=2.0).text("Derivative"));
                        ui.add_space(15.0);
                        
                        ui.heading(egui::RichText::new("Yaw Configuration").strong().color(egui::Color32::WHITE));
                        ui.separator();
                        ui.add_space(10.0);
                        ui.add(egui::Slider::new(&mut self.yaw_p, 0.0..=15.0).text("Proportional"));
                        ui.add(egui::Slider::new(&mut self.yaw_i, 0.0..=10.0).text("Integral"));
                        ui.add(egui::Slider::new(&mut self.yaw_d, 0.0..=2.0).text("Derivative"));
                    });
                });
                
                ui.add_space(15.0);
                
                // Right Column: Feed Forward, TPA, Constraints
                ui.vertical(|ui| {
                    ui.set_width(450.0);
                    
                    panel_frame.show(ui, |ui| {
                        ui.heading(egui::RichText::new("Feed Forward (Stick Response)").strong().color(egui::Color32::WHITE));
                        ui.label(egui::RichText::new("Increases responsiveness by predicting stick motion").color(egui::Color32::from_rgb(150, 150, 150)));
                        ui.separator();
                        ui.add_space(10.0);
                        ui.add(egui::Slider::new(&mut self.ff_roll, 0.0..=150.0).text("Roll FF"));
                        ui.add(egui::Slider::new(&mut self.ff_pitch, 0.0..=150.0).text("Pitch FF"));
                        ui.add(egui::Slider::new(&mut self.ff_yaw, 0.0..=150.0).text("Yaw FF"));
                    });
                    
                    ui.add_space(15.0);
                    
                    panel_frame.show(ui, |ui| {
                        ui.heading(egui::RichText::new("TPA (Throttle PID Attenuation)").strong().color(egui::Color32::WHITE));
                        ui.label(egui::RichText::new("Reduces P/D gains at high throttle to prevent oscillations").color(egui::Color32::from_rgb(150, 150, 150)));
                        ui.separator();
                        ui.add_space(10.0);
                        ui.add(egui::Slider::new(&mut self.tpa_breakpoint, 1000.0..=2000.0).text("TPA Breakpoint"));
                        ui.add(egui::Slider::new(&mut self.tpa_factor, 0.0..=1.0).text("TPA Reduction Factor"));
                    });
                    
                    ui.add_space(15.0);
                    
                    panel_frame.show(ui, |ui| {
                        ui.heading(egui::RichText::new("PID Constraints & Filtering").strong().color(egui::Color32::WHITE));
                        ui.separator();
                        ui.add_space(10.0);
                        ui.add(egui::Slider::new(&mut self.integral_limit, 100.0..=1000.0).text("I-Term Windup Limit"));
                        ui.add(egui::Slider::new(&mut self.d_cutoff_hz, 10.0..=100.0).text("D-Term Lowpass Cutoff (Hz)"));
                    });
                });
            });
        });
        
        // Non-blocking read to clear buffer and show ACKs in terminal
        if let Some(port) = port_handle {
            let mut buf = vec![0; 1024];
            if let Ok(n) = port.read(&mut buf) {
                if n > 0 {
                    if let Ok(s) = String::from_utf8(buf[..n].to_vec()) {
                        print!("{}", s);
                    }
                }
            }
        }
    }
    
    pub fn sync_from_fc(&mut self, port_handle: &mut Option<Box<dyn SerialPort>>) {
        if let Some(port) = port_handle {
            let cmds = [
                "GET_PID_ACRO_RP\n",
                "GET_PID_ANGLE_RP\n",
                "GET_PID_YAW\n",
                "GET_FF\n",
                "GET_TPA\n",
                "GET_I_LIMIT\n",
                "GET_D_CUTOFF\n",
            ];
            for cmd in cmds {
                let _ = port.write(cmd.as_bytes());
                std::thread::sleep(Duration::from_millis(15));
            }
            
            let mut buf = vec![0; 4096];
            std::thread::sleep(Duration::from_millis(200));
            if let Ok(n) = port.read(&mut buf) {
                if let Ok(s) = String::from_utf8(buf[..n].to_vec()) {
                    for line in s.lines() {
                        let parts: Vec<&str> = line.split(',').collect();
                        if parts.is_empty() { continue; }
                        match parts[0] {
                            "PID_ACRO_RP" if parts.len() == 4 => {
                                if let (Ok(p), Ok(i), Ok(d)) = (parts[1].parse::<f32>(), parts[2].parse::<f32>(), parts[3].parse::<f32>()) {
                                    self.acro_p = p / self.master_multiplier; 
                                    self.acro_i = i / self.master_multiplier; 
                                    self.acro_d = d / self.master_multiplier;
                                }
                            }
                            "PID_ANGLE_RP" if parts.len() == 4 => {
                                if let (Ok(p), Ok(i), Ok(d)) = (parts[1].parse::<f32>(), parts[2].parse::<f32>(), parts[3].parse::<f32>()) {
                                    self.angle_p = p / self.master_multiplier; 
                                    self.angle_i = i / self.master_multiplier; 
                                    self.angle_d = d / self.master_multiplier;
                                }
                            }
                            "PID_YAW" if parts.len() == 4 => {
                                if let (Ok(p), Ok(i), Ok(d)) = (parts[1].parse::<f32>(), parts[2].parse::<f32>(), parts[3].parse::<f32>()) {
                                    self.yaw_p = p / self.master_multiplier; 
                                    self.yaw_i = i / self.master_multiplier; 
                                    self.yaw_d = d / self.master_multiplier;
                                }
                            }
                            "FF" if parts.len() == 4 => {
                                if let (Ok(fr), Ok(fp), Ok(fy)) = (parts[1].parse::<f32>(), parts[2].parse::<f32>(), parts[3].parse::<f32>()) {
                                    self.ff_roll = fr; self.ff_pitch = fp; self.ff_yaw = fy;
                                }
                            }
                            "TPA" if parts.len() == 3 => {
                                if let (Ok(bp), Ok(fac)) = (parts[1].parse::<f32>(), parts[2].parse::<f32>()) {
                                    self.tpa_breakpoint = bp; self.tpa_factor = fac;
                                }
                            }
                            "I_LIMIT" if parts.len() == 2 => {
                                if let Ok(il) = parts[1].parse::<f32>() { self.integral_limit = il; }
                            }
                            "D_CUTOFF" if parts.len() == 2 => {
                                if let Ok(dc) = parts[1].parse::<f32>() { self.d_cutoff_hz = dc; }
                            }
                            _ => {}
                        }
                    }
                }
            }
        }
    }
    
    fn send_to_fc(&mut self, port_handle: &mut Option<Box<dyn SerialPort>>) {
        if let Some(port) = port_handle {
            let m = self.master_multiplier;
            let cmd_acro = format!("PID_ACRO_RP,{:.4},{:.4},{:.4}\n", self.acro_p * m, self.acro_i * m, self.acro_d * m);
            let cmd_angle = format!("PID_ANGLE_RP,{:.4},{:.4},{:.4}\n", self.angle_p * m, self.angle_i * m, self.angle_d * m);
            let cmd_yaw = format!("PID_YAW,{:.4},{:.4},{:.4}\n", self.yaw_p * m, self.yaw_i * m, self.yaw_d * m);
            let cmd_ff = format!("SET_FF,{:.4},{:.4},{:.4}\n", self.ff_roll, self.ff_pitch, self.ff_yaw);
            let cmd_tpa = format!("SET_TPA,{:.1},{:.3}\n", self.tpa_breakpoint, self.tpa_factor);
            let cmd_ilimit = format!("SET_I_LIMIT,{:.1}\n", self.integral_limit);
            let cmd_dcutoff = format!("SET_D_CUTOFF,{:.1}\n", self.d_cutoff_hz);
            
            let _ = port.write(cmd_acro.as_bytes()); std::thread::sleep(Duration::from_millis(10));
            let _ = port.write(cmd_angle.as_bytes()); std::thread::sleep(Duration::from_millis(10));
            let _ = port.write(cmd_yaw.as_bytes()); std::thread::sleep(Duration::from_millis(10));
            let _ = port.write(cmd_ff.as_bytes()); std::thread::sleep(Duration::from_millis(10));
            let _ = port.write(cmd_tpa.as_bytes()); std::thread::sleep(Duration::from_millis(10));
            let _ = port.write(cmd_ilimit.as_bytes()); std::thread::sleep(Duration::from_millis(10));
            let _ = port.write(cmd_dcutoff.as_bytes()); std::thread::sleep(Duration::from_millis(10));
            
            // Write to config.h
            config_writer::update_config_define("DEFAULT_PID_P_ROLL_PITCH_ACRO", self.acro_p * m);
            config_writer::update_config_define("DEFAULT_PID_I_ROLL_PITCH_ACRO", self.acro_i * m);
            config_writer::update_config_define("DEFAULT_PID_D_ROLL_PITCH_ACRO", self.acro_d * m);
            
            config_writer::update_config_define("DEFAULT_PID_P_ROLL_PITCH_ANGLE", self.angle_p * m);
            config_writer::update_config_define("DEFAULT_PID_I_ROLL_PITCH_ANGLE", self.angle_i * m);
            config_writer::update_config_define("DEFAULT_PID_D_ROLL_PITCH_ANGLE", self.angle_d * m);
            
            config_writer::update_config_define("DEFAULT_PID_P_YAW", self.yaw_p * m);
            config_writer::update_config_define("DEFAULT_PID_I_YAW", self.yaw_i * m);
            config_writer::update_config_define("DEFAULT_PID_D_YAW", self.yaw_d * m);
            
            config_writer::update_config_define("DEFAULT_FF_ROLL", self.ff_roll);
            config_writer::update_config_define("DEFAULT_FF_PITCH", self.ff_pitch);
            config_writer::update_config_define("DEFAULT_FF_YAW", self.ff_yaw);
            
            config_writer::update_config_define("TPA_BREAKPOINT", self.tpa_breakpoint);
            config_writer::update_config_define("TPA_FACTOR", self.tpa_factor);
            config_writer::update_config_define("PID_INTEGRAL_LIMIT", self.integral_limit);
            config_writer::update_config_define("PID_D_CUTOFF_HZ", self.d_cutoff_hz);

            self.status_msg = "Sent configuration to FC and saved to config.h!".to_string();
        } else {
            self.status_msg = "Cannot send, port not connected.".to_string();
        }
    }
}
