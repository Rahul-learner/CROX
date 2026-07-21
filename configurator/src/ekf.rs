use eframe::egui;
use serialport::SerialPort;
use std::time::Duration;
use std::io::{Read, Write};
use crate::config_writer;

pub struct EkfState {
    pub q_gyro: f32,
    pub q_bias: f32,
    pub r_accel: f32,
    pub status_msg: String,
}

impl Default for EkfState {
    fn default() -> Self {
        Self {
            q_gyro: 0.1,
            q_bias: 0.001,
            r_accel: 10.0,
            status_msg: "Disconnected".to_string(),
        }
    }
}

impl EkfState {
    pub fn ui(&mut self, ui: &mut egui::Ui, _ctx: &egui::Context, port_handle: &mut Option<Box<dyn SerialPort>>) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.heading(egui::RichText::new("EKF & Sensor Tuning").size(24.0).strong().color(egui::Color32::WHITE));
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
                ui.vertical(|ui| {
                    ui.set_width(350.0);
                    panel_frame.show(ui, |ui| {
                        ui.heading(egui::RichText::new("Extended Kalman Filter Matrices").strong().color(egui::Color32::WHITE));
                        ui.label(egui::RichText::new("Process Noise (Q) and Measurement Noise (R)").color(egui::Color32::from_rgb(150, 150, 150)));
                        ui.separator();
                        ui.add_space(10.0);
                        ui.add(egui::Slider::new(&mut self.q_gyro, 0.0..=1.0).text("Q Gyro (Process Noise)"));
                        ui.add(egui::Slider::new(&mut self.q_bias, 0.0..=0.1).text("Q Bias (Drift Noise)"));
                        ui.add(egui::Slider::new(&mut self.r_accel, 0.0..=50.0).text("R Accel (Measurement Noise)"));
                    });
                });
                
                ui.add_space(15.0);
                
                ui.vertical(|ui| {
                    ui.set_width(450.0);
                    panel_frame.show(ui, |ui| {
                        ui.heading(egui::RichText::new("Sensor Calibration").strong().color(egui::Color32::WHITE));
                        ui.label(egui::RichText::new("Calibrate IMU hardware directly").color(egui::Color32::from_rgb(150, 150, 150)));
                        ui.separator();
                        ui.add_space(10.0);
                        
                        if ui.button(egui::RichText::new("Calibrate Accelerometer").size(16.0).color(egui::Color32::from_rgb(52, 152, 219))).clicked() {
                            if let Some(port) = port_handle {
                                let _ = port.write("CALIBRATE_ACCEL\n".as_bytes());
                                self.status_msg = "Requested Accelerometer Calibration. Keep drone still on level surface...".to_string();
                            }
                        }
                        ui.label("Calibrates level attitude using gravity vector on a flat surface.");
                        
                        ui.add_space(5.0);
                        if ui.button(egui::RichText::new("Save Accel Bias to config.h").size(14.0).color(egui::Color32::from_rgb(46, 204, 113))).clicked() {
                            self.fetch_accel_bias(port_handle);
                        }
                        
                        ui.add_space(15.0);
                        
                        if ui.button(egui::RichText::new("Run EKF Noise Calibration").size(16.0).color(egui::Color32::from_rgb(231, 76, 60))).clicked() {
                            if let Some(port) = port_handle {
                                let _ = port.write("CALIBRATE_NOISE\n".as_bytes());
                                self.status_msg = "Requested Noise Calibration. WARNING: Propellers will spin!".to_string();
                            }
                        }
                        ui.label(egui::RichText::new("WARNING: REMOVE PROPELLERS!").strong().color(egui::Color32::from_rgb(231, 76, 60)));
                        ui.label("Spins motors to measure baseline frame vibrations for R matrix.");
                    });
                });
            });
            
            ui.add_space(10.0);
            ui.label(egui::RichText::new(&self.status_msg).color(egui::Color32::from_rgb(46, 204, 113)));
        });
        
        // Drain buffer for terminal output
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
            let _ = port.write("GET_EKF\n".as_bytes());
            std::thread::sleep(Duration::from_millis(100));
            
            let mut buf = vec![0; 1024];
            if let Ok(n) = port.read(&mut buf) {
                if let Ok(s) = String::from_utf8(buf[..n].to_vec()) {
                    for line in s.lines() {
                        let parts: Vec<&str> = line.split(',').collect();
                        if parts.len() == 4 && parts[0] == "EKF" {
                            if let (Ok(qg), Ok(qb), Ok(ra)) = (parts[1].parse::<f32>(), parts[2].parse::<f32>(), parts[3].parse::<f32>()) {
                                self.q_gyro = qg; self.q_bias = qb; self.r_accel = ra;
                                self.status_msg = "EKF parameters synced from FC.".to_string();
                            }
                        }
                    }
                }
            }
        }
    }
    
    fn send_to_fc(&mut self, port_handle: &mut Option<Box<dyn SerialPort>>) {
        if let Some(port) = port_handle {
            let cmd_ekf = format!("EKF,{:.4},{:.5},{:.2}\n", self.q_gyro, self.q_bias, self.r_accel);
            let _ = port.write(cmd_ekf.as_bytes());
            
            // Write to config.h
            config_writer::update_config_define("DEFAULT_Q_GYRO", self.q_gyro);
            config_writer::update_config_define("DEFAULT_Q_BIAS", self.q_bias);
            config_writer::update_config_define("DEFAULT_R_ACCEL", self.r_accel);
            
            self.status_msg = "Sent EKF parameters to FC and saved to config.h!".to_string();
        } else {
            self.status_msg = "Cannot send, port not connected.".to_string();
        }
    }
    
    fn fetch_accel_bias(&mut self, port_handle: &mut Option<Box<dyn SerialPort>>) {
        if let Some(port) = port_handle {
            let _ = port.write(b"GET_ACCEL_BIAS\n");
            std::thread::sleep(Duration::from_millis(50));
            
            let mut buf = vec![0; 256];
            if let Ok(n) = port.read(&mut buf) {
                if let Ok(s) = String::from_utf8(buf[..n].to_vec()) {
                    for line in s.lines() {
                        if line.starts_with("ACCEL_BIAS,") {
                            let parts: Vec<&str> = line.split(',').collect();
                            if parts.len() == 3 {
                                if let (Ok(roll), Ok(pitch)) = (parts[1].parse::<f32>(), parts[2].parse::<f32>()) {
                                    config_writer::update_config_define("DEFAULT_BIAS_ROLL", roll);
                                    config_writer::update_config_define("DEFAULT_BIAS_PITCH", pitch);
                                    self.status_msg = format!("Saved Accel Bias to config.h! Roll: {:.2}, Pitch: {:.2}", roll, pitch);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
