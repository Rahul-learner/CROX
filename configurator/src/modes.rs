use eframe::egui;
use std::io::{Read, Write};
use std::time::{Duration, Instant};
use egui::{Color32, RichText};

pub struct ModesState {
    pub is_armed: bool,
    pub flight_mode: u8, // 0 = Angle, 1 = Acro
    pub loop_time_us: u32,
    pub cpu_freq_khz: u32,
    last_poll: Instant,
    pub status_msg: String,
}

impl Default for ModesState {
    fn default() -> Self {
        Self {
            is_armed: false,
            flight_mode: 0,
            loop_time_us: 0,
            cpu_freq_khz: 0,
            last_poll: Instant::now(),
            status_msg: String::new(),
        }
    }
}

impl ModesState {
    pub fn ui(&mut self, ui: &mut egui::Ui, port_handle: &mut Option<Box<dyn serialport::SerialPort>>) {
        // Poll at 5Hz (modes don't change often)
        if port_handle.is_some() && self.last_poll.elapsed() > Duration::from_millis(200) {
            self.last_poll = Instant::now();
            self.poll_status(port_handle);
        }

        ui.heading(RichText::new("Modes").size(24.0).strong().color(Color32::WHITE));
        ui.add_space(15.0);

        let panel_frame = egui::Frame::none()
            .fill(Color32::from_rgb(30, 35, 45))
            .rounding(10.0)
            .inner_margin(egui::Margin::same(15.0))
            .stroke(egui::Stroke::new(1.0, Color32::from_rgb(50, 55, 65)));

        panel_frame.show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(RichText::new("Arm Status:").size(18.0).strong().color(Color32::from_rgb(200, 200, 200)));
                if self.is_armed {
                    ui.label(RichText::new("ARMED").size(18.0).strong().color(Color32::from_rgb(231, 76, 60)));
                } else {
                    ui.label(RichText::new("DISARMED").size(18.0).strong().color(Color32::from_rgb(46, 204, 113)));
                }
                
                ui.add_space(50.0);
                
                ui.label(RichText::new("Loop Time:").size(14.0).color(Color32::from_rgb(180, 180, 180)));
                ui.label(RichText::new(format!("{} µs", self.loop_time_us)).size(14.0).monospace().color(Color32::WHITE));
                ui.add_space(20.0);
                ui.label(RichText::new("CPU:").size(14.0).color(Color32::from_rgb(180, 180, 180)));
                ui.label(RichText::new(format!("{} MHz", self.cpu_freq_khz / 1000)).size(14.0).monospace().color(Color32::WHITE));
            });
        });

        ui.add_space(30.0);
        
        ui.label(RichText::new("Flight Mode Selection").size(20.0).strong().color(Color32::WHITE));
        ui.label(RichText::new("Select the flight mode. The FC will switch immediately.").color(Color32::from_rgb(150, 150, 150)));
        ui.add_space(20.0);

        let button_size = [250.0, 80.0];
        
        ui.horizontal(|ui| {
            ui.add_space(40.0);
            
            // ANGLE mode button
            let angle_selected = self.flight_mode == 0;
            let angle_fill = if angle_selected {
                Color32::from_rgb(70, 130, 180) // Blue active
            } else {
                Color32::from_rgb(40, 45, 55) // Dark inactive
            };
            let angle_text_color = if angle_selected { Color32::WHITE } else { Color32::from_rgb(150, 150, 150) };
            
            let angle_btn = ui.add_sized(button_size, 
                egui::Button::new(
                    RichText::new("ANGLE MODE\n(Self-leveling)")
                        .size(18.0)
                        .color(angle_text_color)
                        .strong()
                ).fill(angle_fill)
            );
            if angle_btn.clicked() {
                self.set_mode(port_handle, "ANGLE");
            }
            
            ui.add_space(30.0);
            
            // ACRO mode button
            let acro_selected = self.flight_mode == 1;
            let acro_fill = if acro_selected {
                Color32::from_rgb(70, 130, 180) // Blue active
            } else {
                Color32::from_rgb(40, 45, 55) // Dark inactive
            };
            let acro_text_color = if acro_selected { Color32::WHITE } else { Color32::from_rgb(150, 150, 150) };
            
            let acro_btn = ui.add_sized(button_size,
                egui::Button::new(
                    RichText::new("ACRO MODE\n(Rate / Manual)")
                        .size(18.0)
                        .color(acro_text_color)
                        .strong()
                ).fill(acro_fill)
            );
            if acro_btn.clicked() {
                self.set_mode(port_handle, "ACRO");
            }
        });

        ui.add_space(20.0);
        
        if !self.status_msg.is_empty() {
            ui.label(RichText::new(&self.status_msg).color(Color32::from_rgb(46, 204, 113)));
        }

        ui.add_space(30.0);
        ui.separator();
        ui.add_space(10.0);
        
        // Description of modes
        panel_frame.show(ui, |ui| {
            ui.label(RichText::new("Angle Mode").strong().color(Color32::WHITE));
            ui.label(RichText::new("The FC automatically levels the drone when sticks are centered. Good for beginners and stable hovering.").color(Color32::from_rgb(180, 180, 180)));
            ui.add_space(10.0);
            ui.label(RichText::new("Acro Mode").strong().color(Color32::WHITE));
            ui.label(RichText::new("Full rate control — the sticks command rotation rate. No auto-leveling. Required for flips, rolls, and freestyle.").color(Color32::from_rgb(180, 180, 180)));
        });
    }

    fn poll_status(&mut self, port_handle: &mut Option<Box<dyn serialport::SerialPort>>) {
        if let Some(port) = port_handle {
            let _ = port.write(b"GET_STATUS\n");
            let _ = port.flush();
            
            let mut buf = vec![0; 256];
            let _ = port.set_timeout(Duration::from_millis(10));
            if let Ok(n) = port.read(&mut buf) {
                if let Ok(s) = String::from_utf8(buf[..n].to_vec()) {
                    for line in s.lines() {
                        if line.starts_with("STATUS,") {
                            let parts: Vec<&str> = line.split(',').collect();
                            if parts.len() == 5 {
                                if let (Ok(armed), Ok(mode), Ok(loop_us), Ok(cpu)) = (
                                    parts[1].parse::<u8>(), parts[2].parse::<u8>(),
                                    parts[3].parse::<u32>(), parts[4].parse::<u32>(),
                                ) {
                                    self.is_armed = armed != 0;
                                    self.flight_mode = mode;
                                    self.loop_time_us = loop_us;
                                    self.cpu_freq_khz = cpu;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    fn set_mode(&mut self, port_handle: &mut Option<Box<dyn serialport::SerialPort>>, mode: &str) {
        if let Some(port) = port_handle {
            let cmd = format!("SET_MODE,{}\n", mode);
            let _ = port.write(cmd.as_bytes());
            let _ = port.flush();
            self.status_msg = format!("Switched to {} mode", mode);
        }
    }
}
