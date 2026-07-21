use eframe::egui;
use std::io::{Read, Write};
use std::time::{Duration, Instant};
use egui::{Color32, RichText};

pub struct ConfigurationState {
    pub cpu_khz: u32,
    pub pwm_hz: u32,
    pub imu_type: String,
    pub sd_logging: bool,
    pub nrf24: bool,
    pub fetched: bool,
    last_poll: Instant,
}

impl Default for ConfigurationState {
    fn default() -> Self {
        Self {
            cpu_khz: 0,
            pwm_hz: 0,
            imu_type: "Unknown".to_string(),
            sd_logging: false,
            nrf24: false,
            fetched: false,
            last_poll: Instant::now(),
        }
    }
}

impl ConfigurationState {
    pub fn ui(&mut self, ui: &mut egui::Ui, port_handle: &mut Option<Box<dyn serialport::SerialPort>>) {
        // Fetch once when tab is opened
        if !self.fetched && port_handle.is_some() && self.last_poll.elapsed() > Duration::from_millis(500) {
            self.last_poll = Instant::now();
            self.fetch_config(port_handle);
        }

        ui.heading(RichText::new("Configuration").size(24.0).strong().color(Color32::WHITE));
        ui.label(RichText::new("Read-only view of the flight controller's compile-time configuration.").color(Color32::from_rgb(150, 150, 150)));
        ui.add_space(15.0);

        if ui.button("Refresh from FC").clicked() {
            self.fetched = false;
        }

        ui.add_space(15.0);
        
        let panel_frame = egui::Frame::none()
            .fill(Color32::from_rgb(30, 35, 45))
            .rounding(10.0)
            .inner_margin(egui::Margin::same(15.0))
            .stroke(egui::Stroke::new(1.0, Color32::from_rgb(50, 55, 65)));
            
        panel_frame.show(ui, |ui| {
            ui.heading(RichText::new("System").strong().color(Color32::WHITE));
            ui.add_space(5.0);
            ui.separator();
            ui.add_space(5.0);
            egui::Grid::new("config_system_grid").num_columns(2).spacing([40.0, 12.0]).show(ui, |ui| {
                ui.label(RichText::new("Board").color(Color32::from_rgb(180, 180, 180))); ui.label(RichText::new("RP2350 (Pico 2)").strong().color(Color32::WHITE)); ui.end_row();
                ui.label(RichText::new("CPU Frequency").color(Color32::from_rgb(180, 180, 180))); ui.label(RichText::new(format!("{} MHz", self.cpu_khz / 1000)).strong().color(Color32::WHITE)); ui.end_row();
                ui.label(RichText::new("Firmware").color(Color32::from_rgb(180, 180, 180))); ui.label(RichText::new("CROX FC v0.1").strong().color(Color32::WHITE)); ui.end_row();
            });
        });

        ui.add_space(15.0);

        panel_frame.show(ui, |ui| {
            ui.heading(RichText::new("Sensors").strong().color(Color32::WHITE));
            ui.add_space(5.0);
            ui.separator();
            ui.add_space(5.0);
            egui::Grid::new("config_sensors_grid").num_columns(2).spacing([40.0, 12.0]).show(ui, |ui| {
                ui.label(RichText::new("IMU").color(Color32::from_rgb(180, 180, 180))); ui.label(RichText::new(&self.imu_type).strong().color(Color32::WHITE)); ui.end_row();
                ui.label(RichText::new("IMU Bus").color(Color32::from_rgb(180, 180, 180))); ui.label(RichText::new("SPI0").strong().color(Color32::WHITE)); ui.end_row();
                ui.label(RichText::new("Magnetometer").color(Color32::from_rgb(180, 180, 180))); ui.label(RichText::new("None").strong().color(Color32::WHITE)); ui.end_row();
                ui.label(RichText::new("Barometer").color(Color32::from_rgb(180, 180, 180))); ui.label(RichText::new("None").strong().color(Color32::WHITE)); ui.end_row();
                ui.label(RichText::new("GPS").color(Color32::from_rgb(180, 180, 180))); ui.label(RichText::new("None").strong().color(Color32::WHITE)); ui.end_row();
            });
        });

        ui.add_space(15.0);

        panel_frame.show(ui, |ui| {
            ui.heading(RichText::new("Motor & ESC").strong().color(Color32::WHITE));
            ui.add_space(5.0);
            ui.separator();
            ui.add_space(5.0);
            egui::Grid::new("config_motor_grid").num_columns(2).spacing([40.0, 12.0]).show(ui, |ui| {
                ui.label(RichText::new("ESC Protocol").color(Color32::from_rgb(180, 180, 180))); ui.label(RichText::new("Standard PWM").strong().color(Color32::WHITE)); ui.end_row();
                ui.label(RichText::new("PWM Frequency").color(Color32::from_rgb(180, 180, 180))); ui.label(RichText::new(format!("{} Hz", self.pwm_hz)).strong().color(Color32::WHITE)); ui.end_row();
                ui.label(RichText::new("Motor Layout").color(Color32::from_rgb(180, 180, 180))); ui.label(RichText::new("Quad X").strong().color(Color32::WHITE)); ui.end_row();
                ui.label(RichText::new("ESC Range").color(Color32::from_rgb(180, 180, 180))); ui.label(RichText::new("1000 - 2000 µs").strong().color(Color32::WHITE)); ui.end_row();
            });
        });

        ui.add_space(15.0);

        panel_frame.show(ui, |ui| {
            ui.heading(RichText::new("Features").strong().color(Color32::WHITE));
            ui.add_space(5.0);
            ui.separator();
            ui.add_space(5.0);
            egui::Grid::new("config_features_grid").num_columns(2).spacing([40.0, 12.0]).show(ui, |ui| {
                ui.label(RichText::new("SD Card Logging").color(Color32::from_rgb(180, 180, 180)));
                let sd_label = if self.sd_logging {
                    RichText::new("ENABLED").color(Color32::from_rgb(46, 204, 113)).strong()
                } else {
                    RichText::new("DISABLED").color(Color32::from_rgb(231, 76, 60))
                };
                ui.label(sd_label);
                ui.end_row();
                
                ui.label(RichText::new("NRF24 Radio").color(Color32::from_rgb(180, 180, 180)));
                let nrf_label = if self.nrf24 {
                    RichText::new("ENABLED").color(Color32::from_rgb(46, 204, 113)).strong()
                } else {
                    RichText::new("DISABLED").color(Color32::from_rgb(231, 76, 60))
                };
                ui.label(nrf_label);
                ui.end_row();
                
                ui.label(RichText::new("Blackbox").color(Color32::from_rgb(180, 180, 180))); ui.label(RichText::new("Flash + SD Card").strong().color(Color32::WHITE)); ui.end_row();
                ui.label(RichText::new("EKF Filter").color(Color32::from_rgb(180, 180, 180))); ui.label(RichText::new("7-state Quaternion EKF").strong().color(Color32::WHITE)); ui.end_row();
            });
        });
    }

    fn fetch_config(&mut self, port_handle: &mut Option<Box<dyn serialport::SerialPort>>) {
        if let Some(port) = port_handle {
            let _ = port.write(b"GET_CONFIG\n");
            let _ = port.flush();
            
            let mut buf = vec![0; 256];
            let _ = port.set_timeout(Duration::from_millis(100));
            std::thread::sleep(Duration::from_millis(50));
            if let Ok(n) = port.read(&mut buf) {
                if let Ok(s) = String::from_utf8(buf[..n].to_vec()) {
                    for line in s.lines() {
                        if line.starts_with("CONFIG,") {
                            let parts: Vec<&str> = line.split(',').collect();
                            if parts.len() == 5 {
                                self.cpu_khz = parts[1].parse().unwrap_or(0);
                                self.pwm_hz = parts[2].parse().unwrap_or(0);
                                self.imu_type = parts[3].to_string();
                                self.sd_logging = parts[4] == "1";
                                // NRF24 is the inverse of SD logging in current config
                                self.nrf24 = !self.sd_logging;
                                self.fetched = true;
                            }
                        }
                    }
                }
            }
        }
    }
}
