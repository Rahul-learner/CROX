use eframe::egui;
use std::io::{Read, Write};
use std::time::Duration;
use egui::{Color32, RichText};

pub struct CliState {
    pub input: String,
    pub log: Vec<String>,
}

impl Default for CliState {
    fn default() -> Self {
        Self {
            input: String::new(),
            log: vec!["Welcome to CROX CLI. Type commands and press Enter.".to_string(),
                       "Examples: GET_EKF, GET_STATUS, GET_RC, GET_MOTORS, REBOOT".to_string(),
                       "---".to_string()],
        }
    }
}

impl CliState {
    pub fn ui(&mut self, ui: &mut egui::Ui, port_handle: &mut Option<Box<dyn serialport::SerialPort>>) {
        ui.heading("CLI — Command Line Interface");
        ui.add_space(5.0);

        // Log area (scrollable)
        let available_height = ui.available_height() - 50.0;
        egui::ScrollArea::vertical()
            .max_height(available_height)
            .auto_shrink([false; 2])
            .stick_to_bottom(true)
            .show(ui, |ui| {
                ui.set_min_width(ui.available_width());
                for line in &self.log {
                    let color = if line.starts_with("> ") {
                        Color32::from_rgb(243, 156, 18) // Sent commands in yellow
                    } else if line.starts_with("ERR") {
                        Color32::from_rgb(231, 76, 60) // Errors in red
                    } else if line.starts_with("ACK") {
                        Color32::from_rgb(46, 204, 113) // Acknowledgements in green
                    } else {
                        Color32::from_rgb(200, 200, 200) // Default light grey
                    };
                    ui.label(RichText::new(line).monospace().color(color));
                }
            });

        ui.separator();

        // Input area
        ui.horizontal(|ui| {
            ui.label(RichText::new(">").monospace().color(Color32::from_rgb(243, 156, 18)));
            
            let input_response = ui.add_sized(
                [ui.available_width() - 80.0, 25.0],
                egui::TextEdit::singleline(&mut self.input)
                    .font(egui::FontId::monospace(14.0))
                    .hint_text("Type command...")
            );
            
            let send_clicked = ui.add_sized([70.0, 25.0],
                egui::Button::new("Send")
                    .fill(Color32::from_rgb(243, 156, 18))
            ).clicked();
            
            if (input_response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter))) || send_clicked {
                if !self.input.is_empty() {
                    let cmd = self.input.clone();
                    self.log.push(format!("> {}", cmd));
                    
                    if let Some(port) = port_handle {
                        let cmd_with_newline = format!("{}\n", cmd);
                        let _ = port.write(cmd_with_newline.as_bytes());
                        let _ = port.flush();
                        
                        // Brief wait then read response
                        std::thread::sleep(Duration::from_millis(100));
                        let mut buf = vec![0; 4096];
                        let _ = port.set_timeout(Duration::from_millis(50));
                        if let Ok(n) = port.read(&mut buf) {
                            if let Ok(s) = String::from_utf8(buf[..n].to_vec()) {
                                for line in s.lines() {
                                    if !line.is_empty() {
                                        self.log.push(line.to_string());
                                    }
                                }
                            }
                        }
                    } else {
                        self.log.push("ERR: Not connected to FC".to_string());
                    }
                    
                    self.input.clear();
                }
                // Re-focus the input
                input_response.request_focus();
            }
        });

        // Also continuously read background data if connected
        if let Some(port) = port_handle {
            let mut buf = vec![0; 1024];
            let _ = port.set_timeout(Duration::from_millis(1));
            if let Ok(n) = port.read(&mut buf) {
                if n > 0 {
                    if let Ok(s) = String::from_utf8(buf[..n].to_vec()) {
                        for line in s.lines() {
                            if !line.is_empty() {
                                self.log.push(line.to_string());
                            }
                        }
                    }
                }
            }
        }
    }
}
