use eframe::egui;
use egui_plot::{Line, Plot, PlotPoints, Legend};
use std::fs::File;
use std::io::{Read, Write};

#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct BlackboxPacket {
    pub loop_iteration: u32,
    pub dt_us: u32,
    
    pub setpoint_roll: i32,
    pub setpoint_pitch: i32,
    pub setpoint_yaw: i32,
    
    pub roll: i32,
    pub pitch: i32,
    pub yaw_rate: i32,
    
    pub gyro_x: i32,
    pub gyro_y: i32,
    pub gyro_z: i32,
    
    pub accel_x: i32,
    pub accel_y: i32,
    pub accel_z: i32,
    
    pub pid_roll: i32,
    pub pid_pitch: i32,
    pub pid_yaw: i32,
    
    pub rc_roll: i32,
    pub rc_pitch: i32,
    pub rc_yaw: i32,
    pub rc_throttle: u16,
    
    pub motor1: u16,
    pub motor2: u16,
    pub motor3: u16,
    pub motor4: u16,
}

#[derive(Default, Clone)]
pub struct FlightData {
    pub time_us: Vec<f64>,
    pub setpoint_roll: Vec<f64>,
    pub setpoint_pitch: Vec<f64>,
    pub setpoint_yaw: Vec<f64>,
    
    pub roll: Vec<f64>,
    pub pitch: Vec<f64>,
    pub yaw_rate: Vec<f64>,
    
    pub gyro_x: Vec<f64>,
    pub gyro_y: Vec<f64>,
    pub gyro_z: Vec<f64>,
    
    pub accel_x: Vec<f64>,
    pub accel_y: Vec<f64>,
    pub accel_z: Vec<f64>,
    
    pub pid_roll: Vec<f64>,
    pub pid_pitch: Vec<f64>,
    pub pid_yaw: Vec<f64>,
    
    pub rc_roll: Vec<f64>,
    pub rc_pitch: Vec<f64>,
    pub rc_yaw: Vec<f64>,
    pub rc_throttle: Vec<f64>,
    
    pub motor1: Vec<f64>,
    pub motor2: Vec<f64>,
    pub motor3: Vec<f64>,
    pub motor4: Vec<f64>,
}

#[derive(Default)]
pub struct BlackboxState {
    pub status_msg: String,
    
    // Download state
    pub is_downloading: bool,
    pub download_progress: f32,
    pub expected_packets: usize,
    pub raw_buffer: Vec<u8>,
    
    // Parsed flights
    pub flights: Vec<FlightData>,
    pub current_flight_idx: usize,
    
    // UI state
    pub show_gyro: bool,
    pub show_setpoint: bool,
    pub show_rc: bool,
    pub show_pid: bool,
    pub show_motors: bool,
}

impl BlackboxState {
    pub fn new() -> Self {
        let mut s = Self::default();
        s.show_gyro = true;
        s.show_setpoint = true;
        s
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, port_handle: &mut Option<Box<dyn serialport::SerialPort>>) {
        ui.heading(egui::RichText::new("Blackbox Explorer (Betaflight Style)").size(24.0).strong().color(egui::Color32::WHITE));
        ui.add_space(10.0);
        
        ui.horizontal(|ui| {
            if ui.button("Get Info").clicked() {
                if let Some(port) = port_handle {
                    self.request_info(port);
                }
            }
            if ui.button("Download All").clicked() {
                if let Some(port) = port_handle {
                    self.start_download(port);
                }
            }
            if ui.button("Clear Flash").clicked() {
                if let Some(port) = port_handle {
                    let _ = port.write(b"CLEAR_BLACKBOX\n");
                    let _ = port.flush();
                    self.status_msg = "Flash cleared".to_string();
                }
            }
        });
        
        if self.is_downloading {
            ui.add(egui::ProgressBar::new(self.download_progress).text(format!("{:.1}%", self.download_progress * 100.0)));
            // Perform download pump
            if let Some(port) = port_handle {
                self.pump_download(port);
            }
        }
        
        ui.label(egui::RichText::new(&self.status_msg).color(egui::Color32::LIGHT_BLUE));
        ui.separator();
        
        if self.flights.is_empty() {
            ui.label("No data loaded. Download from FC or wait for finish.");
            return;
        }
        
        ui.horizontal(|ui| {
            ui.label("Select Flight:");
            egui::ComboBox::from_id_source("flight_selector")
                .selected_text(format!("Flight {}", self.current_flight_idx + 1))
                .show_ui(ui, |ui| {
                    for i in 0..self.flights.len() {
                        ui.selectable_value(&mut self.current_flight_idx, i, format!("Flight {}", i + 1));
                    }
                });
        });
        
        ui.horizontal(|ui| {
            ui.checkbox(&mut self.show_gyro, "Gyro (Roll/Pitch/Yaw)");
            ui.checkbox(&mut self.show_setpoint, "Setpoint (Roll/Pitch/Yaw)");
            ui.checkbox(&mut self.show_rc, "RC Command");
            ui.checkbox(&mut self.show_pid, "PID Output");
            ui.checkbox(&mut self.show_motors, "Motors");
        });
        
        let flight = &self.flights[self.current_flight_idx];
        
        let plot = Plot::new("blackbox_plot")
            .legend(Legend::default())
            .view_aspect(2.0)
            .x_axis_formatter(|mark, _, _| format!("{:.2} s", mark.value));
            
        plot.show(ui, |plot_ui| {
            if self.show_gyro {
                let r: PlotPoints = flight.time_us.iter().zip(flight.roll.iter()).map(|(&t, &v)| [t, v]).collect();
                let p: PlotPoints = flight.time_us.iter().zip(flight.pitch.iter()).map(|(&t, &v)| [t, v]).collect();
                let y: PlotPoints = flight.time_us.iter().zip(flight.yaw_rate.iter()).map(|(&t, &v)| [t, v]).collect();
                plot_ui.line(Line::new(r).name("Gyro Roll").color(egui::Color32::RED));
                plot_ui.line(Line::new(p).name("Gyro Pitch").color(egui::Color32::BLUE));
                plot_ui.line(Line::new(y).name("Gyro Yaw").color(egui::Color32::GREEN));
            }
            
            if self.show_setpoint {
                let r: PlotPoints = flight.time_us.iter().zip(flight.setpoint_roll.iter()).map(|(&t, &v)| [t, v]).collect();
                let p: PlotPoints = flight.time_us.iter().zip(flight.setpoint_pitch.iter()).map(|(&t, &v)| [t, v]).collect();
                let y: PlotPoints = flight.time_us.iter().zip(flight.setpoint_yaw.iter()).map(|(&t, &v)| [t, v]).collect();
                plot_ui.line(Line::new(r).name("Setpoint Roll").color(egui::Color32::from_rgb(255, 128, 128)));
                plot_ui.line(Line::new(p).name("Setpoint Pitch").color(egui::Color32::from_rgb(128, 128, 255)));
                plot_ui.line(Line::new(y).name("Setpoint Yaw").color(egui::Color32::from_rgb(128, 255, 128)));
            }
            
            if self.show_rc {
                let r: PlotPoints = flight.time_us.iter().zip(flight.rc_roll.iter()).map(|(&t, &v)| [t, v]).collect();
                let p: PlotPoints = flight.time_us.iter().zip(flight.rc_pitch.iter()).map(|(&t, &v)| [t, v]).collect();
                let y: PlotPoints = flight.time_us.iter().zip(flight.rc_yaw.iter()).map(|(&t, &v)| [t, v]).collect();
                let thr: PlotPoints = flight.time_us.iter().zip(flight.rc_throttle.iter()).map(|(&t, &v)| [t, v]).collect();
                plot_ui.line(Line::new(r).name("RC Roll").color(egui::Color32::YELLOW));
                plot_ui.line(Line::new(p).name("RC Pitch").color(egui::Color32::KHAKI));
                plot_ui.line(Line::new(y).name("RC Yaw").color(egui::Color32::GOLD));
                plot_ui.line(Line::new(thr).name("RC Throttle").color(egui::Color32::LIGHT_YELLOW));
            }
            
            if self.show_pid {
                let r: PlotPoints = flight.time_us.iter().zip(flight.pid_roll.iter()).map(|(&t, &v)| [t, v]).collect();
                let p: PlotPoints = flight.time_us.iter().zip(flight.pid_pitch.iter()).map(|(&t, &v)| [t, v]).collect();
                let y: PlotPoints = flight.time_us.iter().zip(flight.pid_yaw.iter()).map(|(&t, &v)| [t, v]).collect();
                plot_ui.line(Line::new(r).name("PID Roll").color(egui::Color32::from_rgb(255, 0, 255)));
                plot_ui.line(Line::new(p).name("PID Pitch").color(egui::Color32::from_rgb(180, 0, 255)));
                plot_ui.line(Line::new(y).name("PID Yaw").color(egui::Color32::from_rgb(200, 0, 200)));
            }
            
            if self.show_motors {
                let m1: PlotPoints = flight.time_us.iter().zip(flight.motor1.iter()).map(|(&t, &v)| [t, v]).collect();
                let m2: PlotPoints = flight.time_us.iter().zip(flight.motor2.iter()).map(|(&t, &v)| [t, v]).collect();
                let m3: PlotPoints = flight.time_us.iter().zip(flight.motor3.iter()).map(|(&t, &v)| [t, v]).collect();
                let m4: PlotPoints = flight.time_us.iter().zip(flight.motor4.iter()).map(|(&t, &v)| [t, v]).collect();
                plot_ui.line(Line::new(m1).name("Motor 1").color(egui::Color32::WHITE));
                plot_ui.line(Line::new(m2).name("Motor 2").color(egui::Color32::LIGHT_GRAY));
                plot_ui.line(Line::new(m3).name("Motor 3").color(egui::Color32::DARK_GRAY));
                plot_ui.line(Line::new(m4).name("Motor 4").color(egui::Color32::GRAY));
            }
        });
    }
    
    fn request_info(&mut self, port: &mut Box<dyn serialport::SerialPort>) {
        let _ = port.write(b"GET_BLACKBOX_INFO\n");
        let _ = port.flush();
        let _ = port.set_timeout(std::time::Duration::from_millis(2000));
        
        let mut byte_buf = [0u8; 1];
        let mut resp = String::new();
        while let Ok(1) = port.read(&mut byte_buf) {
            let c = byte_buf[0] as char;
            resp.push(c);
            if c == '\n' { break; }
        }
        
        if resp.starts_with("BLACKBOX_INFO") {
            let parts: Vec<&str> = resp.trim().split(',').collect();
            if parts.len() == 3 {
                if let Ok(packets) = parts[1].parse::<usize>() {
                    self.status_msg = format!("Found {} packets ready to download", packets);
                    self.expected_packets = packets;
                }
            }
        }
    }
    
    pub fn start_download(&mut self, port: &mut Box<dyn serialport::SerialPort>) {
        let _ = port.write(b"GET_BLACKBOX\n");
        let _ = port.flush();
        let _ = port.set_timeout(std::time::Duration::from_millis(5000));
        
        let mut byte_buf = [0u8; 1];
        let mut resp = String::new();
        while let Ok(1) = port.read(&mut byte_buf) {
            let c = byte_buf[0] as char;
            resp.push(c);
            if c == '\n' { break; }
        }
        
        if resp.starts_with("BLACKBOX_BIN_START") {
            let parts: Vec<&str> = resp.trim().split(',').collect();
            if parts.len() == 3 {
                if let Ok(packets) = parts[1].parse::<usize>() {
                    self.expected_packets = packets;
                    self.is_downloading = true;
                    self.download_progress = 0.0;
                    self.raw_buffer.clear();
                    self.status_msg = format!("Downloading {} packets...", packets);
                    let _ = port.set_timeout(std::time::Duration::from_millis(100)); // Non-blocking reads
                }
            }
        }
    }
    
    fn pump_download(&mut self, port: &mut Box<dyn serialport::SerialPort>) {
        let packet_size = std::mem::size_of::<BlackboxPacket>();
        let target_size = self.expected_packets * packet_size;
        
        let mut buf = vec![0u8; 4096];
        if let Ok(n) = port.read(&mut buf) {
            if n > 0 {
                self.raw_buffer.extend_from_slice(&buf[..n]);
                if target_size > 0 {
                    self.download_progress = (self.raw_buffer.len() as f32 / target_size as f32).clamp(0.0, 1.0);
                }
                
                // Look for END marker
                let search_len = std::cmp::min(self.raw_buffer.len(), 100);
                let tail = &self.raw_buffer[self.raw_buffer.len() - search_len..];
                if let Some(idx) = tail.windows(16).position(|w| w == b"BLACKBOX_BIN_END") {
                    self.is_downloading = false;
                    let end_pos = self.raw_buffer.len() - search_len + idx;
                    self.raw_buffer.truncate(end_pos);
                    self.parse_binary();
                }
            }
        }
    }
    
    fn parse_binary(&mut self) {
        self.flights.clear();
        self.current_flight_idx = 0;
        
        let packet_size = std::mem::size_of::<BlackboxPacket>();
        if self.raw_buffer.len() % packet_size != 0 {
            self.status_msg = format!("Warning: Buffer size {} is not a multiple of packet size {}", self.raw_buffer.len(), packet_size);
        }
        
        let num_packets = self.raw_buffer.len() / packet_size;
        
        let mut current_flight = FlightData::default();
        let mut cum_time_us: f64 = 0.0;
        
        for i in 0..num_packets {
            let start = i * packet_size;
            let end = start + packet_size;
            let chunk = &self.raw_buffer[start..end];
            
            // Unsafe transmute to struct
            let packet: BlackboxPacket = unsafe { std::ptr::read_unaligned(chunk.as_ptr() as *const _) };
            
            if packet.dt_us == 0xFFFFFFFF {
                // EOF marker, separate flight
                if !current_flight.time_us.is_empty() {
                    self.flights.push(current_flight);
                    current_flight = FlightData::default();
                    cum_time_us = 0.0;
                }
                continue;
            }
            
            cum_time_us += packet.dt_us as f64 / 1_000_000.0;
            current_flight.time_us.push(cum_time_us);
            
            current_flight.setpoint_roll.push(packet.setpoint_roll as f64 / 100.0);
            current_flight.setpoint_pitch.push(packet.setpoint_pitch as f64 / 100.0);
            current_flight.setpoint_yaw.push(packet.setpoint_yaw as f64 / 100.0);
            
            current_flight.roll.push(packet.roll as f64 / 100.0);
            current_flight.pitch.push(packet.pitch as f64 / 100.0);
            current_flight.yaw_rate.push(packet.yaw_rate as f64 / 100.0);
            
            current_flight.gyro_x.push(packet.gyro_x as f64 / 1000.0);
            current_flight.gyro_y.push(packet.gyro_y as f64 / 1000.0);
            current_flight.gyro_z.push(packet.gyro_z as f64 / 1000.0);
            
            current_flight.accel_x.push(packet.accel_x as f64 / 1000.0);
            current_flight.accel_y.push(packet.accel_y as f64 / 1000.0);
            current_flight.accel_z.push(packet.accel_z as f64 / 1000.0);
            
            current_flight.pid_roll.push(packet.pid_roll as f64 / 100.0);
            current_flight.pid_pitch.push(packet.pid_pitch as f64 / 100.0);
            current_flight.pid_yaw.push(packet.pid_yaw as f64 / 100.0);
            
            current_flight.rc_roll.push(packet.rc_roll as f64 / 100.0);
            current_flight.rc_pitch.push(packet.rc_pitch as f64 / 100.0);
            current_flight.rc_yaw.push(packet.rc_yaw as f64 / 100.0);
            current_flight.rc_throttle.push(packet.rc_throttle as f64);
            
            current_flight.motor1.push(packet.motor1 as f64);
            current_flight.motor2.push(packet.motor2 as f64);
            current_flight.motor3.push(packet.motor3 as f64);
            current_flight.motor4.push(packet.motor4 as f64);
        }
        
        if !current_flight.time_us.is_empty() {
            self.flights.push(current_flight);
        }
        
        self.status_msg = format!("Parsed {} flights from {} packets", self.flights.len(), num_packets);
        
        // Save to file for easy export
        if let Ok(mut f) = File::create("blackbox_dump.bin") {
            let _ = f.write_all(&self.raw_buffer);
        }
    }
}
