mod setup;
mod tuning;
mod blackbox;
mod receiver;
mod motors;
mod modes;
mod configuration;
mod cli;
mod ekf;
pub mod config_writer;

use eframe::egui;
use std::time::Duration;
use egui::{Color32, RichText, Frame, Margin};

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_fullscreen(true)
            .with_title("CROX Configurator"),
        ..Default::default()
    };
    eframe::run_native(
        "CROX Configurator",
        options,
        Box::new(|cc| {
            let mut visuals = egui::Visuals::dark();
            visuals.panel_fill = Color32::from_rgb(18, 20, 25);
            visuals.window_fill = Color32::from_rgb(25, 28, 35);
            visuals.widgets.noninteractive.bg_fill = Color32::from_rgb(30, 35, 45);
            visuals.widgets.noninteractive.bg_stroke = egui::Stroke::new(1.0, Color32::from_rgb(50, 55, 65));
            visuals.widgets.inactive.bg_fill = Color32::from_rgb(40, 45, 55);
            visuals.widgets.inactive.rounding = egui::Rounding::same(8.0);
            visuals.widgets.hovered.bg_fill = Color32::from_rgb(55, 65, 80);
            visuals.widgets.hovered.rounding = egui::Rounding::same(8.0);
            visuals.widgets.active.bg_fill = Color32::from_rgb(70, 130, 180);
            visuals.widgets.active.rounding = egui::Rounding::same(8.0);
            visuals.selection.bg_fill = Color32::from_rgb(60, 120, 200);
            visuals.window_rounding = egui::Rounding::same(12.0);
            
            cc.egui_ctx.set_visuals(visuals);
            
            // Adjust fonts to be slightly larger for modern look
            let mut fonts = egui::FontDefinitions::default();
            for family in fonts.font_data.values_mut() {
                // We'll just rely on default fonts but egui allows sizing in UI, 
                // we will style text individually in tabs.
            }
            cc.egui_ctx.set_fonts(fonts);
            
            Box::<ConfiguratorApp>::default()
        }),
    )
}

#[derive(PartialEq, Clone, Copy)]
pub enum ConnectionMode {
    Disconnected,
    DirectUSB,
}

#[derive(PartialEq, Clone, Copy)]
pub enum LogLevel {
    Info,
    Tx,
    Rx,
    Warn,
    Error,
}

pub struct LogEntry {
    pub timestamp: String,
    pub level: LogLevel,
    pub message: String,
}

use std::sync::Mutex;
pub static LOGS: Mutex<Vec<LogEntry>> = Mutex::new(Vec::new());

pub fn log_event(message: &str, level: LogLevel) {
    if let Ok(mut logs) = LOGS.lock() {
        let now = chrono::Local::now();
        let timestamp = now.format("%H:%M:%S%.3f").to_string();
        logs.push(LogEntry {
            timestamp,
            level,
            message: message.to_string(),
        });
        if logs.len() > 1000 {
            logs.drain(0..100);
        }
    }
}

struct ConfiguratorApp {
    selected_tab: Tab,
    port: Option<Box<dyn serialport::SerialPort>>,
    
    // Connection State
    ports: Vec<String>,
    selected_port: String,
    status_msg: String,
    connection_mode: ConnectionMode,
    auto_connect_attempted: bool,
    
    // Tab States
    setup_state: setup::SetupState,
    tuning_state: tuning::TuningState,
    blackbox_state: blackbox::BlackboxState,
    receiver_state: receiver::ReceiverState,
    motors_state: motors::MotorsState,
    modes_state: modes::ModesState,
    configuration_state: configuration::ConfigurationState,
    cli_state: cli::CliState,
    ekf_state: ekf::EkfState,
    
    // Log Terminal
    log_terminal_open: bool,
}

impl Default for ConfiguratorApp {
    fn default() -> Self {
        Self {
            selected_tab: Tab::Setup,
            port: None,
            ports: Vec::new(),
            selected_port: String::new(),
            status_msg: "Disconnected".to_string(),
            connection_mode: ConnectionMode::Disconnected,
            auto_connect_attempted: false,
            setup_state: setup::SetupState::default(),
            tuning_state: tuning::TuningState::default(),
            blackbox_state: blackbox::BlackboxState::default(),
            receiver_state: receiver::ReceiverState::default(),
            motors_state: motors::MotorsState::default(),
            modes_state: modes::ModesState::default(),
            configuration_state: configuration::ConfigurationState::default(),
            cli_state: cli::CliState::default(),
            ekf_state: ekf::EkfState::default(),
            log_terminal_open: true, // Open by default
        }
    }
}

#[derive(PartialEq, Clone, Copy)]
enum Tab {
    Setup,
    Ports,
    Configuration,
    PowerBattery,
    Tuning,
    EKF,
    Receiver,
    Modes,
    Motors,
    OSD,
    LedStrip,
    Blackbox,
    CLI,
}

impl Tab {
    fn name(&self) -> &'static str {
        match self {
            Tab::Setup => "Setup",
            Tab::Ports => "Ports",
            Tab::Configuration => "Configuration",
            Tab::PowerBattery => "Power & Battery",
            Tab::Tuning => "PID Tuning",
            Tab::EKF => "EKF / Sensors",
            Tab::Receiver => "Receiver",
            Tab::Modes => "Modes",
            Tab::Motors => "Motors",
            Tab::OSD => "OSD",
            Tab::LedStrip => "LED Strip",
            Tab::Blackbox => "Blackbox",
            Tab::CLI => "CLI",
        }
    }
    
    fn is_implemented(&self) -> bool {
        matches!(self, Tab::Setup | Tab::Tuning | Tab::EKF | Tab::Blackbox | 
                       Tab::Receiver | Tab::Motors | Tab::Modes | 
                       Tab::Configuration | Tab::CLI)
    }
    
    fn icon(&self) -> &'static str {
        match self {
            Tab::Setup => "⚙",
            Tab::Ports => "🔌",
            Tab::Configuration => "🛠",
            Tab::PowerBattery => "🔋",
            Tab::Tuning => "📊",
            Tab::EKF => "🧭",
            Tab::Receiver => "📡",
            Tab::Modes => "🎮",
            Tab::Motors => "⚡",
            Tab::OSD => "🖥",
            Tab::LedStrip => "💡",
            Tab::Blackbox => "📦",
            Tab::CLI => "⌨",
        }
    }
}

impl eframe::App for ConfiguratorApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        
        // --- AUTO CONNECT ---
        if !self.auto_connect_attempted {
            self.auto_connect_attempted = true;
            if let Ok(ports) = serialport::available_ports() {
                for p in &ports {
                    self.ports.push(p.port_name.clone());
                }
                
                // Try to find a USB port first
                let target_port = ports.iter()
                    .find(|p| p.port_name.contains("USB") || p.port_name.contains("ACM") || p.port_name.contains("usb"))
                    .or_else(|| ports.first());
                    
                if let Some(port_info) = target_port {
                    self.selected_port = port_info.port_name.clone();
                    match serialport::new(&self.selected_port, 115200).timeout(Duration::from_millis(100)).open() {
                        Ok(mut port) => {
                            self.port = Some(port.try_clone().unwrap());
                            self.status_msg = "Connected (Probing...)".to_string();
                            self.connection_mode = ConnectionMode::DirectUSB; // Default

                            // Probe device type
                            log_event("Probing device type with GET_CONFIG...", LogLevel::Tx);
                            if let Err(e) = port.write_all(b"GET_CONFIG\n") {
                                println!("Probe error: {}", e);
                                log_event(&format!("Probe error: {}", e), LogLevel::Error);
                            }
                            std::thread::sleep(Duration::from_millis(100));
                            let mut buf = [0u8; 1024];
                            if let Ok(n) = port.read(&mut buf) {
                                let resp = String::from_utf8_lossy(&buf[..n]);
                                log_event(&format!("RX: {}", resp.trim()), LogLevel::Rx);
                            }
                            self.connection_mode = ConnectionMode::DirectUSB;
                            self.status_msg = "Connected (Direct USB)".to_string();
                            log_event("Detected Direct USB connection", LogLevel::Info);
                        }
                        Err(e) => {
                            self.status_msg = format!("Auto-connect failed: {}", e);
                            self.connection_mode = ConnectionMode::Disconnected;
                        }
                    }
                }
            }
        }
        
        // --- TOP PANEL ---
        let top_frame = Frame::default()
            .fill(Color32::from_rgb(32, 32, 32))
            .inner_margin(Margin::same(8.0));
            
        egui::TopBottomPanel::top("top_panel").frame(top_frame).show(ctx, |ui| {
            ui.horizontal(|ui| {
                // Logo & Title
                ui.label(
                    RichText::new("CROX")
                        .color(Color32::from_rgb(243, 156, 18))
                        .size(26.0)
                        .strong()
                );
                ui.label(
                    RichText::new("Configurator")
                        .color(Color32::from_rgb(200, 200, 200))
                        .size(18.0)
                );
                
                ui.add_space(20.0);
                
                // Sensor Status
                let inactive = Color32::from_rgb(80, 80, 80);
                let active = Color32::from_rgb(46, 204, 113);
                let connected = self.port.is_some();
                let sensor_color = if connected { active } else { inactive };
                
                ui.label(RichText::new("Gyro").color(sensor_color).size(12.0));
                ui.label(RichText::new("Accel").color(sensor_color).size(12.0));
                ui.label(RichText::new("Mag").color(inactive).size(12.0));
                ui.label(RichText::new("Baro").color(inactive).size(12.0));
                ui.label(RichText::new("GPS").color(inactive).size(12.0));
                
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    // Connect/Disconnect
                    if self.port.is_none() {
                        let btn = ui.add_sized([100.0, 28.0],
                            egui::Button::new(RichText::new("Connect").color(Color32::WHITE).size(13.0))
                                .fill(Color32::from_rgb(46, 204, 113))
                        );
                        if btn.clicked() && !self.selected_port.is_empty() {
                            match serialport::new(&self.selected_port, 115200)
                                .timeout(Duration::from_millis(100))
                                .open() 
                            {
                                Ok(mut port) => {
                                    self.port = Some(port.try_clone().unwrap());
                                    self.status_msg = "Connected (Probing...)".to_string();
                                    self.connection_mode = ConnectionMode::DirectUSB; // Default

                                    // Probe device type
                                    if let Err(e) = port.write_all(b"GET_CONFIG\n") {
                                        println!("Probe error: {}", e);
                                    }
                                    std::thread::sleep(Duration::from_millis(100));
                                    let mut buf = [0u8; 1024];
                                    if let Ok(n) = port.read(&mut buf) {
                                        let resp = String::from_utf8_lossy(&buf[..n]);
                                    }
                                    self.connection_mode = ConnectionMode::DirectUSB;
                                    self.status_msg = "Connected (Direct USB)".to_string();
                                    
                                    // Reset config fetch flag
                                    self.configuration_state.fetched = false;
                                }
                                Err(e) => {
                                    self.status_msg = format!("Error: {}", e);
                                    self.connection_mode = ConnectionMode::Disconnected;
                                }
                            }
                        }
                    } else {
                        let btn = ui.add_sized([100.0, 28.0],
                            egui::Button::new(RichText::new("Disconnect").color(Color32::WHITE).size(13.0))
                                .fill(Color32::from_rgb(231, 76, 60))
                        );
                        if btn.clicked() {
                            self.port = None;
                            self.status_msg = "Disconnected".to_string();
                            self.connection_mode = ConnectionMode::Disconnected;
                            log_event("Disconnected from serial port.", LogLevel::Warn);
                        }
                    }
                    
                    ui.add_space(8.0);
                    
                    // Port selector
                    egui::ComboBox::from_id_source("port_selector")
                        .width(200.0)
                        .selected_text(&self.selected_port)
                        .show_ui(ui, |ui| {
                            for p in &self.ports {
                                ui.selectable_value(&mut self.selected_port, p.clone(), p);
                            }
                        });
                        
                    if ui.add(egui::Button::new(RichText::new("↻").size(16.0).color(Color32::WHITE))
                        .fill(Color32::from_rgb(60, 60, 60))).clicked() {
                        self.ports.clear();
                        if let Ok(ports) = serialport::available_ports() {
                            for p in ports {
                                self.ports.push(p.port_name);
                            }
                        }
                    }
                    
                    // Status indicator
                    let status_color = if self.port.is_some() {
                        Color32::from_rgb(46, 204, 113)
                    } else {
                        Color32::from_rgb(231, 76, 60)
                    };
                    ui.add_space(8.0);
                    let (status_rect, _) = ui.allocate_exact_size(egui::Vec2::new(12.0, 12.0), egui::Sense::hover());
                    ui.painter().circle_filled(status_rect.center(), 6.0, status_color);
                });
            });
        });

        // --- BOTTOM STATUS BAR ---
        let bottom_frame = Frame::default()
            .fill(Color32::from_rgb(32, 32, 32))
            .inner_margin(Margin::same(4.0));
            
        egui::TopBottomPanel::bottom("bottom_panel").frame(bottom_frame).show(ctx, |ui| {
            ui.horizontal(|ui| {
                let status_color = if self.port.is_some() {
                    Color32::from_rgb(46, 204, 113)
                } else {
                    Color32::from_rgb(150, 150, 150)
                };
                ui.label(RichText::new(&self.status_msg).color(status_color).size(12.0));
                
                ui.add_space(10.0);
                if ui.add(egui::Button::new(
                    RichText::new(if self.log_terminal_open { "▼ Logs" } else { "▲ Logs" }).size(12.0)
                ).fill(Color32::TRANSPARENT)).clicked() {
                    self.log_terminal_open = !self.log_terminal_open;
                }
                
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(RichText::new("Firmware: CROX FC v0.1 (Target: RP2350)")
                        .color(Color32::from_rgb(150, 150, 150)).size(11.0));
                });
            });
        });

        // --- LOG TERMINAL PANEL ---
        if self.log_terminal_open {
            let log_frame = Frame::default()
                .fill(Color32::from_rgb(20, 20, 20))
                .inner_margin(Margin::same(8.0));
            egui::TopBottomPanel::bottom("log_terminal")
                .frame(log_frame)
                .resizable(true)
                .default_height(150.0)
                .min_height(50.0)
                .max_height(400.0)
                .show(ctx, |ui| {
                    egui::ScrollArea::vertical().stick_to_bottom(true).show(ui, |ui| {
                        ui.set_width(ui.available_width());
                        if let Ok(logs) = LOGS.lock() {
                            for log in logs.iter() {
                                let color = match log.level {
                                    LogLevel::Info => Color32::from_rgb(200, 200, 200),
                                    LogLevel::Tx => Color32::from_rgb(100, 150, 255),
                                    LogLevel::Rx => Color32::from_rgb(100, 255, 150),
                                    LogLevel::Warn => Color32::from_rgb(255, 200, 50),
                                    LogLevel::Error => Color32::from_rgb(255, 100, 100),
                                };
                                let text = format!("[{}] {}", log.timestamp, log.message);
                                ui.label(RichText::new(text).color(color).family(egui::FontFamily::Monospace).size(12.0));
                            }
                        }
                    });
                });
        }

        // --- LEFT SIDEBAR ---
        let side_frame = Frame::default()
            .fill(Color32::from_rgb(40, 40, 40))
            .inner_margin(Margin::same(0.0));
            
        egui::SidePanel::left("left_panel").frame(side_frame).exact_width(200.0).show(ctx, |ui| {
            ui.add_space(15.0);
            
            let tabs = [
                Tab::Setup, Tab::Ports, Tab::Configuration, Tab::PowerBattery,
                Tab::Tuning, Tab::EKF, Tab::Receiver, Tab::Modes, Tab::Motors,
                Tab::OSD, Tab::LedStrip, Tab::Blackbox, Tab::CLI
            ];
            
            for tab in tabs {
                let is_selected = self.selected_tab == tab;
                let text_color = if is_selected {
                    Color32::WHITE
                } else if tab.is_implemented() {
                    Color32::from_rgb(200, 200, 200)
                } else {
                    Color32::from_rgb(100, 100, 100)
                };
                
                let (rect, response) = ui.allocate_exact_size(egui::Vec2::new(180.0, 40.0), egui::Sense::click());
                
                // Add some padding from left side
                let content_rect = rect.shrink2(egui::Vec2::new(10.0, 0.0));
                
                if is_selected {
                    let bg_rect = rect.shrink2(egui::Vec2::new(8.0, 2.0));
                    ui.painter().rect_filled(bg_rect, 6.0, Color32::from_rgb(70, 130, 180));
                } else if response.hovered() {
                    let bg_rect = rect.shrink2(egui::Vec2::new(8.0, 2.0));
                    ui.painter().rect_filled(bg_rect, 6.0, Color32::from_rgb(55, 65, 80));
                }
                
                if response.clicked() && tab.is_implemented() {
                    if self.selected_tab != tab {
                        log_event(&format!("Switched to {} tab", tab.name()), LogLevel::Info);
                        self.selected_tab = tab;
                        match tab {
                            Tab::Tuning => self.tuning_state.sync_from_fc(&mut self.port),
                            Tab::EKF => self.ekf_state.sync_from_fc(&mut self.port),
                            Tab::Receiver => self.receiver_state.sync_from_fc(&mut self.port),
                            Tab::Blackbox => {
                                if let Some(port) = &mut self.port {
                                    self.blackbox_state.start_download(port);
                                }
                            }
                            _ => {}
                        }
                    }
                }
                
                // Icon + Name
                let label = format!("{}  {}", tab.icon(), tab.name());
                ui.painter().text(
                    content_rect.min + egui::Vec2::new(14.0, 12.0),
                    egui::Align2::LEFT_TOP,
                    label,
                    egui::FontId::proportional(15.0),
                    text_color,
                );
                ui.add_space(2.0);
            }
        });

        // --- CENTRAL PANEL ---
        egui::CentralPanel::default().show(ctx, |ui| {
            match self.selected_tab {
                Tab::Setup => self.setup_state.ui(ui, &mut self.port),
                Tab::Tuning => self.tuning_state.ui(ui, ctx, &mut self.port),
                Tab::EKF => self.ekf_state.ui(ui, ctx, &mut self.port),
                Tab::Blackbox => self.blackbox_state.ui(ui, &mut self.port),
                Tab::Receiver => self.receiver_state.ui(ui, &mut self.port),
                Tab::Motors => self.motors_state.ui(ui, &mut self.port),
                Tab::Modes => self.modes_state.ui(ui, &mut self.port),
                Tab::Configuration => self.configuration_state.ui(ui, &mut self.port),
                Tab::CLI => self.cli_state.ui(ui, &mut self.port),
                _ => {
                    ui.heading(format!("{} — Coming Soon", self.selected_tab.name()));
                    ui.label("This feature requires additional hardware not yet wired.");
                }
            }
        });

        // Request continuous repaint at ~50fps when connected to keep UI fresh
        if self.port.is_some() {
            ctx.request_repaint_after(Duration::from_millis(20));
        }
    }
}
