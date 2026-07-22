use bevy::prelude::*;
use bevy::input::mouse::MouseWheel;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use egui_plot::{Legend, Line, Plot, PlotPoints, VLine};
use std::collections::{BTreeSet, HashMap, VecDeque};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Mutex;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use std::io::{Write, BufRead, BufReader};
use std::fs::{File, OpenOptions};

// ----------------------------------------------------------------------------
// Data Structures & Bevy Resources
// ----------------------------------------------------------------------------

#[derive(Clone, Default, Debug)]
struct ImuData {
    timestamp: f64,
    values: HashMap<String, f64>,
}

#[derive(Clone, Debug)]
struct LogEvent {
    timestamp: f64,
    message: String,
}

#[derive(Resource)]
struct SerialConnection {
    history: VecDeque<ImuData>,
    events: Vec<LogEvent>,
    max_history_seconds: f64,
    rx: Option<Mutex<Receiver<ImuData>>>,
    cmd_tx: Option<Sender<String>>,

    log_file: Option<File>,
    is_playback: bool,

    scanned_logs: Vec<String>,
    replay_path: String,
    replay_status: String,
    replay_chunks: Vec<(f64, f64, String)>,
    selected_chunk_index: usize,

    scanned_ports: Vec<String>,
    port_name: String,
    baud_rate: String,
    is_connected: bool,

    available_keys: BTreeSet<String>,
    selected_keys: BTreeSet<String>,
    auto_scroll: bool,
    reset_plot_view: bool,

    q_gyro: f64,
    q_bias: f64,
    r_accel: f64,

    // Acro PID Tuning (Inner Loop)
    pid_acro_rp_p: f64,
    pid_acro_rp_i: f64,
    pid_acro_rp_d: f64,
    
    // Yaw PID Tuning
    pid_y_p: f64,
    pid_y_i: f64,
    pid_y_d: f64,

    // Angle Tuning (Outer Loop)
    angle_strength: f64,
    angle_max_deg: f64,
    angle_max_rate: f64,

    // IMU Bias Tuning
    bias_roll: f64,
    bias_pitch: f64,
    bias_yaw: f64,
}

fn scan_logs() -> Vec<String> {
    let mut logs = Vec::new();
    if let Ok(entries) = std::fs::read_dir("logs") {
        for entry in entries.flatten() {
            if let Ok(file_type) = entry.file_type() {
                if file_type.is_file() {
                    let path = entry.path();
                    if path.extension().and_then(|s| s.to_str()) == Some("log") {
                        if let Ok(metadata) = entry.metadata() {
                            if let Ok(modified) = metadata.modified() {
                                logs.push((path.to_string_lossy().into_owned(), modified));
                            }
                        }
                    }
                }
            }
        }
    }
    // Sort descending by time
    logs.sort_by(|a, b| b.1.cmp(&a.1));
    logs.into_iter().map(|(p, _)| p).collect()
}

impl Default for SerialConnection {
    fn default() -> Self {
        let scanned_logs = scan_logs();
        let default_log = scanned_logs.first().cloned().unwrap_or_default();

        let scanned_ports = serialport::available_ports()
            .map(|ports| ports.into_iter().map(|p| p.port_name).collect::<Vec<_>>())
            .unwrap_or_default();
        let default_port = scanned_ports.first().cloned().unwrap_or_else(|| "/dev/ttyACM0".to_owned());

        let mut conn = Self {
            history: VecDeque::new(),
            events: Vec::new(),
            max_history_seconds: 600.0,
            rx: None,
            cmd_tx: None,
            log_file: None,
            is_playback: false,
            scanned_logs,
            replay_path: default_log,
            replay_status: String::new(),
            replay_chunks: Vec::new(),
            selected_chunk_index: 0,
            scanned_ports,
            port_name: default_port,
            baud_rate: "115200".to_owned(),
            is_connected: false,
            available_keys: BTreeSet::new(),
            selected_keys: BTreeSet::new(),
            auto_scroll: true,
            reset_plot_view: false,
            q_gyro: 0.001,
            q_bias: 0.00001,
            r_accel: 20.0,

            pid_acro_rp_p: 3.5,
            pid_acro_rp_i: 0.1,
            pid_acro_rp_d: 0.15,

            pid_y_p: 4.0,
            pid_y_i: 0.1,
            pid_y_d: 0.0,
            
            angle_strength: 4.5,
            angle_max_deg: 45.0,
            angle_max_rate: 360.0,

            bias_roll: -3.95,
            bias_pitch: 1.87,
            bias_yaw: 0.0,
        };

        if let Ok(content) = std::fs::read_to_string("tuning.cfg") {
            for line in content.lines() {
                let parts: Vec<&str> = line.split('=').collect();
                if parts.len() == 2 {
                    let key = parts[0].trim();
                    if let Ok(val) = parts[1].trim().parse::<f64>() {
                        match key {
                            "q_gyro" => conn.q_gyro = val,
                            "q_bias" => conn.q_bias = val,
                            "r_accel" => conn.r_accel = val,
                            "pid_acro_rp_p" => conn.pid_acro_rp_p = val,
                            "pid_acro_rp_i" => conn.pid_acro_rp_i = val,
                            "pid_acro_rp_d" => conn.pid_acro_rp_d = val,
                            "pid_y_p" => conn.pid_y_p = val,
                            "pid_y_i" => conn.pid_y_i = val,
                            "pid_y_d" => conn.pid_y_d = val,
                            "angle_strength" => conn.angle_strength = val,
                            "angle_max_deg" => conn.angle_max_deg = val,
                            "angle_max_rate" => conn.angle_max_rate = val,
                            "bias_roll" => conn.bias_roll = val,
                            "bias_pitch" => conn.bias_pitch = val,
                            "bias_yaw" => conn.bias_yaw = val,
                            _ => {}
                        }
                    }
                }
            }
        }
        conn
    }
}

impl SerialConnection {
    fn save_tuning(&self) {
        let content = format!(
            "q_gyro={}\nq_bias={}\nr_accel={}\npid_acro_rp_p={}\npid_acro_rp_i={}\npid_acro_rp_d={}\npid_y_p={}\npid_y_i={}\npid_y_d={}\nangle_strength={}\nangle_max_deg={}\nangle_max_rate={}\nbias_roll={}\nbias_pitch={}\nbias_yaw={}",
            self.q_gyro, self.q_bias, self.r_accel,
            self.pid_acro_rp_p, self.pid_acro_rp_i, self.pid_acro_rp_d,
            self.pid_y_p, self.pid_y_i, self.pid_y_d,
            self.angle_strength, self.angle_max_deg, self.angle_max_rate,
            self.bias_roll, self.bias_pitch, self.bias_yaw
        );
        let _ = std::fs::write("tuning.cfg", content);
    }
}

// ----------------------------------------------------------------------------
// Bevy Components
// ----------------------------------------------------------------------------

#[derive(Component)]
struct ImuBoard;

#[derive(Component)]
struct SceneCamera;

// ----------------------------------------------------------------------------
// Logic & Background Threads
// ----------------------------------------------------------------------------

fn parse_imu_data(line: &str, timestamp: f64) -> Option<ImuData> {
    let mut data = ImuData {
        timestamp,
        values: HashMap::new(),
    };

    for part in line.trim().split(',') {
        let kv: Vec<&str> = part.split(':').collect();
        if kv.len() == 2 {
            let key = kv[0].trim().to_string();
            if let Ok(val) = kv[1].trim().parse::<f64>() {
                data.values.insert(key, val);
            }
        }
    }

    if data.values.is_empty() { None } else { Some(data) }
}

fn start_serial_thread(port_name: String, baud_rate: u32, tx: Sender<ImuData>, cmd_rx: Receiver<String>) {
    thread::spawn(move || {
        println!("Attempting to open port {} at {} baud...", port_name, baud_rate);
        let port_result = serialport::new(&port_name, baud_rate).timeout(Duration::from_millis(10)).open();

        match port_result {
            Ok(mut port) => {
                println!("Successfully connected to {}!", port_name);

                // Windows FIX: Assert DTR
                let _ = port.write_data_terminal_ready(true);

                let mut serial_buf: Vec<u8> = vec![0; 1000];
                let mut string_buf = String::new();
                let start = Instant::now();

                loop {
                    if let Ok(cmd) = cmd_rx.try_recv() {
                        let _ = port.write_all(format!("{}\n", cmd).as_bytes());
                        println!("Sent: {}", cmd);
                    }

                    match port.read(serial_buf.as_mut_slice()) {
                        Ok(t) => {
                            if let Ok(s) = std::str::from_utf8(&serial_buf[..t]) {
                                string_buf.push_str(s);
                                while let Some(idx) = string_buf.find('\n') {
                                    let line = string_buf[..idx].to_string();
                                    string_buf = string_buf[idx + 1..].to_string();

                                    // Also print in terminal for debugging
                                    println!("Pico RX: {}", line.trim());

                                    let elapsed = start.elapsed().as_secs_f64();
                                    if let Some(data) = parse_imu_data(&line, elapsed) {
                                        if tx.send(data).is_err() { return; }
                                    }
                                }
                            }
                        }
                        Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => (),
                  Err(e) => {
                      eprintln!("Serial read error: {}", e);
                      break;
                  }
                    }
                }
            }
            Err(e) => eprintln!("Failed to open port {}: {}", port_name, e),
        }
    });
}

// ----------------------------------------------------------------------------
// Bevy Systems
// ----------------------------------------------------------------------------

fn main() {
    App::new()
    .add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "Aerospace IMU Monitor & Recorder".to_string(),
                             resolution: (1200.0, 800.0).into(),
                             mode: bevy::window::WindowMode::BorderlessFullscreen,
                             ..default()
        }),
        ..default()
    }))
    .add_plugins(EguiPlugin)
    .add_plugins(FrameTimeDiagnosticsPlugin)
    .insert_resource(ClearColor(Color::srgb(0.08, 0.08, 0.1)))
    .init_resource::<SerialConnection>()
    .add_systems(Startup, setup_3d_scene)
    .add_systems(Update, (
        read_serial_data,
        ui_system,
        update_3d_model_and_axes,
        draw_aerospace_grid,
        camera_zoom_system,
    ))
    .run();
}

fn setup_3d_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let board = commands.spawn((SpatialBundle::default(), ImuBoard)).id();

    commands.entity(board).with_children(|parent| {
        parent.spawn(PbrBundle {
            mesh: meshes.add(Cuboid::new(0.6, 0.5, 3.0)),
                     material: materials.add(StandardMaterial { base_color: Color::srgb(0.7, 0.7, 0.7), ..default() }),
                     ..default()
        });
        parent.spawn(PbrBundle {
            mesh: meshes.add(Cuboid::new(0.4, 0.3, 1.0)),
                     material: materials.add(StandardMaterial { base_color: Color::srgb(0.9, 0.2, 0.2), ..default() }),
                     transform: Transform::from_xyz(0.0, 0.0, -1.8),
                     ..default()
        });
        parent.spawn(PbrBundle {
            mesh: meshes.add(Cuboid::new(3.5, 0.1, 1.2)),
                     material: materials.add(StandardMaterial { base_color: Color::srgb(0.4, 0.4, 0.4), ..default() }),
                     transform: Transform::from_xyz(0.0, -0.1, 0.2),
                     ..default()
        });
        parent.spawn(PbrBundle {
            mesh: meshes.add(Cuboid::new(0.1, 1.2, 0.8)),
                     material: materials.add(StandardMaterial { base_color: Color::srgb(0.3, 0.3, 0.3), ..default() }),
                     transform: Transform::from_xyz(0.0, 0.6, 1.0),
                     ..default()
        });
    });

    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight { illuminance: 6000.0, shadows_enabled: true, ..default() },
                   transform: Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
                   ..default()
    });

    commands.spawn((
        Camera3dBundle { transform: Transform::from_xyz(4.0, 3.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y), ..default() },
                    SceneCamera,
    ));
}

fn read_serial_data(mut serial_res: ResMut<SerialConnection>) {
    let serial = &mut *serial_res;

    if let Some(rx_mutex) = &serial.rx {
        if let Ok(rx) = rx_mutex.lock() {
            while let Ok(data) = rx.try_recv() {
                // Record Data to File if Logging is active
                if let Some(file) = &mut serial.log_file {
                    let mut parts = Vec::new();
                    for (k, v) in &data.values {
                        parts.push(format!("{}:{}", k, v));
                    }
                    let _ = writeln!(file, "DATA,{:.4},{}", data.timestamp, parts.join(","));
                }

                for key in data.values.keys() {
                    if !serial.available_keys.contains(key) {
                        serial.available_keys.insert(key.clone());
                        if key == "Roll" || key == "Pitch" {
                            serial.selected_keys.insert(key.clone());
                        }
                    }
                }
                serial.history.push_back(data);
            }
        }

        // Only prune history if we are NOT in playback mode
        if !serial.is_playback {
            if let Some(latest) = serial.history.back() {
                let cutoff = latest.timestamp - serial.max_history_seconds;
                while let Some(front) = serial.history.front() {
                    if front.timestamp < cutoff {
                        serial.history.pop_front();
                    } else {
                        break;
                    }
                }

                // Prune old event so they dont stretch the graph
                serial.events.retain(|e| e.timestamp >= cutoff);
            }
        }
    }
}

// Helper Widget: Input Field + Increment/Decrement Buttons
fn tuning_field(ui: &mut egui::Ui, label: &str, val: &mut f64, step: f64) {
    ui.horizontal(|ui| {
        ui.label(label);
        if ui.button(" - ").clicked() { *val -= step; }

        // DragValue acts as a text field when clicked, but visually conforms to horizontal layouts nicely.
        ui.add(egui::DragValue::new(val)
        .speed(step)
        .min_decimals(2)
        .max_decimals(6)
        );

        if ui.button(" + ").clicked() { *val += step; }
    });
}

fn ui_system(
    mut contexts: EguiContexts,
    mut serial: ResMut<SerialConnection>,
    diagnostics: Res<DiagnosticsStore>,
) {
    let ctx = contexts.ctx_mut();
    let fps = diagnostics.get(&FrameTimeDiagnosticsPlugin::FPS).and_then(|fps| fps.smoothed()).unwrap_or(0.0);

    egui::SidePanel::left("controls_panel")
    .resizable(true)
    .default_width(330.0)
    .width_range(300.0..=500.0)
    .show(ctx, |ui| {
        ui.heading("Aerospace IMU Monitor");
        ui.label(egui::RichText::new(format!("Engine FPS: {:.1}", fps)).color(egui::Color32::YELLOW).weak());
        ui.separator();

        egui::Frame::group(ui.style()).show(ui, |ui| {
            ui.heading("Serial Connection");
            ui.add_space(5.0);
            
            ui.horizontal(|ui| {
                ui.label("Port:");
                egui::ComboBox::from_id_source("port_combo")
                    .selected_text(&serial.port_name)
                    .width(120.0)
                    .show_ui(ui, |ui| {
                        let ports = serial.scanned_ports.clone();
                        for port in ports {
                            ui.selectable_value(&mut serial.port_name, port.clone(), &port);
                        }
                    });
                if ui.button("🔄").on_hover_text("Refresh Ports").clicked() {
                    if let Ok(ports) = serialport::available_ports() {
                        serial.scanned_ports = ports.into_iter().map(|p| p.port_name).collect();
                        if !serial.scanned_ports.contains(&serial.port_name) {
                            if let Some(first) = serial.scanned_ports.first() {
                                serial.port_name = first.clone();
                            }
                        }
                    }
                }
                
                ui.label("Baud:");
                ui.add(egui::TextEdit::singleline(&mut serial.baud_rate).desired_width(80.0));
            });
            ui.add_space(10.0);

            if !serial.is_connected {
                if ui.add_sized([ui.available_width(), 30.0], egui::Button::new(egui::RichText::new("▶ Connect & Record").color(egui::Color32::GREEN).strong())).clicked() {
                    let (tx, rx) = mpsc::channel();
                    let (cmd_tx, cmd_rx) = mpsc::channel();

                    // Setup automatic file recording partition
                    let _ = std::fs::create_dir_all("logs");
                    let ts = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs();
                    let filename = format!("logs/session_{}.log", ts);

                    if let Ok(mut file) = OpenOptions::new().create(true).write(true).append(true).open(&filename) {
                        let _ = writeln!(file, "EVENT,0.0,Recording Started");
                        serial.log_file = Some(file);
                    }

                    serial.rx = Some(Mutex::new(rx));
                    serial.cmd_tx = Some(cmd_tx);
                    serial.history.clear();
                    serial.events.clear();
                    serial.available_keys.clear();
                    serial.is_connected = true;
                    serial.is_playback = false;
                    serial.auto_scroll = true;

                    let baud = serial.baud_rate.parse::<u32>().unwrap_or(115200);
                    start_serial_thread(serial.port_name.clone(), baud, tx, cmd_rx);
                }
            } else {
                if ui.add_sized([ui.available_width(), 30.0], egui::Button::new(egui::RichText::new("⏹ Disconnect").color(egui::Color32::RED).strong())).clicked() {
                    if let Some(file) = &mut serial.log_file {
                        let _ = writeln!(file, "EVENT,0.0,Recording Stopped");
                    }
                    serial.rx = None;
                    serial.cmd_tx = None;
                    serial.log_file = None;
                    serial.is_connected = false;
                }
            }
        });

        ui.add_space(10.0);

        // Replay Feature
        if !serial.is_connected {
            egui::Frame::group(ui.style()).show(ui, |ui| {
                ui.heading("Replay Data");
                ui.add_space(5.0);
                ui.horizontal(|ui| {
                    ui.label("File:");
                    egui::ComboBox::from_id_source("log_combo")
                        .selected_text(&serial.replay_path)
                        .width(150.0)
                        .show_ui(ui, |ui| {
                            let logs = serial.scanned_logs.clone();
                            for log in logs {
                                ui.selectable_value(&mut serial.replay_path, log.clone(), &log);
                            }
                        });
                    
                    if ui.button("🔄").on_hover_text("Refresh Logs").clicked() {
                        serial.scanned_logs = scan_logs();
                        if !serial.scanned_logs.contains(&serial.replay_path) {
                            if let Some(first) = serial.scanned_logs.first() {
                                serial.replay_path = first.clone();
                            }
                        }
                    }

                    if ui.button("📂 Load").clicked() {
                        let path = serial.replay_path.trim().to_string();

                        match File::open(&path) {
                            Ok(file) => {
                                serial.history.clear();
                                serial.events.clear();
                                serial.available_keys.clear();

                                let mut loaded_count = 0;
                                let mut prev_ts = -1.0;
                                let mut flight_count = 1;
                                let reader = BufReader::new(file);
                                for line in reader.lines().flatten() {
                                    if line.starts_with("DATA,") {
                                        let parts: Vec<&str> = line.splitn(3, ',').collect();
                                        if parts.len() == 3 {
                                            if let Ok(ts) = parts[1].parse::<f64>() {
                                                if prev_ts >= 0.0 && (ts - prev_ts) > 1.0 {
                                                    flight_count += 1;
                                                    serial.events.push(LogEvent { timestamp: ts, message: format!("Flight {} Started", flight_count) });
                                                }
                                                prev_ts = ts;
                                                
                                                if let Some(data) = parse_imu_data(parts[2], ts) {
                                                    for k in data.values.keys() { serial.available_keys.insert(k.clone()); }
                                                    serial.history.push_back(data);
                                                    loaded_count += 1;
                                                }
                                            }
                                        }
                                    } else if line.starts_with("EVENT,") {
                                        let parts: Vec<&str> = line.splitn(3, ',').collect();
                                        if parts.len() == 3 {
                                            if let Ok(ts) = parts[1].parse::<f64>() {
                                                serial.events.push(LogEvent { timestamp: ts, message: parts[2].to_string() });
                                            }
                                        }
                                    }
                                }
                                serial.is_playback = true;
                                serial.auto_scroll = false;
                                serial.replay_status = format!("✅ Loaded {} data points", loaded_count);

                                serial.events.sort_by(|a, b| a.timestamp.partial_cmp(&b.timestamp).unwrap());

                                let mut chunks = Vec::new();
                                let mut last_ts = serial.history.front().map(|d| d.timestamp).unwrap_or(0.0);
                                let mut last_name = "Initial".to_string();
                                
                                for ev in &serial.events {
                                    if ev.message.starts_with("Uploaded") || ev.message.starts_with("Flight") {
                                        chunks.push((last_ts, ev.timestamp, last_name.clone()));
                                        last_ts = ev.timestamp;
                                        last_name = ev.message.clone();
                                    }
                                }
                                if let Some(last_data) = serial.history.back() {
                                    chunks.push((last_ts, last_data.timestamp, last_name));
                                } else {
                                    chunks.push((last_ts, last_ts, last_name));
                                }
                                serial.replay_chunks = chunks;
                                serial.selected_chunk_index = 0;
                            }
                            Err(e) => {
                                serial.replay_status = format!("❌ Error: {}", e);
                            }
                        }
                    }
                });

                if !serial.replay_status.is_empty() {
                    ui.label(egui::RichText::new(&serial.replay_status).color(egui::Color32::YELLOW));
                }

                if !serial.replay_chunks.is_empty() {
                    ui.add_space(5.0);
                    ui.horizontal(|ui| {
                        ui.label("Chunk:");
                        egui::ComboBox::from_id_source("chunk_combo")
                            .selected_text(
                                if serial.selected_chunk_index == 0 {
                                    "All Data".to_string()
                                } else if let Some(chunk) = serial.replay_chunks.get(serial.selected_chunk_index - 1) {
                                    format!("Chunk {} ({:.0}s)", serial.selected_chunk_index, chunk.1 - chunk.0)
                                } else {
                                    "Unknown".to_string()
                                }
                            )
                            .width(220.0)
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut serial.selected_chunk_index, 0, "All Data");
                                let chunks = serial.replay_chunks.clone();
                                for (i, chunk) in chunks.iter().enumerate() {
                                    ui.selectable_value(
                                        &mut serial.selected_chunk_index,
                                        i + 1,
                                        format!("Chunk {} ({:.0}s) - {}", i + 1, chunk.1 - chunk.0, chunk.2)
                                    );
                                }
                            });
                    });
                }
            });
        }

        if serial.is_playback {
            ui.label(egui::RichText::new("📂 PLAYBACK MODE ACTIVE").color(egui::Color32::GREEN).strong());
        }

        ui.add_space(20.0);

        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.heading("Tuning Parameters");
                if ui.button("💾 Save as Default").clicked() {
                    serial.save_tuning();
                }
            });
            ui.separator();

            egui::Frame::group(ui.style()).show(ui, |ui| {
                ui.collapsing("EKF Parameters", |ui| {
                    tuning_field(ui, "Q Gyro Noise: ", &mut serial.q_gyro, 0.0005);
                    tuning_field(ui, "Q Bias Drift: ", &mut serial.q_bias, 0.0001);
                    tuning_field(ui, "R Accel Noise:", &mut serial.r_accel, 0.01);

                    if ui.button("Upload EKF Params").clicked() {
                        let cmd = format!("EKF,{:.5},{:.5},{:.5}", serial.q_gyro, serial.q_bias, serial.r_accel);
                        if let Some(ref tx) = serial.cmd_tx { let _ = tx.send(cmd.clone()); }

                        // Partition / Annotate Log file
                        if let Some(latest) = serial.history.back() {
                            let ts = latest.timestamp;
                            let msg = format!("Uploaded EKF -> QG:{:.5} QB:{:.5} RA:{:.3}", serial.q_gyro, serial.q_bias, serial.r_accel);
                            serial.events.push(LogEvent { timestamp: ts, message: msg.clone() });
                            if let Some(file) = &mut serial.log_file {
                                let _ = writeln!(file, "EVENT,{:.4},{}", ts, msg);
                            }
                        }
                    }
                });
            });

            ui.add_space(10.0);

            egui::Frame::group(ui.style()).show(ui, |ui| {
                ui.collapsing("PID Parameters (Acro Loop)", |ui| {
                    ui.label(egui::RichText::new("Roll & Pitch (Inner Loop)").strong());
                    tuning_field(ui, "Proportional (P):", &mut serial.pid_acro_rp_p, 0.1);
                    tuning_field(ui, "Integral (I):    ", &mut serial.pid_acro_rp_i, 0.05);
                    tuning_field(ui, "Derivative (D):  ", &mut serial.pid_acro_rp_d, 0.01);

                    if ui.button("Upload Acro Roll/Pitch PID").clicked() {
                        let cmd = format!("PID_ACRO,{:.3},{:.3},{:.3}", serial.pid_acro_rp_p, serial.pid_acro_rp_i, serial.pid_acro_rp_d);
                        if let Some(ref tx) = serial.cmd_tx { let _ = tx.send(cmd); }

                        // Partition / Annotate Log file
                        if let Some(latest) = serial.history.back() {
                            let ts = latest.timestamp;
                            let msg = format!("Uploaded PID_ACRO -> P:{:.2} I:{:.2} D:{:.2}", serial.pid_acro_rp_p, serial.pid_acro_rp_i, serial.pid_acro_rp_d);
                            serial.events.push(LogEvent { timestamp: ts, message: msg.clone() });
                            if let Some(file) = &mut serial.log_file {
                                let _ = writeln!(file, "EVENT,{:.4},{}", ts, msg);
                            }
                        }
                    }

                    ui.add_space(10.0);

                    ui.label(egui::RichText::new("Yaw").strong());
                    tuning_field(ui, "Proportional (P):", &mut serial.pid_y_p, 0.1);
                    tuning_field(ui, "Integral (I):    ", &mut serial.pid_y_i, 0.05);
                    tuning_field(ui, "Derivative (D):  ", &mut serial.pid_y_d, 0.01);

                    if ui.button("Upload Yaw PID").clicked() {
                        let cmd = format!("PID_YAW,{:.3},{:.3},{:.3}", serial.pid_y_p, serial.pid_y_i, serial.pid_y_d);
                        if let Some(ref tx) = serial.cmd_tx { let _ = tx.send(cmd); }

                        if let Some(latest) = serial.history.back() {
                            let ts = latest.timestamp;
                            let msg = format!("Uploaded PID_YAW -> P:{:.2} I:{:.2} D:{:.2}", serial.pid_y_p, serial.pid_y_i, serial.pid_y_d);
                            serial.events.push(LogEvent { timestamp: ts, message: msg.clone() });
                            if let Some(file) = &mut serial.log_file {
                                let _ = writeln!(file, "EVENT,{:.4},{}", ts, msg);
                            }
                        }
                    }
                });
            });

            ui.add_space(10.0);

            egui::Frame::group(ui.style()).show(ui, |ui| {
                ui.collapsing("Angle Mode Parameters (Outer Loop)", |ui| {
                    ui.label(egui::RichText::new("Angle Tuning").strong());
                    tuning_field(ui, "Angle Strength (P):", &mut serial.angle_strength, 0.1);
                    tuning_field(ui, "Max Angle (deg):   ", &mut serial.angle_max_deg, 1.0);
                    tuning_field(ui, "Max Rate (deg/s):  ", &mut serial.angle_max_rate, 5.0);

                    if ui.button("Upload Angle Tuning").clicked() {
                        let cmd = format!("ANGLE_TUNE,{:.3},{:.1},{:.1}", serial.angle_strength, serial.angle_max_deg, serial.angle_max_rate);
                        if let Some(ref tx) = serial.cmd_tx { let _ = tx.send(cmd); }

                        if let Some(latest) = serial.history.back() {
                            let ts = latest.timestamp;
                            let msg = format!("Uploaded ANGLE_TUNE -> Str:{:.2} MaxDeg:{:.1} MaxRate:{:.1}", serial.angle_strength, serial.angle_max_deg, serial.angle_max_rate);
                            serial.events.push(LogEvent { timestamp: ts, message: msg.clone() });
                            if let Some(file) = &mut serial.log_file {
                                let _ = writeln!(file, "EVENT,{:.4},{}", ts, msg);
                            }
                        }
                    }
                });
            });

            ui.add_space(10.0);

            egui::Frame::group(ui.style()).show(ui, |ui| {
                ui.collapsing("IMU Bias Offsets", |ui| {
                    tuning_field(ui, "Roll Bias: ", &mut serial.bias_roll, 0.1);
                    tuning_field(ui, "Pitch Bias:", &mut serial.bias_pitch, 0.1);
                    tuning_field(ui, "Yaw Bias:  ", &mut serial.bias_yaw, 0.1);

                    if ui.button("Upload Bias Params").clicked() {
                        let cmd = format!("BIAS,{:.3},{:.3},{:.3}", serial.bias_roll, serial.bias_pitch, serial.bias_yaw);
                        if let Some(ref tx) = serial.cmd_tx { let _ = tx.send(cmd.clone()); }

                        if let Some(latest) = serial.history.back() {
                            let ts = latest.timestamp;
                            let msg = format!("Uploaded BIAS -> R:{:.2} P:{:.2} Y:{:.2}", serial.bias_roll, serial.bias_pitch, serial.bias_yaw);
                            serial.events.push(LogEvent { timestamp: ts, message: msg.clone() });
                            if let Some(file) = &mut serial.log_file {
                                let _ = writeln!(file, "EVENT,{:.4},{}", ts, msg);
                            }
                        }
                    }
                });
            });

            ui.add_space(20.0);
            ui.heading("Plot Variables");
            ui.separator();

            if serial.available_keys.is_empty() {
                ui.label("Waiting for data stream...");
            } else {
                let keys: Vec<String> = serial.available_keys.iter().cloned().collect();
                let latest_vals = serial.history.back().map(|d| d.values.clone()).unwrap_or_default();

                for key in keys {
                    let mut is_selected = serial.selected_keys.contains(&key);
                    let display_text = if let Some(v) = latest_vals.get(&key) {
                        format!("{}: {:.4}", key, v)
                    } else {
                        key.clone()
                    };

                    if ui.checkbox(&mut is_selected, &display_text).changed() {
                        if is_selected {
                            serial.selected_keys.insert(key);
                        } else {
                            serial.selected_keys.remove(&key);
                        }
                    }
                }
            }
        });
    });

    egui::TopBottomPanel::bottom("graph_panel")
    .resizable(true)
    .default_height(350.0)
    .height_range(200.0..=800.0)
    .show(ctx, |ui| {
        ui.horizontal(|ui| {
            ui.heading("Live EKF Timeline");
            ui.add_space(20.0);
            if !serial.is_playback {
                ui.checkbox(&mut serial.auto_scroll, "Auto-Scroll to Newest");
                ui.label(egui::RichText::new(" (Disable to manual Pan/Zoom with Scroll)").weak());
            } else {
                if !serial.replay_chunks.is_empty() {
                    if ui.button("⏪ Prev Chunk").clicked() {
                        if serial.selected_chunk_index > 0 {
                            serial.selected_chunk_index -= 1;
                        }
                    }
                    if ui.button("Next Chunk ⏩").clicked() {
                        if serial.selected_chunk_index < serial.replay_chunks.len() {
                            serial.selected_chunk_index += 1;
                        }
                    }
                    ui.label(format!("Viewing Chunk {} / {}", serial.selected_chunk_index, serial.replay_chunks.len()));
                }
            }
            ui.add_space(10.0);
            if ui.button("🔄 Reset View").clicked() {
                serial.reset_plot_view = true;
            }
        });

        if serial.history.is_empty() {
            ui.centered_and_justified(|ui| {
                ui.label("Waiting for data... Click Connect or Load Replay.");
            });
        } else {
            let mut plot = Plot::new("imu_plot")
            .legend(Legend::default())
            .allow_drag(true)
            .allow_zoom(true)
            .allow_scroll(true);

            if serial.reset_plot_view {
                plot = plot.reset();
                serial.reset_plot_view = false;
            } else {
                plot = plot.auto_bounds(egui::Vec2b::new(false, true));
            }

            plot.show(ui, |plot_ui| {
                let chunk_range = if serial.selected_chunk_index > 0 && serial.selected_chunk_index <= serial.replay_chunks.len() {
                    let chunk = &serial.replay_chunks[serial.selected_chunk_index - 1];
                    Some((chunk.0, chunk.1))
                } else {
                    None
                };

                for key in &serial.selected_keys {
                    let points: PlotPoints = serial.history
                    .iter()
                    .filter(|d| {
                        if let Some((start, end)) = chunk_range {
                            d.timestamp >= start && d.timestamp <= end
                        } else {
                            true
                        }
                    })
                    .filter_map(|d| d.values.get(key).map(|&v| [d.timestamp, v]))
                    .collect();
                    plot_ui.line(Line::new(points).name(key));
                }

                // Draw Partition Events as Vertical Lines
                for event in &serial.events {
                    if let Some((start, end)) = chunk_range {
                        if event.timestamp < start || event.timestamp > end {
                            continue;
                        }
                    }
                    plot_ui.vline(VLine::new(event.timestamp).color(egui::Color32::YELLOW));
                    plot_ui.text(egui_plot::Text::new(
                        egui_plot::PlotPoint::new(event.timestamp, 0.0), // Placement near center
                                                      event.message.clone(),
                    ).color(egui::Color32::WHITE).anchor(egui::Align2::LEFT_CENTER));
                }

                if serial.auto_scroll && !serial.is_playback {
                    if let Some(latest) = serial.history.back() {
                        let max_t = latest.timestamp;
                        let min_t = f64::max(0.0, max_t - 15.0); // Keep a 15-second trailing window
                        let bounds = plot_ui.plot_bounds();
                        plot_ui.set_plot_bounds(egui_plot::PlotBounds::from_min_max(
                            [min_t, bounds.min()[1]],
                            [max_t, bounds.max()[1]]
                        ));
                    }
                }
            });
        }
    });

    if let Some(latest) = serial.history.back() {
        let roll = latest.values.get("Roll").copied().unwrap_or(0.0);
        let pitch = latest.values.get("Pitch").copied().unwrap_or(0.0);

        egui::Window::new("Orientation")
        .anchor(egui::Align2::RIGHT_TOP, [-20.0, 20.0])
        .collapsible(false)
        .title_bar(false)
        .frame(egui::Frame::window(&ctx.style()).fill(egui::Color32::from_black_alpha(180)))
        .show(ctx, |ui| {
            ui.heading(format!("Roll: {:.2}°", roll));
            ui.heading(format!("Pitch: {:.2}°", pitch));
        });
    }
}

fn draw_aerospace_grid(mut gizmos: Gizmos) {
    let grid_size = 5;
    let ground_y = -2.0;
    for i in -grid_size..=grid_size {
        let f = i as f32 * 1.0;
        let color = if i == 0 { Color::srgb(0.2, 0.2, 0.8) } else { Color::srgb(0.3, 0.3, 0.3) };
        gizmos.line(Vec3::new(f, ground_y, -5.0), Vec3::new(f, ground_y, 5.0), color);
        let color = if i == 0 { Color::srgb(0.8, 0.2, 0.2) } else { Color::srgb(0.3, 0.3, 0.3) };
        gizmos.line(Vec3::new(-5.0, ground_y, f), Vec3::new(5.0, ground_y, f), color);
    }
}

fn camera_zoom_system(
    mut scroll_evr: EventReader<MouseWheel>,
    mut query: Query<&mut Transform, With<SceneCamera>>,
    mut egui_contexts: EguiContexts,
) {
    if egui_contexts.ctx_mut().is_pointer_over_area() || egui_contexts.ctx_mut().wants_pointer_input() {
        return;
    }
    let mut scroll_amount = 0.0;
    for ev in scroll_evr.read() { scroll_amount += ev.y; }
    if scroll_amount != 0.0 {
        if let Ok(mut transform) = query.get_single_mut() {
            let forward = transform.rotation * Vec3::NEG_Z;
            transform.translation += forward * scroll_amount * 0.4;
        }
    }
}

fn update_3d_model_and_axes(
    serial: Res<SerialConnection>,
    mut query: Query<&mut Transform, With<ImuBoard>>,
    mut gizmos: Gizmos,
) {
    if let Some(latest) = serial.history.back() {
        let pitch = latest.values.get("Pitch").copied().unwrap_or(0.0);
        let roll = latest.values.get("Roll").copied().unwrap_or(0.0);

        for mut transform in query.iter_mut() {
            let pitch_rad = (pitch as f32).to_radians();
            let roll_rad = (roll as f32).to_radians();
            transform.rotation = Quat::from_rotation_x(-pitch_rad) * Quat::from_rotation_z(-roll_rad);

            let center = transform.translation;
            let right = transform.right() * 2.5;
            let up = transform.up() * 2.5;
            let forward = transform.rotation * Vec3::NEG_Z * 2.5;

            gizmos.line(center, center + right, Color::srgb(1.0, 0.0, 0.0));
            gizmos.line(center, center + up, Color::srgb(0.0, 1.0, 0.0));
            gizmos.line(center, center + forward, Color::srgb(0.0, 0.0, 1.0));
        }
    }
}
