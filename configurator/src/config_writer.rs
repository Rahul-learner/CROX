use std::fs;
use std::path::PathBuf;

pub fn update_config_define(key: &str, value: f32) {
    let mut config_path = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
    
    // We are likely running in `/mnt/Vol_B/HobbyProject/DroneFC/RP2350/configurator`
    // Need to go up to the project root and then into `config`
    if config_path.ends_with("configurator") {
        config_path.pop();
    }
    config_path.push("config");
    config_path.push("config.h");

    if let Ok(content) = fs::read_to_string(&config_path) {
        let mut new_content = String::new();
        let target_prefix = format!("#define {}", key);
        
        for line in content.lines() {
            if line.starts_with(&target_prefix) {
                // Determine if it needs an 'f' suffix
                let new_line = if value.fract() == 0.0 {
                    format!("#define {} {:.1}f", key, value)
                } else {
                    format!("#define {} {:.5}f", key, value)
                };
                new_content.push_str(&new_line);
                new_content.push('\n');
            } else {
                new_content.push_str(line);
                new_content.push('\n');
            }
        }
        
        let _ = fs::write(&config_path, new_content);
    }
}

pub fn update_config_define_bool(key: &str, value: bool) {
    let mut config_path = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
    
    if config_path.ends_with("configurator") {
        config_path.pop();
    }
    config_path.push("config");
    config_path.push("config.h");

    if let Ok(content) = fs::read_to_string(&config_path) {
        let mut new_content = String::new();
        let target_prefix = format!("#define {}", key);
        
        for line in content.lines() {
            if line.starts_with(&target_prefix) {
                let new_line = format!("#define {} {}", key, if value { "true" } else { "false" });
                new_content.push_str(&new_line);
                new_content.push('\n');
            } else {
                new_content.push_str(line);
                new_content.push('\n');
            }
        }
        
        let _ = fs::write(&config_path, new_content);
    }
}
