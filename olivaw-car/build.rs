//! Injects Wi-Fi / MQTT credentials at build time from a gitignored file.
//!
//! `secrets.toml` (or the file named by `OLIVAW_SECRETS`) holds simple
//! `KEY = "value"` lines. Missing file or keys fall back to empty strings so
//! `cargo check` works on CI without secrets; the firmware refuses to start
//! the uplink when the SSID is empty.

use std::env;
use std::fs;

const KEYS: [(&str, &str); 6] = [
    ("WIFI_SSID", ""),
    ("WIFI_PASS", ""),
    ("MQTT_HOST", ""),
    ("MQTT_PORT", "1883"),
    ("MQTT_USER", ""),
    ("MQTT_PASS", ""),
];

fn main() {
    let path = env::var("OLIVAW_SECRETS").unwrap_or_else(|_| "secrets.toml".to_owned());
    println!("cargo:rerun-if-changed={path}");
    println!("cargo:rerun-if-env-changed=OLIVAW_SECRETS");
    let text = fs::read_to_string(&path).unwrap_or_default();
    for (key, default) in KEYS {
        let value = text
            .lines()
            .map(str::trim)
            .filter(|l| !l.starts_with('#'))
            .find_map(|line| {
                let (k, v) = line.split_once('=')?;
                (k.trim() == key).then(|| v.trim().trim_matches('"').to_owned())
            })
            .unwrap_or_else(|| default.to_owned());
        println!("cargo:rustc-env=OLIVAW_{key}={value}");
    }
}
