//! Build-time credentials (see `build.rs` and `secrets.example.toml`).

/// Wi-Fi network name. Empty → the uplink stays off.
pub const WIFI_SSID: &str = env!("OLIVAW_WIFI_SSID");
/// Wi-Fi password.
pub const WIFI_PASS: &str = env!("OLIVAW_WIFI_PASS");
/// MQTT broker host (IPv4 literal or DNS name).
pub const MQTT_HOST: &str = env!("OLIVAW_MQTT_HOST");
/// MQTT broker port as text (parsed at boot; default 1883).
pub const MQTT_PORT: &str = env!("OLIVAW_MQTT_PORT");
/// MQTT user name (empty → anonymous).
pub const MQTT_USER: &str = env!("OLIVAW_MQTT_USER");
/// MQTT password.
pub const MQTT_PASS: &str = env!("OLIVAW_MQTT_PASS");
