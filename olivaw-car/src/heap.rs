//! Heap regions for esp-radio (BLE, and Wi-Fi + BLE coexistence).
//!
//! The only `unsafe` in this binary lives inside `esp_alloc::heap_allocator!`,
//! which registers a static buffer with the global allocator once at boot.

/// Register the heap. Sizes follow Espressif's ESP32 coexistence example
/// (96 KiB reclaimed + 24 KiB internal); BLE-only would fit in 64 + 36.
#[allow(unsafe_code)]
pub fn init() {
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 96 * 1024);
    esp_alloc::heap_allocator!(size: 24 * 1024);
}

/// Log free/used heap (for the 30 s housekeeping line).
pub fn log_stats() {
    log::info!(
        "heap: {} B used, {} B free",
        esp_alloc::HEAP.used(),
        esp_alloc::HEAP.free()
    );
}
