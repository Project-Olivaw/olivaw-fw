//! Byte-at-a-time parser: response descriptor, then an endless stream of
//! 5-byte scan nodes, with one-byte resynchronisation.

use olivaw_lidar::protocol::descriptor::{
    DATA_TYPE_MEASUREMENT, DESCRIPTOR_LEN, SendMode, parse_descriptor,
};
use olivaw_lidar::protocol::scan_node::{SCAN_NODE_LEN, ScanNode, parse_scan_node};

/// `SCAN_NODE_LEN` as the descriptor's `u32` length field.
const NODE_LEN_U32: u32 = 5;

/// Counters for telemetry and tests.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PumpStats {
    /// Nodes successfully parsed.
    pub nodes: u32,
    /// Bytes discarded while hunting for the descriptor.
    pub descriptor_skips: u32,
    /// Bytes discarded to regain node alignment.
    pub resync_bytes: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    /// Not scanning; bytes are ignored.
    Idle,
    /// Collecting the 7-byte response descriptor that precedes the stream.
    Descriptor,
    /// Collecting 5-byte nodes.
    Nodes,
}

/// The pump. Feed it every byte the UART delivers.
#[derive(Debug, Clone, Copy)]
pub struct Pump {
    state: State,
    buf: [u8; DESCRIPTOR_LEN],
    len: usize,
    stats: PumpStats,
}

impl Default for Pump {
    fn default() -> Self {
        Self::new()
    }
}

impl Pump {
    /// Idle pump; call [`start`](Self::start) after sending `SCAN`.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            state: State::Idle,
            buf: [0; DESCRIPTOR_LEN],
            len: 0,
            stats: PumpStats {
                nodes: 0,
                descriptor_skips: 0,
                resync_bytes: 0,
            },
        }
    }

    /// Expect a response descriptor next (after writing `SCAN`).
    pub fn start(&mut self) {
        self.state = State::Descriptor;
        self.len = 0;
    }

    /// Stop parsing (after writing `STOP`); bytes are ignored until `start`.
    pub fn stop(&mut self) {
        self.state = State::Idle;
        self.len = 0;
    }

    /// `true` once the descriptor has been seen and nodes are flowing.
    #[must_use]
    pub fn is_streaming(&self) -> bool {
        self.state == State::Nodes
    }

    /// Counters so far.
    #[must_use]
    pub const fn stats(&self) -> PumpStats {
        self.stats
    }

    /// Push one byte; returns a node when one completes.
    pub fn push_byte(&mut self, byte: u8) -> Option<ScanNode> {
        match self.state {
            State::Idle => None,
            State::Descriptor => {
                self.buf[self.len] = byte;
                self.len += 1;
                if self.len < DESCRIPTOR_LEN {
                    return None;
                }
                let ok = parse_descriptor(&self.buf)
                    .and_then(|d| d.expect(DATA_TYPE_MEASUREMENT, NODE_LEN_U32, SendMode::Multi))
                    .is_ok();
                if ok {
                    self.state = State::Nodes;
                    self.len = 0;
                } else {
                    self.stats.descriptor_skips += 1;
                    self.buf.copy_within(1.., 0);
                    self.len = DESCRIPTOR_LEN - 1;
                }
                None
            }
            State::Nodes => {
                self.buf[self.len] = byte;
                self.len += 1;
                if self.len < SCAN_NODE_LEN {
                    return None;
                }
                let mut node = [0u8; SCAN_NODE_LEN];
                node.copy_from_slice(&self.buf[..SCAN_NODE_LEN]);
                if let Ok(n) = parse_scan_node(&node) {
                    self.len = 0;
                    self.stats.nodes += 1;
                    return Some(n);
                }
                self.stats.resync_bytes += 1;
                self.buf.copy_within(1..SCAN_NODE_LEN, 0);
                self.len = SCAN_NODE_LEN - 1;
                None
            }
        }
    }

    /// Push a chunk; calls `sink` for every completed node.
    pub fn push(&mut self, bytes: &[u8], mut sink: impl FnMut(ScanNode)) {
        for &b in bytes {
            if let Some(n) = self.push_byte(b) {
                sink(n);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &[u8] =
        include_bytes!("../../../olivaw-lidar/tests/fixtures/c1_scan_1000_nodes.bin");

    fn run(bytes: &[u8]) -> (Pump, Vec<ScanNode>) {
        let mut pump = Pump::new();
        pump.start();
        let mut nodes = Vec::new();
        pump.push(bytes, |n| nodes.push(n));
        (pump, nodes)
    }

    #[test]
    fn parses_the_recorded_c1_stream() {
        let (pump, nodes) = run(FIXTURE);
        assert!(pump.is_streaming());
        assert!(nodes.len() >= 1000, "{} nodes", nodes.len());
        assert!(
            nodes.iter().any(|n| n.start_flag),
            "at least one rotation boundary"
        );
        assert_eq!(pump.stats().descriptor_skips, 0);
        assert_eq!(pump.stats().resync_bytes, 0);
    }

    #[test]
    fn resyncs_after_garbage_before_the_descriptor() {
        let mut bytes = vec![0x00, 0xFF, 0x13];
        bytes.extend_from_slice(FIXTURE);
        let (pump, nodes) = run(&bytes);
        assert_eq!(pump.stats().descriptor_skips, 3);
        assert!(nodes.len() >= 1000);
    }

    #[test]
    fn resyncs_after_a_dropped_byte_mid_stream() {
        let mut bytes = FIXTURE.to_vec();
        bytes.remove(7 + 5 * 300 + 2); // lose one byte inside node 300
        let (pump, nodes) = run(&bytes);
        assert!(pump.stats().resync_bytes > 0);
        assert!(nodes.len() >= 990, "{} nodes", nodes.len());
    }

    #[test]
    fn idle_pump_ignores_bytes_and_chunking_is_irrelevant() {
        let mut pump = Pump::new();
        assert!(pump.push_byte(0xA5).is_none());
        pump.start();
        let mut count = 0;
        for chunk in FIXTURE.chunks(13) {
            pump.push(chunk, |_| count += 1);
        }
        assert_eq!(count, run(FIXTURE).1.len());
        pump.stop();
        assert!(!pump.is_streaming());
    }
}
