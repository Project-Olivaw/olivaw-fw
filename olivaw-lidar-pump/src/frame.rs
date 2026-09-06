//! Groups nodes into whole rotations using the start flag.

use olivaw_lidar::protocol::scan_node::ScanNode;
use olivaw_proto::{ScanFrame, ScanPoint};

/// Accumulates nodes; emits a [`ScanFrame`] whenever a new rotation starts.
///
/// The leading partial rotation (everything before the first start flag) is
/// discarded, matching `olivaw-lidar`'s `Scans` iterator.
#[derive(Debug, Clone)]
pub struct FrameAssembler {
    current: ScanFrame,
    synced: bool,
    seq: u32,
}

impl Default for FrameAssembler {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameAssembler {
    /// Empty assembler.
    #[must_use]
    pub fn new() -> Self {
        Self {
            current: ScanFrame::default(),
            synced: false,
            seq: 0,
        }
    }

    /// Add a node observed at car uptime `now_ms`. Returns the completed
    /// previous rotation when `node` starts a new one.
    pub fn push(&mut self, node: ScanNode, now_ms: u32) -> Option<ScanFrame> {
        let mut done = None;
        if node.start_flag {
            if self.synced && !self.current.points.is_empty() {
                let mut finished = core::mem::take(&mut self.current);
                finished.seq = self.seq;
                finished.t_ms = now_ms;
                self.seq = self.seq.wrapping_add(1);
                done = Some(finished);
            }
            self.synced = true;
            self.current.points.clear();
            self.current.dropped = 0;
        }
        if self.synced {
            let point = ScanPoint {
                angle_q6: node.angle_q6,
                dist_q2: node.distance_q2,
                quality: node.quality,
            };
            if self.current.points.push(point).is_err() {
                self.current.dropped = self.current.dropped.saturating_add(1);
            }
        }
        done
    }

    /// Forget any partial rotation (call on stop/restart).
    pub fn reset(&mut self) {
        self.current = ScanFrame::default();
        self.synced = false;
    }

    /// Points collected in the rotation in progress.
    #[must_use]
    pub fn pending(&self) -> usize {
        self.current.points.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use olivaw_proto::MAX_NODES;

    fn node(start: bool, angle_q6: u16) -> ScanNode {
        ScanNode {
            start_flag: start,
            quality: 40,
            angle_q6,
            distance_q2: 4000,
        }
    }

    #[test]
    fn drops_leading_partial_and_emits_whole_rotations() {
        let mut a = FrameAssembler::new();
        assert!(a.push(node(false, 100), 0).is_none());
        assert!(a.push(node(false, 200), 0).is_none());
        assert!(
            a.push(node(true, 0), 10).is_none(),
            "first start flag: begin, nothing to emit"
        );
        for i in 1..5 {
            assert!(a.push(node(false, i * 64), 10 + u32::from(i)).is_none());
        }
        let f = a.push(node(true, 0), 120).expect("rotation complete");
        assert_eq!(f.seq, 0);
        assert_eq!(f.t_ms, 120);
        assert_eq!(f.points.len(), 5);
        assert_eq!(a.pending(), 1);
        let f2 = a.push(node(true, 0), 240).unwrap();
        assert_eq!(f2.seq, 1);
        assert_eq!(f2.points.len(), 1);
    }

    #[test]
    fn overflow_is_counted_not_lost_silently() {
        let mut a = FrameAssembler::new();
        a.push(node(true, 0), 0);
        for i in 0..(MAX_NODES + 10) {
            #[allow(clippy::cast_possible_truncation)]
            a.push(node(false, (i % 23_000) as u16), 1);
        }
        let f = a.push(node(true, 0), 2).unwrap();
        assert_eq!(f.points.len(), MAX_NODES);
        assert_eq!(f.dropped, 11);
    }

    #[test]
    fn reset_forgets_the_partial_rotation() {
        let mut a = FrameAssembler::new();
        a.push(node(true, 0), 0);
        a.push(node(false, 64), 0);
        a.reset();
        assert_eq!(a.pending(), 0);
        assert!(a.push(node(true, 0), 5).is_none());
    }
}
