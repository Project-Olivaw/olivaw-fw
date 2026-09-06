//! Open-circuit-voltage → state-of-charge for `LiPo` cells.
//!
//! A piecewise-linear table of the resting cell voltage. It is optimistic
//! under load (voltage sags), which is the safe direction for a "please
//! come home" indicator: the number only ever climbs back when the motors
//! stop.

/// (cell millivolts, percent) breakpoints, descending.
const TABLE: [(u16, u8); 12] = [
    (4200, 100),
    (4100, 90),
    (4000, 80),
    (3920, 70),
    (3850, 60),
    (3800, 50),
    (3750, 40),
    (3700, 30),
    (3650, 20),
    (3550, 10),
    (3400, 5),
    (3300, 0),
];

/// State of charge for one cell, percent `0..=100`.
#[must_use]
pub fn cell_percent(cell_mv: u16) -> u8 {
    let Some(&(top_mv, top_pct)) = TABLE.first() else {
        return 0;
    };
    if cell_mv >= top_mv {
        return top_pct;
    }
    for pair in TABLE.windows(2) {
        let (hi_mv, hi_pct) = pair[0];
        let (lo_mv, lo_pct) = pair[1];
        if cell_mv >= lo_mv {
            let span = u32::from(hi_mv - lo_mv);
            let above = u32::from(cell_mv - lo_mv);
            let pct = u32::from(lo_pct) + above * u32::from(hi_pct - lo_pct) / span;
            return u8::try_from(pct).unwrap_or(100);
        }
    }
    0
}

/// State of charge for a pack of `cells` in series, from pack millivolts.
#[must_use]
pub fn pack_percent(pack_mv: u32, cells: u8) -> u8 {
    if cells == 0 {
        return 0;
    }
    let cell = pack_mv / u32::from(cells);
    cell_percent(u16::try_from(cell).unwrap_or(u16::MAX))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn endpoints_and_interpolation() {
        assert_eq!(cell_percent(4250), 100);
        assert_eq!(cell_percent(4200), 100);
        assert_eq!(cell_percent(3300), 0);
        assert_eq!(cell_percent(3000), 0);
        assert_eq!(cell_percent(3800), 50);
        let mid = cell_percent(3850 + 25); // halfway 3850→3920 is 60→70
        assert!((63..=67).contains(&mid), "{mid}");
    }

    #[test]
    fn monotonic_over_the_whole_range() {
        let mut last = 0;
        for mv in (3000..=4300).step_by(10) {
            let p = cell_percent(mv);
            assert!(p >= last, "{mv} mV → {p} < {last}");
            last = p;
        }
    }

    #[test]
    fn pack_uses_average_cell() {
        assert_eq!(pack_percent(12_600, 3), 100);
        assert_eq!(pack_percent(11_400, 3), 50);
        assert_eq!(pack_percent(12_600, 0), 0);
    }
}
