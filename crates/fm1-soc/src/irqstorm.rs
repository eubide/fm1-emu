//! FM1_IRQ_STORM: warn once when a core takes the same interrupt again and again while it never
//! stops being pending, the signature of a handler that returns without clearing its source (an
//! unmodelled acknowledge, a lost enable bit). The UI then freezes or the emulation crawls with no
//! other clue. It only watches: with or without it, a run is the same.
//!
//! A core sees an interrupt cleared when, between two of its runs, that interrupt is not pending
//! (another one may outrank it meanwhile: the storm goes on below). Each run ends before the peripherals advance (where a source is raised again), so a
//! handler that clears its source is seen to, however soon the source comes back.

/// Entries in a row, never cleared in between, that make a storm.
pub const STORM_ENTRIES: u32 = 10_000;

#[derive(Debug, Default)]
pub struct IrqStorm {
    cores: [Streaks; 2],
    warned: bool,
}

/// A core's interrupts taken and still pending since (`live`), and how many times in a row.
#[derive(Debug)]
struct Streaks {
    live: u128,
    entries: [u32; 128],
}

impl Default for Streaks {
    fn default() -> Self {
        Self { live: 0, entries: [0; 128] }
    }
}

impl IrqStorm {
    /// `FM1_IRQ_STORM` set: a watcher; otherwise None.
    pub fn from_env() -> Option<Self> {
        std::env::var_os("FM1_IRQ_STORM").map(|_| Self::default())
    }

    /// Between runs on core `core`: `pending(irq)` says whether `irq` is still pending for it
    /// (`Periph::irq_pending_for`).
    pub fn seen(&mut self, core: u32, pending: impl Fn(u8) -> bool) {
        let s = &mut self.cores[core as usize & 1];
        let mut live = s.live;
        while live != 0 {
            let irq = live.trailing_zeros() as u8;
            live &= live - 1;
            if !pending(irq) {
                s.entries[irq as usize] = 0;
                s.live &= !(1 << irq);
            }
        }
    }

    /// Core `core` takes interrupt `irq` at `handler`, `ms` into the run: the warning, once.
    pub fn taken(&mut self, core: u32, irq: u8, handler: u32, ms: u64) -> Option<String> {
        let s = &mut self.cores[core as usize & 1];
        let i = irq as usize & 127;
        s.live |= 1 << i;
        s.entries[i] += 1;
        if s.entries[i] < STORM_ENTRIES || self.warned {
            return None;
        }
        self.warned = true;
        Some(format!(
            "FM1_IRQ_STORM: core {core} took IRQ {irq} (handler {handler:#010x}) {STORM_ENTRIES} times in a row at {ms} ms, \
             still pending after each: its handler returns without clearing it"
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_interrupt_never_cleared_warns_once() {
        let mut w = IrqStorm::default();
        let mut warnings = 0;
        for _ in 0..3 * STORM_ENTRIES {
            w.seen(0, |n| n == 16);
            warnings += w.taken(0, 16, 0x0200_1234, 5).is_some() as u32;
        }
        assert_eq!(warnings, 1);
    }

    #[test]
    fn the_warning_names_the_core_irq_and_handler() {
        let mut w = IrqStorm::default();
        let msg = (0..STORM_ENTRIES).find_map(|_| w.taken(1, 16, 0x0200_1234, 5)).unwrap();
        assert!(msg.contains("core 1") && msg.contains("IRQ 16") && msg.contains("0x02001234"), "{msg}");
    }

    #[test]
    fn an_interrupt_cleared_between_entries_is_no_storm() {
        let mut w = IrqStorm::default();
        for _ in 0..3 * STORM_ENTRIES {
            assert_eq!(w.taken(0, 16, 0x0200_1234, 5), None);
            w.seen(0, |_| false);
        }
    }

    #[test]
    fn an_interrupt_outranked_while_pending_still_storms() {
        let mut w = IrqStorm::default();
        let mut warned = false;
        for i in 0..STORM_ENTRIES + 100 {
            if i % 100 == 0 {
                w.seen(0, |n| n == 16 || n == 37);
                assert_eq!(w.taken(0, 37, 0x0200_5678, 5), None);
            }
            w.seen(0, |n| n == 16);
            warned |= w.taken(0, 16, 0x0200_1234, 5).is_some();
        }
        assert!(warned);
    }

    #[test]
    fn each_core_has_its_own_streak() {
        let mut w = IrqStorm::default();
        for _ in 0..STORM_ENTRIES - 1 {
            assert_eq!(w.taken(0, 16, 0x0200_1234, 5), None);
            w.seen(1, |_| false);
        }
        assert!(w.taken(0, 16, 0x0200_1234, 5).is_some());
    }
}
