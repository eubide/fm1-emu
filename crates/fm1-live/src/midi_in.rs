//! `--midi`: the computer's MIDI inputs into the MIDI IN jack, followed as ports come and go.
use fm1_soc::Action;
use std::sync::mpsc;

/// Ends the name of `--usb`'s virtual MIDI device: the emulated FM-1's own output, never an input.
pub const EMU_SUFFIX: &str = " (emu)";

pub struct MidiIns {
    filter: String,
    ch: u8,
    tx: mpsc::Sender<Action>,
    conns: Vec<(String, midir::MidiInputConnection<()>)>,
    /// Ports that would not connect: tried again once they have gone and come back.
    failed: Vec<String>,
}

impl MidiIns {
    /// Inputs whose name contains `filter` ("all": every one), their channel messages moved to
    /// channel `ch` (1..16; 0 leaves them).
    pub fn new(filter: &str, ch: u8, tx: mpsc::Sender<Action>) -> Self {
        Self { filter: filter.to_string(), ch, tx, conns: Vec::new(), failed: Vec::new() }
    }

    pub fn is_empty(&self) -> bool {
        self.conns.is_empty()
    }

    /// Drop the inputs that have gone, connect the ones that came.
    pub fn refresh(&mut self) -> Result<(), midir::InitError> {
        let probe = midir::MidiInput::new("fm1-live")?;
        let ports: Vec<_> = probe.ports().into_iter().filter_map(|p| Some((probe.port_name(&p).ok()?, p))).collect();
        let present: Vec<String> = ports.iter().map(|(n, _)| n.clone()).collect();
        self.conns.retain(|(n, _)| {
            let here = present.contains(n);
            if !here {
                eprintln!("MIDI in: {n} gone");
            }
            here
        });
        self.failed.retain(|n| present.contains(n));
        let skip: Vec<String> = self.conns.iter().map(|(n, _)| n.clone()).chain(self.failed.iter().cloned()).collect();
        for name in to_open(&present, &skip, &self.filter) {
            let Some((_, port)) = ports.iter().find(|(n, _)| *n == name) else { continue };
            let mut input = midir::MidiInput::new("fm1-live")?;
            input.ignore(midir::Ignore::None);
            let (tx, ch) = (self.tx.clone(), self.ch);
            let got = input.connect(port, "fm1-live in", move |_, msg, _| {
                for (i, &b) in msg.iter().enumerate() {
                    let b = if i == 0 && ch != 0 && (0x80..0xf0).contains(&b) { (b & 0xf0) | (ch - 1) } else { b };
                    let _ = tx.send(Action::Midi { b });
                }
            }, ());
            match got {
                Ok(c) => {
                    eprintln!("MIDI in: {name}");
                    self.conns.push((name, c));
                }
                Err(e) => {
                    eprintln!("MIDI in: {name}: {e}");
                    self.failed.push(name);
                }
            }
        }
        Ok(())
    }
}

/// The ports of `ports` to connect: `filter`'s, not in `skip`.
fn to_open(ports: &[String], skip: &[String], filter: &str) -> Vec<String> {
    let filter = filter.to_lowercase();
    ports.iter()
        .filter(|n| !n.ends_with(EMU_SUFFIX) && !skip.contains(n))
        .filter(|n| filter == "all" || n.to_lowercase().contains(&filter))
        .cloned()
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn names(n: &[&str]) -> Vec<String> {
        n.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn all_opens_every_port_not_connected_yet() {
        assert_eq!(to_open(&names(&["A", "B"]), &names(&["A"]), "all"), names(&["B"]));
    }

    #[test]
    fn a_name_opens_the_ports_containing_it_whatever_the_case() {
        assert_eq!(to_open(&names(&["KeyStep 37", "IAC Bus 1"]), &[], "keystep"), names(&["KeyStep 37"]));
    }

    #[test]
    fn the_emulated_usb_device_is_never_an_input() {
        assert_eq!(to_open(&names(&["FM-1 (emu)", "A"]), &[], "all"), names(&["A"]));
    }

    /// A virtual source stands in for a keyboard plugged, played, unplugged and plugged back.
    #[cfg(target_os = "macos")]
    #[test]
    fn follows_a_keyboard_across_unplugging() {
        use midir::os::unix::VirtualOutput;
        let name = format!("fm1-live test {}", std::process::id());
        let plug = || midir::MidiOutput::new("fm1-live test").unwrap().create_virtual(&name).unwrap();
        let (tx, rx) = mpsc::channel();
        let mut ins = MidiIns::new(&name, 0, tx);
        let played = |keyboard: &mut midir::MidiOutputConnection| {
            while rx.try_recv().is_ok() {}
            keyboard.send(&[0x90, 60, 100]).unwrap();
            let got: Vec<_> = (0..3).filter_map(|_| rx.recv_timeout(std::time::Duration::from_secs(2)).ok()).collect();
            matches!(got[..], [Action::Midi { b: 0x90 }, Action::Midi { b: 60 }, Action::Midi { b: 100 }])
        };

        let mut keyboard = plug();
        ins.refresh();
        assert!(played(&mut keyboard), "plugged in after the start");

        drop(keyboard);
        ins.refresh();
        assert!(ins.conns.is_empty(), "unplugged");

        let mut keyboard = plug();
        ins.refresh();
        assert!(played(&mut keyboard), "plugged back");
    }
}
