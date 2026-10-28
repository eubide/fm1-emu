//! The FFT / vector unit's model (src/hwdsp.rs) against what an FM-1 returned for the same jobs
//! (tests/hwdsp/fm1.txt: tools/hwdsp.py run --on fm1, then export): every vector case inside the SDK's rule
//! (z no narrower than x) bit for bit, every FFT case whose outputs stay in range within 3 LSB.
//! The inputs are made again from the cases' seeds (fm1-dock's probes/hwdsp.c fill(), hwdsp.py gen()).
use fm1_soc::hwdsp::{HwDsp, Mem};

const WORDS: usize = 2048 + 8;
const X: u32 = 0x1_0000;
const Y: u32 = 0x2_1000;
const Z: u32 = 0x3_2000;
const D: u32 = 0x100;

struct Ram(Vec<u32>);
impl Mem for Ram {
    fn rd(&mut self, a: u32) -> u32 {
        self.0[(a / 4) as usize]
    }
    fn wr(&mut self, a: u32, v: u32) {
        self.0[(a / 4) as usize] = v;
    }
}

fn gen(g: u32, amp: u32, p: u32) -> Vec<u32> {
    let mut x = if p != 0 { p } else { 1 };
    let amp = amp as i32;
    (0..WORDS)
        .map(|i| {
            let v: i32 = match g {
                1 => if i as u32 == p { amp } else { 0 },
                2 => amp,
                3 | 5 => {
                    x ^= x << 13;
                    x ^= x >> 17;
                    x ^= x << 5;
                    if g == 5 {
                        x as i32
                    } else if amp != 0 {
                        ((x >> 1) % (2 * amp as u32 + 1)) as i32 - amp
                    } else {
                        0
                    }
                }
                4 => (i as i32).wrapping_mul(amp).wrapping_add(p as i32),
                _ => 0,
            };
            v as u32
        })
        .collect()
}

/// (id, args, out) of each line of tests/hwdsp/fm1.txt (`id | args | ticks | con | out`)
fn cases(text: &str) -> Vec<(String, Vec<u32>, Vec<u32>)> {
    text.lines()
        .filter(|l| !l.starts_with('#') && !l.is_empty())
        .map(|l| {
            let f: Vec<&str> = l.split(" | ").collect();
            let args = f[1].split(',').map(|t| t.trim().parse::<i64>().unwrap() as u32).collect();
            let out = f.get(4).map_or(vec![], |o| o.split_whitespace().map(|w| u32::from_str_radix(w, 16).unwrap()).collect());
            (f[0].to_string(), args, out)
        })
        .collect()
}

fn run(args: &[u32]) -> Vec<u32> {
    let mut ram = Ram(vec![0; 0x5_0000 / 4]);
    let mut u = HwDsp::default();
    let put = |ram: &mut Ram, at: u32, w: &[u32]| {
        for (k, v) in w.iter().enumerate() {
            ram.0[(at / 4) as usize + k] = *v;
        }
    };
    let (vector, off, n) = if args[0] == 1 {
        put(&mut ram, X, &gen(args[2], args[3], args[4]));
        let same = args[1] & 1 != 0;
        put(&mut ram, D, &[args[1], X, if same { X } else { Z }, 0]);
        (false, args[5], args[6])
    } else {
        put(&mut ram, X, &gen(args[6], args[7], args[8]));
        put(&mut ram, Y, &gen(args[9], args[10], args[11]));
        put(&mut ram, Z, &gen(args[12], args[13], args[14]));
        put(&mut ram, D, &[args[1], X, Y, Z, args[2], args[3], args[4], 0]);
        u.write(&mut ram, 0x1ee0408, args[5], 0);
        (true, args[15], args[16])
    };
    u.write(&mut ram, 0x1ee0404, D, 0);
    u.write(&mut ram, 0x1ee0400, (1 << 8) | (1 << 6) | ((vector as u32) << 3) | 1, 0);
    let at = if !vector && args[1] & 1 != 0 { X } else { Z };
    (0..n).map(|k| ram.rd(at + 4 * (off + k))).collect()
}

#[test]
fn the_model_returns_what_the_fm1_did() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../tests/hwdsp/fm1.txt");
    let text = std::fs::read_to_string(path).expect("tests/hwdsp/fm1.txt");
    let (mut exact, mut near, mut skipped) = (0, 0, 0);
    let mut bad = vec![];
    for (id, args, hw) in cases(&text) {
        if args[0] == 0 {
            continue;
        }
        let model = run(&args);
        if args[0] == 2 {
            let con = args[1];
            let (wx, wz, mode) = ((con >> 6) & 3, (con >> 10) & 3, con & 3);
            // outside what is known: add with accumulate, 64-bit sums, z narrower than x,
            // loops whose increments break the element alignment
            let inc = args[3];
            if (mode == 0 && con & 0x20 != 0) || (wz == 3 && con & 0x20 != 0) || (wz < wx && (mode == 1 || mode == 2))
                || ((inc >> 20) & 0x3ff) % (1 << wz) != 0
            {
                skipped += 1;
                continue;
            }
            if model == hw { exact += 1 } else { bad.push(format!("{id}: not bit-exact")) }
        } else {
            // in range: the inputs that keep every stage under 2^28 (the plan's impulses and 2^14 randoms)
            if !(id.contains("/imp") || id.contains("/rnd14")) {
                skipped += 1;
                continue;
            }
            let d = model.iter().zip(&hw).map(|(a, b)| (*a as i32 as i64 - *b as i32 as i64).abs()).max().unwrap_or(0);
            if d <= 3 { near += 1 } else { bad.push(format!("{id}: off by {d}")) }
        }
    }
    eprintln!("hwdsp vs the FM-1: {exact} vector cases bit-exact, {near} FFT cases within 3, {skipped} outside the model");
    assert!(bad.is_empty(), "{}", bad.join("\n"));
    assert!(exact >= 58 && near >= 102, "too few cases: {exact} / {near}");
}
