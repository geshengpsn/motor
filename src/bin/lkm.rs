use std::{thread::sleep, time::Duration};

use deku::{DekuContainerWrite, DekuRead, DekuWrite};
use socketcan::{CanDataFrame, CanSocket, Frame, Socket};

fn main() {
    let can = CanSocket::open("can1").unwrap();
    can.write_frame(&CanDataFrame::from_raw_id(0x140 + 1, &OPEN).unwrap())
        .unwrap();
    let mut data = [0; 8];
    for i in 0..10 {
        let cmd = RawPosCmd::new(36000 * i);
        cmd.to_slice(&mut data).unwrap();
        can.write_frame(&CanDataFrame::from_raw_id(0x140 + 1, &data).unwrap())
            .unwrap();
        sleep(Duration::from_secs(1));
    }
    can.write_frame(&CanDataFrame::from_raw_id(0x140 + 1, &CLOSE).unwrap())
        .unwrap();
    let frame = can.read_frame().unwrap();
    println!("{frame:?}");
}

const CLOSE: [u8; 8] = [0x80, 0, 0, 0, 0, 0, 0, 0];
const OPEN: [u8; 8] = [0x88, 0, 0, 0, 0, 0, 0, 0];
const STOP: [u8; 8] = [0x81, 0, 0, 0, 0, 0, 0, 0];

#[derive(Debug, PartialEq, DekuRead, DekuWrite)]
#[deku(endian = "little")]
struct RawPosCmd {
    cmd: u8,
    zeros: [u8; 3],
    pos: i32,
}

impl RawPosCmd {
    fn new(pos: i32) -> RawPosCmd {
        RawPosCmd {
            cmd: 0xA3,
            zeros: [0; 3],
            pos,
        }
    }
}
