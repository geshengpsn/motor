use std::{
    collections::VecDeque,
    f64::consts::TAU,
    net::TcpListener,
    sync::{mpsc::channel, Arc, RwLock},
    thread::spawn,
    time::Instant,
};

use deku::prelude::*;
use frame::FdFlags;

use serde::{Deserialize, Serialize};
use socketcan::*;
use tungstenite::{accept, Message};

#[derive(Serialize, Deserialize, Clone, Copy, Debug)]
enum Control {
    Calibration,
    Disable,
}

fn main() -> std::io::Result<()> {
    let avg_window = Arc::new(RwLock::new(VecDeque::from([(0_f64, Instant::now()); 2])));
    let (tx, rx) = channel::<Control>();
    let avg_window_phone = avg_window.clone();

    spawn(move || {
        let server = TcpListener::bind("0.0.0.0:6666").unwrap();
        for stream in server.incoming() {
            let stream = stream.unwrap();
            let mut ws = accept(stream).unwrap();
            let mut window = VecDeque::from([0_f64; 10]);
            loop {
                let msg = ws.read();
                match msg {
                    Ok(msg) => match msg {
                        Message::Text(s) => {
                            match s.as_str() {
                                "Calibration" => {
                                    tx.send(Control::Calibration).unwrap();
                                }
                                "Disable" => {
                                    tx.send(Control::Disable).unwrap();
                                }
                                _ => {}
                            }
                            // let control = serde_json::from_str::<Control>(&s).unwrap();
                            // tx.send(control).unwrap();
                        }
                        Message::Binary(buf) => {
                            let pos = f32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as f64;
                            let pos = if pos == -1. { window[9] } else { pos };
                            window.push_back(pos);
                            window.pop_front();
                            let avg = window.iter().sum::<f64>() / 10.;
                            let time = Instant::now();
                            {
                                let mut write_guard = avg_window_phone.write().unwrap();
                                write_guard.push_back((avg, time));
                                write_guard.pop_front();
                            }
                        }
                        Message::Close(_) => {
                            println!("Close");
                            break;
                        }
                        _ => {}
                    },
                    Err(e) => {
                        println!("{e}");
                        break;
                    }
                }
            }
        }
        // let (stream, _) = server.accept().unwrap();
    });

    // let avg_window_phone = avg_window.clone();
    // let _j = spawn(move || {
    //     let session = zenoh::open(Config::default()).res().unwrap();
    //     let mut window = VecDeque::from([0_f64; 10]);
    //     loop {
    //         let sub = session.declare_subscriber("gripper/angle").res().unwrap();
    //         if let Ok(sample) = sub.recv() {
    //             let pos: f64 = sample.value.try_into().unwrap();
    //             // println!("{pos}");
    //             let pos = if pos == -1. { window[9] } else { pos };
    //             window.push_back(pos);
    //             window.pop_front();
    //             let avg = window.iter().sum::<f64>() / 10.;
    //             let time = Instant::now();
    //             {
    //                 let mut write_guard = avg_window_phone.write().unwrap();
    //                 write_guard.push_back((avg, time));
    //                 write_guard.pop_front();
    //             }
    //         }
    //     }
    // });
    // j.join();

    let mut gripper = Gripper::new("can0", 1).unwrap();
    loop {
        match rx.try_recv() {
            Ok(c) => match c {
                Control::Calibration => {
                    println!("Calibration");
                    gripper.calibration()
                }
                Control::Disable => {
                    println!("Disable");
                    gripper.disable();
                }
            },
            Err(e) => match e {
                std::sync::mpsc::TryRecvError::Empty => {
                    let ((a, a_time), (b, b_time)) = {
                        let read_guard = avg_window.read().unwrap();
                        (read_guard[0], read_guard[1])
                    };
                    let now = Instant::now();
                    let slop = (b - a) / (b_time - a_time).as_secs_f64();
                    let pos = slop * (now - b_time).as_secs_f64() + b;
                    gripper.pos_servo(pos);
                }
                std::sync::mpsc::TryRecvError::Disconnected => {
                    panic!("Disconnected")
                }
            },
        }
    }
    // Ok(())

    // let mut pos = gripper.motor.read_data().unwrap().pos;
    // loop {
    //     let vel = (pos - (gripper.close * 0.5 + gripper.open * 0.5)) * -100.;
    //     pos = gripper
    //         .motor
    //         .write_read(Data::Control(ControlFrame {
    //             pos: 0.,
    //             pos_kp: 0,
    //             pos_kd: 0,
    //             vel,
    //             cur: 1.,
    //             mode: Mode::Vel,
    //             status: Status::Enable,
    //         }))
    //         .unwrap()
    //         .pos;
    // }
}

struct Gripper {
    motor: Motor,
    open: f64,
    close: f64,
}

impl Gripper {
    fn new(ifname: &str, id: u16) -> std::io::Result<Gripper> {
        let motor = Motor::connect(ifname, id)?;
        motor.config(ConfigTerm::CurKp, 1.).unwrap();
        motor.config(ConfigTerm::CurKi, 1.).unwrap();
        motor.config(ConfigTerm::VelKp, 1.).unwrap();
        motor.config(ConfigTerm::VelKi, 1.).unwrap();
        Ok(Gripper {
            motor,
            open: 0.,
            close: 0.,
        })
    }

    fn calibration(&mut self) {
        // open
        let mut window = VecDeque::new();

        loop {
            let data = self
                .motor
                .write_read(Data::Control(ControlFrame {
                    pos: 0.,
                    pos_kp: 0,
                    pos_kd: 0,
                    vel: 10.,
                    cur: 1.,
                    mode: Mode::Vel,
                    status: Status::Enable,
                }))
                .unwrap();
            if window.len() < 100 {
                window.push_back(data.vel);
            } else {
                let avg = window.iter().sum::<f64>() / 100.;
                if avg.abs() < 1e-5 {
                    break;
                } else {
                    window.push_back(data.vel);
                    window.pop_front();
                }
            }
        }
        let open = self.motor.read_data().unwrap().pos;
        // println!("open {open}");
        self.open = open;

        // close
        let mut window = VecDeque::new();
        loop {
            let data = self
                .motor
                .write_read(Data::Control(ControlFrame {
                    pos: 0.,
                    pos_kp: 0,
                    pos_kd: 0,
                    vel: -10.,
                    cur: 1.,
                    mode: Mode::Vel,
                    status: Status::Enable,
                }))
                .unwrap();
            if window.len() < 100 {
                window.push_back(data.vel);
            } else {
                let avg = window.iter().sum::<f64>() / 100.;
                if avg.abs() < 1e-5 {
                    break;
                } else {
                    window.push_back(data.vel);
                    window.pop_front();
                }
            }
        }
        let close = self.motor.read_data().unwrap().pos;
        // println!("close {close}");
        self.close = close;
    }

    fn disable(&self) {
        self.motor
            .write_read(Data::Control(ControlFrame {
                pos: 0.,
                pos_kp: 0,
                pos_kd: 0,
                vel: 0.,
                cur: 0.,
                mode: Mode::Pos,
                status: Status::Disable,
            }))
            .unwrap();
    }

    // pos: 0 - 1
    fn pos_servo(&self, pos: f64) {
        self.motor
            .write_read(Data::Control(ControlFrame {
                pos: self.close * (1. - pos) + self.open * pos,
                pos_kp: 1,
                pos_kd: 1,
                vel: 200.,
                cur: 1.,
                mode: Mode::Pos,
                status: Status::Enable,
            }))
            .unwrap();
    }

    // pos: 0 - 1
    // fn cur_servo(&self, force: f64) {}

    // pos: 0 - 1
    // fn vel_servo(&self, vel: f64) {}
}

struct Motor {
    can: CanFdSocket,
    id: u16,
}

impl Motor {
    fn connect(ifname: &str, id: u16) -> std::io::Result<Motor> {
        let motor = Motor {
            can: CanFdSocket::open(ifname)?,
            id,
        };
        Ok(motor)
    }

    pub fn write_read(&self, data: Data) -> std::io::Result<RecvData> {
        let data = data.to_data();
        self.can.write_frame(
            &CanFdFrame::with_flags(
                Id::Standard(StandardId::new(self.id).unwrap()),
                &data,
                FdFlags::empty(),
            )
            .unwrap(),
        )?;
        let frame = self.can.read_frame().unwrap();
        if let CanAnyFrame::Fd(fdframe) = frame {
            let data = fdframe.data();
            let recv = RecvData::from_data(data).unwrap();
            Ok(recv)
        } else {
            Err(std::io::Error::other("not can fd"))
        }
    }

    pub fn config(&self, term: ConfigTerm, value: f32) -> std::io::Result<RecvData> {
        self.write_read(Data::Config(ConfigFrame { term, value }))
    }

    pub fn read_data(&self) -> std::io::Result<RecvData> {
        self.write_read(Data::Control(ControlFrame::default()))
    }
}

#[derive(Debug, PartialEq, DekuRead, DekuWrite)]
#[deku(endian = "big")]
struct RawSendData {
    pos: [u8; 5],
    vel: i32,
    cur: i16,
    mode: u8,
    status: u8,
    pos_kp: u8,
    pos_kd: u8,
    zero: u8,
}

#[derive(Debug)]
enum Data {
    Config(ConfigFrame),
    Control(ControlFrame),
}

impl Data {
    fn to_data(&self) -> [u8; 16] {
        match self {
            Data::Config(config) => config.to_data(),
            Data::Control(data) => data.to_data(),
        }
    }
}

#[derive(Debug, PartialEq, DekuRead, DekuWrite)]
#[deku(endian = "big")]
struct RawConfigFrame {
    zero: u8,
    term: u8,
    value: f32,
    padding: [u8; 10],
}

#[derive(Debug, Clone, Copy)]
enum ConfigTerm {
    VelKp = 0,
    VelKi = 1,
    CurKp = 2,
    CurKi = 3,
}

#[derive(Debug, Clone)]
struct ConfigFrame {
    term: ConfigTerm,
    value: f32,
}

impl ConfigFrame {
    fn to_data(&self) -> [u8; 16] {
        let mut data = [0u8; 16];
        let _ = RawConfigFrame {
            zero: 0,
            term: self.term as u8,
            value: self.value,
            padding: [0u8; 10],
        }
        .to_slice(&mut data);
        data
    }
}

#[derive(Debug, Clone, Default)]
struct ControlFrame {
    // rad
    pos: f64,

    pos_kp: u8,

    pos_kd: u8,

    // Freq
    vel: f64,

    // A
    cur: f64,

    mode: Mode,

    status: Status,
}

impl ControlFrame {
    pub fn to_data(&self) -> [u8; 16] {
        let p = ((self.pos / TAU) * 1048576.) as i64;
        let bytes = p.to_le_bytes();
        let raw = RawSendData {
            pos: [bytes[4], bytes[3], bytes[2], bytes[1], bytes[0]],
            vel: (self.vel * 8388.608) as i32,
            cur: (self.cur * 327.68) as i16,
            mode: self.mode as u8,
            status: self.status as u8,
            pos_kp: self.pos_kp,
            pos_kd: self.pos_kd,
            zero: 0u8,
        };
        let mut data = [0; 16];
        let _ = raw.to_slice(&mut data);
        data
    }
}

#[derive(Debug, PartialEq, DekuRead, DekuWrite)]
#[deku(endian = "big")]
struct RawRecvData {
    pos: [u8; 5],
    vel: i32,
    cur: i16,
    mode: u8,
    status: u8,
    reserve: u8,
    temp: i8,
    id: u8,
}

#[derive(Debug, Clone, Copy, Default)]
pub enum Mode {
    #[default]
    Cur = 0,
    Vel = 1,
    Pos = 2,
}

impl From<u8> for Mode {
    fn from(value: u8) -> Self {
        match value {
            0 => Mode::Cur,
            1 => Mode::Vel,
            2 => Mode::Pos,
            _ => panic!("should not happend"),
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub enum Status {
    #[default]
    Disable = 0,
    Enable = 1,
}

impl From<u8> for Status {
    fn from(value: u8) -> Self {
        match value {
            0 => Status::Disable,
            1 => Status::Enable,
            _ => panic!("should not happend"),
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct RecvData {
    // rad
    pub pos: f64,

    // Freq
    pub vel: f64,

    // A
    pub cur: f64,

    pub mode: Mode,

    pub status: Status,

    pub temp: i8,

    pub id: u8,
}

impl RecvData {
    fn from_data(data: &[u8]) -> core::result::Result<RecvData, deku::error::DekuError> {
        let (_, raw_data) = RawRecvData::from_bytes((data, 0))?;
        let pos = if raw_data.pos[0] >> 7 == 1 {
            i64::from_be_bytes([
                0xff,
                0xff,
                0xff,
                raw_data.pos[0],
                raw_data.pos[1],
                raw_data.pos[2],
                raw_data.pos[3],
                raw_data.pos[4],
            ])
        } else {
            i64::from_be_bytes([
                0,
                0,
                0,
                raw_data.pos[0],
                raw_data.pos[1],
                raw_data.pos[2],
                raw_data.pos[3],
                raw_data.pos[4],
            ])
        };
        let pos_circle_count = (pos / 1048576) as f64 * TAU;
        let pos_rad = (pos % 1048576) as f64 / 1048576. * TAU;
        Ok(RecvData {
            pos: pos_circle_count + pos_rad,
            vel: raw_data.vel as f64 / 8388.608,
            cur: raw_data.cur as f64 / 327.68,
            mode: Mode::from(raw_data.mode),
            status: Status::from(raw_data.status),
            temp: raw_data.temp,
            id: raw_data.id,
        })
    }
}
