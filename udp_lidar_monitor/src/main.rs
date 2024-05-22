use std::{
    io::{self, ErrorKind},
    net::UdpSocket,
    ops::Deref,
    sync::{
        mpsc::{channel, Receiver, Sender},
        Arc, RwLock,
    },
    thread,
    time::Duration,
};

use eframe::egui::{self, Event, Vec2};
use egui::{Color32, Context, Pos2};

const UDP_PORT: u32 = 3333;

#[derive(Debug, Clone)]
struct LidarPacket {
    idx: u32,
    distance: f32,
    angle: f32,
    quality: u32,
    scan_complete: bool,
}

#[derive(Debug, Clone)]
enum State {
    Startup,
    RequestUDPConnect { address: String },
    FailedUDPConnect,
    SuccessfulUDPConnect,
}

impl State {
    fn io_handle(&self, next_state: Self) -> Self {
        match self {
            Self::RequestUDPConnect { address: _ } => next_state,
            _ => self.clone(),
        }
    }

    fn io_has_message(&self) -> bool {
        match self {
            Self::RequestUDPConnect { address: _ } => true,
            _ => false,
        }
    }

    fn ui_handle(&self, next_state: Self) -> Self {
        match self {
            Self::Startup | Self::FailedUDPConnect | Self::SuccessfulUDPConnect => next_state,
            _ => self.clone(),
        }
    }
    fn ui_has_message(&self) -> bool {
        match self {
            Self::Startup | Self::FailedUDPConnect | Self::SuccessfulUDPConnect => true,
            _ => false,
        }
    }

    fn is(&self, other_state: Self) -> bool {
        match self {
            Self::Startup => match other_state {
                Self::Startup => true,
                _ => false,
            },
            Self::RequestUDPConnect { address: _ } => match other_state {
                Self::RequestUDPConnect { address: _ } => true,
                _ => false,
            },
            Self::SuccessfulUDPConnect => match other_state {
                Self::SuccessfulUDPConnect => true,
                _ => false,
            },
            Self::FailedUDPConnect => match other_state {
                Self::FailedUDPConnect => true,
                _ => false,
            },
        }
    }
}

impl Default for State {
    fn default() -> Self {
        State::Startup
    }
}

fn main() -> Result<(), eframe::Error> {
    let options = eframe::NativeOptions::default();
    let mut state = Arc::new(RwLock::new(State::default()));
    let io_state = state.clone();
    let ui_state = state.clone();
    eframe::run_native(
        "serial-plotter",
        options,
        Box::new(|cc| {
            let (pp_sender, pp_receiver): (Sender<LidarPacket>, Receiver<LidarPacket>) = channel();
            let ctx = cc.egui_ctx.clone();
            let _io_thread = thread::spawn(move || run_io(io_state, ctx, pp_sender));
            Box::<Plot>::new(Plot::new(ui_state, pp_receiver))
        }),
    )
}

fn run_io(state: Arc<RwLock<State>>, ctx: Context, sender: Sender<LidarPacket>) {
    match UdpDecoder::new(format!("192.168.4.2:{UDP_PORT}")) {
        Ok(decoder) => {
            {
                let mut state_handle = state.write().unwrap();
                *state_handle = state_handle.io_handle(State::SuccessfulUDPConnect).into();
            }
            loop {
                if let Ok(lidar_packet) = decoder.recv() {
                    let _ = sender.send(lidar_packet);
                    ctx.request_repaint();
                }
            }
        }
        Err(_) => {
            let mut state_handle = state.write().unwrap();
            *state_handle = state_handle.io_handle(State::FailedUDPConnect).into();
            drop(state_handle);
            while !state.read().unwrap().io_has_message() {
                thread::sleep(Duration::from_millis(10));
            }
            if state.read().unwrap().is(State::RequestUDPConnect {
                address: String::new(),
            }) {}
        }
    }
}

#[derive(Debug)]
struct UdpDecoder {
    socket: UdpSocket,
}

impl LidarPacket {
    fn new(idx: u32, distance: f32, angle: f32, quality: u32, scan_complete: bool) -> Self {
        Self {
            idx,
            distance,
            angle,
            quality,
            scan_complete,
        }
    }
}

impl UdpDecoder {
    fn new<S: AsRef<str>>(udp_address: S) -> io::Result<Self> {
        let socket = UdpSocket::bind(udp_address.as_ref())?;
        Ok(Self { socket })
    }

    fn recv(&self) -> io::Result<LidarPacket> {
        let mut buf = [0u8; 17];
        let (amt, _) = self.socket.recv_from(&mut buf)?;
        if amt < 17 {
            return Err(io::Error::new(
                ErrorKind::InvalidData,
                format!("UDP packet has only {amt} bytes, but must have 17 bytes."),
            ));
        }

        let idx_buf: [u8; 4] = buf[0..4].try_into().expect("wrong length");
        let idx = u32::from_be_bytes(idx_buf);
        let distance_buf: [u8; 4] = buf[4..8].try_into().unwrap();
        let distance = f32::from_be_bytes(distance_buf);
        let angle_buf: [u8; 4] = buf[8..12].try_into().unwrap();
        let angle = f32::from_be_bytes(angle_buf);
        let quality_buf: [u8; 4] = buf[12..16].try_into().unwrap();
        let quality = u32::from_be_bytes(quality_buf);
        let complete = buf[16] != 0;
        println!("[{idx}] distance: {distance}, angle: {angle}, quality: {quality}, complete: {complete}");
        Ok(LidarPacket::new(idx, distance, angle, quality, complete))
    }
}

#[derive(Debug, Clone, PartialEq)]
struct LidarPoint {
    idx: u32,
    angle: f64,
    distance: f64,
    x: f64,
    y: f64,
    color: Color32,
}
impl LidarPoint {
    fn new(idx: u32, angle: f64, distance: f64, x: f64, y: f64, color: impl Into<Color32>) -> Self {
        Self {
            idx,
            angle,
            distance,
            x,
            y,
            color: color.into(),
        }
    }
}

#[derive(Debug, Clone)]
struct PlotData {
    data: Vec<LidarPoint>,
}

impl PlotData {
    fn new() -> Self {
        Self { data: Vec::new() }
    }

    fn push(&mut self, idx: u32, angle: f64, distance: f64, quality: u32) {
        let x = angle.to_radians().cos() * distance;
        let y = angle.to_radians().sin() * distance;
        let color = Color32::from_rgba_unmultiplied(
            255,
            0,
            0,
            (<u32 as TryInto<u8>>::try_into(quality).unwrap() << 1) + 128,
        );

        self.data
            .retain(|dp| dp.idx > idx || dp.angle < angle - 0.1 || dp.angle > angle + 0.1);
        self.data
            .push(LidarPoint::new(idx, angle, distance, x, y, color))
    }
}

impl<'a> IntoIterator for &'a PlotData {
    type Item = &'a LidarPoint;
    type IntoIter = <&'a Vec<LidarPoint> as IntoIterator>::IntoIter;
    fn into_iter(self) -> Self::IntoIter {
        return (&(*self.data)).into_iter();
    }
}

impl Deref for PlotData {
    type Target = [LidarPoint];

    fn deref(&self) -> &[LidarPoint] {
        &self.data[..]
    }
}

impl Into<Vec<[f64; 2]>> for PlotData {
    fn into(self) -> Vec<[f64; 2]> {
        self.data
            .into_iter()
            .map(|data_point| [data_point.x, data_point.y])
            .collect()
    }
}

struct Plot {
    state: Arc<RwLock<State>>,
    in_channel: Receiver<LidarPacket>,
    plot_points: PlotData,
    zoom_factor: Vec2,
}

impl Plot {
    fn new(state: Arc<RwLock<State>>, in_channel: Receiver<LidarPacket>) -> Plot {
        Self {
            state,
            in_channel,
            plot_points: PlotData::new(),
            zoom_factor: Vec2::new(10.0, 10.0),
        }
    }
}

impl eframe::App for Plot {
    fn update(&mut self, ctx: &egui::Context, _: &mut eframe::Frame) {
        while let Ok(lidar_packet) = self.in_channel.try_recv() {
            self.plot_points.push(
                lidar_packet.idx,
                lidar_packet.angle as f64,
                lidar_packet.distance as f64,
                lidar_packet.quality,
            );
        }
        egui::CentralPanel::default().show(ctx, |ui| {
            let (scroll, _, _) = ui.input(|i| {
                let scroll = i.events.iter().find_map(|e| match e {
                    Event::MouseWheel {
                        unit: _,
                        delta,
                        modifiers: _,
                    } => Some(*delta),
                    _ => None,
                });
                (scroll, i.pointer.primary_down(), i.modifiers)
            });

            if let Some(mut scroll) = scroll {
                scroll = Vec2::splat(scroll.x + scroll.y);
                self.zoom_factor.x *= (scroll.x / 10.0).exp();
                self.zoom_factor.y *= (scroll.x / 10.0).exp();
            }

            let painter = ui.painter();
            let clip_rect = painter.clip_rect();
            let mid_x = (clip_rect.max.x - clip_rect.min.x) / 2.0;
            let mid_y = (clip_rect.max.y - clip_rect.min.y) / 2.0;
            for point in &self.plot_points {
                painter.circle_filled(
                    Pos2::new(
                        mid_x + (point.x as f32 / self.zoom_factor.x),
                        mid_y + (point.y as f32 / self.zoom_factor.y),
                    ),
                    2.0,
                    point.color,
                );
            }
        });
    }
}
