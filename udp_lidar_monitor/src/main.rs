use std::{
    io::{self, ErrorKind},
    net::UdpSocket,
    ops::Deref,
    sync::mpsc::{channel, Receiver, Sender},
    thread,
};

use eframe::egui::{self, Event, Vec2};
use egui::{debug_text::print, Color32, Context, Pos2};

mod parsers;

const UDP_ADDRESS: &str = "192.168.4.2";
const UDP_PORT: &str = "3333";

#[derive(Debug, Clone)]
struct LidarPacket {
    idx: u32,
    distance: f32,
    angle: f32,
    quality: u32,
    scan_complete: bool,
}

enum ConnectionState {
    SuccessfulConnection,
    FailedConnection,
}

fn main() -> Result<(), eframe::Error> {
    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "serial-plotter",
        options,
        Box::new(|cc| {
            let (packet_sender, packet_receiver): (Sender<LidarPacket>, Receiver<LidarPacket>) =
                channel();
            let (connection_state_sender, connection_state_receiver): (
                Sender<ConnectionState>,
                Receiver<ConnectionState>,
            ) = channel();
            let (reconnect_sender, reconnect_receiver): (
                Sender<ConnectionData>,
                Receiver<ConnectionData>,
            ) = channel();
            let ctx = cc.egui_ctx.clone();
            let _io_thread = thread::spawn(move || {
                run_io(
                    ConnectionData::new(UDP_ADDRESS, UDP_PORT),
                    ctx,
                    packet_sender,
                    reconnect_receiver,
                    connection_state_sender,
                )
            });
            Box::<Plot>::new(Plot::new(
                packet_receiver,
                reconnect_sender,
                connection_state_receiver,
            ))
        }),
    )
}

#[derive(Debug, Clone)]
struct ConnectionData {
    address: String,
    port: String,
}

impl ConnectionData {
    fn new<S: AsRef<str>>(address: S, port: S) -> Self {
        Self {
            address: String::from(address.as_ref()),
            port: String::from(port.as_ref()),
        }
    }
}

impl From<ConnectionData> for String {
    fn from(connection_data: ConnectionData) -> Self {
        return format!("{}:{}", connection_data.address, connection_data.port);
    }
}

fn run_io(
    connection_data: ConnectionData,
    ctx: Context,
    packet_sender: Sender<LidarPacket>,
    reconnect_receiver: Receiver<ConnectionData>,
    connection_state_sender: Sender<ConnectionState>,
) {
    match UdpDecoder::new(String::from(connection_data)) {
        Ok(decoder) => {
            connection_state_sender
                .send(ConnectionState::SuccessfulConnection)
                .expect("Channel<ConnectionState> already closed.");
            loop {
                if let Ok(lidar_packet) = decoder.recv() {
                    let _ = packet_sender.send(lidar_packet);
                    ctx.request_repaint();
                }
            }
        }
        Err(_) => {
            connection_state_sender
                .send(ConnectionState::FailedConnection)
                .expect("Channel<ConnectionState> already closed.");
            if let Ok(reconnect_data) = reconnect_receiver.recv() {
                println!("Trying to reconnect...");
                run_io(
                    reconnect_data,
                    ctx,
                    packet_sender,
                    reconnect_receiver,
                    connection_state_sender,
                )
            }
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
    packet_receiver: Receiver<LidarPacket>,
    reconnect_sender: Sender<ConnectionData>,
    connection_state_receiver: Receiver<ConnectionState>,
    last_connection_state: ConnectionState,
    connection_data: ConnectionData,
    address_ok: bool,
    port_ok: bool,
    plot_points: PlotData,
    zoom_factor: Vec2,
}

impl Plot {
    fn new(
        packet_receiver: Receiver<LidarPacket>,
        reconnect_sender: Sender<ConnectionData>,
        connection_state_receiver: Receiver<ConnectionState>,
    ) -> Plot {
        let last_connection_state = connection_state_receiver
            .recv()
            .expect("Channel<ConnectionState> hung up!");
        let connection_data = ConnectionData::new(UDP_ADDRESS, UDP_PORT);
        Self {
            packet_receiver,
            reconnect_sender,
            connection_state_receiver,
            last_connection_state,
            connection_data,
            address_ok: true,
            port_ok: true,
            plot_points: PlotData::new(),
            zoom_factor: Vec2::new(10.0, 10.0),
        }
    }
}

impl eframe::App for Plot {
    fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        if let Ok(connection_state) = self.connection_state_receiver.try_recv() {
            self.last_connection_state = connection_state;
        }
        match self.last_connection_state {
            ConnectionState::SuccessfulConnection => self.update_lidar_map(ctx, frame),
            ConnectionState::FailedConnection => self.update_failed_connection(ctx, frame),
        }
    }
}

impl Plot {
    fn update_lidar_map(&mut self, ctx: &egui::Context, _: &mut eframe::Frame) {
        while let Ok(lidar_packet) = self.packet_receiver.try_recv() {
            self.plot_points.push(
                lidar_packet.idx,
                lidar_packet.angle as f64,
                lidar_packet.distance as f64,
                lidar_packet.quality,
            );
        }
        egui::CentralPanel::default().show(ctx, |ui| self.ui_lidar_map(ui));
    }
    fn ui_lidar_map(&mut self, ui: &mut egui::Ui) {
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
    }
    fn update_failed_connection(&mut self, ctx: &egui::Context, _: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| self.ui_reconnect(ui));
    }
    fn ui_reconnect(&mut self, ui: &mut egui::Ui) {
        ui.label("UDP Address");
        let mut address_text_edit = egui::TextEdit::singleline(&mut self.connection_data.address);
        if !self.address_ok {
            address_text_edit = address_text_edit.text_color(Color32::from_rgb(255, 0, 0));
        }
        let address_response = ui.add(address_text_edit);
        if address_response.changed() {
            match parsers::parse_udp_address(&self.connection_data.address) {
                Ok(_) => {
                    self.address_ok = true;
                }
                Err(_) => self.address_ok = false,
            }
        }

        ui.label("UDP Port");
        let mut port_text_edit = egui::TextEdit::singleline(&mut self.connection_data.port);
        if !self.port_ok {
            port_text_edit = port_text_edit.text_color(Color32::from_rgb(255, 0, 0));
        }
        let port_response = ui.add(port_text_edit);
        if port_response.changed() {
            match parsers::parse_udp_port(&self.connection_data.port) {
                Ok(_) => {
                    self.port_ok = true;
                }
                Err(_) => self.port_ok = false,
            }
        }

        if !self.address_ok || !self.port_ok {
            let button = egui::Button::new("Try reconnect");
            ui.add_enabled(false, button);
        } else {
            if ui.button("Try reconnect").clicked() {
                self.reconnect_sender
                    .send(self.connection_data.clone())
                    .expect("Channel<ConnectionData> already closed!");
            }
        }
    }
}
