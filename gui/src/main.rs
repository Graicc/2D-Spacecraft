#![allow(dead_code, unused_variables)]
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::{
    net::UdpSocket,
    sync::{
        mpsc::{sync_channel, Receiver, SyncSender},
        Arc,
    },
    thread,
};

use eframe::egui::{self};
use egui_plot::{Line, PlotPoint, PlotPoints};
use messages::WifiMessage;
use nalgebra::Vector2;

type Float = f64;
type Vec2 = Vector2<Float>;

mod dynamics;

#[cfg(not(target_arch = "wasm32"))]
fn main() -> eframe::Result {
    env_logger::init();

    let native_options = eframe::NativeOptions::default();
    eframe::run_native(
        "2d Spacecraft",
        native_options,
        Box::new(|cc| Ok(Box::new(App::new(cc)))),
    )
}

#[cfg(target_arch = "wasm32")]
fn main() {
    use eframe::wasm_bindgen::JsCast as _;

    // Redirect `log` message to `console.log` and friends:
    eframe::WebLogger::init(log::LevelFilter::Debug).ok();

    let web_options = eframe::WebOptions::default();

    wasm_bindgen_futures::spawn_local(async {
        let document = web_sys::window()
            .expect("No window")
            .document()
            .expect("No document");

        let canvas = document
            .get_element_by_id("the_canvas_id")
            .expect("Failed to find the_canvas_id")
            .dyn_into::<web_sys::HtmlCanvasElement>()
            .expect("the_canvas_id was not a HtmlCanvasElement");

        let start_result = eframe::WebRunner::new()
            .start(
                canvas,
                web_options,
                Box::new(|cc| Ok(Box::new(App::new(cc)))),
            )
            .await;

        // Remove the loading text and spinner:
        if let Some(loading_text) = document.get_element_by_id("loading_text") {
            match start_result {
                Ok(_) => {
                    loading_text.remove();
                }
                Err(e) => {
                    loading_text.set_inner_html(
                        "<p> The app has crashed. See the developer console for details. </p>",
                    );
                    panic!("Failed to start eframe: {e:?}");
                }
            }
        }
    });
}

#[derive(Default)]
struct Plots {
    accel_x: Vec<PlotPoint>,
    accel_y: Vec<PlotPoint>,
    accel_z: Vec<PlotPoint>,
    gyro_x: Vec<PlotPoint>,
    gyro_y: Vec<PlotPoint>,
    gyro_z: Vec<PlotPoint>,
}

struct App {
    socket: Arc<UdpSocket>,
    incoming_receiver: Receiver<messages::WifiMessage>,
    outgoing_sender: SyncSender<messages::WifiMessage>,

    plots: Plots,
    pong_count: u64,
    ip: String,
}

impl App {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let socket = UdpSocket::bind((std::net::Ipv4Addr::UNSPECIFIED, 1234)).unwrap();
        // socket.connect("192.168.137.99:1234").unwrap();
        let socket = Arc::new(socket);

        let (incoming_sender, incoming_receiver) = sync_channel::<messages::WifiMessage>(256);

        let (outgoing_sender, outgoing_receiver) = sync_channel::<messages::WifiMessage>(256);

        let s1 = socket.clone();
        let receiver_ctx = cc.egui_ctx.clone();
        let _receiver_thread = thread::spawn(move || loop {
            // println!("Receiver thread loop");
            incoming_sender
                .send(messages::receive_sync::<messages::WifiMessage>(&s1).unwrap())
                .unwrap();
            receiver_ctx.request_repaint();
            // println!("Receiver thread loop2");
        });

        let s2 = socket.clone();
        let _sender_thread = thread::spawn(move || loop {
            // println!("Sender thread loop");
            messages::send_sync(&s2, &outgoing_receiver.recv().unwrap()).unwrap();
            // println!("Sender thread loop2");
        });

        App {
            socket,
            incoming_receiver,
            outgoing_sender,
            plots: Default::default(),
            pong_count: 0,
            ip: "".to_string(),
        }
    }
}

fn to_ui_pos(v: Vec2) -> egui::Pos2 {
    egui::Vec2::new(v.x as f32, v.y as f32).to_pos2()
}
fn to_ui_vec(v: Vec2) -> egui::Vec2 {
    egui::Vec2::new(v.x as f32, -v.y as f32)
}

fn update_plot(points: &mut Vec<PlotPoint>, new_value: impl Into<f64>) {
    let last_x = points.iter().last().map(|a| a.x).unwrap_or(0.0);
    points.push(PlotPoint::new(last_x + 0.01, new_value));
    if points.len() > 512 {
        points.drain(0..points.len() - 512);
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        while let Ok(message) = self.incoming_receiver.try_recv() {
            match message {
                WifiMessage::IMUReading(data) => {
                    update_plot(&mut self.plots.accel_x, data.acc[0]);
                    update_plot(&mut self.plots.accel_y, data.acc[1]);
                    update_plot(&mut self.plots.accel_z, data.acc[2]);

                    update_plot(&mut self.plots.gyro_x, data.gyro[0]);
                    update_plot(&mut self.plots.gyro_y, data.gyro[1]);
                    update_plot(&mut self.plots.gyro_z, data.gyro[2]);
                }
                WifiMessage::Ping(_) => todo!(),
                WifiMessage::Pong(num) => self.pong_count += num,
            }
        }

        egui::SidePanel::left("send_data").show(ctx, |ui| {
            ui.label("Hello, world!");
            ui.text_edit_singleline(&mut self.ip);
            if ui.button("Connect").clicked() {
                self.socket.connect(&self.ip).unwrap();
            }

            if ui.button("Send ping").clicked() {
                self.outgoing_sender.send(WifiMessage::Ping(1)).unwrap();
            }
            ui.label(format!("Pong count {}", self.pong_count));
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            egui_plot::Plot::new("accel")
                .legend(Default::default())
                .show(ui, |plot_ui| {
                    plot_ui.line(Line::new(
                        "Accel X",
                        PlotPoints::Borrowed(self.plots.accel_x.as_slice()),
                    ));
                    plot_ui.line(Line::new(
                        "Accel Y",
                        PlotPoints::Borrowed(self.plots.accel_y.as_slice()),
                    ));
                    plot_ui.line(Line::new(
                        "Accel Z",
                        PlotPoints::Borrowed(self.plots.accel_z.as_slice()),
                    ));
                    plot_ui.line(Line::new(
                        "Gyro X",
                        PlotPoints::Borrowed(self.plots.gyro_x.as_slice()),
                    ));
                    plot_ui.line(Line::new(
                        "Gyro Y",
                        PlotPoints::Borrowed(self.plots.gyro_y.as_slice()),
                    ));
                    plot_ui.line(Line::new(
                        "Gyro Z",
                        PlotPoints::Borrowed(self.plots.gyro_z.as_slice()),
                    ));
                })
        });
    }
}
