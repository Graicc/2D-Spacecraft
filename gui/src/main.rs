#![allow(dead_code, unused_variables)]
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::{
    net::UdpSocket,
    sync::{
        mpsc::{sync_channel, Receiver, SyncSender},
        Arc,
    },
    thread,
    time::{Duration, Instant},
};

use eframe::egui::{self, InputState, Key, ProgressBar, Slider};
use egui_plot::{Line, PlotPoint, PlotPoints};
use messages::{MotorValues, WifiMessage};
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

    motor_left: Vec<PlotPoint>,
    motor_right: Vec<PlotPoint>,
    motor_base: Vec<PlotPoint>,

    voltage: Vec<PlotPoint>,
}

struct Input {
    left_down: bool,
    right_down: bool,
}

impl Input {
    fn from_app(i: &InputState) -> Self {
        Self {
            left_down: i.key_down(Key::A),
            right_down: i.key_down(Key::D),
        }
    }
}

struct App {
    socket: Arc<UdpSocket>,
    incoming_receiver: Receiver<messages::WifiMessage>,
    outgoing_sender: SyncSender<messages::WifiMessage>,

    plots: Plots,
    pong_count: u64,
    ip: String,

    last_update: Instant,

    phase: u8,

    last_input: Input,
    base_thrust: f32,
    thrust: f32,
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
            // messages::send_sync(&s2, &outgoing_receiver.recv().unwrap()).unwrap();
            let _ = messages::send_sync(&s2, &outgoing_receiver.recv().unwrap());
            // println!("Sender thread loop2");
        });

        // Send a heartbeat
        let s3 = socket.clone();
        let _heartbeat_thread = thread::spawn(move || loop {
            thread::sleep(Duration::from_millis(500));
            let _ = messages::send_sync(&s3, &WifiMessage::HeartBeat);
        });

        App {
            socket,
            incoming_receiver,
            outgoing_sender,
            plots: Default::default(),
            pong_count: 0,
            ip: "".to_string(),
            last_update: Instant::now(),
            phase: 0,
            last_input: Input {
                left_down: false,
                right_down: false,
            },
            base_thrust: 0.0,
            thrust: 0.0,
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
        self.phase = (self.phase + 1) % 20;

        ctx.request_repaint();
        while let Ok(message) = self.incoming_receiver.try_recv() {
            match message {
                WifiMessage::StateUpdate(data) => {
                    self.last_update = Instant::now();

                    update_plot(&mut self.plots.accel_x, data.imu_status.acc[0]);
                    update_plot(&mut self.plots.accel_y, data.imu_status.acc[1]);
                    update_plot(&mut self.plots.accel_z, data.imu_status.acc[2]);

                    update_plot(&mut self.plots.gyro_x, data.imu_status.gyro[0]);
                    update_plot(&mut self.plots.gyro_y, data.imu_status.gyro[1]);
                    update_plot(&mut self.plots.gyro_z, data.imu_status.gyro[2]);

                    update_plot(&mut self.plots.motor_left, data.motor_values.left);
                    update_plot(&mut self.plots.motor_right, data.motor_values.right);
                    update_plot(&mut self.plots.motor_base, data.motor_values.base);

                    update_plot(&mut self.plots.voltage, data.voltage);
                }
                WifiMessage::Pong(num) => self.pong_count += num,
                WifiMessage::HeartBeat | WifiMessage::Ping(_) | WifiMessage::MotorValues(_) => {
                    todo!()
                } // Should never be received
            }
        }

        if self.phase == 0 {
            self.last_input = ctx.input(Input::from_app);
            let motor_values = MotorValues {
                left: if self.last_input.left_down {
                    (u8::MAX as f32 * self.thrust) as u8
                } else {
                    0
                },
                right: if self.last_input.right_down {
                    (u8::MAX as f32 * self.thrust) as u8
                } else {
                    0
                },
                base: (u8::MAX as f32 * self.base_thrust) as u8,
            };
            let _ = self
                .outgoing_sender
                .send(WifiMessage::MotorValues(motor_values));
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

            ui.label(format!(
                "Last update time: {:3}",
                (Instant::now() - self.last_update).as_secs_f32()
            ));

            ui.label("Thrust Slider: ");
            ui.add(Slider::new(&mut self.thrust, 0.0..=1.0));

            ui.label("Base Thrust Slider: ");
            ui.add(Slider::new(&mut self.base_thrust, 0.0..=1.0));

            let left = ProgressBar::new(if self.last_input.left_down { 1.0 } else { 0.0 });
            ui.add(left);

            let right = ProgressBar::new(if self.last_input.right_down { 1.0 } else { 0.0 });
            ui.add(right);
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
                    plot_ui.line(Line::new(
                        "Motor Left",
                        PlotPoints::Borrowed(self.plots.motor_left.as_slice()),
                    ));
                    plot_ui.line(Line::new(
                        "Motor Right",
                        PlotPoints::Borrowed(self.plots.motor_right.as_slice()),
                    ));
                    plot_ui.line(Line::new(
                        "Motor Base",
                        PlotPoints::Borrowed(self.plots.motor_base.as_slice()),
                    ));
                    plot_ui.line(Line::new(
                        "Voltage",
                        PlotPoints::Borrowed(self.plots.voltage.as_slice()),
                    ));
                })
        });
    }
}
