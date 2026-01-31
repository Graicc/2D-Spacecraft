#![allow(dead_code, unused_variables)]
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::net::UdpSocket;

use dynamics::{Simulation, State};
use eframe::{
    egui::{self, Frame, Rect},
    emath::{self},
};
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

struct App {
    simulation: Simulation,
}

impl App {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let simulation = Simulation {
            state: State {
                position: Vec2::zeros(),
                theta: 0.0,
                // velocity: Vec2::new(1.0, 1.0),
                velocity: Vec2::new(0.0, 0.0),
                theta_dot: 0.0,
            },
            ..Default::default()
        };
        App { simulation }
    }
}

fn to_ui_pos(v: Vec2) -> egui::Pos2 {
    egui::Vec2::new(v.x as f32, v.y as f32).to_pos2()
}
fn to_ui_vec(v: Vec2) -> egui::Vec2 {
    egui::Vec2::new(v.x as f32, -v.y as f32)
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("2D Spacecraft");

            egui::SidePanel::left("controls")
                .resizable(true)
                .min_width(230.0)
                .show_inside(ui, |ui| {
                    egui::Grid::new("Impulse")
                        .min_col_width(0.0)
                        .show(ui, |ui| {
                            let kick = 0.1;
                            if ui.small_button("↺").clicked() {
                                self.simulation.state.theta_dot += kick;
                            }
                            if ui.small_button("U").clicked() {
                                self.simulation.state.velocity.y += kick;
                            }
                            if ui.small_button("↻").clicked() {
                                self.simulation.state.theta_dot -= kick;
                            }
                            ui.end_row();

                            if ui.small_button("L").clicked() {
                                self.simulation.state.velocity.x -= kick;
                            }
                            if ui.small_button("·").on_hover_text("Reset").clicked() {
                                self.simulation.state.position = Vec2::zeros();
                                self.simulation.state.velocity = Vec2::zeros();
                            }
                            if ui.small_button("R").clicked() {
                                self.simulation.state.velocity.x += kick;
                            }
                            ui.end_row();

                            ui.label("");
                            if ui.small_button("D").clicked() {
                                self.simulation.state.velocity.y -= kick;
                            }
                        });

                    ui.add_space(30.0);
                    ui.label("RCS Values");

                    ui.horizontal(|ui| {
                        ui.label("X");
                        ui.add(egui::Slider::new(
                            &mut self.simulation.controller.target.force.x,
                            -1.0..=1.0,
                        ));
                        if ui.small_button("↺").clicked() {
                            self.simulation.controller.target.force.x = 0.0;
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label("Y");
                        ui.add(egui::Slider::new(
                            &mut self.simulation.controller.target.force.y,
                            -1.0..=1.0,
                        ));
                        if ui.small_button("↺").clicked() {
                            self.simulation.controller.target.force.y = 0.0;
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label("θ");
                        ui.add(egui::Slider::new(
                            &mut self.simulation.controller.target.force.z,
                            -1.0..=1.0,
                        ));
                        if ui.small_button("↺").clicked() {
                            self.simulation.controller.target.force.z = 0.0;
                        }
                    });

                    if ui.button("Ping").clicked() {
                        let mut socket =
                            UdpSocket::bind((std::net::Ipv4Addr::UNSPECIFIED, 1234)).unwrap();
                        socket.connect("192.168.137.98:1234").unwrap();
                        messages::send_sync(&mut socket, &messages::WifiMessage::Ping(67)).unwrap();
                        let response = messages::receive_sync::<WifiMessage>(&mut socket).unwrap();
                        println!("Response: {response:?}")
                    };
                });

            egui::CentralPanel::default().show_inside(ui, |ui| {
                // Draw the spacecraft
                Frame::canvas(ui.style()).show(ui, |ui| {
                    ui.ctx().request_repaint();
                    let (time, dt) = ui.input(|i| (i.time, i.stable_dt));
                    self.simulation.tick(dt as Float);
                    let desired_size =
                        ui.available_width().min(ui.available_height()) * egui::vec2(1.0, 1.0);
                    let (_id, rect) = ui.allocate_space(desired_size);

                    let visible_width = 20.0;
                    let to_screen = emath::RectTransform::from_to(
                        Rect::from_center_size(
                            egui::Pos2::ZERO,
                            egui::vec2(visible_width, visible_width),
                        )
                        .scale_from_center2(egui::vec2(1.0, -1.0)),
                        rect,
                    );

                    self.simulation.draw(ui, to_screen);
                });
            });
        });
    }
}
