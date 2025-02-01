#![allow(dead_code, unused_variables)]
#![cfg_attr(not(debug_assertions), windows_subsystem= "windows")]

use eframe::{
    egui::{self, Color32, Frame, Rect, Stroke},
    emath,
};
use nalgebra::Vector2;

type Float = f64;
type Vec2 = Vector2<Float>;

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
                velocity: Vec2::new(1.0, 1.0),
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

            // Impulse buttons
            {
                let kick = 0.1;
                ui.horizontal(|ui| {
                    if ui.small_button("↺").clicked() {
                        self.simulation.state.theta_dot -= kick;
                    }
                    if ui.small_button("U").clicked() {
                        self.simulation.state.velocity.y += kick;
                    }
                    if ui.small_button("↻").clicked() {
                        self.simulation.state.theta_dot += kick;
                    }
                });
                ui.horizontal(|ui| {
                    if ui.small_button("L").clicked() {
                        self.simulation.state.velocity.x -= kick;
                    }
                    ui.add_space(30.0);
                    if ui.small_button("R").clicked() {
                        self.simulation.state.velocity.x += kick;
                    }
                });
                ui.horizontal(|ui| {
                    ui.add_space(25.0);
                    if ui.small_button("D").clicked() {
                        self.simulation.state.velocity.y -= kick;
                    }
                });
            }

            // Draw the spacecraft
            Frame::canvas(ui.style()).show(ui, |ui| {
                ui.ctx().request_repaint();
                let (time, dt) = ui.input(|i| (i.time, i.stable_dt));
                self.simulation.tick(dt as Float);
                let desired_size = ui.available_width() * egui::vec2(1.0, 1.0);
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

                let radius: f32 = 0.5;
                let position = to_ui_pos(self.simulation.state.position);
                let velocity = to_ui_vec(self.simulation.state.velocity);
                let front = {
                    let state = &self.simulation.state;
                    to_ui_pos(state.position + state.fowards() * radius as Float)
                };

                ui.painter().circle(
                    to_screen * position,
                    to_screen.scale().x * radius,
                    Color32::TRANSPARENT,
                    Stroke::new(3.0, Color32::WHITE),
                );
                ui.painter().line_segment(
                    [to_screen * position, to_screen * front],
                    Stroke::new(3.0, Color32::WHITE),
                );
                ui.painter().arrow(
                    to_screen * position,
                    velocity * to_screen.scale().x,
                    Stroke::new(1.0, Color32::RED),
                );
            });
        });
    }
}

#[derive(Debug, Default)]
struct State {
    position: Vec2,   // m
    theta: Float,     // radians
    velocity: Vec2,   // m/s
    theta_dot: Float, // raidans/s
}

impl State {
    fn fowards(&self) -> Vec2 {
        Vec2::new(self.theta.sin(), self.theta.cos())
    }
}

fn apply_friction(velocity: &Vec2, friction: Float, dt: Float) -> Vec2 {
    let v_norm = velocity.try_normalize(0.0);
    let v_norm = match v_norm {
        Some(v) => v,
        None => return *velocity,
    };
    let v_friction = v_norm * friction;

    let result = velocity - dt * v_friction;
    if velocity.dot(&result) < 0.0 {
        Vec2::zeros()
    } else {
        result
    }
}

impl State {
    fn tick(&mut self, dt: Float, properties: &Properties) {
        self.position += self.velocity * dt;
        self.theta += self.theta_dot * dt;

        self.velocity = apply_friction(&self.velocity, properties.friction, dt);
        self.theta_dot *= 1.0 - properties.rotational_damping;
    }
}

#[derive(Debug)]
struct Properties {
    mass: Float,               // kg
    moment_of_inertia: Float,  // kg m^2
    friction: Float,           // mu
    rotational_damping: Float, // idk
    gravity: Float,            // m/s^2
}

impl Default for Properties {
    fn default() -> Self {
        Properties {
            mass: 1.0,
            moment_of_inertia: 1.0,
            friction: 0.01,
            rotational_damping: 0.01,
            gravity: 9.8,
        }
    }
}

#[derive(Debug, Default)]
struct Simulation {
    state: State,
    properties: Properties,
}

impl Simulation {
    fn tick(&mut self, dt: f64) {
        self.state.tick(dt, &self.properties);
    }
}
