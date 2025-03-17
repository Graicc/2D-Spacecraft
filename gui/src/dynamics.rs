use crate::{to_ui_pos, to_ui_vec, Float, Vec2};

use eframe::{
    egui::{self, Color32, Frame, Rect, Stroke},
    emath::{self, RectTransform},
};
use nalgebra::{Rotation2, Vector2};

#[derive(Debug, Default)]
pub struct State {
    pub position: Vec2,   // m
    pub theta: Float,     // radians
    pub velocity: Vec2,   // m/s
    pub theta_dot: Float, // radians/s
}

impl State {
    fn forwards(&self) -> Vec2 {
        Vec2::new(self.theta.cos(), self.theta.sin())
    }

    fn up(&self) -> Vec2 {
        Vec2::new(-self.theta.sin(), self.theta.cos())
    }

    fn local_position_to_world(&self, location: Vec2) -> Vec2 {
        let rot = Rotation2::new(self.theta);
        (rot * location) + self.position
    }

    fn local_vec_to_world(&self, vec: Vec2) -> Vec2 {
        let rot = Rotation2::new(self.theta);
        rot * vec
    }
}

#[derive(Debug)]
pub struct OffsetDir {
    pub x: Float,
    pub y: Float,
    pub dir_x: Float,
    pub dir_y: Float,
}

impl OffsetDir {
    fn new(x: Float, y: Float, dir_x: Float, dir_y: Float) -> Self {
        Self { x, y, dir_x, dir_y }
    }
}

const RCS_COUNT: usize = 6;

#[derive(Debug)]
pub struct Properties {
    pub mass: Float,               // kg
    pub moment_of_inertia: Float,  // kg m^2
    pub friction: Float,           // mu
    pub rotational_damping: Float, // idk
    pub gravity: Float,            // m/s^2
    pub local_xyt_to_rcs: nalgebra::Matrix<
        Float,
        nalgebra::Const<RCS_COUNT>,
        nalgebra::Const<3>,
        nalgebra::ArrayStorage<Float, RCS_COUNT, 3>,
    >,
    pub rcs_locations: [OffsetDir; RCS_COUNT],
}

impl Default for Properties {
    fn default() -> Self {
        Properties {
            mass: 1.0,
            moment_of_inertia: 1.0,
            friction: 0.01,
            rotational_damping: 0.0,
            gravity: 9.8,
            #[rustfmt::skip]
            local_xyt_to_rcs: nalgebra::Matrix::<Float, nalgebra::Const<RCS_COUNT>, nalgebra::Const<3>, _>::from_row_slice(&[
                0.0, -0.5, 1.0,
				0.0, -0.5, -1.0,
				-1.0, 0.0, 0.0,
				0.0, 0.5, 1.0,
				0.0, 0.5, -1.0,
                1.0, 0.0, 0.0,
			]),
            rcs_locations: [
                OffsetDir::new(-1.0, 0.1, 0.0, 1.0),
                OffsetDir::new(1.0, 0.1, 0.0, 1.0),
                OffsetDir::new(1.0, 0.0, 1.0, 0.0),
                OffsetDir::new(1.0, -0.1, 0.0, -1.0),
                OffsetDir::new(-1.0, -0.1, 0.0, -1.0),
                OffsetDir::new(-1.0, 0.0, -1.0, 0.0),
            ],
        }
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct Target {
    /// Target force in global xyt
    pub force: nalgebra::Vector3<Float>,
}

#[derive(Debug, Default)]
pub struct Controller {
    /// Target state to reach
    pub target: Target,
    /// Last response (only used for ui)
    last_value: Response,
}

#[derive(Debug, Default, Clone, Copy)]
struct Response {
    rcs: nalgebra::SVector<Float, RCS_COUNT>,
}

impl Controller {
    fn new() -> Self {
        Self::default()
    }

    fn set_target(&mut self, target: Target) {
        self.target = target;
    }

    fn tick(&mut self, state: &State, properties: &Properties) -> Response {
        let force_local_xyt = nalgebra::Matrix3::new_rotation(-state.theta) * self.target.force;

        let rcs = properties.local_xyt_to_rcs * force_local_xyt;
        let rcs = (rcs + rcs.abs()) / 2.0;

        let response = Response { rcs };
        self.last_value = response;
        response
    }
}

fn apply_friction(velocity: &Vec2, friction: Float, dt: Float) -> Vec2 {
    let v_norm = match velocity.try_normalize(0.0) {
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
    fn apply_local_impulse(&mut self, location: Vec2, impulse: Vec2) {
        self.velocity += self.local_vec_to_world(impulse);

        let angular_impulse = nalgebra::Matrix2::from_columns(&[location, impulse]).determinant();
        self.theta_dot += angular_impulse;
    }

    fn tick(&mut self, dt: Float, response: &Response, properties: &Properties) {
        // Euler integration
        self.position += self.velocity * dt;
        self.theta += self.theta_dot * dt;

        self.velocity = apply_friction(&self.velocity, properties.friction, dt);
        self.theta_dot *= 1.0 - properties.rotational_damping;

        // Apply RCS
        let rcs = response.rcs;
        for (i, &v) in rcs.iter().enumerate() {
            let offset = &properties.rcs_locations[i];

            let impulse = v * dt / properties.mass;

            self.apply_local_impulse(
                Vec2::new(offset.x, offset.y),
                -Vec2::new(offset.dir_x, offset.dir_y) * impulse,
            );
        }
    }
}

#[derive(Debug, Default)]
pub struct Simulation {
    pub state: State,
    pub controller: Controller,
    pub properties: Properties,
}

impl Simulation {
    pub fn tick(&mut self, dt: Float) {
        let response = self.controller.tick(&self.state, &self.properties);
        self.state.tick(dt, &response, &self.properties);
    }

    pub fn draw(&self, ui: &mut egui::Ui, to_screen: RectTransform) {
        let radius: f32 = 0.5;
        let position = to_ui_pos(self.state.position);
        let velocity = to_ui_vec(self.state.velocity);
        let front = to_ui_pos(self.state.position + self.state.up() * radius as Float);

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
            Stroke::new(2.0, Color32::GREEN),
        );

        let rcs = self.controller.last_value.rcs;

        for (i, &v) in rcs.iter().enumerate() {
            let offset = &self.properties.rcs_locations[i];

            let local_dir = Vec2::new(v * offset.dir_x, v * offset.dir_y);
            let world_start = self
                .state
                .local_position_to_world(Vec2::new(offset.x, offset.y));

            let world_dir = self.state.local_vec_to_world(local_dir);

            ui.painter().arrow(
                to_screen * to_ui_pos(world_start),
                to_screen.scale().x * to_ui_vec(world_dir),
                Stroke::new(2.0, Color32::RED),
            );
        }
    }
}
