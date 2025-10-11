use core::ops::{Add, Sub};

#[derive(Debug, Default, Clone)]
pub struct State {
    pub px: f32,
    pub py: f32,
    pub pt: f32,
    pub vx: f32,
    pub vy: f32,
    pub vt: f32,
}

impl Add for State {
    type Output = State;

    fn add(self, rhs: Self) -> Self::Output {
        State {
            px: self.px + rhs.px,
            py: self.py + rhs.py,
            pt: self.pt + rhs.pt,
            vx: self.vx + rhs.vx,
            vy: self.vy + rhs.vy,
            vt: self.vt + rhs.vt,
        }
    }
}

impl Sub for State {
    type Output = State;

    fn sub(self, rhs: Self) -> Self::Output {
        State {
            px: self.px - rhs.px,
            py: self.py - rhs.py,
            pt: self.pt - rhs.pt,
            vx: self.vx - rhs.vx,
            vy: self.vy - rhs.vy,
            vt: self.vt - rhs.vt,
        }
    }
}
