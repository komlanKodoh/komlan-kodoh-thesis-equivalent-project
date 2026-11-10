//! Portable quad state and control layers shared by the native and browser apps.

pub mod controls;
mod quad;
mod world;

pub use controls::{MissionPlan, SafetyConfig};
pub use quad::*;
pub use world::{Meters, ObjectId, WorldObject, WorldSnapshot};

/// Magnitude of a held pilot direction's force, in newtons.
pub const PILOT_FORCE: f32 = 18.0;

/// Normalized pilot or autopilot input to a quadcopter flight controller.
/// Pitch, roll, and yaw are signed; throttle ranges from zero to one.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FlightControls {
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
    pub throttle: f32,
}

impl FlightControls {
    pub fn new(pitch: f32, roll: f32, yaw: f32, throttle: f32) -> Self {
        Self {
            pitch: pitch.clamp(-1.0, 1.0),
            roll: roll.clamp(-1.0, 1.0),
            yaw: yaw.clamp(-1.0, 1.0),
            throttle: throttle.clamp(0.0, 1.0),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);

    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Rotates a vector about the world vertical axis.
    pub fn rotate_z(self, angle: f32) -> Self {
        let (sin, cos) = angle.sin_cos();
        Self::new(
            cos * self.x - sin * self.y,
            sin * self.x + cos * self.y,
            self.z,
        )
    }

    pub fn length(self) -> f32 {
        self.dot(self).sqrt()
    }

    pub fn distance(self, other: Self) -> f32 {
        (self - other).length()
    }

    pub fn normalize_or_zero(self) -> Self {
        let length = self.length();
        if length > f32::EPSILON {
            self / length
        } else {
            Self::ZERO
        }
    }

    pub fn clamp_length_max(self, maximum: f32) -> Self {
        let length = self.length();
        if length > maximum {
            self * (maximum / length)
        } else {
            self
        }
    }

    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }
}

impl std::ops::Add for Vec3 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl std::ops::Sub for Vec3 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl std::ops::Mul<f32> for Vec3 {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl std::ops::Div<f32> for Vec3 {
    type Output = Self;
    fn div(self, rhs: f32) -> Self {
        self * rhs.recip()
    }
}

impl std::iter::Sum for Vec3 {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(Self::ZERO, |sum, item| sum + item)
    }
}
