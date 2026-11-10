//! Final safety override: check the projected path and brake against velocity.

use crate::{FlightControls, FlightState, Vec3, WorldObject};

/// Braking limits and downstream flight-controller calibration, in SI units.
#[derive(Clone, Copy, Debug)]
pub struct SafetyConfig {
    pub lookahead_seconds: f32,
    /// Horizontal radius covering the body and rotor guards.
    pub vehicle_radius_m: f32,
    pub braking_acceleration: f32,
    pub braking_ramp_distance_m: f32,
    pub full_braking_distance_m: f32,
    pub max_tilt_radians: f32,
    pub control_angle_radians: f32,
    /// Full-throttle thrust divided by vehicle mass, in m/s².
    pub max_thrust_acceleration: f32,
}

impl Default for SafetyConfig {
    fn default() -> Self {
        Self {
            lookahead_seconds: 5.0,
            vehicle_radius_m: 0.9,
            braking_acceleration: 2.5,
            braking_ramp_distance_m: 12.0,
            full_braking_distance_m: 1.5,
            max_tilt_radians: 0.3,
            control_angle_radians: 0.3,
            max_thrust_acceleration: 24.0,
        }
    }
}

impl SafetyConfig {
    fn valid(self) -> bool {
        [
            self.lookahead_seconds,
            self.vehicle_radius_m,
            self.braking_acceleration,
            self.braking_ramp_distance_m,
            self.full_braking_distance_m,
            self.max_tilt_radians,
            self.control_angle_radians,
            self.max_thrust_acceleration,
        ]
        .iter()
        .all(|value| value.is_finite() && *value > 0.0)
            && self.full_braking_distance_m < self.braking_ramp_distance_m
            && self.max_tilt_radians <= self.control_angle_radians
            && self.control_angle_radians <= 0.8
            && self.max_thrust_acceleration > 9.81
    }
}

/// Owns final flight controls after the collision check.
pub trait SafetyLayer {
    fn update_controls(
        &mut self,
        state: FlightState,
        requested: FlightControls,
        objects: &[WorldObject],
    );
    fn controls(&self) -> FlightControls;
}

pub(crate) struct ClearanceSafety {
    config: SafetyConfig,
    controls: FlightControls,
    safety_override: bool,
    debug_message: Option<String>,
    enabled: bool,
    angle_mode: bool,
}

impl Default for ClearanceSafety {
    fn default() -> Self {
        Self {
            config: SafetyConfig::default(),
            controls: FlightControls::default(),
            safety_override: false,
            debug_message: None,
            enabled: true,
            angle_mode: true,
        }
    }
}

impl ClearanceSafety {
    pub(crate) fn set_config(&mut self, config: SafetyConfig) -> Result<(), &'static str> {
        if !config.valid() {
            return Err("invalid safety limits or flight-controller calibration");
        }
        self.config = config;
        Ok(())
    }

    pub(crate) fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }
    pub(crate) fn enabled(&self) -> bool {
        self.enabled
    }
    pub(crate) fn debug_message(&self) -> Option<&str> {
        self.debug_message.as_deref()
    }

    pub(crate) fn overriding(&self) -> bool {
        self.safety_override
    }
    pub(crate) fn set_angle_mode(&mut self, enabled: bool) {
        self.angle_mode = enabled;
    }

    fn override_message(
        &self,
        state: FlightState,
        destination: Vec3,
        object: &WorldObject,
        requested: FlightControls,
    ) -> String {
        format!(
            "SAFETY OVERRIDE: {:.1}-second path (vehicle radius {:.2} m) intersects obstacle {:?} ({})\nClearance: {:.2} m, emergency strength: {:.0}%\nPosition: {:?}\nVelocity: {:?}\nProjected position: {:?}\nObstacle: {:?}, radius {:.2} m, height {:.2} m\nRequested: {:?}\nBraking: {:?}",
            self.config.lookahead_seconds,
            self.config.vehicle_radius_m,
            object.id,
            object.label,
            self.clearance(state.position, object),
            self.urgency(self.clearance(state.position, object)) * 100.0,
            state.position,
            state.velocity,
            destination,
            object.position,
            object.radius_m,
            object.height_m,
            requested,
            self.controls
        )
    }

    fn braking_acceleration(&self, velocity: Vec3) -> Vec3 {
        // Limit magnitude without changing the direction opposite to velocity.
        let limit = self
            .config
            .braking_acceleration
            .min(9.81 * self.config.max_tilt_radians.sin())
            .min(self.config.max_thrust_acceleration - 9.81);
        (velocity * -2.4).clamp_length_max(limit)
    }

    fn urgency(&self, clearance: f32) -> f32 {
        ((self.config.braking_ramp_distance_m - clearance)
            / (self.config.braking_ramp_distance_m - self.config.full_braking_distance_m))
            .clamp(0.0, 1.0)
    }

    fn clearance(&self, position: Vec3, object: &WorldObject) -> f32 {
        let radial = (position.x - object.position.x).hypot(position.y - object.position.y)
            - object.radius_m;
        let vertical = (-position.z).max(position.z - object.height_m).max(0.0);
        radial.max(0.0).hypot(vertical) - self.config.vehicle_radius_m
    }

    fn braking_thrust(&self, velocity: Vec3, urgency: f32) -> Vec3 {
        let acceleration = self.braking_acceleration(velocity);
        let normal = Vec3::new(acceleration.x, acceleration.y, acceleration.z + 9.81);
        let horizontal = Vec3::new(-velocity.x, -velocity.y, 0.0);
        let emergency = if horizontal.length() > 0.1 {
            let tilt = self.config.max_tilt_radians;
            (horizontal.normalize_or_zero() * tilt.sin() + Vec3::new(0.0, 0.0, tilt.cos()))
                * self.config.max_thrust_acceleration
        } else if velocity.z < 0.0 {
            Vec3::new(0.0, 0.0, self.config.max_thrust_acceleration)
        } else {
            Vec3::ZERO
        };
        normal * (1.0 - urgency) + emergency * urgency
    }

    fn braking_throttle(&self, state: FlightState, magnitude: f32) -> f32 {
        let pitch = state.attitude.x;
        let roll = state.attitude.y;
        let current_direction =
            Vec3::new(roll.sin() * pitch.cos(), -pitch.sin(), 0.0).rotate_z(state.attitude.z);
        // Do not boost existing tilt into the obstacle while the flight controller turns.
        let magnitude = if current_direction.dot(state.velocity) > 0.05 {
            magnitude.min(self.braking_thrust(state.velocity, 0.0).length())
        } else {
            magnitude
        };
        (magnitude / self.config.max_thrust_acceleration).sqrt()
    }

    fn braking_controls(&self, state: FlightState, urgency: f32) -> FlightControls {
        let thrust = self
            .braking_thrust(state.velocity, urgency)
            .rotate_z(-state.attitude.z);
        let magnitude = thrust.length();
        let pitch = if magnitude > 0.0 {
            (-thrust.y / magnitude).asin()
        } else {
            0.0
        };
        let roll = thrust.x.atan2(thrust.z);
        let throttle = self.braking_throttle(state, magnitude);
        let (pitch, roll) = if self.angle_mode {
            (
                pitch / self.config.control_angle_radians,
                roll / self.config.control_angle_radians,
            )
        } else {
            (
                (pitch - state.attitude.x) * 3.0 / 1.5,
                (roll - state.attitude.y) * 3.0 / 1.5,
            )
        };
        FlightControls::new(pitch, roll, 0.0, throttle)
    }
}

impl SafetyLayer for ClearanceSafety {
    fn controls(&self) -> FlightControls {
        self.controls
    }

    fn update_controls(
        &mut self,
        state: FlightState,
        requested: FlightControls,
        objects: &[WorldObject],
    ) {
        self.controls = requested;
        self.debug_message = None;
        let destination = state.position + state.velocity * self.config.lookahead_seconds;
        let obstacle = objects
            .iter()
            .filter(|object| {
                segment_intersects(
                    state.position,
                    destination,
                    object,
                    self.config.vehicle_radius_m,
                )
            })
            .min_by(|a, b| {
                self.clearance(state.position, a)
                    .total_cmp(&self.clearance(state.position, b))
            });
        if self.enabled && state.velocity.length() > 0.0 {
            if let Some(object) = obstacle {
                self.controls = self
                    .braking_controls(state, self.urgency(self.clearance(state.position, object)));

                if self.controls != requested {
                    self.debug_message =
                        Some(self.override_message(state, destination, object, requested));
                }
            }
        }
        self.safety_override = self.controls != requested;
    }
}

/// Clip the line to the obstacle's height, then check its closest point to the axis.
fn segment_intersects(start: Vec3, end: Vec3, object: &WorldObject, padding: f32) -> bool {
    let radius = object.radius_m + padding;
    let delta = end - start;
    let (low, high) = if delta.z.abs() <= 1e-9 {
        if start.z < 0.0 || start.z > object.height_m {
            return false;
        }
        (0.0, 1.0)
    } else {
        let bottom = -start.z / delta.z;
        let top = (object.height_m - start.z) / delta.z;
        (bottom.min(top).max(0.0), bottom.max(top).min(1.0))
    };
    if low > high {
        return false;
    }
    let offset = Vec3::new(
        start.x - object.position.x,
        start.y - object.position.y,
        0.0,
    );
    let movement = Vec3::new(delta.x, delta.y, 0.0);
    if escaping_contact(start, delta, offset, movement, object, radius) {
        return false;
    }
    let t = (-offset.dot(movement) / movement.dot(movement).max(1e-9)).clamp(low, high);
    (offset + movement * t).length() <= radius
}

// Starting inside a conservative obstacle bound must still allow outward motion.
fn escaping_contact(
    start: Vec3,
    delta: Vec3,
    offset: Vec3,
    movement: Vec3,
    object: &WorldObject,
    radius: f32,
) -> bool {
    start.z >= 0.0
        && start.z <= object.height_m
        && offset.length() <= radius
        && offset.dot(movement) >= 0.0
        && (movement.length() > 1e-6 || delta.z > 0.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ObjectId;

    fn tower(x: f32, y: f32) -> WorldObject {
        WorldObject {
            id: ObjectId(1),
            position: Vec3::new(x, y, 20.0),
            radius_m: 3.0,
            height_m: 40.0,
            label: "Tower".into(),
        }
    }

    fn state(velocity: Vec3) -> FlightState {
        FlightState {
            position: Vec3::new(0.0, 0.0, 10.0),
            velocity,
            ..FlightState::default()
        }
    }

    #[test]
    fn checks_the_configured_lookahead_and_vehicle_radius() {
        let mut safety = ClearanceSafety::default();
        let state = state(Vec3::new(6.0, 0.0, 0.0));
        let requested = FlightControls::default();
        safety.update_controls(state, requested, &[tower(34.0, 0.0)]);
        assert_eq!(safety.controls(), requested);
        safety.update_controls(state, requested, &[tower(33.8, 0.0)]);
        assert!(safety.overriding());
        assert!(safety.controls().roll < 0.0);
        safety.update_controls(state, requested, &[tower(10.0, 4.0), tower(-10.0, 0.0)]);
        assert!(!safety.overriding());
    }

    #[test]
    fn braking_uses_body_axes_after_the_drone_yaws() {
        let safety = ClearanceSafety::default();
        let mut state = state(Vec3::new(6.0, 0.0, 0.0));
        state.attitude.z = std::f32::consts::FRAC_PI_2;
        let controls = safety.braking_controls(state, 0.0);
        assert!(controls.pitch < 0.0);
        assert!(controls.roll.abs() < 1e-5);
    }

    #[test]
    fn brakes_in_the_opposite_direction_without_steering_or_climbing() {
        let safety = ClearanceSafety::default();
        for velocity in [Vec3::new(8.0, 4.0, 2.0), Vec3::new(0.0, 0.0, -5.0)] {
            let acceleration = safety.braking_acceleration(velocity);
            assert!(acceleration.dot(velocity) < 0.0);
            assert!(
                (acceleration.normalize_or_zero() + velocity.normalize_or_zero()).length() < 1e-5
            );
            assert!(acceleration.length() <= safety.config.braking_acceleration + 1e-5);
        }
    }

    #[test]
    fn intersects_vertical_paths_and_ignores_paths_above_the_roof() {
        let object = tower(0.0, 0.0);
        assert!(segment_intersects(
            Vec3::new(0.0, 0.0, 50.0),
            Vec3::new(0.0, 0.0, 35.0),
            &object,
            0.0
        ));
        assert!(!segment_intersects(
            Vec3::new(0.0, 0.0, 50.0),
            Vec3::new(0.0, 0.0, 45.0),
            &object,
            0.0
        ));
        assert!(!segment_intersects(
            Vec3::new(-10.0, 0.0, 50.0),
            Vec3::new(10.0, 0.0, 50.0),
            &object,
            0.0
        ));
        assert!(segment_intersects(
            Vec3::new(-10.0, 0.0, 10.0),
            Vec3::new(10.0, 0.0, 10.0),
            &object,
            0.0
        ));
    }

    #[test]
    fn contact_allows_escape_but_still_brakes_motion_further_inside() {
        let mut safety = ClearanceSafety::default();
        let requested = FlightControls::new(0.0, -0.3, 0.0, 0.65);
        let objects = [tower(2.0, 0.0)];
        safety.update_controls(state(Vec3::new(1.0, 0.0, 0.0)), requested, &objects);
        assert!(safety.overriding());
        safety.update_controls(state(Vec3::new(-1.0, 0.0, 0.0)), requested, &objects);
        assert!(!safety.overriding());
        assert_eq!(safety.controls(), requested);
    }

    #[test]
    fn braking_ramps_with_proximity_and_reaches_full_throttle() {
        let mut safety = ClearanceSafety::default();
        let state = state(Vec3::new(6.0, 0.0, 0.0));
        let mut previous = 0.0;
        for distance in [20.0, 12.0, 8.0, 5.0] {
            safety.update_controls(state, FlightControls::default(), &[tower(distance, 0.0)]);
            assert!(safety.controls().throttle > previous);
            assert!(safety.controls().roll < 0.0);
            previous = safety.controls().throttle;
        }
        assert!((previous - 1.0).abs() < 1e-5);
    }

    #[test]
    fn most_urgent_obstacle_wins_regardless_of_object_order() {
        let mut safety = ClearanceSafety::default();
        let state = state(Vec3::new(6.0, 0.0, 0.0));
        safety.update_controls(
            state,
            FlightControls::default(),
            &[tower(25.0, 0.0), tower(5.0, 0.0)],
        );
        let first = safety.controls();
        safety.update_controls(
            state,
            FlightControls::default(),
            &[tower(5.0, 0.0), tower(25.0, 0.0)],
        );
        assert_eq!(first, safety.controls());
        assert!((first.throttle - 1.0).abs() < 1e-5);
    }

    #[test]
    fn waits_for_tilt_to_reverse_before_boosting_throttle() {
        let safety = ClearanceSafety::default();
        let mut state = state(Vec3::new(6.0, 0.0, 0.0));
        state.attitude.y = 0.3;
        assert!(safety.braking_controls(state, 1.0).throttle < 0.8);
        state.attitude.y = -0.3;
        assert!((safety.braking_controls(state, 1.0).throttle - 1.0).abs() < 1e-5);
    }

    #[test]
    fn emergency_does_not_add_upward_thrust_to_brake_pure_ascent() {
        let safety = ClearanceSafety::default();
        let thrust = safety.braking_thrust(Vec3::new(0.0, 0.0, 4.0), 1.0);
        assert_eq!(thrust, Vec3::ZERO);
        let thrust = safety.braking_thrust(Vec3::new(0.0, 0.0, -4.0), 1.0);
        assert_eq!(thrust.z, safety.config.max_thrust_acceleration);
    }

    #[test]
    fn preserves_controls_at_rest_and_when_disabled() {
        let mut safety = ClearanceSafety::default();
        let requested = FlightControls::new(0.0, 1.0, 0.0, 0.8);
        safety.update_controls(state(Vec3::ZERO), requested, &[tower(10.0, 0.0)]);
        assert_eq!(safety.controls(), requested);
        safety.set_enabled(false);
        safety.update_controls(
            state(Vec3::new(6.0, 0.0, 0.0)),
            requested,
            &[tower(10.0, 0.0)],
        );
        assert_eq!(safety.controls(), requested);
        assert!(!safety.overriding());
    }
}
