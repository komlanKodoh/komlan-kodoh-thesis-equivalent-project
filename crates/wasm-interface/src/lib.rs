//! Browser boundary for the platform-independent [`quad_core`] controller.
//!
//! This crate converts JavaScript arrays into core values, advances a simulated
//! ESC, and returns forces and feedback for browser physics.

use quad_core::{
    ControlFrame, FlightControls, FlightState, ObjectId, Quad, SafetyConfig, Vec3, WorldObject,
    WorldSnapshot,
};
use quad_sim::FcEscStack;
use wasm_bindgen::prelude::*;

/// Stateful JavaScript handle for one quad controller and mission goal.
#[wasm_bindgen]
pub struct BrowserController {
    quad: Quad,
    flight_stack: FcEscStack,
    goal: Vec3,
}

#[wasm_bindgen]
impl BrowserController {
    /// Creates a controller with the default mission target and safety enabled.
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        let mut quad = Quad::new();
        quad.set_tilt_input_limit(0.9);
        quad.set_safety_config(SafetyConfig {
            vehicle_radius_m: 1.25,
            control_angle_radians: 0.7,
            max_tilt_radians: 0.7,
            ..SafetyConfig::default()
        })
        .expect("valid browser safety limits");
        let mut flight_stack = FcEscStack::default();
        flight_stack.set_angle_limit(0.7);
        quad.enable_course_planning(800.0);
        Self {
            quad,
            flight_stack,
            goal: Vec3::new(650.0, 650.0, 18.0),
        }
    }

    /// Sets the mission target in world-space meters.
    pub fn set_goal(&mut self, x: f32, y: f32, z: f32) {
        self.goal = Vec3::new(x, y, z);
    }

    /// Enables or disables goal-seeking autopilot.
    pub fn set_autopilot(&mut self, enabled: bool) {
        self.quad.set_autopilot(enabled);
    }

    pub fn set_angle_mode(&mut self, enabled: bool) {
        self.flight_stack.set_angle_mode(enabled);
        self.quad.set_safety_angle_mode(enabled);
    }

    pub fn reset_flight_stack(&mut self) {
        self.flight_stack.reset();
    }

    /// Enables or disables the core safety constraint.
    pub fn set_safety(&mut self, enabled: bool) {
        self.quad.set_safety_enabled(enabled);
    }

    /// Returns the active collision override reason and input values.
    pub fn safety_debug(&self) -> Option<String> {
        self.quad.safety_debug().map(str::to_owned)
    }

    /// Returns whether the core safety constraint is enabled.
    pub fn safety_enabled(&self) -> bool {
        self.quad.safety_enabled()
    }

    /// Returns the same held-direction pilot force used by the native UI.
    /// Sets normalized pitch, roll, yaw and throttle pilot channels.
    pub fn set_manual_controls(&mut self, pitch: f32, roll: f32, yaw: f32, throttle: f32) {
        self.quad
            .set_manual_controls(FlightControls::new(pitch, roll, yaw, throttle));
    }

    /// Sets cruise speed in meters per second; the core clamps it to 1–40.
    pub fn set_cruise_speed(&mut self, speed: f32) {
        self.quad.set_cruise_speed(speed);
    }

    /// Advances the core controller by one physics step.
    ///
    /// `state` is `[position xyz, velocity xyz]`; objects are obstacle rows.
    /// Output contains four channels, four motor RPM values, thrust, torque,
    /// target delta, and whether safety actually overrode the controls.
    pub fn compute_controls(&mut self, state: &[f32], objects: &[f32], dt: f32) -> Vec<f32> {
        let frame = browser_frame(state, objects, self.goal, dt);
        let flight_state = frame.state;
        let output = self.quad.step(frame);
        let motors = self.flight_stack.step(output.controls, flight_state, dt);
        vec![
            output.controls.pitch,
            output.controls.roll,
            output.controls.yaw,
            output.controls.throttle,
            motors.rpm[0],
            motors.rpm[1],
            motors.rpm[2],
            motors.rpm[3],
            motors.thrust,
            motors.roll_torque,
            motors.pitch_torque,
            motors.yaw_torque,
            output.mission_plan.relative_target.x,
            output.mission_plan.relative_target.y,
            output.mission_plan.relative_target.z,
            f32::from(self.quad.safety_overriding()),
        ]
    }
}

impl Default for BrowserController {
    fn default() -> Self {
        Self::new()
    }
}

fn browser_frame(state: &[f32], data: &[f32], goal: Vec3, dt: f32) -> ControlFrame {
    let position = vector_at(state, 0);
    let velocity = vector_at(state, 3);
    ControlFrame {
        observer: ObjectId(0),
        state: FlightState {
            position,
            velocity,
            goal,
            attitude: vector_at(state, 6),
            angular_velocity: vector_at(state, 9),
        },
        world: WorldSnapshot::new(parse_objects(data)),
        delta_seconds: dt,
    }
}

fn vector_at(values: &[f32], start: usize) -> Vec3 {
    Vec3::new(
        values.get(start).copied().unwrap_or_default(),
        values.get(start + 1).copied().unwrap_or_default(),
        values.get(start + 2).copied().unwrap_or_default(),
    )
}

fn parse_objects(data: &[f32]) -> Vec<WorldObject> {
    data.chunks_exact(5)
        .enumerate()
        .map(|(index, row)| WorldObject {
            id: ObjectId(index as u32 + 1),
            position: Vec3::new(row[0], row[1], row[2]),
            radius_m: row[3],
            height_m: row[4],
            label: "Obstacle".into(),
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn autopilot_commands_takeoff_from_zero_velocity_without_safety_override() {
        let mut controller = BrowserController::new();
        controller.set_autopilot(true);
        controller.set_cruise_speed(30.0);
        let state = [12.0, 15.0, 0.18, 0.0, 0.0, 0.0];
        let output = controller.compute_controls(&state, &[], 1.0 / 60.0);
        assert_eq!(output.len(), 16);
        assert_eq!(output[15], 0.0);
        assert!(output[3] > (9.81_f32 / 24.0).sqrt());
        let output = controller.compute_controls(&state, &[], 1.0);
        assert!(output[8] > 9.81);
    }

    #[test]
    fn feedback_reports_actual_override_and_clears_when_safe() {
        let mut controller = BrowserController::new();
        controller.set_manual_controls(0.0, 0.0, 0.0, 0.65);
        let state = [0.0, 0.0, 10.0, 6.0, 0.0, 0.0];
        let obstacle = [20.0, 0.0, 20.0, 3.0, 40.0];
        assert_eq!(
            controller.compute_controls(&state, &obstacle, 0.016)[15],
            1.0
        );
        assert_eq!(controller.compute_controls(&state, &[], 0.016)[15], 0.0);
        controller.set_safety(false);
        assert_eq!(
            controller.compute_controls(&state, &obstacle, 0.016)[15],
            0.0
        );
    }

    #[test]
    fn browser_simulates_motors_from_controls_and_resets_them() {
        let mut controller = BrowserController::new();
        controller.set_manual_controls(0.0, 0.0, 0.0, 0.5);
        let output = controller.compute_controls(&[0.0; 12], &[], 0.08);
        assert_eq!(output.len(), 16);
        assert_eq!(&output[..4], &[0.0, 0.0, 0.0, 0.5]);
        assert_eq!(&output[4..8], &[6000.0; 4]);
        assert_eq!(output[8], 6.0);
        assert_eq!(&output[9..12], &[0.0; 3]);
        controller.reset_flight_stack();
        let output = controller.compute_controls(&[0.0; 12], &[], 0.0);
        assert_eq!(&output[4..12], &[0.0; 8]);
    }
}
