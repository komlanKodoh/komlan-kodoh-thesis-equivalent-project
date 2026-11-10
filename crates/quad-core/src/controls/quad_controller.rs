use crate::{FlightControls, FlightState, MissionPlan, Vec3};

/// Converts movement intent into normalized pitch, roll, yaw, and throttle.
/// The four channel values are in that order and each is bounded to its range.
pub trait QuadController {
    fn update_controls(
        &mut self,
        state: FlightState,
        mission_plan: MissionPlan,
        delta_seconds: f32,
    );
    fn controls(&self) -> FlightControls;
}

/// Proportional velocity controller that converts world movement into pilot channels.
pub(crate) struct DefaultQuadController {
    controls: FlightControls,
    manual_controls: FlightControls,
    reversal_elapsed: Vec3,
    heading_aligned: bool,
    gain: f32,
    max_force: f32,
    tilt_input_limit: f32,
}

impl DefaultQuadController {
    pub(crate) fn new(gain: f32, max_force: f32) -> Self {
        Self {
            controls: FlightControls::default(),
            manual_controls: FlightControls::default(),
            reversal_elapsed: Vec3::ZERO,
            heading_aligned: false,
            gain,
            max_force,
            tilt_input_limit: 0.35,
        }
    }

    pub(crate) fn set_manual_controls(&mut self, controls: FlightControls) {
        self.manual_controls = controls;
        self.use_manual_controls();
    }

    pub(crate) fn use_manual_controls(&mut self) {
        self.controls = self.manual_controls;
        self.reversal_elapsed = Vec3::ZERO;
        self.heading_aligned = false;
    }

    pub(crate) fn set_throttle(&mut self, throttle: f32) {
        self.manual_controls.throttle = throttle.clamp(0.0, 1.0);
        self.use_manual_controls();
    }

    pub(crate) fn update_manual_heading(&mut self, state: FlightState) {
        self.controls = self.manual_controls;
        if self.manual_controls.yaw.abs() < 0.05 {
            self.controls.yaw = heading_bias(state);
        }
    }

    fn velocity_correction(&mut self, velocity: Vec3, desired: Vec3, dt: f32) -> Vec3 {
        let dt = if dt.is_finite() { dt.max(0.0) } else { 0.0 };
        let boost = Vec3::new(
            reversal_multiplier(&mut self.reversal_elapsed.x, velocity.x, desired.x, dt),
            reversal_multiplier(&mut self.reversal_elapsed.y, velocity.y, desired.y, dt),
            reversal_multiplier(&mut self.reversal_elapsed.z, velocity.z, desired.z, dt),
        );
        let error = desired - velocity;
        Vec3::new(error.x * boost.x, error.y * boost.y, error.z * boost.z) * self.gain
    }

    /// Turn before translating; hysteresis avoids switching at a single angle.
    fn aligned_velocity(&mut self, state: FlightState, desired: Vec3) -> (Vec3, f32) {
        if desired.x.hypot(desired.y) < 0.01 {
            self.heading_aligned = false;
            return (Vec3::new(0.0, 0.0, desired.z), 0.0);
        }
        let error = heading_error(state, desired);
        let resume = 12.0_f32.to_radians();
        let stop = 20.0_f32.to_radians();
        if error.abs() <= resume {
            self.heading_aligned = true;
        } else if error.abs() >= stop {
            self.heading_aligned = false;
        }
        let scale = if self.heading_aligned {
            ((stop - error.abs()) / (stop - resume)).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let velocity = Vec3::new(desired.x * scale, desired.y * scale, desired.z);
        (velocity, yaw_control(state, error))
    }

    pub(crate) fn set_tilt_input_limit(&mut self, limit: f32) {
        self.tilt_input_limit = limit.clamp(0.0, 1.0);
    }
}

impl QuadController for DefaultQuadController {
    fn controls(&self) -> FlightControls {
        self.controls
    }

    fn update_controls(
        &mut self,
        state: FlightState,
        mission_plan: MissionPlan,
        delta_seconds: f32,
    ) {
        let (desired, yaw) = self.aligned_velocity(state, mission_plan.desired_momentum);
        let acceleration = self
            .velocity_correction(state.velocity, desired, delta_seconds)
            .clamp_length_max(self.max_force)
            .rotate_z(-state.attitude.z);

        self.controls = FlightControls::new(
            (-acceleration.y / 9.81).clamp(-self.tilt_input_limit, self.tilt_input_limit),
            (acceleration.x / 9.81).clamp(-self.tilt_input_limit, self.tilt_input_limit),
            yaw,
            ((acceleration.z + 9.81).clamp(0.0, 24.0) / 24.0).sqrt(),
        );
    }
}

/// Body +Y is the nose. Avoid chasing an undefined heading while hovering.
fn heading_bias(state: FlightState) -> f32 {
    if state.velocity.x.hypot(state.velocity.y) < 0.5 {
        return 0.0;
    }
    yaw_control(state, heading_error(state, state.velocity))
}

fn heading_error(state: FlightState, direction: Vec3) -> f32 {
    let error = (-direction.x).atan2(direction.y) - state.attitude.z;
    error.sin().atan2(error.cos())
}

fn yaw_control(state: FlightState, error: f32) -> f32 {
    (error * 1.5 - state.angular_velocity.z * 0.2).clamp(-0.6, 0.6)
}

/// Add one unit of gain per opposing second, up to four times the normal gain.
fn reversal_multiplier(elapsed: &mut f32, velocity: f32, desired: f32, dt: f32) -> f32 {
    if velocity * desired < -0.0001 {
        *elapsed = (*elapsed + dt).min(3.0);
    } else {
        *elapsed = 0.0;
    }
    1.0 + *elapsed
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nose_bias_faces_motion_and_is_quiet_at_rest() {
        let mut state = FlightState::default();
        assert_eq!(heading_bias(state), 0.0);
        state.velocity = Vec3::new(2.0, 0.0, 0.0);
        assert!(heading_bias(state) < 0.0);
        state.attitude.z = -std::f32::consts::FRAC_PI_2;
        assert!(heading_bias(state).abs() < 1e-5);
        state.attitude.z = std::f32::consts::PI - 0.05;
        state.velocity = Vec3::new(0.1, -2.0, 0.0);
        assert!(heading_bias(state).abs() < 0.2);
    }

    #[test]
    fn world_direction_is_preserved_when_the_nose_turns() {
        let mut controller = DefaultQuadController::new(2.4, 32.0);
        let mut state = FlightState::default();
        state.attitude.z = -std::f32::consts::FRAC_PI_2;
        let plan = MissionPlan {
            desired_momentum: Vec3::new(1.0, 0.0, 0.0),
            ..Default::default()
        };
        controller.update_controls(state, plan, 0.016);
        assert!(controller.controls().pitch < 0.0);
        assert!(controller.controls().roll.abs() < 1e-5);
    }

    #[test]
    fn manual_yaw_takes_priority_over_heading_bias() {
        let mut controller = DefaultQuadController::new(2.4, 32.0);
        let state = FlightState {
            velocity: Vec3::new(2.0, 0.0, 0.0),
            ..Default::default()
        };
        controller.update_manual_heading(state);
        assert!(controller.controls().yaw < 0.0);
        controller.set_manual_controls(FlightControls::new(0.0, 0.0, 0.3, 0.6));
        controller.update_manual_heading(state);
        assert_eq!(controller.controls().yaw, 0.3);
    }

    #[test]
    fn opposing_motion_progressively_increases_correction_and_resets() {
        let mut controller = DefaultQuadController::new(2.4, 32.0);
        let state = FlightState {
            velocity: Vec3::new(-0.1, 0.0, 0.0),
            attitude: Vec3::new(0.0, 0.0, -std::f32::consts::FRAC_PI_2),
            ..Default::default()
        };
        let plan = MissionPlan {
            desired_momentum: Vec3::new(0.1, 0.0, 0.0),
            ..Default::default()
        };
        controller.update_controls(state, plan, 0.0);
        let normal = -controller.controls().pitch;
        controller.update_controls(state, plan, 1.0);
        let first = -controller.controls().pitch;
        controller.update_controls(state, plan, 1.0);
        assert!(first > normal);
        assert!(-controller.controls().pitch > first);
        controller.update_controls(FlightState { velocity: Vec3::ZERO, ..state }, plan, 0.1);
        assert_eq!(controller.reversal_elapsed, Vec3::ZERO);
        controller.update_controls(state, plan, 0.0);
        assert_eq!(-controller.controls().pitch, normal);
    }

    #[test]
    fn ramp_is_time_based_and_only_boosts_opposing_axes() {
        let mut slow = DefaultQuadController::new(2.4, 32.0);
        let mut fast = DefaultQuadController::new(2.4, 32.0);
        let velocity = Vec3::new(-0.1, 0.1, 0.0);
        let desired = Vec3::new(0.1, 0.1, 0.0);
        for _ in 0..10 {
            slow.velocity_correction(velocity, desired, 0.1);
        }
        for _ in 0..100 {
            fast.velocity_correction(velocity, desired, 0.01);
        }
        assert!((slow.reversal_elapsed.x - fast.reversal_elapsed.x).abs() < 1e-5);
        assert_eq!(slow.reversal_elapsed.y, 0.0);
        slow.velocity_correction(velocity, desired, 100.0);
        assert_eq!(slow.reversal_elapsed.x, 3.0);
        slow.use_manual_controls();
        assert_eq!(slow.reversal_elapsed, Vec3::ZERO);
    }
}
