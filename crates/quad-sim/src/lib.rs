//! Simulated flight controller, ESCs, and motors for native and browser physics.

use quad_core::{FlightControls, FlightState};

/// Body-frame forces and torques produced by the four simulated motors.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotorWrench {
    pub thrust: f32,
    pub roll_torque: f32,
    pub pitch_torque: f32,
    pub yaw_torque: f32,
    pub rpm: [f32; 4],
}

/// Configurable four-rotor mixer and ESC/motor response model.
#[derive(Clone, Copy, Debug)]
pub struct FcEscStack {
    pub max_rpm: f32,
    pub arm_m: f32,
    pub max_thrust_n: f32,
    pub yaw_torque_n_m: f32,
    pub response_seconds: f32,
    angle_mode: bool,
    angle_limit: f32,
    rpm: [f32; 4],
}

impl Default for FcEscStack {
    fn default() -> Self {
        Self {
            max_rpm: 12000.0,
            arm_m: 0.594,
            max_thrust_n: 6.0,
            yaw_torque_n_m: 0.08,
            response_seconds: 0.08,
            angle_mode: true,
            angle_limit: 0.30,
            rpm: [0.0; 4],
        }
    }
}

impl FcEscStack {
    pub fn set_angle_mode(&mut self, enabled: bool) {
        self.angle_mode = enabled;
    }

    pub fn set_angle_limit(&mut self, radians: f32) {
        self.angle_limit = radians.clamp(0.1, 0.8);
    }

    /// Advances motor speeds and returns their combined body-frame wrench.
    /// Motor order: front-left, front-right, rear-right, rear-left.
    pub fn step(&mut self, controls: FlightControls, state: FlightState, dt: f32) -> MotorWrench {
        let mix = self.motor_mix(controls, state);
        let alpha = if self.response_seconds <= 0.0 {
            1.0
        } else {
            (dt.max(0.0) / self.response_seconds).clamp(0.0, 1.0)
        };
        for (rpm, target) in self.rpm.iter_mut().zip(mix) {
            *rpm += (target.clamp(0.0, 1.0) * self.max_rpm - *rpm) * alpha;
        }
        self.wrench()
    }

    fn motor_mix(&self, controls: FlightControls, state: FlightState) -> [f32; 4] {
        let c = FlightControls::new(
            controls.pitch,
            controls.roll,
            controls.yaw,
            controls.throttle,
        );
        let desired_pitch_rate = self.desired_rate(c.pitch, state.attitude.x);
        let desired_roll_rate = self.desired_rate(c.roll, state.attitude.y);
        let pitch_error = desired_pitch_rate - state.angular_velocity.x;
        let roll_error = desired_roll_rate - state.angular_velocity.y;
        let yaw_error = c.yaw * 0.7 - state.angular_velocity.z;
        let p = (pitch_error * 0.02).clamp(-0.06, 0.06);
        let r = (roll_error * 0.02).clamp(-0.06, 0.06);
        let y = (yaw_error * 0.025).clamp(-0.035, 0.035);
        [
            c.throttle + p + r + y,
            c.throttle + p - r - y,
            c.throttle - p - r + y,
            c.throttle - p + r - y,
        ]
    }

    fn desired_rate(&self, stick: f32, angle: f32) -> f32 {
        if self.angle_mode {
            ((stick * self.angle_limit - angle) * 3.0).clamp(-1.5, 1.5)
        } else {
            stick * 1.5
        }
    }

    /// Stops all motors, for reset and teleport operations.
    pub fn reset(&mut self) {
        self.rpm = [0.0; 4];
    }

    fn wrench(&self) -> MotorWrench {
        let thrusts = self
            .rpm
            .map(|rpm| (rpm / self.max_rpm).powi(2) * self.max_thrust_n);
        MotorWrench {
            thrust: thrusts.iter().sum(),
            roll_torque: (thrusts[0] - thrusts[1] - thrusts[2] + thrusts[3]) * self.arm_m * 0.7071,
            pitch_torque: (thrusts[0] + thrusts[1] - thrusts[2] - thrusts[3]) * self.arm_m * 0.7071,
            yaw_torque: (thrusts[0] - thrusts[1] + thrusts[2] - thrusts[3]) * self.yaw_torque_n_m,
            rpm: self.rpm,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn angle_corrections_act_on_their_own_axes() {
        let state = FlightState {
            attitude: quad_core::Vec3::new(0.1, 0.0, 0.0),
            ..FlightState::default()
        };
        let mut stack = FcEscStack::default();
        let pitch = stack.step(FlightControls::new(0.0, 0.0, 0.0, 0.7), state, 1.0);
        assert!(pitch.pitch_torque < 0.0);
        assert!(pitch.roll_torque.abs() < 1e-5);

        stack.reset();
        let state = FlightState {
            attitude: quad_core::Vec3::new(0.0, 0.1, 0.0),
            ..FlightState::default()
        };
        let roll = stack.step(FlightControls::new(0.0, 0.0, 0.0, 0.7), state, 1.0);
        assert!(roll.roll_torque < 0.0);
        assert!(roll.pitch_torque.abs() < 1e-5);
    }
}
