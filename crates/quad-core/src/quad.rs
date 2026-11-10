//! Default layer implementations and the controller that connects them.

use crate::controls::{
    ClearanceSafety, DefaultQuadController, GoalSeekingQuadMissionPlanner, QuadController,
    QuadMissionPlanner, QuadSensors, RangeQuadSensors, SafetyConfig, SafetyLayer,
};

use crate::{FlightControls, MissionPlan, ObjectId, Vec3, WorldObject, WorldSnapshot};

/// Position, velocity, and destination supplied to the mission_planner layer.
/// Position, velocity, and goal use world axes. Attitude is body-frame tilt
/// about X and Y, with world yaw about Z; angular velocity uses body axes.
#[derive(Clone, Copy, Debug, Default)]
pub struct FlightState {
    pub position: Vec3,
    pub velocity: Vec3,
    pub goal: Vec3,
    pub attitude: Vec3,
    pub angular_velocity: Vec3,
}

/// Vehicle values needed by the mission planner.
#[derive(Clone, Copy, Debug, Default)]
pub struct MissionState {
    pub position: Vec3,
    pub velocity: Vec3,
    pub goal: Vec3,
}

impl From<FlightState> for MissionState {
    fn from(state: FlightState) -> Self {
        Self {
            position: state.position,
            velocity: state.velocity,
            goal: state.goal,
        }
    }
}

/// All data needed to calculate one controller output.
///
/// A simulation head builds this from its physics state and world, then passes
/// it to [`Quad::step`]. `delta_seconds` is the elapsed physics time.
#[derive(Clone, Debug)]
pub struct ControlFrame {
    pub observer: ObjectId,
    pub state: FlightState,
    pub world: WorldSnapshot,
    pub delta_seconds: f32,
}

/// Flight controls and feedback returned for one control frame.
///
/// Hardware sends the controls to its flight controller; simulations feed them
/// to a simulated flight controller and ESC. Other fields support telemetry.
#[derive(Clone, Debug, Default)]
pub struct ControlOutput {
    pub controls: FlightControls,
    pub observed: Vec<WorldObject>,
    pub mission_plan: MissionPlan,
}

/// Runs sensors, mission planning, quad control, and final safety override.
///
/// Sensors and safety run on every control step, including manual flight.
pub struct Quad {
    quad_sensors: RangeQuadSensors,
    mission_planner: GoalSeekingQuadMissionPlanner,
    quad_controller: DefaultQuadController,
    safety: ClearanceSafety,
    autopilot: bool,
}

impl Quad {
    /// Creates the default controller with autopilot off and safety enabled.
    pub fn new() -> Self {
        Self {
            quad_sensors: RangeQuadSensors::new(250.0),
            mission_planner: GoalSeekingQuadMissionPlanner::new(4.2),
            quad_controller: DefaultQuadController::new(2.4, 32.0),
            safety: ClearanceSafety::default(),
            autopilot: false,
        }
    }

    /// Enables or disables goal-seeking autopilot.
    pub fn set_autopilot(&mut self, enabled: bool) {
        self.autopilot = enabled;

        if !enabled {
            self.quad_controller.use_manual_controls();
            self.mission_planner.reset();
            self.quad_sensors.clear();
        }
    }

    /// Sets the four pilot channels and switches the controller to manual mode.
    pub fn set_manual_controls(&mut self, controls: FlightControls) {
        self.quad_controller.set_manual_controls(controls);
        self.set_autopilot(false);
    }

    /// Sets retained manual throttle while preserving the other stick channels.
    pub fn set_throttle(&mut self, throttle: f32) {
        self.quad_controller.set_throttle(throttle);
        self.set_autopilot(false);
    }

    /// Enables or disables the final flight-control safety override.
    pub fn set_safety_enabled(&mut self, enabled: bool) {
        self.safety.set_enabled(enabled);
    }

    /// Configures emergency limits and downstream flight-controller calibration.
    pub fn set_safety_config(&mut self, config: SafetyConfig) -> Result<(), &'static str> {
        self.safety.set_config(config)
    }

    /// Matches safety's stick interpretation to the downstream flight controller.
    pub fn set_safety_angle_mode(&mut self, enabled: bool) {
        self.safety.set_angle_mode(enabled);
    }

    /// Sets cruise speed, clamped to the supported range of 1–40 m/s.
    pub fn set_cruise_speed(&mut self, speed: f32) {
        self.mission_planner.set_cruise_speed(speed);
    }

    /// Configures the maximum autopilot stick command.
    pub fn set_tilt_input_limit(&mut self, autopilot_input: f32) {
        self.quad_controller.set_tilt_input_limit(autopilot_input);
    }

    /// Enables full-course routing for the browser simulation.
    pub fn enable_course_planning(&mut self, map_size: f32) {
        self.quad_sensors = RangeQuadSensors::visible_within(250.0);
        self.mission_planner.set_route_map_size(map_size);
    }

    /// Reports whether goal-seeking autopilot is active.
    pub fn autopilot_enabled(&self) -> bool {
        self.autopilot
    }

    /// Reports whether the safety constraint is active.
    pub fn safety_enabled(&self) -> bool {
        self.safety.enabled()
    }

    /// Returns the current safety override reason and input values for debugging.
    pub fn safety_debug(&self) -> Option<&str> {
        self.safety.debug_message()
    }

    /// Reports whether safety changed the controls on the latest step.
    pub fn safety_overriding(&self) -> bool {
        self.safety.overriding()
    }

    /// Calculates flight controls and feedback for one frame.
    ///
    /// Safety has the final say over manual and autonomous flight controls.
    pub fn step(&mut self, frame: ControlFrame) -> ControlOutput {
        self.observe(&frame);
        if self.autopilot {
            self.refresh_plan(frame.state);
            self.update_controls(frame.state, frame.delta_seconds);
        } else {
            self.quad_controller.update_manual_heading(frame.state);
        }

        self.safety.update_controls(
            frame.state,
            self.quad_controller.controls(),
            self.quad_sensors.observed(),
        );
        self.output()
    }

    /// Collects a snapshot of the results owned by the control components.
    pub fn output(&self) -> ControlOutput {
        ControlOutput {
            controls: self.safety.controls(),
            observed: self.quad_sensors.observed().to_vec(),
            mission_plan: self.mission_planner.mission_plan(),
        }
    }

    /// Returns the mission planner's latest plan, before the final safety override.
    pub fn plan(&self) -> MissionPlan {
        self.mission_planner.mission_plan()
    }

    fn observe(&mut self, frame: &ControlFrame) {
        self.quad_sensors
            .observe(&frame.world, frame.observer, frame.state.position);
    }

    fn refresh_plan(&mut self, state: FlightState) {
        self.mission_planner
            .plan(state.into(), self.quad_sensors.observed());
    }

    fn update_controls(&mut self, state: FlightState, delta_seconds: f32) {
        self.quad_controller.update_controls(
            state,
            self.mission_planner.mission_plan(),
            delta_seconds,
        );
    }
}

impl Default for Quad {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn autonomous_frame() -> ControlFrame {
        ControlFrame {
            observer: ObjectId(0),
            state: FlightState {
                goal: Vec3::new(100.0, 0.0, 0.0),
                ..Default::default()
            },
            world: WorldSnapshot::default(),
            delta_seconds: 0.1,
        }
    }

    #[test]
    fn components_retain_their_results() {
        let mut quad = Quad::new();
        quad.set_autopilot(true);
        let mut frame = autonomous_frame();
        let output = quad.step(frame.clone());
        assert!(quad.mission_planner.mission_plan().desired_momentum.x > 0.0);
        assert_eq!(quad.quad_controller.controls(), output.controls);
        assert_eq!(quad.quad_sensors.observed().len(), output.observed.len());
        frame.state.goal = Vec3::new(-100.0, 0.0, 0.0);
        quad.step(frame);
        assert!(quad.plan().desired_momentum.x < 0.0);
        quad.set_manual_controls(FlightControls::default());
        assert_eq!(quad.plan().desired_momentum, Vec3::ZERO);
        assert!(quad.quad_sensors.observed().is_empty());
    }

    #[test]
    fn disabling_autopilot_restores_the_controllers_manual_channels() {
        let mut quad = Quad::new();
        let manual = FlightControls::new(0.2, -0.3, 0.1, 0.6);
        quad.set_manual_controls(manual);
        quad.set_autopilot(true);
        quad.step(autonomous_frame());
        quad.set_throttle(0.7);
        let output = quad.step(autonomous_frame());
        assert_eq!(output.controls, FlightControls::new(0.2, -0.3, 0.1, 0.7));
        assert_eq!(output.mission_plan.desired_momentum, Vec3::ZERO);
    }

    fn collision_frame() -> ControlFrame {
        let mut frame = autonomous_frame();
        frame.state.position.z = 10.0;
        frame.state.velocity.x = 6.0;
        frame.world = WorldSnapshot::new(vec![WorldObject {
            id: ObjectId(1),
            position: Vec3::new(20.0, 0.0, 20.0),
            radius_m: 3.0,
            height_m: 40.0,
            label: "Tower".into(),
        }]);
        frame
    }

    #[test]
    fn safety_overrides_manual_controls_without_changing_pilot_input() {
        let mut quad = Quad::new();
        let pilot = FlightControls::new(0.0, 1.0, 0.1, 0.65);
        quad.set_manual_controls(pilot);
        let output = quad.step(collision_frame());
        assert!(output.controls.roll < 0.0);
        assert_eq!(quad.quad_controller.controls(), pilot);
        quad.set_safety_enabled(false);
        assert_eq!(quad.step(collision_frame()).controls, pilot);
    }

    #[test]
    fn safety_overrides_autopilot_without_changing_its_plan() {
        let mut quad = Quad::new();
        quad.set_autopilot(true);
        quad.set_cruise_speed(30.0);
        let output = quad.step(collision_frame());
        assert!(output.mission_plan.desired_momentum.x > 6.0);
        assert!(quad.quad_controller.controls().roll < 0.0);
        assert!(quad.quad_controller.controls().yaw < 0.0);
        assert!(output.controls.roll < 0.0);
        assert_eq!(
            quad.plan().desired_momentum,
            output.mission_plan.desired_momentum
        );
    }

    #[test]
    fn manual_step_observes_without_planning() {
        let mut quad = Quad::new();
        let controls = FlightControls::new(0.2, -0.3, 0.1, 0.6);
        quad.set_manual_controls(controls);
        quad.set_safety_enabled(false);
        let world = WorldSnapshot::new(vec![WorldObject {
            id: ObjectId(1),
            position: Vec3::new(1.0, 0.0, 10.0),
            radius_m: 5.0,
            height_m: 20.0,
            label: "Building".into(),
        }]);
        let output = quad.step(ControlFrame {
            observer: ObjectId(0),
            state: FlightState::default(),
            world,
            delta_seconds: 1.0 / 60.0,
        });
        assert_eq!(output.controls, controls);
        assert_eq!(output.observed.len(), 1);
        assert_eq!(output.mission_plan.desired_momentum, Vec3::ZERO);
    }
}
