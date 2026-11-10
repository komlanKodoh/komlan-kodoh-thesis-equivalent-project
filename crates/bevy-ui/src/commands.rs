use bevy::prelude::*;
use quad_core::FlightControls;
use std::collections::VecDeque;

#[derive(Clone, Copy, Debug)]
pub enum WorldCommand {
    Teleport(Vec3),
    ResetVehicle,
}

#[derive(Resource, Default)]
pub struct WorldCommandQueue(pub VecDeque<WorldCommand>);

#[derive(Component)]
pub struct QuadBody;

#[derive(Resource, Default)]
pub struct SimulationMode(pub bool);

#[derive(Clone, Copy, Debug)]
pub enum QuadCommand {
    SetAutopilot(bool),
    SetAngleMode(bool),
    ToggleSafety,
    SetManualControls(FlightControls),
    SetCruiseSpeed(f32),
    SetMissionGoal(Vec3),
    SetLayerVisible(LayerKind, bool),
    Reset,
}

#[derive(Clone, Copy, Debug)]
pub enum StageEditCommand {
    AddObstacle { center: Vec3, size: Vec3 },
    RemoveLastObstacle,
}

#[derive(Resource, Default)]
pub struct StageEditCommandQueue(pub VecDeque<StageEditCommand>);

#[derive(Clone, Copy, Debug)]
pub enum LayerKind {
    QuadSensors,
    QuadMissionPlanner,
    QuadController,
    Safety,
}

#[derive(Resource, Default)]
pub struct QuadCommandQueue(pub VecDeque<QuadCommand>);

#[derive(Resource, Clone, Debug)]
pub struct Telemetry {
    pub position: Vec3,
    pub velocity: Vec3,
    pub elapsed: f32,
    pub distance: f32,
    pub clearance: f32,
    pub target_delta: Vec3,
    pub running: bool,
    pub safety_enabled: bool,
}

impl Default for Telemetry {
    fn default() -> Self {
        Self {
            position: Vec3::new(12.0, 15.0, 0.18),
            velocity: Vec3::ZERO,
            elapsed: 0.0,
            distance: 0.0,
            clearance: f32::INFINITY,
            target_delta: Vec3::ZERO,
            running: false,
            safety_enabled: true,
        }
    }
}

#[derive(Resource, Debug)]
pub struct VisualSettings {
    pub quad_sensors: bool,
    pub quad_mission_planner: bool,
    pub quad_controller: bool,
    pub safety: bool,
}

impl Default for VisualSettings {
    fn default() -> Self {
        Self {
            quad_sensors: true,
            quad_mission_planner: true,
            quad_controller: true,
            safety: true,
        }
    }
}
