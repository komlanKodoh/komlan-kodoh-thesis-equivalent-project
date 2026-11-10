//! Contracts for the stages of the quad control pipeline.
//!
//! Each layer receives plain data and retains its own results. The layers do not access
//! physics engines, rendering, or platform APIs. [`Quad`](crate::Quad)
//! connects sensors, mission planning, quad control, and final safety override.
//!
//!

mod quad_controller;
mod quad_mission_planner;
mod quad_sensors;
mod route_planner;
mod safety;

pub use quad_controller::QuadController;
pub use quad_mission_planner::{MissionPlan, QuadMissionPlanner};
pub use quad_sensors::QuadSensors;
pub use safety::{SafetyConfig, SafetyLayer};

pub(crate) use quad_controller::DefaultQuadController;
pub(crate) use quad_mission_planner::GoalSeekingQuadMissionPlanner;
pub(crate) use quad_sensors::RangeQuadSensors;
pub(crate) use safety::ClearanceSafety;
