use super::route_planner;
use crate::{MissionState, Vec3, WorldObject};

/// Movement intent produced by mission_planner and consumed by quad_controller.
///
/// `relative_target` points from the vehicle toward the current target.
/// `desired_momentum` is the desired velocity vector in meters per second.
#[derive(Clone, Copy, Debug, Default)]
pub struct MissionPlan {
    pub relative_target: Vec3,
    pub desired_momentum: Vec3,
}

/// Produces and retains the final mission plan, for the controller.
pub trait QuadMissionPlanner {
    /// Saves the next mission plan using current state and perceived objects.
    ///
    /// `state.goal` is the world-space mission target. The saved target is
    /// relative to the current position; desired momentum is a velocity vector.
    fn plan(&mut self, state: MissionState, objects: &[WorldObject]);
    fn mission_plan(&self) -> MissionPlan;
}

/// Default mission planning implementation that seeks the goal and biases away from obstacles.
pub(crate) struct GoalSeekingQuadMissionPlanner {
    mission_plan: MissionPlan,
    cruise_speed: f32,
    route_map_size: Option<f32>,
}

impl GoalSeekingQuadMissionPlanner {
    /// Creates goal seeking with a cruise velocity in meters per second.
    pub(crate) fn new(cruise_speed: f32) -> Self {
        Self {
            mission_plan: MissionPlan::default(),
            cruise_speed,
            route_map_size: None,
        }
    }

    /// Changes the cruise velocity, clamped to the core's 1–40 m/s range.
    pub(crate) fn set_cruise_speed(&mut self, speed: f32) {
        self.cruise_speed = speed.clamp(1.0, 40.0);
    }

    pub(crate) fn reset(&mut self) {
        self.mission_plan = MissionPlan::default();
    }

    pub(crate) fn set_route_map_size(&mut self, size: f32) {
        self.route_map_size = Some(size);
    }

    fn route_plan(
        &self,
        state: MissionState,
        objects: &[WorldObject],
        map_size: f32,
    ) -> MissionPlan {
        let (waypoint, next) = route_planner::route(state.position, state.goal, objects, map_size)
            .unwrap_or((state.position, None));
        let toward = crate::Vec3::new(
            waypoint.x - state.position.x,
            waypoint.y - state.position.y,
            0.0,
        );
        let goal_distance = horizontal_distance(state.position, state.goal);
        let mut speed = self.cruise_speed.min(braking_speed(goal_distance, 0.0));
        if let Some(next) = next {
            let after = crate::Vec3::new(next.x - waypoint.x, next.y - waypoint.y, 0.0);
            let turn = turn_angle(toward, after);
            let corner_speed = (self.cruise_speed * (1.0 - turn / 1.57)).max(5.0);
            speed = speed.min(braking_speed(toward.length(), corner_speed));
        }
        MissionPlan {
            relative_target: waypoint - state.position,
            desired_momentum: toward.normalize_or_zero() * speed
                + crate::Vec3::new(0.0, 0.0, (state.goal.z - state.position.z).clamp(-4.0, 4.0)),
        }
    }
}

impl QuadMissionPlanner for GoalSeekingQuadMissionPlanner {
    fn mission_plan(&self) -> MissionPlan {
        self.mission_plan
    }

    fn plan(&mut self, state: MissionState, objects: &[WorldObject]) {
        self.mission_plan = self.requested_plan(state, objects);
    }
}

impl GoalSeekingQuadMissionPlanner {
    fn requested_plan(&self, state: MissionState, objects: &[WorldObject]) -> MissionPlan {
        if let Some(map_size) = self.route_map_size {
            return self.route_plan(state, objects, map_size);
        }
        let mut target = state.goal - state.position;
        repel_from_objects(&mut target, state.position, objects);
        MissionPlan {
            relative_target: target,
            desired_momentum: target.normalize_or_zero() * target.length().min(self.cruise_speed),
        }
    }
}

fn horizontal_distance(a: crate::Vec3, b: crate::Vec3) -> f32 {
    ((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt()
}

fn braking_speed(distance: f32, end_speed: f32) -> f32 {
    (end_speed * end_speed + 6.0 * (distance - 10.0).max(0.0)).sqrt()
}

fn turn_angle(before: crate::Vec3, after: crate::Vec3) -> f32 {
    before
        .normalize_or_zero()
        .dot(after.normalize_or_zero())
        .clamp(-1.0, 1.0)
        .acos()
}

fn repel_from_objects(target: &mut crate::Vec3, position: crate::Vec3, objects: &[WorldObject]) {
    // Increase clearance when an object is less than 15 m away.
    for object in objects {
        let away = position - object.position;
        let clearance = away.length() - object.radius_m - 5.0;
        if clearance < 15.0 && clearance > 0.1 {
            *target = *target + away.normalize_or_zero() * (15.0 - clearance) * 2.0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cruise_speed_can_reach_thirty_meters_per_second() {
        let mut planner = GoalSeekingQuadMissionPlanner::new(4.2);
        planner.set_cruise_speed(30.0);
        let state = MissionState {
            goal: crate::Vec3::new(200.0, 0.0, 0.0),
            ..MissionState::default()
        };
        planner.plan(state, &[]);
        assert_eq!(planner.mission_plan().desired_momentum.length(), 30.0);
    }
}
