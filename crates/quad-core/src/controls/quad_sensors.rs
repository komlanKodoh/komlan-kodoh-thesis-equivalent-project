use crate::{ObjectId, Vec3, WorldObject, WorldSnapshot};

/// Selects the world objects available to the controller for one frame.
///
/// Implementations retain objects the observer can currently perceive.
/// They receive an immutable world snapshot and must not alter simulation state.
pub trait QuadSensors {
    /// Saves observed objects for an observer at `position`, in meters.
    ///
    /// The observer ID identifies the controlled vehicle so it can be excluded
    /// from the saved objects, which the mission planner uses for this frame.
    fn observe(&mut self, world: &WorldSnapshot, observer: ObjectId, position: Vec3);
    fn observed(&self) -> &[WorldObject];
}

/// Default quad sensors implementation that observes objects within a radius.
pub(crate) struct RangeQuadSensors {
    observed: Vec<WorldObject>,
    radius: f32,
    line_of_sight: bool,
}

impl RangeQuadSensors {
    /// Creates a range filter with `radius` expressed in meters.
    pub(crate) fn new(radius: f32) -> Self {
        Self {
            observed: Vec::new(),
            radius,
            line_of_sight: false,
        }
    }

    pub(crate) fn clear(&mut self) {
        self.observed.clear();
    }

    /// Uses an omnidirectional sensor with range and tower occlusion.
    pub(crate) fn visible_within(radius: f32) -> Self {
        Self {
            observed: Vec::new(),
            radius,
            line_of_sight: true,
        }
    }
}

impl QuadSensors for RangeQuadSensors {
    fn observed(&self) -> &[WorldObject] {
        &self.observed
    }

    /// Uses the snapshot's distance filter and excludes the observer itself.
    fn observe(&mut self, world: &WorldSnapshot, observer: ObjectId, position: Vec3) {
        self.observed = world
            .objects_within(observer, position, crate::Meters(self.radius))
            .into_iter()
            .filter(|target| !self.line_of_sight || visible(position, target, world.objects()))
            .collect();
    }
}

fn visible(position: Vec3, target: &WorldObject, objects: &[WorldObject]) -> bool {
    let dx = target.position.x - position.x;
    let dy = target.position.y - position.y;
    let distance_sq = dx * dx + dy * dy;
    objects
        .iter()
        .filter(|object| object.id != target.id)
        .all(|object| {
            let ox = object.position.x - position.x;
            let oy = object.position.y - position.y;
            let t = ((ox * dx + oy * dy) / distance_sq.max(1e-6)).clamp(0.0, 1.0);
            let gap_sq = (ox - t * dx).powi(2) + (oy - t * dy).powi(2);
            t >= 1.0 || gap_sq >= object.radius_m.powi(2)
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn browser_sensor_hides_tower_behind_another() {
        let tower = |id, x, y| WorldObject {
            id: ObjectId(id),
            position: Vec3::new(x, y, 15.0),
            radius_m: 6.0,
            height_m: 30.0,
            label: "Tower".into(),
        };
        let world = WorldSnapshot::new(vec![
            tower(1, 40.0, 0.0),
            tower(2, 80.0, 0.0),
            tower(3, 80.0, 30.0),
        ]);
        let mut sensors = RangeQuadSensors::visible_within(180.0);
        sensors.observe(&world, ObjectId(0), Vec3::ZERO);
        let seen = sensors.observed();
        assert_eq!(seen.len(), 2);
        assert!(seen.iter().all(|object| object.id != ObjectId(2)));
    }
}
