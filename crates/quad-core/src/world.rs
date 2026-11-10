use crate::Vec3;

#[derive(Clone, Copy, Debug, Default, Eq, Hash, PartialEq)]
pub struct ObjectId(pub u32);

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Meters(pub f32);

#[derive(Clone, Debug, PartialEq)]
pub struct WorldObject {
    pub id: ObjectId,
    pub position: Vec3,
    pub radius_m: f32,
    pub height_m: f32,
    pub label: String,
}

#[derive(Clone, Debug, Default)]
pub struct WorldSnapshot {
    objects: Vec<WorldObject>,
}

impl WorldSnapshot {
    pub fn new(objects: Vec<WorldObject>) -> Self {
        Self { objects }
    }

    pub fn objects_within(
        &self,
        observer: ObjectId,
        origin: Vec3,
        radius: Meters,
    ) -> Vec<WorldObject> {
        self.objects
            .iter()
            .filter(|object| object.id != observer)
            .filter(|object| object.position.distance(origin) - object.radius_m <= radius.0)
            .cloned()
            .collect()
    }

    pub fn objects(&self) -> &[WorldObject] {
        &self.objects
    }
}
