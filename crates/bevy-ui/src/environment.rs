use crate::{QuadBody, SimulationMode, StageEditCommand, StageEditCommandQueue};
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use quad_core::{ObjectId, Vec3 as ControlVec3, WorldObject, WorldSnapshot};

#[derive(Resource, Default)]
pub struct SimulationWorld {
    pub objects: Vec<WorldObject>,
}

impl SimulationWorld {
    pub fn snapshot(&self) -> WorldSnapshot {
        WorldSnapshot::new(self.objects.clone())
    }

    pub fn replace(&mut self, object: WorldObject) {
        if let Some(existing) = self.objects.iter_mut().find(|item| item.id == object.id) {
            *existing = object;
        } else {
            self.objects.push(object);
        }
    }

    pub fn remove(&mut self, id: ObjectId) {
        self.objects.retain(|object| object.id != id);
    }
}

#[derive(Resource, Clone, Copy, Debug)]
pub struct MissionGoal(pub Vec3);

impl Default for MissionGoal {
    fn default() -> Self {
        Self(Vec3::new(145.0, 145.0, 18.0))
    }
}

pub struct EnvironmentPlugin;

impl Plugin for EnvironmentPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SimulationWorld>()
            .init_resource::<MissionGoal>()
            .init_resource::<StageEditCommandQueue>()
            .add_systems(Startup, spawn_environment)
            .add_systems(FixedUpdate, sync_world_objects.after(PhysicsSet::Writeback))
            .add_systems(
                Update,
                (
                    handle_stage_edits,
                    sync_mission_marker,
                    orbit_camera,
                    draw_world_grid,
                ),
            );
    }
}

#[derive(Component)]
struct OrbitCamera {
    target: Vec3,
    was_flying: bool,
    yaw: f32,
    pitch: f32,
    distance: f32,
}

const BUILDINGS: [(f32, f32, f32, f32, f32); 9] = [
    (16.0, 56.0, 9.8, 8.0, 9.8),
    (58.0, 66.0, 8.4, 13.0, 12.6),
    (92.0, 18.0, 8.4, 9.0, 7.0),
    (100.0, 60.0, 8.4, 16.0, 8.4),
    (144.0, 104.0, 5.6, 18.0, 15.4),
    (62.0, 120.0, 9.8, 10.0, 8.4),
    (94.0, 138.0, 8.4, 12.0, 7.0),
    (18.0, 108.0, 7.0, 8.0, 8.4),
    (102.0, 108.0, 9.8, 15.0, 12.6),
];

#[derive(Component)]
struct StageObstacle(ObjectId);

#[derive(Component)]
struct MissionTargetMarker;

fn spawn_environment(
    mut commands: Commands,
    mut world: ResMut<SimulationWorld>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    spawn_lighting(&mut commands);
    spawn_ground(&mut commands, &mut meshes, &mut materials);
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(1.4))),
        MeshMaterial3d(materials.add(Color::srgb(1.0, 0.78, 0.3))),
        Transform::from_translation(Vec3::new(145.0, 145.0, 18.0)),
        MissionTargetMarker,
    ));
    for (x, y, width, height, depth) in BUILDINGS {
        spawn_building(
            &mut commands,
            &mut world,
            &mut meshes,
            &mut materials,
            x,
            y,
            width,
            height,
            depth,
        );
    }
}

fn spawn_lighting(commands: &mut Commands) {
    let target = Vec3::new(100.0, 100.0, 0.0);
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(100.0, 20.0, 100.0).looking_at(target, Vec3::Z),
        OrbitCamera {
            target,
            was_flying: false,
            yaw: 0.0,
            pitch: 0.35,
            distance: 140.0,
        },
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 12_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(-25.0, -15.0, 40.0).looking_at(Vec3::ZERO, Vec3::Z),
    ));
    commands.insert_resource(GlobalAmbientLight {
        color: Color::srgb(0.6, 0.8, 0.86),
        brightness: 180.0,
        ..default()
    });
}

fn spawn_ground(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) {
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(200.0, 200.0, 0.3))),
        MeshMaterial3d(materials.add(Color::srgb(0.08, 0.16, 0.18))),
        Transform::from_xyz(100.0, 100.0, -0.15),
        RigidBody::Fixed,
        Collider::cuboid(100.0, 100.0, 0.15),
    ));
}

fn spawn_building(
    commands: &mut Commands,
    world: &mut SimulationWorld,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    x: f32,
    y: f32,
    width: f32,
    height: f32,
    depth: f32,
) {
    let center = Vec3::new(x, y, height / 2.0);
    let entity = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(width, depth, height))),
            MeshMaterial3d(materials.add(Color::srgb(0.18, 0.28, 0.30))),
            Transform::from_translation(center),
            RigidBody::Fixed,
            Collider::cuboid(width / 2.0, depth / 2.0, height / 2.0),
        ))
        .id();
    let id = ObjectId(entity.index().index() as u32);
    commands.entity(entity).insert(StageObstacle(id));
    world.replace(WorldObject {
        id,
        position: to_control(center),
        radius_m: width.hypot(depth) / 2.0,
        height_m: height,
        label: "Building".into(),
    });
}

fn handle_stage_edits(
    mut commands: Commands,
    mut queue: ResMut<StageEditCommandQueue>,
    mut world: ResMut<SimulationWorld>,
    obstacles: Query<(Entity, &StageObstacle)>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    while let Some(edit) = queue.0.pop_front() {
        match edit {
            StageEditCommand::AddObstacle { center, size } => spawn_building(
                &mut commands,
                &mut world,
                &mut meshes,
                &mut materials,
                center.x,
                center.y,
                size.x,
                size.z,
                size.y,
            ),
            StageEditCommand::RemoveLastObstacle => {
                if let Some((entity, obstacle)) = obstacles.iter().last() {
                    world.remove(obstacle.0);
                    commands.entity(entity).despawn();
                }
            }
        }
    }
}

fn sync_mission_marker(
    goal: Res<MissionGoal>,
    mut markers: Query<&mut Transform, With<MissionTargetMarker>>,
) {
    for mut transform in &mut markers {
        transform.translation = goal.0;
    }
}

fn orbit_camera(
    mode: Res<SimulationMode>,
    time: Res<Time>,
    motion: Res<bevy::input::mouse::AccumulatedMouseMotion>,
    scroll: Res<bevy::input::mouse::AccumulatedMouseScroll>,
    buttons: Res<ButtonInput<MouseButton>>,
    mut camera: Query<(&mut Transform, &mut OrbitCamera), Without<QuadBody>>,
    vehicle: Query<&Transform, With<QuadBody>>,
) {
    let (Ok((mut transform, mut orbit)), Ok(vehicle)) = (camera.single_mut(), vehicle.single())
    else {
        return;
    };
    if mode.0 {
        follow_vehicle(&mut transform, vehicle, time.delta_secs());
        orbit.was_flying = true;
        return;
    }
    if orbit.was_flying {
        orbit.target = Vec3::new(100.0, 100.0, 0.0);
        orbit.was_flying = false;
    }
    update_orbit(&mut orbit, &motion, &scroll, &buttons, &transform);
    let rotation = Quat::from_rotation_z(orbit.yaw) * Quat::from_rotation_x(orbit.pitch);
    transform.translation =
        orbit.target + rotation * Vec3::new(0.0, -orbit.distance, orbit.distance * 0.55);
    transform.look_at(orbit.target, Vec3::Z);
}

fn follow_vehicle(camera: &mut Transform, vehicle: &Transform, dt: f32) {
    let mut forward = vehicle.rotation * Vec3::Y;
    forward.z = 0.0;
    forward = forward.normalize_or_zero();
    if forward == Vec3::ZERO {
        forward = Vec3::Y;
    }
    let desired = vehicle.translation - forward * 8.0 + Vec3::Z * 3.0;
    camera.translation = camera.translation.lerp(desired, 1.0 - (-6.0 * dt).exp());
    camera.look_at(vehicle.translation + forward * 2.0 + Vec3::Z, Vec3::Z);
}

fn update_orbit(
    orbit: &mut OrbitCamera,
    motion: &bevy::input::mouse::AccumulatedMouseMotion,
    scroll: &bevy::input::mouse::AccumulatedMouseScroll,
    buttons: &ButtonInput<MouseButton>,
    transform: &Transform,
) {
    if buttons.pressed(MouseButton::Left) {
        orbit.yaw -= motion.delta.x * 0.005;
        orbit.pitch = (orbit.pitch + motion.delta.y * 0.004).clamp(-0.1, 1.2);
    }
    if buttons.pressed(MouseButton::Right) {
        let scale = orbit.distance * 0.002;
        orbit.target += transform.right() * -motion.delta.x * scale;
        orbit.target += transform.up() * motion.delta.y * scale;
    }
    orbit.distance = (orbit.distance - scroll.delta.y * 1.5).clamp(20.0, 140.0);
}

fn draw_world_grid(mut gizmos: Gizmos) {
    let color = Color::srgba(0.34, 0.61, 0.62, 0.14);
    for coordinate in (0..=200).step_by(10) {
        let point = coordinate as f32;
        gizmos.line(
            Vec3::new(point, 0.0, 0.02),
            Vec3::new(point, 200.0, 0.02),
            color,
        );
        gizmos.line(
            Vec3::new(0.0, point, 0.02),
            Vec3::new(200.0, point, 0.02),
            color,
        );
    }
}

fn sync_world_objects(
    mut world: ResMut<SimulationWorld>,
    objects: Query<(Entity, &Transform), Changed<Transform>>,
) {
    for (entity, transform) in &objects {
        if let Some(existing) = world
            .objects
            .iter()
            .find(|object| object.id == ObjectId(entity.index().index() as u32))
            .cloned()
        {
            world.replace(WorldObject {
                position: to_control(transform.translation),
                ..existing
            });
        }
    }
}

pub fn to_control(value: Vec3) -> ControlVec3 {
    ControlVec3::new(value.x, value.y, value.z)
}
pub fn from_control(value: ControlVec3) -> Vec3 {
    Vec3::new(value.x, value.y, value.z)
}
