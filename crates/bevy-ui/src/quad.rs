use crate::environment::{from_control, to_control};
use crate::{
    LayerKind, MissionGoal, QuadBody, QuadCommand, QuadCommandQueue, SimulationMode,
    SimulationWorld, Telemetry, VisualSettings,
};
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use quad_core::{ControlFrame, FlightControls, FlightState, ObjectId, Quad, WorldObject};
use quad_sim::FcEscStack;

#[derive(Component)]
struct QuadRuntime {
    core: Quad,
    flight_stack: FcEscStack,
    mission_plan: Vec3,
    observed: Vec<WorldObject>,
    rpm: [f32; 4],
}

#[derive(Component)]
struct Rotor(usize);

pub struct QuadPlugin;

impl Plugin for QuadPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_quad)
            .add_systems(
                FixedUpdate,
                (process_commands, control_quad)
                    .chain()
                    .before(PhysicsSet::SyncBackend),
            )
            .add_systems(Update, (update_visual_feedback, spin_rotors));
    }
}

fn spawn_quad(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let body = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.34, 0.58, 0.07))),
            MeshMaterial3d(materials.add(Color::srgb(0.09, 0.14, 0.17))),
            Transform::from_xyz(12.0, 15.0, 0.18),
            RigidBody::Dynamic,
            quad_collider(),
            ColliderMassProperties::Mass(1.0),
            Velocity::zero(),
            GravityScale(0.0),
            ExternalForce::default(),
            Damping {
                linear_damping: 1.7,
                angular_damping: 5.0,
            },
            QuadBody,
            QuadRuntime {
                core: Quad::new(),
                flight_stack: FcEscStack::default(),
                mission_plan: Vec3::ZERO,
                observed: Vec::new(),
                rpm: [0.0; 4],
            },
        ))
        .id();
    add_rotor_visuals(&mut commands, body, &mut meshes, &mut materials);
    add_fpv_body(&mut commands, body, &mut meshes, &mut materials);
}

fn quad_collider() -> Collider {
    let mut parts = vec![(
        Vec3::ZERO,
        Quat::IDENTITY,
        Collider::cuboid(0.6, 0.4, 0.175),
    )];
    for (x, y) in [(-0.42, 0.42), (0.42, 0.42), (0.42, -0.42), (-0.42, -0.42)] {
        parts.push((
            Vec3::new(x, y, 0.11),
            Quat::from_rotation_x(std::f32::consts::FRAC_PI_2),
            Collider::cylinder(0.025, 0.29),
        ));
    }
    parts.push((
        Vec3::new(0.0, 0.36, 0.14),
        Quat::IDENTITY,
        Collider::cuboid(0.16, 0.16, 0.14),
    ));
    parts.push((
        Vec3::new(0.0, -0.09, 0.14),
        Quat::IDENTITY,
        Collider::cuboid(0.13, 0.18, 0.07),
    ));
    Collider::compound(parts)
}

fn add_fpv_body(
    commands: &mut Commands,
    body: Entity,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) {
    let dark = materials.add(Color::srgb(0.05, 0.08, 0.11));
    commands.entity(body).with_children(|parent| {
        parent.spawn((
            Mesh3d(meshes.add(Cuboid::new(0.22, 0.15, 0.16))),
            MeshMaterial3d(dark.clone()),
            Transform::from_xyz(0.0, 0.36, 0.14),
        ));
        parent.spawn((
            Mesh3d(meshes.add(Cuboid::new(0.23, 0.34, 0.12))),
            MeshMaterial3d(dark),
            Transform::from_xyz(0.0, -0.09, 0.14),
        ));
    });
    add_fpv_cage(commands, body, meshes, materials);
    add_fpv_lens(commands, body, meshes, materials);
}

fn add_fpv_cage(
    commands: &mut Commands,
    body: Entity,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) {
    let orange = materials.add(Color::srgb(1.0, 0.45, 0.12));
    commands.entity(body).with_children(|parent| {
        for x in [-0.14, 0.14] {
            parent.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.035, 0.24, 0.23))),
                MeshMaterial3d(orange.clone()),
                Transform::from_xyz(x, 0.34, 0.14),
            ));
        }
        parent.spawn((
            Mesh3d(meshes.add(Cuboid::new(0.31, 0.24, 0.025))),
            MeshMaterial3d(orange),
            Transform::from_xyz(0.0, 0.34, 0.265),
        ));
    });
}

fn add_fpv_lens(
    commands: &mut Commands,
    body: Entity,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) {
    let dark = materials.add(Color::srgb(0.05, 0.08, 0.11));
    let glass = materials.add(Color::srgb(0.15, 0.55, 0.8));
    commands.entity(body).with_children(|parent| {
        parent.spawn((
            Mesh3d(meshes.add(Cylinder::new(0.085, 0.075))),
            MeshMaterial3d(dark),
            Transform::from_xyz(0.0, 0.465, 0.15),
        ));
        parent.spawn((
            Mesh3d(meshes.add(Cylinder::new(0.061, 0.008))),
            MeshMaterial3d(glass),
            Transform::from_xyz(0.0, 0.507, 0.15),
        ));
    });
}

fn add_rotor_visuals(
    commands: &mut Commands,
    body: Entity,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) {
    let arm = meshes.add(Cuboid::new(0.09, 1.19, 0.07));
    let blade = meshes.add(Cuboid::new(0.49, 0.055, 0.018));
    let hub = meshes.add(Cuboid::new(0.13, 0.13, 0.09));
    let dark = materials.add(Color::srgb(0.18, 0.27, 0.3));
    let prop = materials.add(Color::srgb(0.78, 0.88, 0.86));
    commands.entity(body).with_children(|parent| {
        for (index, (x, y)) in [
            (-0.42_f32, 0.42_f32),
            (0.42, 0.42),
            (0.42, -0.42),
            (-0.42, -0.42),
        ]
        .into_iter()
        .enumerate()
        {
            parent.spawn((
                Mesh3d(arm.clone()),
                MeshMaterial3d(dark.clone()),
                Transform::from_rotation(Quat::from_rotation_z(-x.atan2(y))),
            ));
            parent.spawn((
                Mesh3d(hub.clone()),
                MeshMaterial3d(dark.clone()),
                Transform::from_xyz(x, y, 0.05),
            ));
            parent.spawn((
                Mesh3d(blade.clone()),
                MeshMaterial3d(prop.clone()),
                Transform::from_xyz(x, y, 0.11),
                Rotor(index),
            ));
        }
    });
}

fn spin_rotors(
    time: Res<Time>,
    quads: Query<&QuadRuntime>,
    mut rotors: Query<(&Rotor, &mut Transform)>,
) {
    let Ok(quad) = quads.single() else {
        return;
    };
    for (rotor, mut transform) in &mut rotors {
        let direction = if rotor.0 % 2 == 0 { 1.0 } else { -1.0 };
        transform.rotate_z(
            direction * quad.rpm[rotor.0] * std::f32::consts::TAU / 60.0 * time.delta_secs(),
        );
    }
}

fn process_commands(
    mut queue: ResMut<QuadCommandQueue>,
    mut quads: Query<&mut QuadRuntime>,
    mut goal: ResMut<MissionGoal>,
    mut telemetry: ResMut<Telemetry>,
    mut visuals: ResMut<VisualSettings>,
) {
    let Ok(mut quad) = quads.single_mut() else {
        return;
    };
    while let Some(command) = queue.0.pop_front() {
        apply_command(command, &mut quad, &mut goal, &mut telemetry, &mut visuals);
    }
}

fn apply_command(
    command: QuadCommand,
    quad: &mut QuadRuntime,
    goal: &mut MissionGoal,
    telemetry: &mut Telemetry,
    visuals: &mut VisualSettings,
) {
    match command {
        QuadCommand::SetAutopilot(active) => quad.core.set_autopilot(active),
        QuadCommand::SetAngleMode(active) => {
            quad.flight_stack.set_angle_mode(active);
            quad.core.set_safety_angle_mode(active);
        }
        QuadCommand::ToggleSafety => {
            let enabled = !quad.core.safety_enabled();
            quad.core.set_safety_enabled(enabled);
            visuals.safety = enabled;
        }
        QuadCommand::SetManualControls(controls) => quad.core.set_manual_controls(controls),
        QuadCommand::SetCruiseSpeed(speed) => quad.core.set_cruise_speed(speed),
        QuadCommand::SetMissionGoal(target) => goal.0 = target,
        QuadCommand::SetLayerVisible(layer, visible) => set_layer_visible(visuals, layer, visible),
        QuadCommand::Reset => reset_quad(quad, goal, telemetry),
    }
}

fn reset_quad(quad: &mut QuadRuntime, goal: &mut MissionGoal, telemetry: &mut Telemetry) {
    quad.flight_stack.reset();
    quad.core.set_manual_controls(FlightControls::default());
    quad.mission_plan = Vec3::ZERO;
    quad.observed.clear();
    quad.rpm = [0.0; 4];
    telemetry.elapsed = 0.0;
    *goal = MissionGoal::default();
}

fn set_layer_visible(settings: &mut VisualSettings, layer: LayerKind, visible: bool) {
    match layer {
        LayerKind::QuadSensors => settings.quad_sensors = visible,
        LayerKind::QuadMissionPlanner => settings.quad_mission_planner = visible,
        LayerKind::QuadController => settings.quad_controller = visible,
        LayerKind::Safety => settings.safety = visible,
    }
}

fn control_quad(
    time: Res<Time<Fixed>>,
    mode: Res<SimulationMode>,
    world: Res<SimulationWorld>,
    goal: Res<MissionGoal>,
    mut telemetry: ResMut<Telemetry>,
    mut quads: Query<(
        Entity,
        &mut QuadRuntime,
        &Transform,
        &mut Velocity,
        &mut GravityScale,
        &mut ExternalForce,
    )>,
) {
    let Ok((entity, mut quad, transform, mut velocity, mut gravity, mut force)) =
        quads.single_mut()
    else {
        return;
    };
    if !mode.0 {
        quad.flight_stack.reset();
        quad.rpm = [0.0; 4];
        gravity.0 = 0.0;
        velocity.linear = Vec3::ZERO;
        velocity.angular = Vec3::ZERO;
        force.force = Vec3::ZERO;
        force.torque = Vec3::ZERO;
        return;
    }
    gravity.0 = 1.0;
    let body_up = transform.rotation.inverse() * Vec3::Z;
    let body_rates = transform.rotation.inverse() * velocity.angular;
    let forward = transform.rotation * Vec3::Y;
    let state = FlightState {
        position: to_control(transform.translation),
        velocity: to_control(velocity.linear),
        goal: to_control(goal.0),
        attitude: quad_core::Vec3::new(
            body_up.y.atan2(body_up.z),
            (-body_up.x).atan2(body_up.z),
            (-forward.x).atan2(forward.y),
        ),
        angular_velocity: to_control(body_rates),
    };
    update_quad_control(
        &mut quad,
        &world,
        state,
        entity,
        time.delta_secs(),
        transform.rotation,
        &mut force,
    );
    update_telemetry(&mut telemetry, state, goal.0, &quad, time.delta_secs());
}

fn update_quad_control(
    quad: &mut QuadRuntime,
    world: &SimulationWorld,
    state: FlightState,
    entity: Entity,
    dt: f32,
    rotation: Quat,
    external: &mut ExternalForce,
) {
    let frame = ControlFrame {
        observer: ObjectId(entity.index().index() as u32),
        state,
        world: world.snapshot(),
        delta_seconds: dt,
    };
    let output = quad.core.step(frame);
    let motors = quad.flight_stack.step(output.controls, state, dt);
    quad.mission_plan = from_control(output.mission_plan.relative_target);
    quad.observed = output.observed;
    quad.rpm = motors.rpm;
    external.force = rotation * Vec3::Z * motors.thrust;
    external.torque =
        rotation * Vec3::new(motors.pitch_torque, motors.roll_torque, motors.yaw_torque);
}

fn update_telemetry(
    telemetry: &mut Telemetry,
    state: FlightState,
    goal: Vec3,
    quad: &QuadRuntime,
    dt: f32,
) {
    telemetry.position = from_control(state.position);
    telemetry.velocity = from_control(state.velocity);
    telemetry.elapsed += dt;
    telemetry.target_delta = quad.mission_plan;
    telemetry.clearance = quad
        .observed
        .iter()
        .map(|item| state.position.distance(item.position) - item.radius_m)
        .fold(f32::INFINITY, f32::min);
    telemetry.running = quad.core.autopilot_enabled();
    telemetry.safety_enabled = quad.core.safety_enabled();
    telemetry.distance = telemetry.position.distance(goal);
}

fn update_visual_feedback(
    mut gizmos: Gizmos,
    visuals: Res<VisualSettings>,
    quads: Query<(&Transform, &Velocity, &QuadRuntime), With<QuadBody>>,
) {
    let Ok((transform, velocity, quad)) = quads.single() else {
        return;
    };
    draw_layers(
        &mut gizmos,
        transform.translation,
        velocity.linear,
        &quad.observed,
        quad.mission_plan,
        &visuals,
    );
}

fn draw_layers(
    gizmos: &mut Gizmos,
    position: Vec3,
    velocity: Vec3,
    observed: &[WorldObject],
    target: Vec3,
    visuals: &VisualSettings,
) {
    draw_mission_plan(gizmos, position, target, visuals.quad_mission_planner);
    draw_quad_control(gizmos, position, velocity, visuals.quad_controller);
    draw_safety_layer(gizmos, position, velocity, visuals.safety);
    draw_quad_sensors(gizmos, position, observed, visuals.quad_sensors);
}

fn draw_mission_plan(gizmos: &mut Gizmos, position: Vec3, target: Vec3, enabled: bool) {
    if enabled {
        gizmos.line(position, position + target, Color::srgb(0.35, 0.88, 0.55));
    }
}

fn draw_quad_control(gizmos: &mut Gizmos, position: Vec3, velocity: Vec3, enabled: bool) {
    if enabled {
        gizmos.line(position, position + velocity, Color::srgb(1.0, 0.67, 0.25));
    }
}

fn draw_safety_layer(gizmos: &mut Gizmos, position: Vec3, velocity: Vec3, enabled: bool) {
    if enabled {
        draw_safety(gizmos, position, velocity);
    }
}

fn draw_quad_sensors(gizmos: &mut Gizmos, position: Vec3, objects: &[WorldObject], enabled: bool) {
    if enabled {
        draw_observations(gizmos, position, objects);
    }
}

fn draw_safety(gizmos: &mut Gizmos, position: Vec3, velocity: Vec3) {
    let color = Color::srgb(0.7, 0.45, 0.95);
    gizmos.line(position, position + velocity * 1.2, color);
}

fn draw_observations(gizmos: &mut Gizmos, position: Vec3, objects: &[WorldObject]) {
    for object in objects {
        let target = from_control(object.position);
        gizmos.line(position, target, Color::srgba(0.25, 0.82, 0.9, 0.24));
    }
}
