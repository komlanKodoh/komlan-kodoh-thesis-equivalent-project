use crate::{QuadBody, WorldCommand, WorldCommandQueue};
use bevy::prelude::*;
use bevy_rapier3d::plugin::RapierContextInitialization;
use bevy_rapier3d::prelude::*;

pub struct SimulationPhysicsPlugin;

impl Plugin for SimulationPhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<WorldCommandQueue>()
            .insert_resource(Time::<Fixed>::from_hz(60.0))
            .insert_resource(TimestepMode::Fixed {
                dt: 1.0 / 60.0,
                substeps: 1,
            })
            .add_plugins(rapier_plugin())
            .add_systems(
                FixedUpdate,
                apply_world_commands.before(PhysicsSet::SyncBackend),
            );
    }
}

fn rapier_plugin() -> RapierPhysicsPlugin<NoUserData> {
    RapierPhysicsPlugin::<NoUserData>::default()
        .with_custom_initialization(
            RapierContextInitialization::InitializeDefaultRapierContext {
                integration_parameters: default(),
                rapier_configuration: RapierConfiguration {
                    gravity: Vec3::new(0.0, 0.0, -9.81),
                    ..RapierConfiguration::new(1.0)
                },
            },
        )
        .in_fixed_schedule()
}

fn apply_world_commands(
    mut queue: ResMut<WorldCommandQueue>,
    mut vehicles: Query<(&mut Transform, &mut Velocity, &mut ExternalForce), With<QuadBody>>,
) {
    let Ok((mut transform, mut velocity, mut force)) = vehicles.single_mut() else {
        return;
    };
    while let Some(command) = queue.0.pop_front() {
        match command {
            WorldCommand::Teleport(position) => {
                transform.translation = position;
                velocity.linear = Vec3::ZERO;
                force.force = Vec3::ZERO;
            }
            WorldCommand::ResetVehicle => {
                transform.translation = Vec3::new(12.0, 15.0, 0.18);
                velocity.linear = Vec3::ZERO;
                force.force = Vec3::ZERO;
            }
        }
    }
}
