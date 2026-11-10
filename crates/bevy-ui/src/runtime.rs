use crate::ui::FlightUiPlugin;
use crate::{
    EnvironmentPlugin, QuadCommandQueue, QuadPlugin, SimulationPhysicsPlugin, Telemetry,
    VisualSettings, WorldCommandQueue,
};
use bevy::prelude::*;
use bevy::window::WindowMode;

pub fn run() {
    build_app().run();
}

fn build_app() -> App {
    let mut window = Window {
        title: "Aether Flight Simulation".into(),
        fit_canvas_to_parent: true,
        ..default()
    };
    configure_window(&mut window);

    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(window),
        ..default()
    }))
    .add_plugins((
        SimulationPhysicsPlugin,
        EnvironmentPlugin,
        QuadPlugin,
        FlightUiPlugin,
    ))
    .init_resource::<QuadCommandQueue>()
    .init_resource::<WorldCommandQueue>()
    .init_resource::<Telemetry>()
    .init_resource::<VisualSettings>();
    app
}

fn configure_window(window: &mut Window) {
    #[cfg(target_arch = "wasm32")]
    {
        window.canvas = Some("#world".into());
        window.prevent_default_event_handling = false;
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        window.mode = WindowMode::BorderlessFullscreen(MonitorSelection::Primary);
    }
}
