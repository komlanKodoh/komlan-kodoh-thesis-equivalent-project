use crate::{
    LayerKind, QuadCommand, QuadCommandQueue, SimulationMode, StageEditCommand,
    StageEditCommandQueue, Telemetry, VisualSettings, WorldCommand, WorldCommandQueue,
};
use bevy::prelude::*;
use bevy::{
    input::mouse::{MouseScrollUnit, MouseWheel},
    picking::hover::HoverMap,
};
use quad_core::FlightControls;

const PANEL: Color = Color::srgba(0.035, 0.075, 0.09, 0.92);
const TEXT: Color = Color::srgb(0.82, 0.92, 0.9);

pub struct FlightUiPlugin;

impl Plugin for FlightUiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PositionEditor>()
            .init_resource::<StageEditor>()
            .init_resource::<InputForce>()
            .init_resource::<SimulationMode>()
            .init_resource::<FlightMode>()
            .init_resource::<HudVisibility>()
            .add_systems(Startup, build_hud)
            .add_systems(
                Update,
                (
                    button_actions,
                    pilot_input,
                    update_hud,
                    update_layer_buttons,
                    toggle_hud,
                ),
            )
            .add_systems(Update, send_scroll_events)
            .add_observer(on_scroll_handler);
    }
}

#[derive(Resource)]
struct PositionEditor(Vec3);

impl Default for PositionEditor {
    fn default() -> Self {
        Self(Vec3::new(12.0, 15.0, 0.18))
    }
}

#[derive(Resource)]
struct StageEditor {
    goal: Vec3,
    center: Vec3,
    size: Vec3,
    cruise_speed: f32,
}

impl Default for StageEditor {
    fn default() -> Self {
        Self {
            goal: Vec3::new(145.0, 145.0, 18.0),
            center: Vec3::new(100.0, 100.0, 0.0),
            size: Vec3::new(8.0, 8.0, 8.0),
            cruise_speed: 4.2,
        }
    }
}

#[derive(Resource, Default)]
struct InputForce(FlightControls);

#[derive(Resource)]
struct HudVisibility(bool);

impl Default for HudVisibility {
    fn default() -> Self {
        Self(true)
    }
}

#[derive(Component)]
struct HudRoot;

#[derive(Resource)]
struct FlightMode(bool);

impl Default for FlightMode {
    fn default() -> Self {
        Self(true)
    }
}

#[derive(Clone, Copy, Component)]
enum Action {
    Autopilot,
    FlightMode,
    Reset,
    Teleport,
    AdjustX(f32),
    AdjustY(f32),
    AdjustZ(f32),
    Layer(LayerKind),
    Safety,
    Pilot(Vec3),
    AdjustGoal(Vec3),
    SetGoal,
    AdjustStageCenter(Vec3),
    AdjustStageSize(Vec3),
    AddObstacle,
    RemoveObstacle,
    AdjustCruiseSpeed(f32),
}

#[derive(Clone, Copy, Component)]
enum Readout {
    Status,
    Position,
    Speed,
    Goal,
    Safety,
    MissionTarget,
    StageCenter,
    StageSize,
    CruiseSpeed,
}

fn build_hud(mut commands: Commands) {
    commands.spawn((
        Camera2d,
        Camera {
            order: 10,
            ..default()
        },
    ));
    let panel = commands.spawn((panel_node(), HudRoot)).id();
    commands.entity(panel).with_children(|ui| {
        ui.spawn((
            Text::new("AETHER  /  QUAD–001"),
            font(20.0),
            TextColor(TEXT),
        ));
        spawn_readout(ui, "Awaiting departure", Readout::Status);
        spawn_stats(ui);
        spawn_mission_controls(ui);
        spawn_stage_editor(ui);
        spawn_section(ui, "PILOT CONTROLS");
        spawn_pilot_buttons(ui);
        spawn_position_controls(ui);
        spawn_section(ui, "CONTROL LAYERS  ·  60 Hz / 1 Hz");
        spawn_layer_controls(ui);
        spawn_section(ui, "DRAG TO ORBIT  ·  SCROLL TO ZOOM  ·  H TO HIDE HUD");
    });
}

fn spawn_mission_controls(ui: &mut ChildSpawnerCommands) {
    spawn_section(ui, "MISSION CONTROL  ·  MAP 200 × 200 M");
    spawn_button(ui, "START / STOP SIMULATION", Action::Autopilot);
    spawn_readout(ui, "CRUISE SPEED   4.2 m/s", Readout::CruiseSpeed);
    spawn_adjust_row(
        ui,
        "CRUISE",
        Action::AdjustCruiseSpeed(-0.5),
        Action::AdjustCruiseSpeed(0.5),
    );
    spawn_readout(ui, "TARGET   145 / 145 / 18 m", Readout::MissionTarget);
    spawn_adjust_row(
        ui,
        "TARGET X",
        Action::AdjustGoal(Vec3::new(-10.0, 0.0, 0.0)),
        Action::AdjustGoal(Vec3::new(10.0, 0.0, 0.0)),
    );
    spawn_adjust_row(
        ui,
        "TARGET Y",
        Action::AdjustGoal(Vec3::new(0.0, -10.0, 0.0)),
        Action::AdjustGoal(Vec3::new(0.0, 10.0, 0.0)),
    );
    spawn_adjust_row(
        ui,
        "TARGET Z",
        Action::AdjustGoal(Vec3::new(0.0, 0.0, -5.0)),
        Action::AdjustGoal(Vec3::new(0.0, 0.0, 5.0)),
    );
    spawn_button(ui, "SET MISSION TARGET", Action::SetGoal);
}

fn spawn_stage_editor(ui: &mut ChildSpawnerCommands) {
    spawn_section(ui, "STAGE EDITOR  ·  ADD / REMOVE BUILDINGS");
    spawn_readout(ui, "BUILDING CENTER   100 / 100 m", Readout::StageCenter);
    spawn_adjust_row(
        ui,
        "CENTER X",
        Action::AdjustStageCenter(Vec3::new(-10.0, 0.0, 0.0)),
        Action::AdjustStageCenter(Vec3::new(10.0, 0.0, 0.0)),
    );
    spawn_adjust_row(
        ui,
        "CENTER Y",
        Action::AdjustStageCenter(Vec3::new(0.0, -10.0, 0.0)),
        Action::AdjustStageCenter(Vec3::new(0.0, 10.0, 0.0)),
    );
    spawn_readout(ui, "BUILDING SIZE   8 × 8 × 8 m", Readout::StageSize);
    spawn_adjust_row(
        ui,
        "WIDTH",
        Action::AdjustStageSize(Vec3::new(-2.0, 0.0, 0.0)),
        Action::AdjustStageSize(Vec3::new(2.0, 0.0, 0.0)),
    );
    spawn_adjust_row(
        ui,
        "DEPTH",
        Action::AdjustStageSize(Vec3::new(0.0, -2.0, 0.0)),
        Action::AdjustStageSize(Vec3::new(0.0, 2.0, 0.0)),
    );
    spawn_adjust_row(
        ui,
        "HEIGHT",
        Action::AdjustStageSize(Vec3::new(0.0, 0.0, -2.0)),
        Action::AdjustStageSize(Vec3::new(0.0, 0.0, 2.0)),
    );
    spawn_button(ui, "ADD BUILDING", Action::AddObstacle);
    spawn_button(ui, "REMOVE LAST BUILDING", Action::RemoveObstacle);
}

fn panel_node() -> impl Bundle {
    (
        Node {
            position_type: PositionType::Absolute,
            top: px(20),
            right: px(20),
            width: px(330),
            max_height: percent(94),
            overflow: Overflow::scroll_y(),
            flex_direction: FlexDirection::Column,
            row_gap: px(12),
            padding: UiRect::all(px(18)),
            border: UiRect::all(px(1)),
            ..default()
        },
        BackgroundColor(PANEL),
        BorderColor::all(Color::srgba(0.4, 0.8, 0.75, 0.28)),
        ScrollPosition::default(),
    )
}

#[derive(EntityEvent, Debug)]
#[entity_event(propagate, auto_propagate)]
struct HudScroll {
    entity: Entity,
    delta: Vec2,
}

fn send_scroll_events(
    mut wheel: MessageReader<MouseWheel>,
    hover_map: Res<HoverMap>,
    mut commands: Commands,
) {
    for event in wheel.read() {
        let scale = if event.unit == MouseScrollUnit::Line {
            21.0
        } else {
            1.0
        };
        let delta = -Vec2::new(event.x, event.y) * scale;
        for entities in hover_map.values() {
            for entity in entities.keys().copied() {
                commands.trigger(HudScroll { entity, delta });
            }
        }
    }
}

fn on_scroll_handler(
    mut scroll: On<HudScroll>,
    mut nodes: Query<(&mut ScrollPosition, &Node, &ComputedNode)>,
) {
    let Ok((mut position, node, computed)) = nodes.get_mut(scroll.entity) else {
        return;
    };
    if node.overflow.y != OverflowAxis::Scroll {
        return;
    }
    let max = (computed.content_size().y - computed.size().y) * computed.inverse_scale_factor();
    position.y = (position.y + scroll.delta.y).clamp(0.0, max.max(0.0));
    scroll.propagate(false);
}

fn spawn_stats(ui: &mut ChildSpawnerCommands) {
    spawn_readout(ui, "POSITION   12.0 / 15.0 / 0.2 m", Readout::Position);
    spawn_readout(ui, "SPEED      0.0 m/s", Readout::Speed);
    spawn_readout(ui, "GOAL       0.0 m", Readout::Goal);
    spawn_readout(ui, "SAFETY     ACTIVE", Readout::Safety);
}

fn spawn_section(ui: &mut ChildSpawnerCommands, title: &str) {
    ui.spawn((
        Text::new(title),
        font(10.0),
        TextColor(Color::srgb(0.4, 0.78, 0.74)),
    ));
}

fn spawn_adjust_row(ui: &mut ChildSpawnerCommands, label: &str, down: Action, up: Action) {
    ui.spawn(Node {
        flex_direction: FlexDirection::Row,
        align_items: AlignItems::Center,
        column_gap: px(6),
        ..default()
    })
    .with_children(|row| {
        row.spawn((
            Text::new(label),
            font(9.0),
            TextColor(TEXT),
            Node {
                width: px(82),
                ..default()
            },
        ));
        spawn_button(row, "−", down);
        spawn_button(row, "+", up);
    });
}

fn spawn_readout(ui: &mut ChildSpawnerCommands, title: &str, kind: Readout) {
    ui.spawn((Text::new(title), font(13.0), TextColor(TEXT), kind));
}

fn spawn_pilot_buttons(ui: &mut ChildSpawnerCommands) {
    spawn_button(ui, "RESET VEHICLE", Action::Reset);
    spawn_button(ui, "THROTTLE UP", Action::Pilot(Vec3::Z));
    spawn_button(ui, "THROTTLE DOWN", Action::Pilot(Vec3::NEG_Z));
    spawn_button(ui, "YAW LEFT", Action::Pilot(Vec3::NEG_X));
    spawn_button(ui, "YAW RIGHT", Action::Pilot(Vec3::X));
    spawn_section(ui, "W/S throttle · A/D yaw · I/K pitch · J/L roll");
    spawn_button(ui, "ANGLE MODE · ON", Action::FlightMode);
}

fn spawn_position_controls(ui: &mut ChildSpawnerCommands) {
    spawn_section(ui, "POSITION OVERRIDE  /  XYZ METERS");
    spawn_button(ui, "X −", Action::AdjustX(-1.0));
    spawn_button(ui, "X +", Action::AdjustX(1.0));
    spawn_button(ui, "Y −", Action::AdjustY(-1.0));
    spawn_button(ui, "Y +", Action::AdjustY(1.0));
    spawn_button(ui, "Z −", Action::AdjustZ(-1.0));
    spawn_button(ui, "Z +", Action::AdjustZ(1.0));
    spawn_button(ui, "SET POSITION", Action::Teleport);
}

fn spawn_layer_controls(ui: &mut ChildSpawnerCommands) {
    spawn_button(
        ui,
        "QUAD SENSORS  ·  CYAN",
        Action::Layer(LayerKind::QuadSensors),
    );
    spawn_button(
        ui,
        "QUAD MISSION PLANNER  ·  GREEN",
        Action::Layer(LayerKind::QuadMissionPlanner),
    );
    spawn_button(
        ui,
        "QUAD CONTROLLER  ·  AMBER",
        Action::Layer(LayerKind::QuadController),
    );
    spawn_button(ui, "SAFETY      ·  VIOLET", Action::Safety);
}

fn spawn_button(ui: &mut ChildSpawnerCommands, label: &str, action: Action) {
    ui.spawn((
        Button,
        Node {
            min_height: px(34),
            padding: UiRect::axes(px(10), px(7)),
            ..default()
        },
        BackgroundColor(Color::srgba(0.25, 0.56, 0.53, 0.25)),
        action,
        children![(Text::new(label), font(10.0), TextColor(TEXT))],
    ));
}

fn font(size: f32) -> TextFont {
    TextFont {
        font_size: FontSize::Px(size),
        ..default()
    }
}

fn button_actions(
    mut buttons: Query<(&Interaction, &Action), (Changed<Interaction>, With<Button>)>,
    mut quad: ResMut<QuadCommandQueue>,
    mut world: ResMut<WorldCommandQueue>,
    mut stage: ResMut<StageEditCommandQueue>,
    mut position: ResMut<PositionEditor>,
    mut editor: ResMut<StageEditor>,
    mut visuals: ResMut<VisualSettings>,
    mut mode: ResMut<SimulationMode>,
    mut flight_mode: ResMut<FlightMode>,
) {
    for (interaction, action) in &mut buttons {
        if *interaction == Interaction::Pressed {
            apply_action(
                *action,
                &mut quad,
                &mut world,
                &mut stage,
                &mut position,
                &mut editor,
                &mut visuals,
                &mut mode,
                &mut flight_mode,
            );
        }
    }
}

fn apply_action(
    action: Action,
    quad: &mut QuadCommandQueue,
    world: &mut WorldCommandQueue,
    stage: &mut StageEditCommandQueue,
    position: &mut PositionEditor,
    editor: &mut StageEditor,
    visuals: &mut VisualSettings,
    mode: &mut SimulationMode,
    flight_mode: &mut FlightMode,
) {
    match action {
        Action::Autopilot => {
            mode.0 = !mode.0;
            quad.0.push_back(QuadCommand::SetAutopilot(mode.0));
        }
        Action::FlightMode => {
            flight_mode.0 = !flight_mode.0;
            quad.0.push_back(QuadCommand::SetAngleMode(flight_mode.0));
        }
        Action::Reset => {
            mode.0 = false;
            reset_action(quad, world, position, editor);
        }
        Action::Teleport => world.0.push_back(WorldCommand::Teleport(position.0)),
        Action::AdjustX(delta) => position.0.x = (position.0.x + delta).max(0.0),
        Action::AdjustY(delta) => position.0.y = (position.0.y + delta).max(0.0),
        Action::AdjustZ(delta) => position.0.z = (position.0.z + delta).max(0.18),
        Action::Layer(layer) => toggle_layer(visuals, layer),
        Action::Safety => toggle_safety(quad, visuals),
        Action::Pilot(axis) => {
            quad.0
                .push_back(QuadCommand::SetManualControls(FlightControls::new(
                    axis.y,
                    axis.x,
                    axis.x,
                    if axis.z > 0.0 { 0.65 } else { 0.0 },
                )))
        }
        action => apply_editor_action(action, quad, stage, editor),
    }
}

fn reset_action(
    quad: &mut QuadCommandQueue,
    world: &mut WorldCommandQueue,
    position: &mut PositionEditor,
    editor: &mut StageEditor,
) {
    quad.0.push_back(QuadCommand::Reset);
    quad.0.push_back(QuadCommand::SetCruiseSpeed(4.2));
    world.0.push_back(WorldCommand::ResetVehicle);
    position.0 = Vec3::new(12.0, 15.0, 0.18);
    editor.goal = Vec3::new(145.0, 145.0, 18.0);
    editor.cruise_speed = 4.2;
}

fn toggle_safety(quad: &mut QuadCommandQueue, visuals: &mut VisualSettings) {
    quad.0.push_back(QuadCommand::ToggleSafety);
    toggle_layer(visuals, LayerKind::Safety);
}

fn apply_editor_action(
    action: Action,
    quad: &mut QuadCommandQueue,
    stage: &mut StageEditCommandQueue,
    editor: &mut StageEditor,
) {
    match action {
        Action::AdjustGoal(delta) => editor.goal = clamp_goal(editor.goal + delta),
        Action::SetGoal => quad.0.push_back(QuadCommand::SetMissionGoal(editor.goal)),
        Action::AdjustStageCenter(delta) => editor.center = clamp_center(editor.center + delta),
        Action::AdjustStageSize(delta) => editor.size = clamp_size(editor.size + delta),
        Action::AddObstacle => stage.0.push_back(StageEditCommand::AddObstacle {
            center: editor.center,
            size: editor.size,
        }),
        Action::RemoveObstacle => stage.0.push_back(StageEditCommand::RemoveLastObstacle),
        Action::AdjustCruiseSpeed(delta) => {
            editor.cruise_speed = (editor.cruise_speed + delta).clamp(1.0, 12.0);
            quad.0
                .push_back(QuadCommand::SetCruiseSpeed(editor.cruise_speed));
        }
        _ => {}
    }
}

fn clamp_goal(value: Vec3) -> Vec3 {
    Vec3::new(
        value.x.clamp(0.0, 200.0),
        value.y.clamp(0.0, 200.0),
        value.z.clamp(0.0, 80.0),
    )
}

fn clamp_center(value: Vec3) -> Vec3 {
    Vec3::new(value.x.clamp(0.0, 200.0), value.y.clamp(0.0, 200.0), 0.0)
}

fn clamp_size(value: Vec3) -> Vec3 {
    Vec3::new(
        value.x.clamp(2.0, 40.0),
        value.y.clamp(2.0, 40.0),
        value.z.clamp(2.0, 40.0),
    )
}

fn toggle_layer(settings: &mut VisualSettings, layer: LayerKind) {
    match layer {
        LayerKind::QuadSensors => settings.quad_sensors = !settings.quad_sensors,
        LayerKind::QuadMissionPlanner => {
            settings.quad_mission_planner = !settings.quad_mission_planner
        }
        LayerKind::QuadController => settings.quad_controller = !settings.quad_controller,
        LayerKind::Safety => settings.safety = !settings.safety,
    }
}

fn pilot_input(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut input: ResMut<InputForce>,
    mut queue: ResMut<QuadCommandQueue>,
    mut mode: ResMut<SimulationMode>,
) {
    let has_pilot_input = [
        KeyCode::KeyW,
        KeyCode::KeyA,
        KeyCode::KeyS,
        KeyCode::KeyD,
        KeyCode::KeyI,
        KeyCode::KeyJ,
        KeyCode::KeyK,
        KeyCode::KeyL,
    ]
    .into_iter()
    .any(|key| keys.pressed(key));
    if has_pilot_input && !mode.0 {
        mode.0 = true;
        queue.0.push_back(QuadCommand::SetAutopilot(false));
    }
    let mut controls = FlightControls::new(
        axis(keys.pressed(KeyCode::KeyI), keys.pressed(KeyCode::KeyK)),
        axis(keys.pressed(KeyCode::KeyL), keys.pressed(KeyCode::KeyJ)),
        axis(keys.pressed(KeyCode::KeyD), keys.pressed(KeyCode::KeyA)),
        input.0.throttle
            + (axis(keys.pressed(KeyCode::KeyW), keys.pressed(KeyCode::KeyS))
                * time.delta_secs()
                * 0.5),
    );
    controls.throttle = controls.throttle.clamp(0.0, 1.0);
    if controls != input.0 {
        input.0 = controls;
        queue.0.push_back(QuadCommand::SetManualControls(controls));
    }
}

fn axis(positive: bool, negative: bool) -> f32 {
    f32::from(positive) - f32::from(negative)
}

fn toggle_hud(
    keys: Res<ButtonInput<KeyCode>>,
    mut state: ResMut<HudVisibility>,
    mut hud: Query<&mut Visibility, With<HudRoot>>,
) {
    if keys.just_pressed(KeyCode::KeyH) {
        state.0 = !state.0;
        for mut visibility in &mut hud {
            *visibility = if state.0 {
                Visibility::Visible
            } else {
                Visibility::Hidden
            };
        }
    }
}

fn update_hud(
    state: Res<Telemetry>,
    editor: Res<StageEditor>,
    mut texts: Query<(&mut Text, &Readout)>,
) {
    let speed = state.velocity.length();
    for (mut text, kind) in &mut texts {
        **text = readout_text(*kind, &state, &editor, speed);
    }
}

fn readout_text(kind: Readout, state: &Telemetry, editor: &StageEditor, speed: f32) -> String {
    match kind {
        Readout::Status => {
            if state.running {
                "AUTONOMOUS FLIGHT".into()
            } else {
                "PILOT CONTROL".into()
            }
        }
        Readout::Position => format!(
            "POSITION   {:.1} / {:.1} / {:.1} m",
            state.position.x, state.position.y, state.position.z
        ),
        Readout::Speed => format!("SPEED      {speed:.1} m/s"),
        Readout::Goal => format!("GOAL       {:.1} m", state.distance),
        Readout::Safety => {
            if !state.safety_enabled {
                "SAFETY     DISABLED".into()
            } else {
                "SAFETY     ACTIVE".into()
            }
        }
        Readout::MissionTarget => format!(
            "TARGET   {:.0} / {:.0} / {:.0} m",
            editor.goal.x, editor.goal.y, editor.goal.z
        ),
        Readout::StageCenter => {
            format!("CENTER   {:.0} / {:.0} m", editor.center.x, editor.center.y)
        }
        Readout::StageSize => format!(
            "SIZE   {:.0} × {:.0} × {:.0} m",
            editor.size.x, editor.size.y, editor.size.z
        ),
        Readout::CruiseSpeed => format!("CRUISE SPEED   {:.1} m/s", editor.cruise_speed),
    }
}

fn update_layer_buttons(
    visuals: Res<VisualSettings>,
    mode: Res<SimulationMode>,
    flight_mode: Res<FlightMode>,
    mut buttons: Query<(&Action, &Children, &mut BackgroundColor), With<Button>>,
    mut labels: Query<&mut Text>,
) {
    for (action, children, mut background) in &mut buttons {
        let state = if matches!(*action, Action::FlightMode) {
            Some(("FLIGHT MODE", flight_mode.0))
        } else if matches!(*action, Action::Autopilot) {
            Some(("SIMULATION", mode.0))
        } else {
            layer_button_state(*action, &visuals)
        };
        let Some((name, enabled)) = state else {
            continue;
        };
        background.0 = if enabled {
            Color::srgba(0.25, 0.56, 0.53, 0.5)
        } else {
            Color::srgba(0.25, 0.56, 0.53, 0.12)
        };
        for child in children.iter() {
            if let Ok(mut text) = labels.get_mut(child) {
                **text = if name == "FLIGHT MODE" {
                    if enabled {
                        "ANGLE MODE · ON".into()
                    } else {
                        "ACRO MODE · ON".into()
                    }
                } else if name == "SIMULATION" {
                    if enabled {
                        "RETURN TO PLANNING".into()
                    } else {
                        "START SIMULATION".into()
                    }
                } else {
                    format!("{name}  ·  {}", if enabled { "ON" } else { "OFF" })
                };
            }
        }
    }
}

fn layer_button_state(action: Action, visuals: &VisualSettings) -> Option<(&'static str, bool)> {
    match action {
        Action::Layer(LayerKind::QuadSensors) => Some(("QUAD SENSORS", visuals.quad_sensors)),
        Action::Layer(LayerKind::QuadMissionPlanner) => {
            Some(("QUAD MISSION PLANNER", visuals.quad_mission_planner))
        }
        Action::Layer(LayerKind::QuadController) => {
            Some(("QUAD CONTROLLER", visuals.quad_controller))
        }
        Action::Safety | Action::Layer(LayerKind::Safety) => Some(("SAFETY", visuals.safety)),
        _ => None,
    }
}
