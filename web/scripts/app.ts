import * as THREE from 'three';
import { DEFAULT_GOAL, MAP_SIZE, PhysicsWorld } from './physics.js';
import { FlightScene } from './scene.js';
import { SceneEditor } from './editor.js';

let sceneEditor: SceneEditor | undefined;

const FIXED_STEP = 1 / 60;
let simulationActive = false;
let activeGoal = { ...DEFAULT_GOAL };
const runtime = document.querySelector<HTMLElement>('#runtime');
const canvas = document.querySelector<HTMLCanvasElement>('#world');

type BrowserController = {
  set_autopilot(enabled: boolean): void;
  set_angle_mode(enabled: boolean): void;
  reset_flight_stack(): void;
  set_goal(x: number, y: number, z: number): void;
  set_cruise_speed(speed: number): void;
  set_manual_controls(pitch: number, roll: number, yaw: number, throttle: number): void;
  set_safety(enabled: boolean): void;
  safety_enabled(): boolean;
  safety_debug(): string | undefined;
  compute_controls(state: Float32Array, obstacles: Float32Array, dt: number): Float32Array;
};

type BrowserEngine = {
  default(): Promise<void>;
  BrowserController: new () => BrowserController;
};

async function start(): Promise<void> {
  if (!canvas) throw new Error('Missing world canvas');
  const engine = await import('../native/aether_control.js') as unknown as BrowserEngine;
  await engine.default();
  const controller = new engine.BrowserController();
  controller.set_goal(DEFAULT_GOAL.x, DEFAULT_GOAL.y, DEFAULT_GOAL.z);
  controller.set_cruise_speed(30);
  const physics = await PhysicsWorld.create();
  const scene = new FlightScene(canvas);
  bindControls(controller, physics, scene);
  scene.setMissionTarget(DEFAULT_GOAL);
  sceneEditor = new SceneEditor(physics, scene, target => {
    applyGoal(controller, scene, target);
    updateReadouts(physics, controller);
  }, () => !simulationActive);
  updateReadouts(physics, controller);
  setRuntime('SYSTEM ONLINE  ·  FLIGHT STUDIO');
  animate(controller, physics, scene);
}

function animate(controller: BrowserController, physics: PhysicsWorld, scene: FlightScene): void {
  let previous = performance.now();
  let lastRender = previous;
  let accumulator = 0;
  let lastOutput = new Float32Array(16);
  const tick = (now: number): void => {
    accumulator += Math.min((now - previous) / 1000, 0.1);
    previous = now;
    while (accumulator >= FIXED_STEP && simulationActive) {
      lastOutput = controlPhysics(controller, physics);
      accumulator -= FIXED_STEP;
    }
    if (!simulationActive) accumulator = 0;
    refreshView(physics, scene, lastOutput, Math.min((now - lastRender) / 1000, 0.05));
    lastRender = now;
    requestAnimationFrame(tick);
  };
  requestAnimationFrame(tick);
}

function controlPhysics(controller: BrowserController, physics: PhysicsWorld): Float32Array {
  const position = physics.quad.translation();
  const velocity = physics.quad.linvel();
  const rotation = new THREE.Quaternion(
    physics.quad.rotation().x, physics.quad.rotation().y,
    physics.quad.rotation().z, physics.quad.rotation().w);
  const forward = new THREE.Vector3(0, 1, 0).applyQuaternion(rotation);
  const heading = Math.atan2(-forward.x, forward.y);
  const inverse = rotation.invert();
  const up = new THREE.Vector3(0, 0, 1).applyQuaternion(inverse);
  const rates = physics.quad.angvel();
  const angular = new THREE.Vector3(rates.x, rates.y, rates.z).applyQuaternion(inverse);
  const state = new Float32Array([position.x, position.y, position.z, velocity.x, velocity.y, velocity.z,
    Math.atan2(up.y, up.z), Math.atan2(-up.x, up.z), heading, angular.x, angular.y, angular.z]);
  const output = controller.compute_controls(state, physics.obstacleData(), FIXED_STEP) as Float32Array;
  physics.step({ thrust: output[8], roll: output[9], pitch: output[10], yaw: output[11] });
  updateReadouts(physics, controller);
  return output;
}

function refreshView(physics: PhysicsWorld, scene: FlightScene, output: Float32Array, dt: number): void {
  sceneEditor?.update();
  scene.syncWorld(physics, simulationActive ? output.subarray(4, 8) : new Float32Array(4), dt);
  const point = physics.quad.translation();
  const origin = new THREE.Vector3(point.x, point.y, point.z);
  const target = new THREE.Vector3(output[12], output[13], output[14]);
  const velocity = physics.quad.linvel();
  const observed = physics.obstacles.map(({ body }) => body.translation())
    .filter(item => Math.hypot(item.x - point.x, item.y - point.y, item.z - point.z) < 120)
    .map(item => new THREE.Vector3(item.x, item.y, item.z));
  scene.showFeedback(origin, { target, velocity: new THREE.Vector3(velocity.x, velocity.y, velocity.z),
    observed, safetyOverriding: simulationActive && output[15] > 0.5 }, layerVisibility());
  scene.render();
}

function bindControls(controller: BrowserController, physics: PhysicsWorld, scene: FlightScene): void {
  bindPilotInput(controller, scene);
  bindAutopilot(controller, scene);
  bindPositionEditor(controller, physics, scene);
  bindMissionEditor(controller, scene);
  bindLayerSwitches();
  bindHudCollapse();
  bindSafety(controller);
  bindSpeed(controller);
}

function bindPilotInput(controller: BrowserController, scene: FlightScene): void {
  const keyboardKeys = new Set<string>();
  const sticks = { leftX: 0, leftY: 0, rightX: 0, rightY: 0 };
  let angleMode = true;
  let throttle = 0;
  const set = () => {
    if (keyboardKeys.has('w')) throttle = Math.min(1, throttle + 0.02);
    if (keyboardKeys.has('s')) throttle = Math.max(0, throttle - 0.02);
    const pitch = Math.max(-1, Math.min(1, Number(keyboardKeys.has('i')) - Number(keyboardKeys.has('k')) + sticks.rightY));
    const roll = Math.max(-1, Math.min(1, Number(keyboardKeys.has('l')) - Number(keyboardKeys.has('j')) + sticks.rightX));
    const yaw = Math.max(-1, Math.min(1, Number(keyboardKeys.has('d')) - Number(keyboardKeys.has('a')) + sticks.leftX));
    const moving = pitch !== 0 || roll !== 0 || yaw !== 0 || keyboardKeys.has('w') || keyboardKeys.has('s') || sticks.leftY !== 0;
    if (moving && !simulationActive) {
      simulationActive = true;
      scene.setFlying(true);
      updateSimulationButton();
    }
    if (moving && element<HTMLButtonElement>('#autopilot').dataset.enabled === 'true') setAutopilotUi(controller, false);
    if (moving || element<HTMLButtonElement>('#autopilot').dataset.enabled !== 'true') {
      controller.set_manual_controls(pitch, roll, yaw, throttle);
    }
    positionStick('left', yaw, throttle * 2 - 1);
    positionStick('right', roll, pitch);
    const display = document.querySelector<HTMLElement>('#throttle-readout');
    if (display) display.textContent = `THROTTLE  ${Math.round(throttle * 100)}% · ${angleMode ? 'ANGLE' : 'ACRO'} MODE`;
  };
  window.setInterval(set, 50);
  bindJoystickSticks(sticks, value => {
    throttle = Math.max(0, Math.min(1, throttle + value));
  }, set);
  element<HTMLButtonElement>('#flight-mode').addEventListener('click', event => {
    angleMode = !angleMode;
    controller.set_angle_mode(angleMode);
    const button = event.currentTarget as HTMLButtonElement;
    button.textContent = angleMode ? 'ANGLE MODE · ON' : 'ACRO MODE · ON';
    button.setAttribute('aria-pressed', String(angleMode));
    set();
  });
  document.addEventListener('keydown', event => {
    const key = pilotKey(event);
    if (key) { event.preventDefault(); keyboardKeys.add(key); set(); }
  }, true);
  document.addEventListener('keyup', event => {
    const key = pilotKey(event);
    if (key) { event.preventDefault(); keyboardKeys.delete(key); set(); }
  }, true);
  document.addEventListener('focusin', event => {
    if (event.target instanceof HTMLInputElement) { keyboardKeys.clear(); set(); }
  });
  window.addEventListener('blur', () => { keyboardKeys.clear(); sticks.leftX = sticks.leftY = sticks.rightX = sticks.rightY = 0; set(); });
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) { keyboardKeys.clear(); sticks.leftX = sticks.leftY = sticks.rightX = sticks.rightY = 0; set(); }
  });
  element<HTMLButtonElement>('#reset').addEventListener('click', () => {
    keyboardKeys.clear();
    sticks.leftX = sticks.leftY = sticks.rightX = sticks.rightY = 0;
    throttle = 0;
    set();
  });
}

function positionStick(side: string, x: number, y: number): void {
  const stick = element<HTMLElement>(`[data-stick="${side}"]`);
  const knob = stick.querySelector<HTMLElement>('.stick-knob');
  if (!knob) return;
  const radius = stick.getBoundingClientRect().width * 0.36;
  const scale = Math.max(1, Math.hypot(x, y));
  knob.style.transform = `translate(calc(-50% + ${x / scale * radius}px), calc(-50% - ${y / scale * radius}px))`;
}

function bindJoystickSticks(sticks: { leftX: number; leftY: number; rightX: number; rightY: number }, throttle: (delta: number) => void, apply: () => void): void {
  for (const element of document.querySelectorAll<HTMLElement>('[data-stick]')) {
    const left = element.dataset.stick === 'left';
    let previousY = 0;
    const move = (event: PointerEvent) => {
      const rect = element.getBoundingClientRect();
      const radius = rect.width * 0.36;
      const dx = event.clientX - (rect.left + rect.width / 2);
      const dy = event.clientY - (rect.top + rect.height / 2);
      const magnitude = Math.max(1, Math.hypot(dx, dy));
      const x = Math.max(-1, Math.min(1, dx / radius * Math.min(1, radius / magnitude)));
      const y = Math.max(-1, Math.min(1, -dy / radius * Math.min(1, radius / magnitude)));
      if (left) { sticks.leftX = x; sticks.leftY = y; throttle((y - previousY) * 0.5); previousY = y; }
      else { sticks.rightX = x; sticks.rightY = y; }
      apply();
    };
    element.addEventListener('pointerdown', event => {
      event.preventDefault(); element.setPointerCapture(event.pointerId); element.classList.add('active');
      previousY = 0; move(event);
    });
    element.addEventListener('pointermove', event => { if (element.hasPointerCapture(event.pointerId)) move(event); });
    const release = (event: PointerEvent) => {
      if (!element.hasPointerCapture(event.pointerId)) return;
      element.releasePointerCapture(event.pointerId); element.classList.remove('active');
      if (left) { sticks.leftX = sticks.leftY = 0; }
      else { sticks.rightX = sticks.rightY = 0; }
      apply();
    };
    element.addEventListener('pointerup', release);
    element.addEventListener('pointercancel', release);
  }
}

function pilotKey(event: KeyboardEvent): string | undefined {
  const keys: Record<string, string> = { KeyW: 'w', KeyA: 'a', KeyS: 's', KeyD: 'd', KeyI: 'i', KeyJ: 'j', KeyK: 'k', KeyL: 'l' };
  const codeKey = keys[event.code];
  if (codeKey) return codeKey;
  const key = event.key.toLowerCase();
  return ['w', 'a', 's', 'd', 'i', 'j', 'k', 'l'].includes(key) ? key : undefined;
}

function applyGoal(controller: BrowserController, scene: FlightScene, target: { x: number; y: number; z: number }): void {
  controller.set_goal(target.x, target.y, target.z);
  activeGoal = target;
  scene.setMissionTarget(target);
  setPositionInputsForGoal(target.x, target.y, target.z);
}

function bindMissionEditor(controller: BrowserController, scene: FlightScene): void {
  element<HTMLButtonElement>('#set-goal').addEventListener('click', () => {
    const target = { x: boundedInput('#goal-x', 0, MAP_SIZE), y: boundedInput('#goal-y', 0, MAP_SIZE), z: boundedInput('#goal-z', 0, 80) };
    applyGoal(controller, scene, target);
    setRuntime(`MISSION TARGET  ·  ${target.x.toFixed(0)} / ${target.y.toFixed(0)} / ${target.z.toFixed(0)} M`);
  });
}

function bindAutopilot(controller: BrowserController, scene: FlightScene): void {
  element<HTMLButtonElement>('#autopilot').addEventListener('click', () => {
    simulationActive = !simulationActive;
    scene.setFlying(simulationActive);
    setAutopilotUi(controller, simulationActive);
    updateSimulationButton();
    if (!simulationActive) controller.reset_flight_stack();
  });
}

function setAutopilotUi(controller: BrowserController, enabled: boolean): void {
  const button = element<HTMLButtonElement>('#autopilot');
  button.dataset.enabled = String(enabled);
  updateSimulationButton();
  controller.set_autopilot(enabled);
}

function updateSimulationButton(): void {
  element<HTMLButtonElement>('#autopilot').textContent = simulationActive ? 'RETURN TO PLANNING' : 'START SIMULATION';
}

function bindPositionEditor(controller: BrowserController, physics: PhysicsWorld, scene: FlightScene): void {
  setPositionInputs(12, 15, 0.18);
  element<HTMLButtonElement>('#set-position').addEventListener('click', () => {
    physics.teleport({ x: numberInput('#pos-x'), y: numberInput('#pos-y'), z: Math.max(0.18, numberInput('#pos-z')) });
  });
  element<HTMLButtonElement>('#reset').addEventListener('click', () => {
    simulationActive = false;
    scene.setFlying(false);
    controller.reset_flight_stack();
    controller.set_autopilot(false);
    controller.set_goal(DEFAULT_GOAL.x, DEFAULT_GOAL.y, DEFAULT_GOAL.z);
    activeGoal = { ...DEFAULT_GOAL };
    controller.set_cruise_speed(30);
    controller.set_manual_controls(0, 0, 0, 0);
    physics.teleport({ x: 12, y: 15, z: 0.18 });
    scene.setMissionTarget(DEFAULT_GOAL);
  updateReadouts(physics, controller);
    setPositionInputs(12, 15, 0.18);
    setPositionInputsForGoal(DEFAULT_GOAL.x, DEFAULT_GOAL.y, DEFAULT_GOAL.z);
    element<HTMLInputElement>('#speed').value = '30';
    setText('#speed-value', '30.0 m/s');
    updateSafetyDebug(controller);
  const autopilot = element<HTMLButtonElement>('#autopilot');
    autopilot.dataset.enabled = 'false';
    autopilot.textContent = 'START SIMULATION';
  });
}

function setPositionInputsForGoal(x: number, y: number, z: number): void {
  element<HTMLInputElement>('#goal-x').value = String(x);
  element<HTMLInputElement>('#goal-y').value = String(y);
  element<HTMLInputElement>('#goal-z').value = String(z);
}

function bindLayerSwitches(): void {
  for (const button of document.querySelectorAll<HTMLButtonElement>('[data-layer]:not(#safety)')) {
    button.addEventListener('click', () => {
      const active = button.getAttribute('aria-pressed') !== 'true';
      button.setAttribute('aria-pressed', String(active));
    });
  }
}

function bindHudCollapse(): void {
  element<HTMLButtonElement>('#collapse').addEventListener('click', () => {
    const panel = element<HTMLElement>('#hud');
    const collapsed = panel.classList.toggle('collapsed');
    element<HTMLButtonElement>('#collapse').textContent = collapsed ? 'OPEN HUD' : 'COLLAPSE';
  });
}

function bindSafety(controller: BrowserController): void {
  element<HTMLButtonElement>('#safety').addEventListener('click', event => {
    const button = event.currentTarget as HTMLButtonElement;
    const active = button.getAttribute('aria-pressed') !== 'true';
    button.setAttribute('aria-pressed', String(active));
    controller.set_safety(active);
  });
}

function bindSpeed(controller: BrowserController): void {
  element<HTMLInputElement>('#speed').addEventListener('input', event => {
    const speed = Number((event.currentTarget as HTMLInputElement).value);
    controller.set_cruise_speed(speed);
    element<HTMLElement>('#speed-value').textContent = `${speed.toFixed(1)} m/s`;
  });
}

function updateSafetyDebug(controller: BrowserController): void {
  const panel = element<HTMLElement>('#safety-debug');
  const message = simulationActive ? controller.safety_debug() : undefined;
  if (message && panel.hidden) console.warn(message);
  panel.hidden = !message;
  panel.textContent = message ?? '';
}

function updateReadouts(physics: PhysicsWorld, controller: BrowserController): void {
  const position = physics.quad.translation();
  const velocity = physics.quad.linvel();
  const autopilot = element<HTMLButtonElement>('#autopilot').dataset.enabled === 'true';
  setText('#mode', !simulationActive ? 'PLANNING' : autopilot ? 'AUTONOMOUS FLIGHT' : 'PILOT CONTROL');
  setText('#position', `${position.x.toFixed(1)} / ${position.y.toFixed(1)} / ${position.z.toFixed(1)} m`);
  setText('#speed-readout', `${Math.hypot(velocity.x, velocity.y).toFixed(1)} m/s`);
  setText('#goal-distance', `${Math.hypot(position.x - activeGoal.x, position.y - activeGoal.y, position.z - activeGoal.z).toFixed(1)} m`);
  setText('#safety-state', controller.safety_enabled() ? 'PROTECTION ACTIVE' : 'PROTECTION OFF');
}

function layerVisibility(): Record<string, boolean> {
  const active = (name: string) => element<HTMLButtonElement>(`[data-layer="${name}"]`).getAttribute('aria-pressed') === 'true';
  return { quadSensors: active('quad-sensors'), quadMissionPlanner: active('quad-mission-planner'), quadController: active('quad-controller'), safety: active('safety') };
}

function setPositionInputs(x: number, y: number, z: number): void {
  element<HTMLInputElement>('#pos-x').value = x.toFixed(1);
  element<HTMLInputElement>('#pos-y').value = y.toFixed(1);
  element<HTMLInputElement>('#pos-z').value = z.toFixed(1);
}

function axis(keys: Set<string>, positive: string, negative: string): number {
  return Number(keys.has(positive)) - Number(keys.has(negative));
}

function manualForce(keys: Set<string>, magnitude: number): THREE.Vector3 {
  return new THREE.Vector3(axis(keys, 'd', 'a'), axis(keys, 'w', 's'), axis(keys, 'k', 'j'))
    .normalize().multiplyScalar(magnitude);
}

function numberInput(selector: string): number {
  return Number(element<HTMLInputElement>(selector).value) || 0;
}

function boundedInput(selector: string, min: number, max: number): number {
  const input = element<HTMLInputElement>(selector);
  const value = Math.min(max, Math.max(min, numberInput(selector)));
  input.value = String(value);
  return value;
}

function element<T extends HTMLElement>(selector: string): T {
  const result = document.querySelector<T>(selector);
  if (!result) throw new Error(`Missing UI element: ${selector}`);
  return result;
}

function setText(selector: string, text: string): void {
  element<HTMLElement>(selector).textContent = text;
}

function setRuntime(message: string): void {
  if (runtime) runtime.textContent = message;
}

void start().catch(error => setRuntime(`STARTUP FAILED  ·  ${String(error)}`));
