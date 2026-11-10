import * as THREE from 'three';
import { MAP_SIZE, type PhysicsWorld } from './physics.js';
import type { FlightScene } from './scene.js';

type Tool = 'orbit' | 'select' | 'goal' | 'building';
type Goal = { x: number; y: number; z: number };
type Building = { x: number; y: number; width: number; depth: number; height: number };

export class SceneEditor {
  private tool: Tool = 'orbit';
  private selected = -1;
  private readonly ray = new THREE.Raycaster();
  private readonly plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
  private readonly preview = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1),
    new THREE.MeshBasicMaterial({ color: 0x9bafff, transparent: true, opacity: 0.35, depthWrite: false }));
  private readonly outline = new THREE.Box3Helper(new THREE.Box3(), 0xb9fff0);
  private pointer?: { id: number; x: number; y: number; building: number; offset: THREE.Vector2 };
  private active = false;
  private readonly history: Building[][] = [];

  constructor(private physics: PhysicsWorld, private scene: FlightScene,
    private setGoal: (goal: Goal) => void, private canEdit: () => boolean) {
    scene.scene.add(this.preview, this.outline);
    this.preview.visible = this.outline.visible = false;
    this.bindTools();
    this.bindInspector();
    this.bindPointer();
  }

  private bindTools(): void {
    document.querySelectorAll<HTMLButtonElement>('[data-tool]').forEach(button => {
      button.addEventListener('click', () => this.setTool(button.dataset.tool as Tool));
    });
    ui('#pick-goal').onclick = () => this.setTool('goal');
    ui('#place-building').onclick = () => this.setTool('building');
    document.querySelectorAll<HTMLButtonElement>('[data-preset]').forEach(button => {
      button.onclick = () => {
        const [width, depth, height] = button.dataset.preset!.split(',').map(Number);
        write('stage-width', width); write('stage-depth', depth); write('stage-height', height);
        this.setTool('building');
      };
    });
    window.addEventListener('keydown', event => this.keydown(event));
    ui('#undo-edit').onclick = () => this.undo();
  }

  private setTool(tool: Tool): void {
    if (!this.canEdit() && tool !== 'orbit') return;
    this.tool = tool;
    const canvas = this.scene.renderer.domElement;
    canvas.dataset.editTool = tool === 'orbit' ? '' : tool;
    canvas.style.cursor = tool === 'orbit' ? 'grab' : 'crosshair';
    this.scene.controls.mouseButtons.LEFT = tool === 'orbit' ? THREE.MOUSE.ROTATE : null;
    this.scene.controls.touches.ONE = tool === 'orbit' ? THREE.TOUCH.ROTATE : null;
    this.preview.visible = false;
    document.querySelectorAll<HTMLButtonElement>('[data-tool]').forEach(button => {
      button.setAttribute('aria-pressed', String(button.dataset.tool === tool));
    });
    const messages = { orbit: 'Drag to orbit · Right drag to pan · Scroll to zoom',
      select: 'Click a building to select · Drag it or click the ground to move · Esc to exit',
      goal: 'Click the ground to place your goal · Altitude comes from Target Z · Esc to exit',
      building: 'Click to place buildings · Choose a preset or adjust dimensions · Esc to exit' };
    this.status(messages[tool]);
  }

  private bindInspector(): void {
    ui('#apply-building').onclick = () => {
      if (!this.canEdit() || this.selected < 0) return;
      this.remember();
      this.replace(this.selected, dimensions());
      this.select(this.selected);
      this.status('Building updated');
    };
    ui('#delete-building').onclick = () => this.deleteSelected();
    ui('#add-obstacle').onclick = () => {
      if (!this.canEdit()) return;
      this.remember();
      this.add(dimensions());
    };
    ui('#remove-obstacle').onclick = () => {
      if (!this.canEdit() || !this.physics.obstacles.length) return;
      this.remember();
      this.physics.removeLastObstacle();
      this.select(-1);
    };
  }

  private bindPointer(): void {
    const canvas = this.scene.renderer.domElement;
    canvas.addEventListener('pointerdown', event => {
      if (event.button === 0 && this.tool !== 'orbit' && this.canEdit()) {
        event.stopImmediatePropagation(); this.down(event);
      }
    }, true);
    canvas.addEventListener('pointermove', event => {
      if (this.pointer?.id === event.pointerId) event.stopImmediatePropagation();
      this.move(event);
    }, true);
    canvas.addEventListener('pointerup', event => {
      if (this.pointer?.id === event.pointerId) event.stopImmediatePropagation();
      this.up(event);
    }, true);
    canvas.addEventListener('pointercancel', () => this.cancel());
    canvas.addEventListener('pointerleave', () => { if (!this.pointer) this.preview.visible = false; });
  }

  private down(event: PointerEvent): void {
    if (event.button !== 0 || this.tool === 'orbit' || !this.canEdit()) return;
    const point = this.ground(event);
    if (!point) return;
    const building = this.tool === 'select' ? this.hitBuilding() : -1;
    if (building >= 0) this.select(building);
    const body = this.physics.obstacles[building]?.body.translation();
    const offset = body ? new THREE.Vector2(body.x - point.x, body.y - point.y) : new THREE.Vector2();
    this.pointer = { id: event.pointerId, x: event.clientX, y: event.clientY, building, offset };
    this.scene.renderer.domElement.setPointerCapture(event.pointerId);
  }

  private move(event: PointerEvent): void {
    if (!this.canEdit() || this.tool === 'orbit') return;
    const point = this.ground(event);
    if (!point) { this.preview.visible = false; return; }
    if (this.pointer?.building !== undefined && this.pointer.building >= 0) point.add(
      new THREE.Vector3(this.pointer.offset.x, this.pointer.offset.y, 0));
    this.showPreview(point);
  }

  private up(event: PointerEvent): void {
    const pointer = this.pointer;
    if (!pointer || pointer.id !== event.pointerId) return;
    const point = this.ground(event);
    const dragged = Math.hypot(event.clientX - pointer.x, event.clientY - pointer.y) > 4;
    if (point && this.canEdit()) {
      point.x += pointer.offset.x; point.y += pointer.offset.y;
      if (this.tool === 'goal' && !dragged) this.placeGoal(point);
      if (this.tool === 'building' && !dragged) { this.remember(); this.add(this.at(point)); }
      if (this.tool === 'select' && this.selected >= 0 && (pointer.building < 0 || dragged)) {
        this.remember();
        this.replace(this.selected, this.at(point));
        this.select(this.selected);
        this.status('Building moved · Adjust dimensions in the stage editor');
      }
    }
    this.cancel();
  }

  private cancel(): void {
    const canvas = this.scene.renderer.domElement;
    if (this.pointer && canvas.hasPointerCapture(this.pointer.id)) canvas.releasePointerCapture(this.pointer.id);
    this.pointer = undefined;
    this.preview.visible = false;
  }

  private ground(event: PointerEvent): THREE.Vector3 | null {
    const rect = this.scene.renderer.domElement.getBoundingClientRect();
    const cursor = new THREE.Vector2((event.clientX - rect.left) / rect.width * 2 - 1,
      -(event.clientY - rect.top) / rect.height * 2 + 1);
    this.ray.setFromCamera(cursor, this.scene.camera);
    const point = this.ray.ray.intersectPlane(this.plane, new THREE.Vector3());
    if (!point || point.x < 0 || point.y < 0 || point.x > MAP_SIZE || point.y > MAP_SIZE) return null;
    return point;
  }

  private hitBuilding(): number {
    const meshes = this.scene.scene.children.filter(mesh => Number.isInteger(mesh.userData.obstacleIndex));
    return this.ray.intersectObjects(meshes, false)[0]?.object.userData.obstacleIndex ?? -1;
  }

  private at(point: THREE.Vector3): Building {
    const size = dimensions();
    return { ...size, x: this.coordinate(point.x, size.width / 2), y: this.coordinate(point.y, size.depth / 2) };
  }

  private coordinate(value: number, margin = 0): number {
    const snap = (ui('#editor-snap') as HTMLInputElement).checked;
    return Math.max(margin, Math.min(MAP_SIZE - margin, snap ? Math.round(value / 5) * 5 : value));
  }

  private showPreview(point: THREE.Vector3): void {
    if (this.tool === 'select' && this.selected < 0) return;
    const goal = this.tool === 'goal';
    const size = this.at(point);
    const z = read('goal-z', 0, 80);
    this.preview.position.set(goal ? this.coordinate(point.x) : size.x,
      goal ? this.coordinate(point.y) : size.y, goal ? z : size.height / 2);
    this.preview.scale.set(goal ? 3 : size.width, goal ? 3 : size.depth, goal ? 3 : size.height);
    this.preview.material.color.set(goal ? 0xffd166 : 0x9bafff);
    this.preview.visible = true;
  }

  private placeGoal(point: THREE.Vector3): void {
    const goal = { x: this.coordinate(point.x), y: this.coordinate(point.y), z: read('goal-z', 0, 80) };
    this.setGoal(goal);
    this.status(`Goal placed at ${goal.x.toFixed(0)}, ${goal.y.toFixed(0)} · Altitude ${goal.z} m`);
  }

  private add(size: Building): void {
    this.physics.addBuilding(size.x, size.y, size.width, size.depth, size.height);
    this.select(this.physics.obstacles.length - 1);
    this.status('Building placed · Click to place another, or Select / move to reposition');
  }

  private replace(index: number, size: Building): void {
    this.physics.updateBuilding(index, size.x, size.y, size.width, size.depth, size.height);
  }

  private select(index: number): void {
    this.selected = index;
    const obstacle = this.physics.obstacles[index];
    ui('#selection-info').textContent = obstacle ? `Building ${index + 1} selected` : 'New building';
    for (const id of ['apply-building', 'delete-building']) (ui(`#${id}`) as HTMLButtonElement).disabled = !obstacle;
    this.outline.visible = !!obstacle;
    if (!obstacle) return;
    const point = obstacle.body.translation();
    write('stage-x', point.x); write('stage-y', point.y);
    write('stage-width', obstacle.width); write('stage-depth', obstacle.depth); write('stage-height', obstacle.height);
    this.outline.box.setFromCenterAndSize(new THREE.Vector3(point.x, point.y, point.z),
      new THREE.Vector3(obstacle.width + 0.3, obstacle.depth + 0.3, obstacle.height + 0.3));
  }

  private deleteSelected(): void {
    if (!this.canEdit() || this.selected < 0) return;
    this.remember();
    this.physics.removeObstacle(this.selected);
    this.select(-1);
    this.status('Building deleted · Undo to restore');
  }

  private remember(): void {
    this.history.push(this.physics.obstacles.map(obstacle => ({ ...obstacle.body.translation(),
      width: obstacle.width, depth: obstacle.depth, height: obstacle.height })));
    if (this.history.length > 30) this.history.shift();
    (ui('#undo-edit') as HTMLButtonElement).disabled = false;
  }

  private undo(): void {
    if (!this.canEdit()) return;
    const previous = this.history.pop();
    if (!previous) return;
    while (this.physics.obstacles.length) this.physics.removeLastObstacle();
    previous.forEach(size => this.physics.addBuilding(size.x, size.y, size.width, size.depth, size.height));
    this.select(-1);
    (ui('#undo-edit') as HTMLButtonElement).disabled = !this.history.length;
    this.status('Building edit undone');
  }

  private keydown(event: KeyboardEvent): void {
    if ((event.target as HTMLElement).matches('input, textarea, select, [contenteditable]')) return;
    if (event.key === 'Escape') { this.cancel(); this.setTool('orbit'); this.select(-1); }
    if (event.key === 'Delete' || event.key === 'Backspace') {
      if (this.canEdit() && this.selected >= 0) { event.preventDefault(); this.deleteSelected(); }
    }
    if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === 'z' && this.canEdit()) {
      event.preventDefault(); this.undo();
    }
  }

  update(): void {
    const active = !this.canEdit();
    if (active === this.active) return;
    this.active = active;
    if (active) { this.cancel(); this.setTool('orbit'); this.select(-1); }
    document.querySelectorAll<HTMLButtonElement>('[data-tool]:not([data-tool="orbit"]), [data-preset], #pick-goal, #place-building, #add-obstacle, #remove-obstacle')
      .forEach(button => { button.disabled = active; });
    this.status(active ? 'Simulation running · Return to planning to edit the scene' : 'Planning mode · Choose a tool to edit the scene');
  }

  private status(message: string): void { ui('#editor-status').textContent = message; }
}

function ui(selector: string): HTMLElement {
  const element = document.querySelector<HTMLElement>(selector);
  if (!element) throw new Error(`Missing editor element: ${selector}`);
  return element;
}

function read(id: string, min: number, max: number): number {
  const input = ui(`#${id}`) as HTMLInputElement;
  const value = Math.min(max, Math.max(min, Number(input.value) || min));
  input.value = String(value);
  return value;
}

function write(id: string, value: number): void {
  (ui(`#${id}`) as HTMLInputElement).value = String(Math.round(value * 10) / 10);
}

function dimensions(): Building {
  return { x: read('stage-x', 0, MAP_SIZE), y: read('stage-y', 0, MAP_SIZE),
    width: read('stage-width', 1, 40), depth: read('stage-depth', 1, 40), height: read('stage-height', 1, 80) };
}
