import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { MAP_SIZE, QUAD_SCALE, type PhysicsWorld } from './physics.js';

export type LayerFeedback = { target: THREE.Vector3; velocity: THREE.Vector3; observed: THREE.Vector3[]; safetyOverriding: boolean };

export class FlightScene {
  readonly scene = new THREE.Scene();
  readonly camera = new THREE.PerspectiveCamera(52, 1, 0.5, 5000);
  readonly renderer: THREE.WebGLRenderer;
  readonly controls: OrbitControls;
  readonly quadMesh: THREE.Group;
  private flying = false;
  private lastQuadPosition = new THREE.Vector3(12, 15, 0.18);
  private readonly obstacles: THREE.Mesh[] = [];
  private readonly heightLabels: THREE.Sprite[] = [];
  private readonly orbitSurfaces: THREE.Object3D[] = [];
  private readonly raycaster = new THREE.Raycaster();
  private readonly orbitMarker = makeOrbitMarker();
  private readonly targetArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(), 1, 0x61e68f);
  private readonly velocityArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(), 1, 0x72d8ff);
  private readonly observationLines = new THREE.Group();
  private readonly safetyRing: THREE.Mesh;
  private readonly targetMarker: THREE.Mesh;
  private readonly goalBeacon = new THREE.Group();
  constructor(canvas: HTMLCanvasElement) {
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.setClearColor(0x11152c);
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.25;
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(400, -300, 700);
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.target.set(400, 400, 0);
    this.controls.enableDamping = true;
    this.controls.enablePan = true;
    this.controls.screenSpacePanning = false;
    this.controls.maxPolarAngle = Math.PI / 2 - 0.035;
    this.controls.minDistance = 2;
    this.controls.maxDistance = 1500;
    canvas.addEventListener('pointerdown', event => {
      if (event.button === 0 && !this.flying && !canvas.dataset.editTool) this.setOrbitPointFromView();
    }, true);
    window.addEventListener('pointerup', () => { this.orbitMarker.visible = false; });
    window.addEventListener('pointercancel', () => { this.orbitMarker.visible = false; });
    const ambient = new THREE.HemisphereLight(0xbacaff, 0x423052, 2.4);
    ambient.position.set(0, 0, 1);
    this.scene.add(ambient);
    this.addSunlight();
    this.addGround();

    this.quadMesh = makeQuad();
    this.orbitSurfaces.push(this.quadMesh);
    this.safetyRing = makeSafetyRing();
    this.safetyRing.visible = false;
    this.targetMarker = new THREE.Mesh(new THREE.OctahedronGeometry(1.4), new THREE.MeshBasicMaterial({ color: 0xffd166 }));
    this.scene.add(this.quadMesh, this.targetArrow, this.velocityArrow, this.observationLines, this.safetyRing, this.targetMarker, this.orbitMarker);
  }

  syncWorld(physics: PhysicsWorld, rpm: Float32Array, dt: number): void {
    this.syncObstacleMeshes(physics);
    const position = physics.quad.translation();
    this.quadMesh.position.set(position.x, position.y, position.z);
    const rotation = physics.quad.rotation();
    this.quadMesh.quaternion.set(rotation.x, rotation.y, rotation.z, rotation.w);
    const rotors = this.quadMesh.userData.rotors as THREE.Group[];
    rotors.forEach((rotor, index) => {
      const direction = index % 2 === 0 ? 1 : -1;
      rotor.rotation.z += direction * (rpm[index] ?? 0) * Math.PI * 2 / 60 * dt;
    });
    if (this.flying) this.followQuad();
    this.lastQuadPosition.copy(this.quadMesh.position);
    this.obstacles.forEach((mesh, index) => {
      this.refreshObstacle(mesh, physics, index);
      const point = physics.obstacles[index]?.body.translation();
      if (point) mesh.position.set(point.x, point.y, point.z);
    });
  }

  setFlying(active: boolean): void {
    this.flying = active;
    this.orbitMarker.visible = false;
    this.controls.enablePan = !active;
    if (active) {
      const forward = new THREE.Vector3(0, 1, 0).applyQuaternion(this.quadMesh.quaternion);
      forward.z = 0;
      if (forward.lengthSq() < 0.01) forward.set(0, 1, 0);
      forward.normalize();
      this.controls.target.copy(this.quadMesh.position).add(new THREE.Vector3(0, 0, 1));
      this.camera.position.copy(this.quadMesh.position).addScaledVector(forward, -5).add(new THREE.Vector3(0, 0, 2.5));
      this.lastQuadPosition.copy(this.quadMesh.position);
    } else {
      this.controls.target.set(400, 400, 0);
      this.camera.position.set(400, -300, 700);
    }
    this.controls.update();
  }

  private followQuad(): void {
    const movement = this.quadMesh.position.clone().sub(this.lastQuadPosition);
    this.camera.position.add(movement);
    this.controls.target.add(movement);
  }

  private setOrbitPointFromView(): void {
    this.raycaster.setFromCamera(new THREE.Vector2(0, 0), this.camera);
    const hit = this.raycaster.intersectObjects(this.orbitSurfaces, true)[0];
    if (hit) {
      this.controls.target.copy(hit.point);
      this.orbitMarker.position.copy(hit.point);
      this.orbitMarker.visible = true;
      this.controls.update();
    }
  }

  setMissionTarget(point: { x: number; y: number; z: number }): void {
    this.targetMarker.position.set(point.x, point.y, point.z);
    this.goalBeacon.children.forEach(child => {
      (child as THREE.Line).geometry.dispose();
      ((child as THREE.Line).material as THREE.Material).dispose();
    });
    this.goalBeacon.clear();
    const beam = line(new THREE.Vector3(point.x, point.y, 0.1),
      new THREE.Vector3(point.x, point.y, point.z), 0xffd166, 0.8);
    const ring = new THREE.Mesh(new THREE.RingGeometry(4.5, 5.5, 48),
      new THREE.MeshBasicMaterial({ color: 0xffd166, side: THREE.DoubleSide }));
    ring.position.set(point.x, point.y, 0.12);
    this.goalBeacon.add(beam, ring);
    this.scene.add(this.goalBeacon);
  }

  showFeedback(origin: THREE.Vector3, feedback: LayerFeedback, visible: Record<string, boolean>): void {
    const distance = this.camera.position.distanceTo(origin);
    const worldPerPixel = 2 * distance * Math.tan(THREE.MathUtils.degToRad(this.camera.fov / 2))
      / Math.max(1, this.renderer.domElement.clientHeight);
    setArrow(this.targetArrow, origin, feedback.target, visible.quadMissionPlanner, worldPerPixel);
    this.velocityArrow.position.copy(origin);
    setArrow(this.velocityArrow, origin, feedback.velocity, visible.quadController, worldPerPixel);
    this.setObservations(origin, feedback.observed, visible.quadSensors);
    this.safetyRing.position.copy(origin);
    this.safetyRing.visible = visible.safety && feedback.safetyOverriding;
    (this.safetyRing.material as THREE.MeshBasicMaterial).color.set(0xff596b);
  }

  render(): void {
    const { clientWidth, clientHeight } = this.renderer.domElement;
    if (clientWidth && clientHeight) {
      this.renderer.setSize(clientWidth, clientHeight, false);
      this.camera.aspect = clientWidth / clientHeight;
      this.camera.updateProjectionMatrix();
    }
    this.controls.update();
    this.updateHeightLabels();
    if (this.orbitMarker.visible) {
      this.orbitMarker.quaternion.copy(this.camera.quaternion);
      this.orbitMarker.scale.setScalar(this.camera.position.distanceTo(this.orbitMarker.position) * 0.018);
    }
    this.renderer.render(this.scene, this.camera);
  }

  private addSunlight(): void {
    const light = new THREE.DirectionalLight(0xffd8bc, 3.2);
    light.position.set(-25, -15, 40);
    this.scene.add(light);
  }

  private addGround(): void {
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(8000, 8000), makeGroundMaterial());
    ground.position.set(MAP_SIZE / 2, MAP_SIZE / 2, 0);
    this.scene.add(ground);
    this.orbitSurfaces.push(ground);
  }

  private syncObstacleMeshes(physics: PhysicsWorld): void {
    while (this.obstacles.length > physics.obstacles.length) {
      const mesh = this.obstacles.pop();
      if (mesh) {
        this.scene.remove(mesh);
        this.orbitSurfaces.splice(this.orbitSurfaces.indexOf(mesh), 1);
        const label = this.heightLabels.pop();
        if (label) {
          label.material.map?.dispose();
          label.material.dispose();
        }
        mesh.geometry.dispose();
        (mesh.material as THREE.Material).dispose();
      }
    }
    while (this.obstacles.length < physics.obstacles.length) {
      const obstacle = physics.obstacles[this.obstacles.length];
      const mesh = new THREE.Mesh(new THREE.BoxGeometry(obstacle.width, obstacle.depth, obstacle.height), buildingMaterial(this.obstacles.length));
      const label = makeHeightLabel(obstacle.height);
      label.position.z = obstacle.height / 2 + 2;
      mesh.add(label);
      this.heightLabels.push(label);
      this.obstacles.push(mesh);
      this.orbitSurfaces.push(mesh);
      this.scene.add(mesh);
    }
  }

  private refreshObstacle(mesh: THREE.Mesh, physics: PhysicsWorld, index: number): void {
    const obstacle = physics.obstacles[index];
    const size = (mesh.geometry as THREE.BoxGeometry).parameters;
    mesh.userData.obstacleIndex = index;
    if (size.width === obstacle.width && size.height === obstacle.depth && size.depth === obstacle.height) return;
    mesh.geometry.dispose();
    mesh.geometry = new THREE.BoxGeometry(obstacle.width, obstacle.depth, obstacle.height);
    const old = this.heightLabels[index];
    mesh.remove(old);
    old.material.map?.dispose();
    old.material.dispose();
    const label = makeHeightLabel(obstacle.height);
    label.position.z = obstacle.height / 2 + 2;
    mesh.add(label);
    this.heightLabels[index] = label;
  }

  private updateHeightLabels(): void {
    const pixelsToWorld = 2 * Math.tan(THREE.MathUtils.degToRad(this.camera.fov / 2))
      / Math.max(1, this.renderer.domElement.clientHeight);
    this.obstacles.forEach((mesh, index) => {
      const label = this.heightLabels[index];
      const size = (mesh.geometry as THREE.BoxGeometry).parameters;
      const delta = this.quadMesh.position.clone().sub(mesh.position);
      const distance = Math.hypot(Math.max(0, Math.abs(delta.x) - size.width / 2),
        Math.max(0, Math.abs(delta.y) - size.height / 2),
        Math.max(0, Math.abs(delta.z) - size.depth / 2));
      label.visible = distance < 60;
      if (!label.visible) return;
      const point = mesh.position.clone().add(label.position);
      const scale = this.camera.position.distanceTo(point) * pixelsToWorld;
      label.scale.set(76 * scale, 26 * scale, 1);
      label.material.opacity = 1 - THREE.MathUtils.smoothstep(distance, 45, 60);
    });
  }

  private setObservations(origin: THREE.Vector3, points: THREE.Vector3[], visible: boolean): void {
    this.observationLines.children.forEach(child => {
      const segment = child as THREE.Line;
      segment.geometry.dispose();
      (segment.material as THREE.Material).dispose();
    });
    this.observationLines.clear();
    if (!visible) return;
    for (const point of points) this.observationLines.add(line(origin, point, 0x56d4e7, 0.42));
  }
}

function makeOrbitMarker(): THREE.Group {
  const marker = new THREE.Group();
  const material = new THREE.MeshBasicMaterial({ color: 0xffd166, depthTest: false });
  marker.add(new THREE.Mesh(new THREE.TorusGeometry(0.7, 0.045, 8, 40), material));
  marker.add(new THREE.Mesh(new THREE.SphereGeometry(0.1, 12, 8), material));
  marker.visible = false;
  marker.renderOrder = 10;
  return marker;
}

function makeQuad(): THREE.Group {
  const group = new THREE.Group();
  group.scale.setScalar(QUAD_SCALE);
  quadPart(group, new THREE.BoxGeometry(0.34, 0.58, 0.035), 0x18252b, [0, 0, -0.035]);
  quadPart(group, new THREE.BoxGeometry(0.30, 0.48, 0.035), 0x263a42, [0, 0, 0.065]);
  quadPart(group, new THREE.BoxGeometry(0.23, 0.34, 0.12), 0x343d48, [0, -0.09, 0.14]);
  for (const y of [-0.18, 0.02]) {
    quadPart(group, new THREE.BoxGeometry(0.25, 0.035, 0.13), 0xff963e, [0, y, 0.14]);
  }
  addCameraCage(group);
  quadPart(group, new THREE.BoxGeometry(0.14, 0.025, 0.04), 0xff3c5e, [0, -0.30, 0.04]);
  const antenna = quadPart(group, new THREE.CylinderGeometry(0.012, 0.012, 0.23, 8), 0x20242a, [0, -0.26, 0.23]);
  antenna.rotation.x = Math.PI / 2;
  quadPart(group, new THREE.SphereGeometry(0.04, 10, 8), 0xff963e, [0, -0.26, 0.35]);
  const positions = [[-0.42, 0.42], [0.42, 0.42], [0.42, -0.42], [-0.42, -0.42]];
  group.userData.rotors = positions.map(([x, y]) => addMotor(group, x, y));
  return group;
}

function quadPart(group: THREE.Group, geometry: THREE.BufferGeometry, color: number, position: number[]): THREE.Mesh {
  const mesh = new THREE.Mesh(geometry, material(color, 0.32));
  mesh.position.set(position[0], position[1], position[2]);
  group.add(mesh);
  return mesh;
}

// +Y is the front: orange cage, exposed lens, and matching front rotor guards.
function addCameraCage(group: THREE.Group): void {
  for (const x of [-0.14, 0.14]) {
    quadPart(group, new THREE.BoxGeometry(0.035, 0.24, 0.23), 0xff963e, [x, 0.34, 0.14]);
  }
  quadPart(group, new THREE.BoxGeometry(0.31, 0.24, 0.025), 0xff963e, [0, 0.34, 0.265]);
  quadPart(group, new THREE.BoxGeometry(0.22, 0.15, 0.16), 0x101820, [0, 0.36, 0.14]);
  quadPart(group, new THREE.CylinderGeometry(0.085, 0.085, 0.075, 24), 0x151d26, [0, 0.465, 0.15]);
  quadPart(group, new THREE.CylinderGeometry(0.061, 0.061, 0.008, 24), 0x358eb7, [0, 0.507, 0.15]);
  quadPart(group, new THREE.BoxGeometry(0.20, 0.025, 0.03), 0xffd776, [0, 0.455, 0.04]);
}

function addMotor(group: THREE.Group, x: number, y: number): THREE.Group {
  const arm = new THREE.Mesh(new THREE.BoxGeometry(0.085, Math.hypot(x, y) * 2, 0.065), material(0x263e46, 0.4));
  arm.rotation.z = -Math.atan2(x, y);
  group.add(arm);
  const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.095, 0.11, 0.1, 16), material(0x152b31, 0.3));
  hub.rotation.x = Math.PI / 2;
  hub.position.set(x, y, 0.05);
  const guard = new THREE.Mesh(new THREE.TorusGeometry(0.275, 0.012, 6, 32), material(y > 0 ? 0xff963e : 0x526575, 0.38));
  guard.position.set(x, y, 0.11);
  const light = new THREE.Mesh(new THREE.SphereGeometry(0.035, 10, 8), new THREE.MeshBasicMaterial({ color: y > 0 ? 0x6effd4 : 0xff7168 }));
  light.position.set(x, y, 0.16);
  const rotor = new THREE.Group();
  rotor.position.set(x, y, 0.11);
  rotor.add(new THREE.Mesh(new THREE.BoxGeometry(0.49, 0.05, 0.016), material(0xcce7e3, 0.22)));
  group.add(hub, guard, light, rotor);
  return rotor;
}

function makeSafetyRing(): THREE.Mesh {
  const ring = new THREE.Mesh(new THREE.TorusGeometry(5, 0.035, 6, 80), new THREE.MeshBasicMaterial({ color: 0xb284ff }));
  ring.rotation.x = Math.PI / 2;
  return ring;
}

function setArrow(arrow: THREE.ArrowHelper, origin: THREE.Vector3, vector: THREE.Vector3, visible: boolean, worldPerPixel: number): void {
  arrow.position.copy(origin);
  arrow.visible = visible && vector.length() > 0.05;
  if (arrow.visible) arrow.setDirection(vector.clone().normalize());
  const length = THREE.MathUtils.clamp(vector.length() * 4, 12, 48) * worldPerPixel;
  arrow.setLength(length, length * 0.25, length * 0.13);
}

function line(start: THREE.Vector3, end: THREE.Vector3, color: number, opacity: number): THREE.Line {
  const geometry = new THREE.BufferGeometry().setFromPoints([start, end]);
  return new THREE.Line(geometry, new THREE.LineBasicMaterial({ color, transparent: true, opacity }));
}

function material(color: number, roughness: number): THREE.MeshStandardMaterial {
  return new THREE.MeshStandardMaterial({ color, roughness, metalness: 0.32 });
}

function buildingMaterial(index: number): THREE.MeshStandardMaterial {
  const palette = [0x6675d9, 0x42b8c7, 0xc67fad, 0xe49a77, 0x8d8ddd];
  return new THREE.MeshStandardMaterial({
    color: palette[index % palette.length], roughness: 0.65, metalness: 0.12,
    emissive: palette[index % palette.length], emissiveIntensity: 0.08,
  });
}

function makeGroundMaterial(): THREE.ShaderMaterial {
  return new THREE.ShaderMaterial({
    vertexShader: `varying vec2 groundPoint;
      void main() {
        vec4 world = modelMatrix * vec4(position, 1.0);
        groundPoint = world.xy;
        gl_Position = projectionMatrix * viewMatrix * world;
      }`,
    fragmentShader: groundFragmentShader(),
  });
}

function groundFragmentShader(): string {
  return `varying vec2 groundPoint;
      float grid(float spacing) {
        vec2 cell = groundPoint / spacing;
        vec2 footprint = max(fwidth(cell), vec2(0.0001));
        vec2 distanceToLine = abs(fract(cell - 0.5) - 0.5) / footprint;
        float fade = 1.0 - smoothstep(0.25, 0.9, max(footprint.x, footprint.y));
        return (1.0 - smoothstep(0.0, 1.2, min(distanceToLine.x, distanceToLine.y))) * fade;
      }
      void main() {
        float stage = step(0.0, groundPoint.x) * step(groundPoint.x, 800.0)
          * step(0.0, groundPoint.y) * step(groundPoint.y, 800.0);
        vec3 base = mix(vec3(0.035, 0.045, 0.085), vec3(0.075, 0.09, 0.16), stage);
        base += grid(10.0) * vec3(0.025, 0.045, 0.065) * stage;
        base += grid(100.0) * vec3(0.075, 0.12, 0.18) * stage;
        float edge = min(min(abs(groundPoint.x), abs(groundPoint.y)),
          min(abs(groundPoint.x - 800.0), abs(groundPoint.y - 800.0)));
        base += (1.0 - smoothstep(0.0, 1.2, edge)) * vec3(0.16, 0.38, 0.42) * stage;
        gl_FragColor = vec4(base, 1.0);
        #include <tonemapping_fragment>
        #include <colorspace_fragment>
      }`;
}

function makeHeightLabel(height: number): THREE.Sprite {
  const canvas = document.createElement('canvas');
  canvas.width = 228;
  canvas.height = 78;
  const context = canvas.getContext('2d');
  if (!context) throw new Error('Height labels require a 2D canvas');
  context.fillStyle = 'rgba(16, 21, 43, 0.92)';
  context.beginPath();
  context.roundRect(2, 2, 224, 74, 18);
  context.fill();
  context.strokeStyle = '#8999cf';
  context.lineWidth = 2;
  context.stroke();
  context.fillStyle = '#edf0ff';
  context.font = '500 36px monospace';
  context.textAlign = 'center';
  context.textBaseline = 'middle';
  context.fillText(`${height.toFixed(1)} m`, 114, 40);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const label = new THREE.Sprite(new THREE.SpriteMaterial({
    map: texture, transparent: true, depthWrite: false, toneMapped: false,
  }));
  label.visible = false;
  return label;
}
