import RAPIER from '@dimforge/rapier3d-compat';

export const MAP_SIZE = 800;
export const QUAD_SCALE = 1.4;
export const DEFAULT_GOAL = { x: 650, y: 650, z: 18 };
const COURSE_COLUMNS = 12;

function generateCourse(): number[][] {
  const cell = MAP_SIZE / COURSE_COLUMNS;
  const buildings: number[][] = [];
  for (let row = 0; row < COURSE_COLUMNS; row++) {
    for (let column = 0; column < COURSE_COLUMNS; column++) {
      const x = (column + 0.5) * cell + (Math.random() - 0.5) * 28;
      const y = (row + 0.5) * cell + (Math.random() - 0.5) * 28;
      if (Math.hypot(x - 12, y - 15) < 40 || Math.hypot(x - DEFAULT_GOAL.x, y - DEFAULT_GOAL.y) < 40) continue;
      buildings.push([x, y, 8 + Math.random() * 10, 18 + Math.random() * 24, 8 + Math.random() * 10]);
    }
  }
  return buildings;
}

export type Vector = { x: number; y: number; z: number };
export type Wrench = { thrust: number; roll: number; pitch: number; yaw: number };
export type Obstacle = { body: RAPIER.RigidBody; radius: number; width: number; depth: number; height: number };

export class PhysicsWorld {
  readonly world: RAPIER.World;
  readonly quad: RAPIER.RigidBody;
  readonly obstacles: Obstacle[] = [];

  private constructor(world: RAPIER.World, quad: RAPIER.RigidBody) {
    this.world = world;
    this.quad = quad;
  }

  static async create(): Promise<PhysicsWorld> {
    await RAPIER.init();
    const world = new RAPIER.World({ x: 0, y: 0, z: -9.81 });
    world.timestep = 1 / 60;
    const physics = new PhysicsWorld(world, makeQuad(world));
    addGround(world);
    for (const [x, y, width, height, depth] of generateCourse()) {
      physics.addBuilding(x, y, width, depth, height);
    }
    return physics;
  }

  addBuilding(x: number, y: number, width: number, depth: number, height: number): void {
    const body = this.world.createRigidBody(
      RAPIER.RigidBodyDesc.fixed().setTranslation(x, y, height / 2),
    );
    this.world.createCollider(RAPIER.ColliderDesc.cuboid(width / 2, depth / 2, height / 2), body);
    this.obstacles.push({ body, radius: Math.hypot(width, depth) / 2, width, depth, height });
  }

  removeObstacle(index: number): void {
    const [obstacle] = this.obstacles.splice(index, 1);
    if (obstacle) this.world.removeRigidBody(obstacle.body);
  }

  removeLastObstacle(): void {
    if (this.obstacles.length) this.removeObstacle(this.obstacles.length - 1);
  }

  updateBuilding(index: number, x: number, y: number, width: number, depth: number, height: number): void {
    const obstacle = this.obstacles[index];
    if (!obstacle) return;
    this.addBuilding(x, y, width, depth, height);
    const replacement = this.obstacles.pop()!;
    this.world.removeRigidBody(obstacle.body);
    this.obstacles[index] = replacement;
  }

  step(wrench: Wrench): void {
    this.quad.resetForces(true);
    this.quad.resetTorques(true);
    const rotation = this.quad.rotation();
    const thrust = rotateVector(rotation, { x: 0, y: 0, z: wrench.thrust });
    this.quad.addForce(thrust, true);
    this.quad.addTorque(rotateVector(rotation, { x: wrench.pitch, y: wrench.roll, z: wrench.yaw }), true);
    this.world.step();
  }

  teleport(position: Vector): void {
    this.quad.resetForces(true);
    this.quad.resetTorques(true);
    this.quad.setTranslation(position, true);
    this.quad.setLinvel({ x: 0, y: 0, z: 0 }, true);
    this.quad.setAngvel({ x: 0, y: 0, z: 0 }, true);
  }

  obstacleData(): Float32Array {
    const values = this.obstacles.flatMap(({ body, radius, height }) => {
      const point = body.translation();
      return [point.x, point.y, point.z, radius, height];
    });
    return new Float32Array(values);
  }
}

function rotateVector(q: { x: number; y: number; z: number; w: number }, v: Vector): Vector {
  const tx = 2 * (q.y * v.z - q.z * v.y);
  const ty = 2 * (q.z * v.x - q.x * v.z);
  const tz = 2 * (q.x * v.y - q.y * v.x);
  return { x: v.x + q.w * tx + q.y * tz - q.z * ty,
    y: v.y + q.w * ty + q.z * tx - q.x * tz,
    z: v.z + q.w * tz + q.x * ty - q.y * tx };
}

function makeQuad(world: RAPIER.World): RAPIER.RigidBody {
  const body = world.createRigidBody(
    RAPIER.RigidBodyDesc.dynamic().setTranslation(12, 15, 0.18)
      .setLinearDamping(0.12).setAngularDamping(5.0),
  );
  world.createCollider(RAPIER.ColliderDesc.cuboid(0.18 * QUAD_SCALE, 0.28 * QUAD_SCALE, 0.1).setMass(1), body);
  for (const [x, y] of [[-0.42, 0.42], [0.42, 0.42], [0.42, -0.42], [-0.42, -0.42]]) {
    world.createCollider(RAPIER.ColliderDesc.cylinder(0.025 * QUAD_SCALE, 0.29 * QUAD_SCALE)
      .setRotation({ x: Math.SQRT1_2, y: 0, z: 0, w: Math.SQRT1_2 })
      .setTranslation(x * QUAD_SCALE, y * QUAD_SCALE, 0.11 * QUAD_SCALE).setMass(0), body);
  }
  world.createCollider(RAPIER.ColliderDesc.cuboid(0.16 * QUAD_SCALE, 0.16 * QUAD_SCALE, 0.14 * QUAD_SCALE)
    .setTranslation(0, 0.36 * QUAD_SCALE, 0.14 * QUAD_SCALE).setMass(0), body);
  world.createCollider(RAPIER.ColliderDesc.cuboid(0.13 * QUAD_SCALE, 0.18 * QUAD_SCALE, 0.07 * QUAD_SCALE)
    .setTranslation(0, -0.09 * QUAD_SCALE, 0.14 * QUAD_SCALE).setMass(0), body);
  return body;
}

function addGround(world: RAPIER.World): void {
  const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, -0.15));
  world.createCollider(RAPIER.ColliderDesc.cuboid(100000, 100000, 0.15), body);
}
