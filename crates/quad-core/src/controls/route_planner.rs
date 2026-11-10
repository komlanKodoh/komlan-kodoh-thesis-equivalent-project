use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::{Vec3, WorldObject};

const CELL: f32 = 20.0;
const CLEARANCE: f32 = 10.0;

struct Grid {
    width: usize,
    blocked: Vec<bool>,
}

#[derive(Clone, Copy)]
struct Entry {
    cost: f32,
    node: usize,
}

impl PartialEq for Entry {
    fn eq(&self, other: &Self) -> bool {
        self.node == other.node && self.cost == other.cost
    }
}
impl Eq for Entry {}
impl PartialOrd for Entry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Entry {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.total_cmp(&self.cost)
    }
}

pub(crate) fn route(
    start: Vec3,
    goal: Vec3,
    objects: &[WorldObject],
    map_size: f32,
) -> Option<(Vec3, Option<Vec3>)> {
    if !segment_clear(start, start, objects) {
        return escape_waypoint(start, goal, objects, map_size).map(|point| (point, None));
    }
    if segment_clear(start, goal, objects) {
        return Some((goal, None));
    }
    let grid = Grid::new(map_size, objects);
    let path = grid.search(start, goal)?;
    let turn = (1..path.len())
        .rev()
        .find(|&index| segment_clear(start, path[index], objects))?;
    let next = (turn + 1..path.len())
        .rev()
        .find(|&index| segment_clear(path[turn], path[index], objects));
    Some((path[turn], next.map(|index| path[index])))
}

/// Recover from contact or a clearance zone before asking the grid for a route.
fn escape_waypoint(start: Vec3, goal: Vec3, objects: &[WorldObject], size: f32) -> Option<Vec3> {
    let mut best = None;
    let mut best_cost = f32::INFINITY;
    for distance in [CELL, CELL * 2.0, CELL * 4.0] {
        for index in 0..32 {
            let angle = index as f32 * std::f32::consts::TAU / 32.0;
            let point = start + Vec3::new(angle.cos(), angle.sin(), 0.0) * distance;
            if point.x < 0.0 || point.y < 0.0 || point.x > size || point.y > size {
                continue;
            }
            if !segment_clear(point, point, objects) || !escape_clear(start, point, objects) {
                continue;
            }
            let cost = distance + horizontal_distance(point, goal) * 0.05;
            if cost < best_cost {
                best = Some(point);
                best_cost = cost;
            }
        }
    }
    best
}

fn escape_clear(start: Vec3, end: Vec3, objects: &[WorldObject]) -> bool {
    let motion = Vec3::new(end.x - start.x, end.y - start.y, 0.0);
    objects.iter().all(|object| {
        let away = Vec3::new(
            start.x - object.position.x,
            start.y - object.position.y,
            0.0,
        );
        if away.length() < object.radius_m + CLEARANCE {
            away.dot(motion) >= 0.0
        } else {
            segment_clear(start, end, std::slice::from_ref(object))
        }
    })
}

impl Grid {
    fn new(map_size: f32, objects: &[WorldObject]) -> Self {
        let width = (map_size / CELL).ceil() as usize;
        let blocked = (0..width * width)
            .map(|node| {
                let point = cell_center(node, width);
                objects.iter().any(|object| {
                    horizontal_distance(point, object.position) < object.radius_m + CLEARANCE
                })
            })
            .collect();
        Self { width, blocked }
    }

    fn search(&self, start: Vec3, goal: Vec3) -> Option<Vec<Vec3>> {
        let first = self.index(start);
        let last = self.index(goal);
        let mut scores = vec![f32::INFINITY; self.blocked.len()];
        let mut previous = vec![usize::MAX; self.blocked.len()];
        let mut visited = vec![false; self.blocked.len()];
        let mut open = BinaryHeap::new();
        scores[first] = 0.0;
        open.push(Entry {
            cost: 0.0,
            node: first,
        });
        while let Some(Entry { node, .. }) = open.pop() {
            if visited[node] {
                continue;
            }
            if node == last {
                return Some(reconstruct(node, first, &previous, self.width));
            }
            visited[node] = true;
            self.expand(node, last, &mut scores, &mut previous, &mut open);
        }
        None
    }

    fn expand(
        &self,
        node: usize,
        goal: usize,
        scores: &mut [f32],
        previous: &mut [usize],
        open: &mut BinaryHeap<Entry>,
    ) {
        let x = node % self.width as usize;
        let y = node / self.width as usize;
        for dy in -1..=1 {
            for dx in -1..=1 {
                if dx == 0 && dy == 0 {
                    continue;
                }
                let nx = x as isize + dx;
                let ny = y as isize + dy;
                if !self.can_step(x, y, nx, ny) {
                    continue;
                }
                let next = ny as usize * self.width + nx as usize;
                let cost = scores[node] + CELL * ((dx * dx + dy * dy) as f32).sqrt();
                if cost >= scores[next] {
                    continue;
                }
                scores[next] = cost;
                previous[next] = node;
                open.push(Entry {
                    cost: cost
                        + horizontal_distance(
                            cell_center(next, self.width),
                            cell_center(goal, self.width),
                        ),
                    node: next,
                });
            }
        }
    }

    fn can_step(&self, x: usize, y: usize, nx: isize, ny: isize) -> bool {
        if nx < 0 || ny < 0 || nx >= self.width as isize || ny >= self.width as isize {
            return false;
        }
        let next = ny as usize * self.width + nx as usize;
        if self.blocked[next] {
            return false;
        }
        if nx != x as isize && ny != y as isize {
            !self.blocked[y * self.width + nx as usize]
                && !self.blocked[ny as usize * self.width + x]
        } else {
            true
        }
    }

    fn index(&self, point: Vec3) -> usize {
        let x = (point.x / CELL).floor().clamp(0.0, self.width as f32 - 1.0) as usize;
        let y = (point.y / CELL).floor().clamp(0.0, self.width as f32 - 1.0) as usize;
        y * self.width + x
    }
}

fn reconstruct(mut node: usize, first: usize, previous: &[usize], width: usize) -> Vec<Vec3> {
    let mut path = vec![cell_center(node, width)];
    while node != first {
        node = previous[node];
        path.push(cell_center(node, width));
    }
    path.reverse();
    path
}

fn cell_center(node: usize, width: usize) -> Vec3 {
    Vec3::new(
        (node % width) as f32 * CELL + CELL / 2.0,
        (node / width) as f32 * CELL + CELL / 2.0,
        0.0,
    )
}

fn horizontal_distance(a: Vec3, b: Vec3) -> f32 {
    ((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt()
}

fn segment_clear(start: Vec3, end: Vec3, objects: &[WorldObject]) -> bool {
    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let length_sq = dx * dx + dy * dy;
    objects.iter().all(|object| {
        let t = if length_sq > 0.0 {
            ((object.position.x - start.x) * dx + (object.position.y - start.y) * dy) / length_sq
        } else {
            0.0
        }
        .clamp(0.0, 1.0);
        let nearest = Vec3::new(start.x + t * dx, start.y + t * dy, 0.0);
        horizontal_distance(nearest, object.position) >= object.radius_m + CLEARANCE
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ObjectId;

    #[test]
    fn contact_recovery_exits_clearance_and_then_reaches_the_goal() {
        let mut position = Vec3::new(88.0, 100.0, 10.0);
        let goal = Vec3::new(190.0, 100.0, 10.0);
        let objects = [WorldObject {
            id: ObjectId(1),
            position: Vec3::new(100.0, 100.0, 20.0),
            radius_m: 12.0,
            height_m: 40.0,
            label: "Tower".into(),
        }];
        for _ in 0..600 {
            if horizontal_distance(position, goal) < 1.0 {
                return;
            }
            let (waypoint, _) = route(position, goal, &objects, 200.0).expect("escape or route");
            assert!(escape_clear(position, waypoint, &objects));
            let next = position + (waypoint - position).normalize_or_zero();
            assert!(horizontal_distance(next, objects[0].position) >= 12.0);
            position = next;
        }
        panic!("planner failed to resume its mission after contact");
    }

    #[test]
    fn route_goes_around_tower_instead_of_over_it() {
        let start = Vec3::new(10.0, 100.0, 2.0);
        let goal = Vec3::new(190.0, 100.0, 18.0);
        let tower = WorldObject {
            id: ObjectId(1),
            position: Vec3::new(100.0, 100.0, 20.0),
            radius_m: 12.0,
            height_m: 40.0,
            label: "Tower".into(),
        };
        let (waypoint, next) = route(start, goal, &[tower.clone()], 200.0).unwrap();
        assert_ne!(waypoint, goal);
        assert!(next.is_some());
        assert!(segment_clear(start, waypoint, &[tower]));
    }
}
