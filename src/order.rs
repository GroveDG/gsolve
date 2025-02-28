use std::{collections::HashMap, mem::take};

use crate::{
    constraints::TargetedConstraint,
    figure::{Figure, CID, PID},
};
use std::collections::HashSet;

fn targets(figure: &Figure, cid: CID, known: &HashSet<PID>) -> Vec<(PID, TargetedConstraint)> {
    let (constraint, c_points) = &figure.constraints[cid];
    constraint.targets(&c_points, c_points.iter().map(|p| known.contains(p)))
}

#[derive(Debug, Clone, Default)]
struct PointStatus {
    cids: HashSet<CID>,
    support: Vec<TargetedConstraint>,
    disamb: Vec<TargetedConstraint>,
}
impl PointStatus {
    pub(super) fn add_constraint(&mut self, cid: CID, constraint: TargetedConstraint) -> bool {
        if !self.cids.insert(cid) {
            return false;
        }
        if constraint.discretizing() {
            self.support.push(constraint);
            true
        } else {
            self.disamb.push(constraint);
            false
        }
    }
    pub(super) fn discretized(&self) -> bool {
        self.support.len() >= 2
    }
}

fn expand_tree<'a>(
    figure: &Figure,
    known: &HashSet<PID>,
    point: PID,
    statuses: &mut HashMap<PID, PointStatus>,
) -> Vec<PID> {
    let mut new_points = Vec::new();
    for &cid in &figure.points[point] {
        for (target, constraint) in targets(figure, cid, known) {
            if !statuses.contains_key(&target) {
                statuses.insert(target, PointStatus::default());
            }
            let status = statuses.get_mut(&target).unwrap();
            // Add constraint to target.
            // Skip if constraint is already applied
            // or non-discretizing.
            if status.add_constraint(cid, constraint) {
                continue;
            }
            // Push if just discretized.
            if status.discretized() {
                new_points.push(target)
            }
        }
    }
    new_points
}

fn compute_tree<'a>(
    root: PID,
    orbiter: PID,
    cid: CID,
    constraint: TargetedConstraint,
    figure: &Figure,
) -> (Vec<(PID, Vec<TargetedConstraint>)>, HashSet<PID>) {
    // Set state for root and orbiter
    let mut statuses = {
        let mut orbiter_status = PointStatus::default();
        orbiter_status.add_constraint(cid, constraint);
        HashMap::from_iter([(root, PointStatus::default()), (orbiter, orbiter_status)])
    };
    let mut points: HashSet<PID> = HashSet::from_iter([root, orbiter]);
    let mut i = 1;
    let mut order = Vec::from_iter([root, orbiter]);

    while i < order.len() {
        let point = order[i];
        // Mark as known.
        points.insert(point);
        // Add new points to queue/order.
        order.append(&mut expand_tree(figure, &points, point, &mut statuses));
        i += 1;
    }

    // In order of solving...
    let order = order
        .into_iter()
        .map(|p| {
            // Take status.
            let mut status = statuses.remove(&p).unwrap();
            // Take supporting constraints from status.
            let mut support = take(&mut status.support);
            // Push disambiguating constraints to support.
            support.extend(take(&mut status.disamb));
            (p, support)
        })
        .collect();
    (order, points)
}

fn compute_forest(figure: &mut Figure) -> Vec<Vec<(PID, Vec<TargetedConstraint>)>> {
    let mut forest: Vec<(
        Vec<(PID, Vec<TargetedConstraint>)>, // order
        HashSet<PID>,                        // contained
    )> = Vec::new();

    // For each point as root with their constraints...
    for (root, cids) in figure.points.iter().enumerate() {
        // Assume root as known.
        let known = HashSet::from_iter([root]);
        // For each constraint...
        for cid in cids.iter().copied() {
            // For each target...
            for (orbiter, constraint) in targets(&figure, cid, &known) {
                // Skip if tree would be a subset of an existing tree.
                if forest
                    .iter()
                    .any(|(_, p)| p.contains(&root) && p.contains(&orbiter))
                {
                    continue;
                }
                // Compute this pair's tree.
                let (order, points) = compute_tree(root, orbiter, cid, constraint, &figure);
                // Discard subtrees.
                forest.retain(|(_, p)| !points.is_superset(p));
                // Add this new tree.
                forest.push((order, points));
            }
        }
    }

    let (orders, _): (_, Vec<_>) = forest.into_iter().unzip();
    orders
}

pub fn order_bfs(figure: &mut Figure) -> Vec<(PID, Vec<TargetedConstraint>)> {
    let forest = compute_forest(figure);

    println!("{:?}", forest);
    assert_eq!(forest.len(), 1);

    forest.into_iter().flatten().collect()
}
