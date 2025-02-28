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
        self.support.len() == 2
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
            if !status.add_constraint(cid, constraint) {
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

fn compute_forest(figure: &mut Figure) -> Vec<Vec<(PID, Vec<TargetedConstraint>)>> {
    let mut forest: Vec<(
        Vec<(PID, Vec<TargetedConstraint>)>, // order
        HashSet<PID>,                        // contained
    )> = Vec::new();

    // For each point as root with their constraints...
    for root in 0..figure.points.len() {
        // Assume root as known.
        let order = vec![root];
        let points = {
            let mut points = HashSet::new();
            points.insert(root);
            points
        };

        let mut statuses = HashMap::new();
        expand_tree(figure, &points, root, &mut statuses);
        let orbiters: Vec<_> = statuses.keys().copied().collect();
        statuses.insert(root, Default::default());

        for orbiter in orbiters {
            // Skip if tree would be a subset of an existing tree.
            if forest
                .iter()
                .any(|(_, p)| p.contains(&root) && p.contains(&orbiter))
            {
                continue;
            }

            let mut order = order.clone();
            let mut points = points.clone();
            let mut statuses = statuses.clone();

            order.push(orbiter);
            points.insert(orbiter);

            // Compute tree.
            let mut i = 1;
            while i < order.len() {
                println!("{:?}", order);
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

            // Discard subtrees.
            forest.retain(|(_, p)| !points.is_superset(p));
            // Add this new tree.
            forest.push((order, points));
        }
    }

    let (orders, _): (_, Vec<_>) = forest.into_iter().unzip();
    orders
}

pub fn order_bfs(figure: &mut Figure) -> Vec<(PID, Vec<TargetedConstraint>)> {
    let mut forest = compute_forest(figure);

    for tree in forest.iter() {
        for (point, constraints) in tree {
            println!("{:?} {:?}", point, constraints);
        }
    }
    debug_assert_eq!(forest.len(), 1);
    let order = forest.remove(0);
    debug_assert_eq!(order.len(), figure.points.len());

    order
}
