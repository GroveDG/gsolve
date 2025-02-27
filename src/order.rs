use std::collections::HashMap;

use crate::{constraints::TargetedConstraint, figure::{Figure, CID, PID}};
use std::{collections::HashSet, hash::RandomState, iter::repeat};

use itertools::Itertools;

type HashMapSet<K, V> = HashMap<K, HashSet<V>>;

fn targets(figure: &Figure, cid: CID, known: &HashSet<PID>) -> Vec<PID> {
    let (constraint, c_points) = &figure.constraints[cid];
    let c_points: Vec<_> = c_points
        .iter()
        .map(|p| (known.contains(p), *p))
        .collect();
    constraint.targets(&c_points)
}

fn expand_tree<'a>(
    figure: &Figure,
    known: &HashSet<PID>,
    point: PID,
    support: &mut HashMap<PID, Vec<CID>>,
) -> Vec<PID> {
    let mut new_points = Vec::new();
    for &cid in &figure.points[point] {
        for t in targets(figure, cid, known) {
            if !support.contains_key(&t) {
                support.insert(t, Vec::new());
            }
            let sup_vec = support.get_mut(&t).unwrap();
            // Skip if constraint is already applied.
            if sup_vec.contains(&cid) {
                continue;
            }
            // Add constraint to target.
            sup_vec.push(cid);
            // Skip if non-discretizing.
            if !figure.constraints[cid].0.discretizing() {
                continue;
            }
            // Push if just discretized.
            let mut discrete: usize = 0;
            for cid in sup_vec {
                if figure.constraints[*cid].0.discretizing() {
                    discrete += 1;
                }
            }
            if discrete == 2 {
                new_points.push(t)
            }
        }
    }
    new_points
}

fn compute_tree<'a>(
    root: PID,
    orbiter: PID,
    figure: &Figure,
) -> (Vec<(PID, Vec<CID>)>, HashSet<PID>) {
    let mut support = HashMap::new();
    let mut points: HashSet<PID> = HashSet::from_iter([root]);

    expand_tree(figure, &points, root, &mut support);
    points.insert(orbiter);

    let mut i = 1;
    let mut order = Vec::from_iter([root, orbiter]);
    while i < order.len() {
        let point = order[i];
        // Mark as known.
        points.insert(point);
        // Add new points to queue/order.
        order.append(&mut expand_tree(figure, &points, point, &mut support));
        i += 1;
    }
    (
        order
            .into_iter()
            .map(|p| (p, support.remove(&p).unwrap_or_default()))
            .collect(),
        points,
    )
}

fn root_pairs<'a>(figure: &'a Figure) -> impl Iterator<Item = (PID, PID)> {
    let mut neighbors: HashMapSet<PID, PID> = HashMap::new();
    for p in 0..figure.points.len() {
        let known_points: HashSet<PID, RandomState> = {
            let mut known_points = HashSet::new();
            known_points.insert(p);
            known_points
        };
        let mut n = HashSet::new();
        for &cid in figure.points[p].iter() {
            for t in targets(figure, cid, &known_points) {
                n.insert(t);
            }
        }
        neighbors.insert(p, n);
    }
    neighbors
        .into_iter()
        .map(|(p, targets)| repeat(p).zip(targets))
        .flatten()
}

fn compute_forest(figure: &mut Figure) -> Vec<Vec<(PID, Vec<CID>)>> {
    let mut forest: Vec<(
        Vec<(PID, Vec<CID>)>, // order
        HashSet<PID>,         // contained
    )> = Vec::new();

    for (root, orbiter) in root_pairs(figure) {
        // If this root pair is contained in any tree, skip it.
        if forest
            .iter()
            .any(|(_, p)| p.contains(&root) && p.contains(&orbiter))
        {
            continue;
        }
        // Compute this pair's tree.
        let (order, points) = compute_tree(root, orbiter, &figure);
        // Discard subtrees.
        forest.retain(|(_, p)| !points.is_superset(p));
        // Add this new tree.
        forest.push((order, points));
    }

    let (orders, _): (_, Vec<_>) = forest.into_iter().unzip();
    orders
}

pub fn order_bfs(figure: &mut Figure) -> Vec<(PID, Vec<TargetedConstraint>)> {
    let forest = compute_forest(figure).into_iter().flatten().collect_vec();

    let mut mapping: HashMap<PID, usize> = HashMap::new();
    let mut order: Vec<Vec<CID>> = Vec::new();
    for (id, mut cids) in forest {
        if mapping.contains_key(&id) {
            // incorrect: moves constraints backwards.
            order[mapping[&id]].append(&mut cids);
            panic!("multiple trees")
        } else {
            mapping.insert(id, order.len());

            // Move non-discretizing to back.
            let mut non: Vec<_>;
            (cids, non) = cids.into_iter().partition(|cid| {
                figure
                    .constraints[*cid].0
                    .discretizing()
            });
            cids.append(&mut non);

            order.push(cids);
        }
    }

    order
}
