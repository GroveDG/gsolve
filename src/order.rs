//! Orders a [Figure] into a collection of target points ([PID]) and
//! collections of [TargetedConstraints][TargetedConstraint] which
//! are used in [solving][crate::solve].

use std::hash::RandomState;

use indexmap::{IndexSet, indexset};
use petgraph::graph::NodeIndex;

use crate::constraints::{EqualitySystem, GeoFn, Quantity};
use std::collections::HashSet;

pub fn order_bfs(eq_sys: &EqualitySystem) -> Option<Vec<Box<GeoFn>>> {
    let mut forest: Vec<(&(usize, usize), Vec<usize>, Vec<Vec<Quantity>>)> = Vec::new();
    for (segment, quantities) in eq_sys.segments.iter() {
        let Some(distance) = quantities.distance else {
            continue;
        };
        let Some(orientation) = quantities.orientation else {
            continue;
        };
        for (_, kp, _) in forest.iter() {
            if kp.contains(&segment.0) && kp.contains(&segment.1) {
                continue;
            }
        }

        let mut known_points = indexset! {segment.0, segment.1};
        let mut order: Vec<Vec<Quantity>> = vec![Vec::new(); eq_sys.points.len()];
        let mut known_quantities = indexset! {distance, orientation};
        let mut neighbor_quantities = IndexSet::new();

        for kq in known_quantities {
            for n in eq_sys.graph.neighbors(kq) {
                if !neighbor_quantities.insert(n) {
                    continue;
                }
            }
        }
        for nq in neighbor_quantities {
            let quantity = eq_sys.graph.node_weight(nq)?;
            let points = match quantity {
                Quantity::Distance(a, b) => {
                    if !known_quantities.insert(nq) {
                        continue;
                    }
                }
                Quantity::Orientation(a, b) => {}
            };
            for point in points {
                order[point].push(*quantity);
                if order[point].len() == 2 {
                    known_points.push(point);
                }
            }
        }
        forest.retain(|(s, _, _)| !known_points.contains(&s.0) || !known_points.contains(&s.1));
        forest.push((segment, known_points, order));
    }

    todo!()
}
