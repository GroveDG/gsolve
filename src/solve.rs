//! Takes an order from and [order fn][crate::order] and solves for the
//! positions of the points.

use crate::{
    constraints::TargetedConstraint,
    math::{
        geo::{choose, meet, Geo, TwoD},
        Vector,
    },
    PID,
};

fn iter_brute(
    order: &Vec<(PID, Vec<TargetedConstraint>)>,
    positions: &mut Vec<Vector>,
    i: usize,
) -> Result<(), ()> {
    if i >= order.len() {
        return Ok(());
    };
    let (id, geos) = &order[i];
    let geo = geos
        .iter()
        .map(|constraint| constraint.geo(positions))
        .reduce(meet)
        .unwrap_or_else(|| vec![Geo::Two(TwoD::All)]);
    for g in geo {
        positions[*id] = choose(g);
        if iter_brute(order, positions, i + 1).is_ok() {
            return Ok(());
        }
    }
    return Err(());
}

/// Uses a brute force approach.
/// 
/// Solving is straightforward with ordering completed beforehand. All applicable
/// possibility spaces for the next point are intersected. Select an intersection
/// point to use as this point's position. If there are no intersection points, 
/// backtrack.
///
/// A currently unimplemented technique is to select the intersection point based
/// on which is the closest to a future point's current possibility space. This 
/// should keep long chains from wandering off when they will need to loop back 
/// around to another point.
pub fn solve_brute(order: Vec<(PID, Vec<TargetedConstraint>)>) -> Result<Vec<Vector>, String> {
    let mut positions = vec![Vector::ZERO; order.len()];
    if iter_brute(&order, &mut positions, 0).is_ok() {
        Ok(positions)
    } else {
        Err("solve failed".to_string())
    }
}
