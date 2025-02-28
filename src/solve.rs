use std::collections::HashMap;

use crate::{
    constraints::TargetedConstraint,
    figure::{Figure, CID},
    math::{
        geo::{choose, meet, Geo, TwoD},
        Vector,
    },
    util::locate,
    PID,
};

fn iter_brute(
    order: &Vec<(PID, Vec<TargetedConstraint>)>,
    positions: &mut Vec<Vector>,
    i: usize,
) -> Result<(), ()> {
    if order.len() <= i {
        return Ok(());
    };
    let geo = order[i]
        .1
        .iter()
        .map(|constraint| constraint.geo(positions))
        .reduce(meet)
        .unwrap_or_else(|| vec![Geo::Two(TwoD::All)]);
    for g in geo {
        positions[i] = choose(g);
        if iter_brute(order, positions, i + 1).is_ok() {
            return Ok(());
        }
    }
    return Err(());
}

pub fn solve_brute(order: Vec<(PID, Vec<TargetedConstraint>)>) -> Result<Vec<Vector>, String> {
    let mut positions = vec![Vector::ZERO; order.len()];
    if iter_brute(&order, &mut positions, 0).is_ok() {
        return Ok(positions);
    } else {
        Err("solve failed".to_string())
    }
}
