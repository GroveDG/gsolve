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
    let id = order[i].0;
    let geos = &order[i].1;
    let geo = geos
        .iter()
        .map(|constraint| constraint.geo(positions))
        .reduce(meet)
        .unwrap_or_else(|| vec![Geo::Two(TwoD::All)]);
    for g in geo {
        println!("{:?}", g);
        positions[id] = choose(g);
        if iter_brute(order, positions, i + 1).is_ok() {
            return Ok(());
        }
    }
    return Err(());
}

pub fn solve_brute(order: Vec<(PID, Vec<TargetedConstraint>)>) -> Result<Vec<Vector>, String> {
    let mut positions = vec![Vector::ZERO; order.len()];
    if iter_brute(&order, &mut positions, 0).is_ok() {
        Ok(positions)
    } else {
        Err("solve failed".to_string())
    }
}
