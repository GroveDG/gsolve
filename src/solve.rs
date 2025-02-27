use std::collections::HashMap;

use crate::{
    figure::{Figure, CID}, math::{
        geo::{choose, meet, Geo, TwoD},
        Vector,
    }, util::locate
};

fn iter_brute(
    order: &Vec<Vec<CID>>,
    figure: &mut Figure,
    positions: &mut Vec<Vector>,
    i: usize,
) -> Result<(), ()> {
    if order.len() <= i {
        return Ok(());
    };
    let geo = order[i]
        .iter()
        .map(|&cid| {
            let (c, c_points) = figure.constraints[cid];
            let t_ind = locate(&c_points, &i).unwrap();
            c.geo(&positions[..i], t_ind)
        })
        .reduce(meet)
        .unwrap_or_else(|| vec![Geo::Two(TwoD::All)]);
    for g in geo {
        positions[i] = choose(g);
        if iter_brute(order, figure, positions, i + 1).is_ok() {
            return Ok(());
        }
    }
    return Err(());
}

pub fn solve_brute(
    figure: &mut Figure,
    order: Vec<Vec<CID>>,
) -> Result<Vec<Vector>, String> {
    let mut positions = vec![Vector::ZERO; order.len()];
    if iter_brute(&order, figure, &mut positions, 0).is_ok() {
        return Ok(positions);
    } else {
        Err("solve failed".to_string())
    }
}
