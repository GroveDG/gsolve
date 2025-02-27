use std::collections::HashMap;

use super::constraints::Constraint;

pub type PID = usize;
pub type CID = usize;

#[derive(Debug, Default)]
pub struct Figure {
    pub(crate) constraints: Vec<(Constraint, Vec<PID>)>,
    pub(crate) points: Vec<Vec<CID>>,
}

impl Figure {
    pub fn new_point(&mut self) -> PID {
        self.points.push(Vec::new());
        self.points.len() - 1
    }

    pub fn add_constraint(&mut self, constraint: Constraint, points: Vec<PID>) {
        let cid = self.constraints.len();
        for pid in &points {
            self.points.get_mut(*pid).unwrap().push(cid);
        }
        self.constraints.push((constraint, points));
    }

    pub fn map_ids(&mut self, mapping: &HashMap<PID, PID>) {
        for (_, points) in self.constraints.iter_mut() {
            for point in points {
                *point = mapping[point];
            }
        }
        let points = std::mem::take(&mut self.points);
        let mut ordered_points = vec![None; self.points.len()];
        for (i, point) in points.into_iter().enumerate() {
            ordered_points[i] = Some(point);
        }
        self.points = ordered_points.into_iter().map(|p| p.unwrap()).collect();
    }
}
