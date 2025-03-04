use itertools::Itertools;

use super::constraints::Constraint;

/// Internal point IDs.
/// 
/// Currently [`usize`].
pub type PID = usize;
/// Internal constraint IDs.
/// 
/// Currently [`usize`].
pub type CID = usize;

/// A geometric figure comprised of points and [constraints][Constraint].
#[derive(Debug, Default, Clone)]
pub struct Figure {
    pub(crate) constraints: Vec<(Constraint, Vec<PID>)>,
    pub(crate) points: Vec<Vec<CID>>,
}

impl Figure {
    /// Add a new point.
    /// 
    /// Useful for creating mappings from point names to [point IDs][PID].
    pub fn new_point(&mut self) -> PID {
        self.points.push(Vec::new());
        self.points.len() - 1
    }

    /// Add a [`Constraint`].
    /// 
    /// Points are specified seprately to allow mapping points to constraints and vice versa.
    /// Points must be specified in an order dependent on the type of [`Constraint`].
    pub fn add_constraint(&mut self, constraint: Constraint, points: Vec<PID>) {
        let cid = self.constraints.len();
        for pid in points.iter().unique() {
            self.points.get_mut(*pid).unwrap().push(cid);
        }
        self.constraints.push((constraint, points));
    }
}
