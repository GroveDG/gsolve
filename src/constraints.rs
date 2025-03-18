use std::{collections::HashMap, hash::Hash};

use petgraph::graph::{DiGraph, NodeIndex};

use crate::math::{geo::Geo, invert_ops, solve_for_left, Number, Op, Vector};

/// Internal point IDs.
///
/// Currently [`usize`].
pub type PID = usize;
/// Internal constraint IDs.
///
/// Currently [`usize`].
pub type CID = usize;

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum Quantity {
    Constant(Number),
    Distance(PID, PID),
    Orientation(PID, PID),
}
impl Eq for Quantity {}
impl Hash for Quantity {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        core::mem::discriminant(self).hash(state);
        match self {
            Self::Constant(n) => n.to_be_bytes().hash(state),
            Self::Distance(a, b) | Self::Orientation(a, b) => {
                a.hash(state);
                b.hash(state);
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Quantities {
    pub(crate) distance: Option<NodeIndex<u32>>,
    pub(crate) orientation: Option<NodeIndex<u32>>,
}

#[derive(Debug, Clone, Default)]
pub struct EqualitySystem {
    pub(crate) graph: DiGraph<Quantity, Vec<Op>>,
    pub(crate) segments: HashMap<(PID, PID), Quantities>,
    pub(crate) points: Vec<Vec<NodeIndex<u32>>>,
}
impl EqualitySystem {
    pub fn get_or_add_quantity(&mut self, quantity: Quantity) -> NodeIndex<u32> {
        let segment = match quantity {
            Quantity::Constant(_) => {
                return self.graph.add_node(quantity);
            },
            Quantity::Orientation(a, b) | Quantity::Distance(a, b) => (a, b),
        };
        {
            let m = segment.0.max(segment.1);
            if m >= self.points.len() {
                self.points.resize_with(m, Default::default);
            }
        }
        if !self.segments.contains_key(&segment) {
            self.segments.insert(segment, Quantities::default());
        }
        let quantities = self.segments.get_mut(&segment).unwrap();
        let slot = match quantity {
            Quantity::Distance(..) => &mut quantities.distance,
            Quantity::Orientation(..) => &mut quantities.orientation,
        };
        if slot.is_none() {
            let n = self.graph.add_node(quantity);
            *slot = Some(n);
            self.points[segment.0].push(n);
            self.points[segment.1].push(n);
        }
        slot.unwrap()
    }
    pub fn add_equality(&mut self, exprs: [(Quantity, Vec<Op>); 2]) {
        let p0 = self.get_or_add_quantity(exprs[0].0);
        let p1 = self.get_or_add_quantity(exprs[1].0);
        if self.graph.contains_edge(p0, p1) {
            return;
        }
        let ltr = solve_for_left(&exprs[0].1, &exprs[1].1);
        self.graph.add_edge(p0, p1, invert_ops(&ltr));
        self.graph.add_edge(p0, p1, ltr);
    }
}

pub type GeoFn = dyn Fn(Vec<Vector>) -> Vec<Geo>;

// impl TargetedConstraint {
//     pub(crate) fn discretizing(&self) -> bool {
//         match self {
//             Self::Chirality(..) => false,
//             _ => true,
//         }
//     }
//     pub(crate) fn geo(self, pos: &Vec<Vector>) -> Vec<Geo> {
//         match self {
//             TargetedConstraint::Element(m, element) => match element {
//                 TargetedElement::Distance(center) => vec![Geo::One(OneD::Circle {
//                     c: pos[center],
//                     r: m,
//                 })],
//                 TargetedElement::Vertex(p0, p1) => {
//                     let (v, d) = (pos[p1] - pos[p0]).unit_mag();
//                     debug_assert_ne!(d, 0.0);
//                     let r = d / 2.0 / m.sin();
//                     let mid = (pos[p0] + pos[p1]) / 2.0;
//                     let a = r * m.cos();
//                     if a.about_zero() {
//                         vec![Geo::One(OneD::Circle { c: mid, r })]
//                     } else {
//                         let v_a = v.perp() * a;
//                         vec![
//                             Geo::One(OneD::Circle { c: mid + v_a, r }),
//                             Geo::One(OneD::Circle { c: mid - v_a, r }),
//                         ]
//                     }
//                 }
//                 TargetedElement::Orient(base_origin, base_other, origin) => {
//                     let base = (pos[base_other] - pos[base_origin]).unit();
//                     let v_pos = base.rot(m);
//                     let v_neg = base.rot(-m);
//                     let l = if base_origin == origin {
//                         0.0
//                     } else {
//                         Number::NEG_INFINITY
//                     };
//                     vec![
//                         Geo::One(OneD::Linear {
//                             o: pos[origin],
//                             v: v_pos,
//                             l,
//                         }),
//                         Geo::One(OneD::Linear {
//                             o: pos[origin],
//                             v: v_neg,
//                             l,
//                         }),
//                     ]
//                 }
//             },
//             TargetedConstraint::Chirality(pol, p0, p1, p2, p3, p4) => {
//                 let measured = (pos[p1] - pos[p0]).cross(pos[p2] - pos[p1]).signum();
//                 let n = {
//                     let mut n = (pos[p4] - pos[p3]).unit().perp() * measured;
//                     if pol == Polarity::Anti {
//                         n = -n;
//                     }
//                     n
//                 };
//                 vec![Geo::Two(TwoD::Half { o: pos[p4], n })]
//             }
//         }
//     }
// }

// mod targeting {
//     use std::f64::consts::PI;

//     use crate::{PID, math::Number};

//     use super::{Polarity, TargetedConstraint, TargetedElement};
//     use itertools::Itertools;

//     pub(super) fn distance(
//         r: Number,
//         points: &[PID],
//         mut knowledge: impl Iterator<Item = bool>,
//     ) -> Option<Vec<(PID, TargetedConstraint)>> {
//         let (unknown, known) = match knowledge.next_tuple()? {
//             (false, true) => (points[0], points[1]),
//             (true, false) => (points[1], points[0]),
//             _ => return None,
//         };
//         Some(vec![(
//             unknown,
//             TargetedConstraint::Element(r, TargetedElement::Distance(known)),
//         )])
//     }

//     pub(super) fn angle(
//         m: Number,
//         points: &[PID],
//         mut knowledge: impl Iterator<Item = bool>,
//     ) -> Option<Vec<(PID, TargetedConstraint)>> {
//         let (unknown, vertex, known) = match knowledge.next_tuple()? {
//             (false, true, true) => (points[0], points[1], points[2]),
//             (true, true, false) => (points[2], points[1], points[0]),
//             (true, false, true) => {
//                 return Some(vec![(
//                     points[1],
//                     TargetedConstraint::Element(m, TargetedElement::Vertex(points[0], points[2])),
//                 )]);
//             }
//             _ => return None,
//         };
//         Some(vec![(
//             unknown,
//             TargetedConstraint::Element(m, TargetedElement::Orient(vertex, known, vertex)),
//         )])
//     }

//     pub(super) fn parallel(
//         points: &[PID],
//         mut knowledge: impl Iterator<Item = bool>,
//     ) -> Option<Vec<(PID, TargetedConstraint)>> {
//         let mut known_line = None;
//         let mut targets = Vec::new();
//         for line in points.chunks_exact(2) {
//             let (known, unknown) = match knowledge.next_tuple()? {
//                 (true, false) => (line[0], line[1]),
//                 (false, true) => (line[1], line[0]),
//                 (true, true) if known_line.is_none() => {
//                     known_line = Some((line[0], line[1]));
//                     continue;
//                 }
//                 _ => continue,
//             };
//             targets.push((known, unknown));
//         }
//         let Some((known_0, known_1)) = known_line else {
//             return None;
//         };
//         let mut constraints = Vec::new();
//         for (known, unknown) in targets {
//             constraints.push((
//                 unknown,
//                 TargetedConstraint::Element(
//                     0.,
//                     if known == known_0 {
//                         TargetedElement::Orient(known_1, known_0, known)
//                     } else {
//                         TargetedElement::Orient(known_0, known_1, known)
//                     },
//                 ),
//             ));
//         }
//         Some(constraints)
//     }

//     pub(super) fn perpendicular(
//         points: &[PID],
//         mut knowledge: impl Iterator<Item = bool>,
//     ) -> Option<Vec<(PID, TargetedConstraint)>> {
//         let mut known_line = None;
//         let mut targets = Vec::new();
//         for (idx, line) in points.chunks_exact(2).enumerate() {
//             let (known, unknown) = match knowledge.next_tuple()? {
//                 (true, false) => (line[0], line[1]),
//                 (false, true) => (line[1], line[0]),
//                 (true, true) if known_line.is_none() => {
//                     known_line = Some((idx, line[0], line[1]));
//                     continue;
//                 }
//                 _ => continue,
//             };
//             targets.push((idx, known, unknown));
//         }
//         let Some((known_idx, known_0, known_1)) = known_line else {
//             return None;
//         };
//         let mut constraints = Vec::new();
//         for (target_idx, known, unknown) in targets {
//             constraints.push((
//                 unknown,
//                 TargetedConstraint::Element(
//                     if (target_idx % 2) == (known_idx % 2) {
//                         0.
//                     } else {
//                         PI / 2.
//                     },
//                     if known == known_0 {
//                         TargetedElement::Orient(known_1, known_0, known)
//                     } else {
//                         TargetedElement::Orient(known_0, known_1, known)
//                     },
//                 ),
//             ));
//         }

//         Some(constraints)
//     }

//     pub(super) fn collinear(
//         points: &[PID],
//         knowledge: impl Iterator<Item = bool>,
//     ) -> Option<Vec<(PID, TargetedConstraint)>> {
//         let mut line = Vec::new();
//         let mut unknown = Vec::new();
//         for (&point, known) in points.into_iter().zip(knowledge) {
//             if known {
//                 line.push(point)
//             } else {
//                 unknown.push(point);
//             }
//         }
//         if line.len() < 2 {
//             return None;
//         }
//         let line =
//             TargetedConstraint::Element(0., TargetedElement::Orient(line[0], line[1], line[1]));
//         let mut constraints = Vec::new();
//         for target in unknown {
//             constraints.push((target, line));
//         }
//         Some(constraints)
//     }

//     pub(super) fn chirality(
//         polarities: &[Polarity],
//         points: &[PID],
//         mut knowledge: impl Iterator<Item = bool>,
//     ) -> Option<Vec<(PID, TargetedConstraint)>> {
//         let mut known_angle = None;
//         let mut targets = Vec::new();
//         for (angle, pol) in points.chunks_exact(3).zip(polarities) {
//             let (known, unknown) = match knowledge.next_tuple()? {
//                 (true, true, false) => ((angle[0], angle[1]), angle[2]),
//                 (true, false, true) => ((angle[2], angle[0]), angle[1]),
//                 (false, true, true) => ((angle[1], angle[2]), angle[0]),
//                 (true, true, true) => {
//                     known_angle = Some((pol, angle[0], angle[1], angle[2]));
//                     continue;
//                 }
//                 _ => {
//                     continue;
//                 }
//             };
//             targets.push((pol, known, unknown));
//         }
//         let Some((known_pol, known_0, known_1, known_2)) = known_angle else {
//             return None;
//         };
//         let mut constraints = Vec::new();
//         for (pol, known, unknown) in targets {
//             constraints.push((
//                 unknown,
//                 TargetedConstraint::Chirality(
//                     if pol == known_pol {
//                         Polarity::Pro
//                     } else {
//                         Polarity::Anti
//                     },
//                     known_0,
//                     known_1,
//                     known_2,
//                     known.0,
//                     known.1,
//                 ),
//             ));
//         }
//         Some(constraints)
//     }
// }
