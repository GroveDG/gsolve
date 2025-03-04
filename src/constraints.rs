use crate::{
    figure::PID,
    math::{
        geo::{line_from_points, Geo, OneD, TwoD},
        AboutEq, Number, Vector,
    },
};

/// Indicates a 1D direction.
///
/// This is used ambiguously for [`Constraint::Chirality`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Polarity {
    /// Negative to positive.
    Pro,
    /// Positive to negative.
    Anti,
}

/// Constraints that are user-friendly.
///
/// Constraints are specified intuitively to users in an untargeted way.
/// This is as opposed to [`TargetedConstraint`] which is used for solving.
///
/// Used with [Figure::add_constraint][`crate::Figure::add_constraint`].
/// Each constraint has its own point ordering.
#[derive(Debug, Clone)]
pub enum Constraint {
    /// A geometric element with its specified measure.
    /// 
    /// Point order is dependent on the [`Element`].
    Element(Number, Element),
    /// A set of parallel lines.
    /// 
    /// Lines are specified as pairs of points.
    Parallel,
    /// A set of perpendicular lines.
    /// 
    /// Lines are specified as pairs of points. Lines are perpendicular to their neighbors, so lines alterante direction.
    Perpendicular,
    /// A set of points which all lie on the same line.
    /// 
    /// Points are unordered.
    Collinear,
    /// A set of angles with their relative directions.
    /// 
    /// Angles are specified in triples of points. Angles with the same polarity are in the same direction.
    /// Angles with opposite polarity are in opposite directions.
    Chirality(Vec<Polarity>),
}
#[derive(Debug, Clone, Copy)]

/// A geometric element (used with [Constraint]).
pub enum Element {
    /// The distance between two points.
    Distance,
    /// The angle between two coterminal segments.
    Angle,
}

impl Constraint {
    pub(crate) fn targets(
        &self,
        points: &[PID],
        knowledge: impl Iterator<Item = bool>,
    ) -> Vec<(PID, TargetedConstraint)> {
        use targeting::*;

        match self {
            Constraint::Element(m, element) => match element {
                Element::Distance => distance(*m, points, knowledge),
                Element::Angle => angle(*m, points, knowledge),
            },
            Constraint::Parallel => parallel(points, knowledge),
            Constraint::Perpendicular => perpendicular(points, knowledge),
            Constraint::Collinear => collinear(points, knowledge),
            Constraint::Chirality(polarities) => chirality(&polarities, points, knowledge),
        }
        .unwrap_or_else(Vec::new)
    }
}

/// Constraints used for solving.
/// 
/// TargetedConstraints are [Constraints][Constraint] that have been ordered and targeted.
/// This makes them essentially commands which are simply executed in order when solving.
/// 
/// [PIDs][PID] specified are known points.
/// The target is the only unknown point which is specified separately.
/// 
/// TargetedConstraints should be generated with an order function like [order_bfs][crate::order_bfs].
#[derive(Debug, Clone, Copy)]
pub enum TargetedConstraint {
    /// A geometric element with its specified measure.
    Element(Number, TargetedElement),
    /// The first two points form a known line.
    /// The third point and target make a parallel line.
    Parallel(PID, PID, PID),
    /// The first two points form a known line.
    /// The third point and target make a perpendicular line.
    Perpendicular(PID, PID, PID),
    /// The known points make a line which the target also lies on.
    Collinear(PID, PID),
    /// The first three points are a known angle.
    /// The next two points and the target form an angle.
    /// The direction of these angles is the same if the
    /// [Polarity] is [Pro][Polarity::Pro] and opposite
    /// if the [Polarity] is [Anti][Polarity::Anti].
    Chirality(Polarity, PID, PID, PID, PID, PID),
}

/// A target geometric element (used with [TargetedConstraint]).
#[derive(Debug, Clone, Copy)]
pub enum TargetedElement {
    /// The distance between the known point and the target.
    Distance(PID),
    /// The angle between the two known points where the target is the vertex.
    AngleEnd(PID, PID),
    /// The angle between the second known point and the target
    /// where the first known point is the vertex.
    AngleVertex(PID, PID),
}
impl TargetedConstraint {
    pub(crate) fn discretizing(&self) -> bool {
        match self {
            Self::Chirality(..) => false,
            _ => true,
        }
    }
    pub(crate) fn geo(self, pos: &Vec<Vector>) -> Vec<Geo> {
        match self {
            TargetedConstraint::Element(m, element) => match element {
                TargetedElement::Distance(center) => vec![Geo::One(OneD::Circle {
                    c: pos[center],
                    r: m,
                })],
                TargetedElement::AngleEnd(origin, other) => {
                    let base = (pos[other] - pos[origin]).unit();
                    let v_pos = base.rot(m);
                    let v_neg = base.rot(-m);
                    vec![
                        Geo::One(OneD::Linear {
                            o: pos[origin],
                            v: v_pos,
                            l: 0.0,
                        }),
                        Geo::One(OneD::Linear {
                            o: pos[origin],
                            v: v_neg,
                            l: 0.0,
                        }),
                    ]
                }
                TargetedElement::AngleVertex(p0, p1) => {
                    let (v, d) = (pos[p1] - pos[p0]).unit_mag();
                    debug_assert_ne!(d, 0.0);
                    let r = d / 2.0 / m.sin();
                    let mid = (pos[p0] + pos[p1]) / 2.0;
                    let a = r * m.cos();
                    if a.about_zero() {
                        vec![Geo::One(OneD::Circle { c: mid, r })]
                    } else {
                        let v_a = v.perp() * a;
                        vec![
                            Geo::One(OneD::Circle { c: mid + v_a, r }),
                            Geo::One(OneD::Circle { c: mid - v_a, r }),
                        ]
                    }
                }
            },
            TargetedConstraint::Parallel(p0, p1, origin) => vec![Geo::One(OneD::Linear {
                o: pos[origin],
                v: (pos[p1] - pos[p0]).unit(),
                l: Number::NEG_INFINITY,
            })],
            TargetedConstraint::Perpendicular(p0, p1, origin) => vec![Geo::One(OneD::Linear {
                o: pos[origin],
                v: (pos[p1] - pos[p0]).unit().perp(),
                l: Number::NEG_INFINITY,
            })],
            TargetedConstraint::Collinear(p0, p1) => vec![Geo::One(line_from_points(
                pos[p0],
                pos[p1],
                Number::NEG_INFINITY,
            ))],
            TargetedConstraint::Chirality(pol, p0, p1, p2, p3, p4) => {
                let measured = (pos[p1] - pos[p0]).cross(pos[p2] - pos[p1]).signum();
                let n = {
                    let mut n = (pos[p4] - pos[p3]).unit().perp() * measured;
                    if pol == Polarity::Anti {
                        n = -n;
                    }
                    n
                };
                vec![Geo::Two(TwoD::Half { o: pos[p4], n })]
            }
        }
    }
}

mod targeting {
    use crate::{math::Number, PID};

    use super::{Polarity, TargetedConstraint, TargetedElement};
    use itertools::Itertools;

    pub(super) fn distance(
        r: Number,
        points: &[PID],
        mut knowledge: impl Iterator<Item = bool>,
    ) -> Option<Vec<(PID, TargetedConstraint)>> {
        let (unknown, known) = match knowledge.next_tuple()? {
            (false, true) => (points[0], points[1]),
            (true, false) => (points[1], points[0]),
            _ => return None,
        };
        Some(vec![(
            unknown,
            TargetedConstraint::Element(r, TargetedElement::Distance(known)),
        )])
    }

    pub(super) fn angle(
        m: Number,
        points: &[PID],
        mut knowledge: impl Iterator<Item = bool>,
    ) -> Option<Vec<(PID, TargetedConstraint)>> {
        let (unknown, vertex, known) = match knowledge.next_tuple()? {
            (false, true, true) => (points[0], points[1], points[2]),
            (true, true, false) => (points[2], points[1], points[0]),
            (true, false, true) => {
                return Some(vec![(
                    points[1],
                    TargetedConstraint::Element(
                        m,
                        TargetedElement::AngleVertex(points[0], points[2]),
                    ),
                )])
            }
            _ => return None,
        };
        Some(vec![(
            unknown,
            TargetedConstraint::Element(m, TargetedElement::AngleEnd(vertex, known)),
        )])
    }

    pub(super) fn parallel(
        points: &[PID],
        mut knowledge: impl Iterator<Item = bool>,
    ) -> Option<Vec<(PID, TargetedConstraint)>> {
        let mut known_line = None;
        let mut targets = Vec::new();
        for line in points.chunks_exact(2) {
            let (known, unknown) = match knowledge.next_tuple()? {
                (true, false) => (line[0], line[1]),
                (false, true) => (line[1], line[0]),
                (true, true) if known_line.is_none() => {
                    known_line = Some((line[0], line[1]));
                    continue;
                }
                _ => continue,
            };
            targets.push((known, unknown));
        }
        let Some((known_0, known_1)) = known_line else {
            return None;
        };
        let mut constraints = Vec::new();
        for (known, unknown) in targets {
            constraints.push((
                unknown,
                TargetedConstraint::Parallel(known_0, known_1, known),
            ));
        }
        Some(constraints)
    }

    pub(super) fn perpendicular(
        points: &[PID],
        mut knowledge: impl Iterator<Item = bool>,
    ) -> Option<Vec<(PID, TargetedConstraint)>> {
        let mut known_line = None;
        let mut targets = Vec::new();
        for (idx, line) in points.chunks_exact(2).enumerate() {
            let (known, unknown) = match knowledge.next_tuple()? {
                (true, false) => (line[0], line[1]),
                (false, true) => (line[1], line[0]),
                (true, true) if known_line.is_none() => {
                    known_line = Some((idx, line[0], line[1]));
                    continue;
                }
                _ => continue,
            };
            targets.push((idx, known, unknown));
        }
        let Some((known_idx, known_0, known_1)) = known_line else {
            return None;
        };
        let mut constraints = Vec::new();
        for (target_idx, known, unknown) in targets {
            constraints.push((
                unknown,
                if (target_idx % 2) == (known_idx % 2) {
                    TargetedConstraint::Parallel(known_0, known_1, known)
                } else {
                    TargetedConstraint::Perpendicular(known_0, known_1, known)
                },
            ));
        }
        Some(constraints)
    }

    pub(super) fn collinear(
        points: &[PID],
        knowledge: impl Iterator<Item = bool>,
    ) -> Option<Vec<(PID, TargetedConstraint)>> {
        let mut line = Vec::new();
        let mut unknown = Vec::new();
        for (&point, known) in points.into_iter().zip(knowledge) {
            if known {
                line.push(point)
            } else {
                unknown.push(point);
            }
        }
        if line.len() < 2 {
            return None;
        }
        let line = TargetedConstraint::Collinear(line[0], line[1]);
        let mut constraints = Vec::new();
        for target in unknown {
            constraints.push((target, line));
        }
        Some(constraints)
    }

    pub(super) fn chirality(
        polarities: &[Polarity],
        points: &[PID],
        mut knowledge: impl Iterator<Item = bool>,
    ) -> Option<Vec<(PID, TargetedConstraint)>> {
        let mut known_angle = None;
        let mut targets = Vec::new();
        for (angle, pol) in points.chunks_exact(3).zip(polarities) {
            let (known, unknown) = match knowledge.next_tuple()? {
                (true, true, false) => ((angle[0], angle[1]), angle[2]),
                (true, false, true) => ((angle[2], angle[0]), angle[1]),
                (false, true, true) => ((angle[1], angle[2]), angle[0]),
                (true, true, true) => {
                    known_angle = Some((pol, angle[0], angle[1], angle[2]));
                    continue;
                }
                _ => {
                    continue;
                }
            };
            targets.push((pol, known, unknown));
        }
        let Some((known_pol, known_0, known_1, known_2)) = known_angle else {
            return None;
        };
        let mut constraints = Vec::new();
        for (pol, known, unknown) in targets {
            constraints.push((
                unknown,
                TargetedConstraint::Chirality(
                    if pol == known_pol {
                        Polarity::Pro
                    } else {
                        Polarity::Anti
                    },
                    known_0,
                    known_1,
                    known_2,
                    known.0,
                    known.1,
                ),
            ));
        }
        Some(constraints)
    }
}
