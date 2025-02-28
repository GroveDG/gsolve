use crate::{
    figure::PID,
    math::{
        geo::{line_from_points, Geo, OneD, TwoD},
        AboutEq, Number, Vector,
    },
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Polarity {
    Pro,
    Anti,
}

#[derive(Debug, Clone)]
pub enum Constraint {
    Element(Number, Element),
    Parallel,
    Perpendicular,
    Collinear,
    Chirality(Vec<Polarity>),
}
#[derive(Debug, Clone, Copy)]

pub enum Element {
    Distance,
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

#[derive(Debug, Clone, Copy)]
pub enum TargetedConstraint {
    Element(Number, TargetedElement),
    Parallel(PID, PID, PID),
    Perpendicular(PID, PID, PID),
    Collinear(PID, PID),
    Chirality(Polarity, PID, PID, PID, PID, PID),
}
#[derive(Debug, Clone, Copy)]
pub enum TargetedElement {
    Distance(PID),
    AngleEnd(PID, PID),
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
                let measured = if (pos[p1] - pos[p0]).cross(pos[p2] - pos[p0]).signum() == -1. {
                    Polarity::Anti
                } else {
                    Polarity::Pro
                };
                let n = {
                    let mut n = (pos[p4] - pos[p3]).unit();
                    if measured != pol {
                        n = n.perp();
                    }
                    n
                };
                vec![Geo::Two(TwoD::Half { o: pos[p3], n })]
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
                (true, false, true) => ((angle[0], angle[2]), angle[1]),
                (false, true, true) => ((angle[1], angle[2]), angle[0]),
                (true, true, true) if known_angle.is_none() => {
                    known_angle = Some((pol, angle[0], angle[1], angle[2]));
                    continue;
                }
                _ => continue,
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
