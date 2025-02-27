use std::fmt::Display;

use itertools::Itertools;

use crate::{
    figure::{Figure, PID},
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
#[derive(Debug, Clone, Copy)]
pub(crate) enum TargetedConstraint {
    Element(Number, TargetedElement),
    Parallel(PID, PID, PID),
    Perpendicular(PID, PID, PID),
    Collinear(PID, PID),
    Chirality(Polarity, PID, PID, PID, PID, PID),
}
#[derive(Debug, Clone, Copy)]
enum TargetedElement {
    Distance(PID),
    AngleEnd(PID, PID),
    AngleVertex(PID, PID),
}

impl Constraint {
    pub(crate) fn targets(
        self,
        points: &[PID],
        knowledge: &[bool],
    ) -> Vec<(PID, TargetedConstraint)> {
        match self {
            Constraint::Element(m, element) => match element {
                Element::Distance => distance(m, points, knowledge),
                Element::Angle => angle(m, points, knowledge),
            },
            Constraint::Parallel => parallel(points, knowledge),
            Constraint::Perpendicular => perpendicular(points, knowledge),
            Constraint::Collinear => collinear(points, knowledge),
            Constraint::Chirality(polarities) => chirality(&polarities, points, knowledge),
        }
    }
    pub(crate) fn discretizing(&self) -> bool {
        match self {
            Self::Chirality(..) => false,
            _ => true,
        }
    }
}

fn distance(r: Number, points: &[PID], knowledge: &[bool]) -> Vec<(PID, TargetedConstraint)> {
    let (unknown, known) = match knowledge {
        [false, true] => (points[0], points[1]),
        [true, false] => (points[1], points[0]),
        _ => return Vec::new(),
    };
    vec![(
        unknown,
        TargetedConstraint::Element(r, TargetedElement::Distance(known)),
    )]
}

fn angle(m: Number, points: &[PID], knowledge: &[bool]) -> Vec<(PID, TargetedConstraint)> {
    let (unknown, vertex, known) = match knowledge {
        [false, true, true] => (points[0], points[1], points[2]),
        [true, true, false] => (points[2], points[1], points[0]),
        [true, false, true] => {
            return vec![(
                points[1],
                TargetedConstraint::Element(m, TargetedElement::AngleVertex(points[0], points[2])),
            )]
        }
        _ => return Vec::new(),
    };
    vec![(
        unknown,
        TargetedConstraint::Element(m, TargetedElement::AngleEnd(vertex, known)),
    )]
}

fn parallel(points: &[PID], knowledge: &[bool]) -> Vec<(PID, TargetedConstraint)> {
    let mut known_line = None;
    let mut targets = Vec::new();
    for (line, line_know) in points.chunks_exact(2).zip(knowledge.chunks_exact(2)) {
        let (known, unknown) = match line_know {
            [true, false] => (line[0], line[1]),
            [false, true] => (line[1], line[0]),
            [true, true] if known_line.is_none() => {
                known_line = Some((line[0], line[1]));
                continue;
            }
            _ => continue,
        };
        targets.push((known, unknown));
    }
    let Some((known_0, known_1)) = known_line else {
        return Vec::new();
    };
    let mut constraints = Vec::new();
    for (known, unknown) in targets {
        constraints.push((
            unknown,
            TargetedConstraint::Parallel(known_0, known_1, known),
        ));
    }
    constraints
}

fn perpendicular(points: &[PID], knowledge: &[bool]) -> Vec<(PID, TargetedConstraint)> {
    let mut known_line = None;
    let mut targets = Vec::new();
    for (idx, (line, line_know)) in points
        .chunks_exact(2)
        .zip(knowledge.chunks_exact(2))
        .enumerate()
    {
        let (known, unknown) = match line_know {
            [true, false] => (line[0], line[1]),
            [false, true] => (line[1], line[0]),
            [true, true] if known_line.is_none() => {
                known_line = Some((idx, line[0], line[1]));
                continue;
            }
            _ => continue,
        };
        targets.push((idx, known, unknown));
    }
    let Some((known_idx, known_0, known_1)) = known_line else {
        return Vec::new();
    };
    let mut constraints = Vec::new();
    for (target_idx, known, unknown) in targets {
        constraints.push((
            unknown,
            if (target_idx % 2) == (known_idx % 2) {
                TargetedConstraint::Parallel(known_0, known_1, known)
            } else {
                TargetedConstraint::Perpendicular(known_0, known_1, known)
            }
        ));
    }
    constraints
}

fn collinear(points: &[PID], knowledge: &[bool]) -> Vec<(PID, TargetedConstraint)> {
    let mut line = Vec::new();
    let mut unknown = Vec::new();
    for (&point, &known) in points.into_iter().zip(knowledge.into_iter()) {
        if known {
            line.push(point)
        } else {
            unknown.push(point);
        }
    }
    if line.len() < 2 {
        return vec![];
    }
    let line = TargetedConstraint::Collinear(line[0], line[1]);
    let mut constraints = Vec::new();
    for target in unknown {
        constraints.push((target, line));
    }
    constraints
}

fn chirality(polarities: &[Polarity], points: &[PID], knowledge: &[bool]) -> Vec<(PID, TargetedConstraint)> {
    let mut known_angle = None;
    let mut targets = Vec::new();
    for ((angle, angle_know), pol) in points
        .chunks_exact(3)
        .zip(knowledge.chunks_exact(3))
        .zip(polarities)
    {
        let (known, unknown) = match angle_know {
            [true, true, false] => ((angle[0], angle[1]), angle[2]),
            [true, false, true] => ((angle[0], angle[2]), angle[1]),
            [false, true, true] => ((angle[1], angle[2]), angle[0]),
            [true, true, true] if known_angle.is_none() => {
                known_angle = Some((pol, angle[0], angle[1], angle[2]));
                continue;
            }
            _ => continue,
        };
        targets.push((pol, known, unknown));
    }
    let Some((known_pol, known_0, known_1, known_2)) = known_angle else {
        return Vec::new();
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
                known_0, known_1, known_2, known.0, known.1
            )
        ));
    }
    constraints
}