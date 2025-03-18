use crate::math::vector::Vector;
use itertools::Itertools;

use super::{AboutEq, Number};

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum Geo {
    Point(Vector),
    Ray(Vector, Vector),
    Circle(Vector, Number),
    // All,
}

fn close_on_ray(o: Vector, v: Vector, p: Vector) -> Vector {
    let t = (p - o).dot(v).max(0.);
    along_line(o, v, t)
}

fn along_line(o: Vector, v: Vector, t: Number) -> Vector {
    o + v * t
}

pub(crate) fn meet(g0: Vec<Geo>, g1: Vec<Geo>) -> Vec<Geo> {
    g0.iter()
        .cartesian_product(g1)
        .map(|(&g0, g1)| intersect(g0, g1))
        .concat()
}

fn intersect(g0: Geo, g1: Geo) -> Vec<Geo> {
    match (g0, g1) {
        (g, Geo::Point(p)) | (Geo::Point(p), g) => {
            if dist(p, g).about_zero() {
                // The point is close enough.
                vec![Geo::Point(p)]
            } else {
                // The point is not close.
                vec![]
            }
        }
        (Geo::Ray(o0, v0), Geo::Ray(o1, v1)) => {
            // https://math.stackexchange.com/a/406895
            let b = o1 - o0;
            // Using Cramer's Rule
            let a = Vector { x: v0.x, y: -v1.x }.cross(Vector { x: v0.y, y: -v1.y });
            if a == 0.0 {
                // The rays are parallel.
                return vec![];
            }
            let t0 = Vector { x: b.x, y: -v1.x }.cross(Vector { x: b.y, y: -v1.y }) / a;
            let t1 = Vector { x: v0.x, y: b.x }.cross(Vector { x: v0.y, y: b.y }) / a;
            if t0 < 0. || t1 < 0. {
                // The rays intersect before one of their starts.
                vec![]
            } else {
                // The rays intersect.
                vec![Geo::Point(along_line(o0, v0, t0))]
            }
        }
        (Geo::Circle(c, r), Geo::Ray(o, v)) | (Geo::Ray(o, v), Geo::Circle(c, r)) => {
            // https://w.wiki/A6Jn
            let o_c = o - c;
            let v_o_c = v.dot(o_c);
            let delta = v_o_c.powi(2) - (o_c.mag().powi(2) - r.powi(2));
            if delta.is_sign_negative() {
                // No intersection.
                vec![]
            } else if delta.about_zero() {
                // The line is tangent.
                vec![-v_o_c]
            } else {
                // The line passes through.
                let sqrt_delta = delta.sqrt();
                vec![-v_o_c + sqrt_delta, -v_o_c - sqrt_delta]
            }
            .into_iter()
            .filter_map(|t| {
                if t >= 0. {
                    Some(Geo::Point(along_line(o, v, t)))
                } else {
                    None
                }
            })
            .collect()
        }
        (Geo::Circle(c0, r0), Geo::Circle(c1, r1)) => {
            // https://stackoverflow.com/a/3349134
            let (dir, d) = (c1 - c0).unit_mag();
            // One circle contains the other.
            if d < (r0 - r1).abs() {
                return vec![];
            }
            // The circles are separated.
            if d > (r0 + r1) {
                return vec![];
            }
            let a = (r0.powi(2) - r1.powi(2) + d.powi(2)) / (2.0 * d);
            let c = c0 + dir * a;
            // The circles touch at one point.
            if d.about_eq(r0 + r1) {
                return vec![Geo::Point(c)];
            }
            let h = (r0.powi(2) - a.powi(2)).sqrt();
            let h_v = dir.perp() * h;
            // The circles overlap at two points.
            vec![Geo::Point(c + h_v), Geo::Point(c - h_v)]
        }
        // (Geo::All, g) | (g, Geo::All) => vec![g],
    }
}

pub(crate) fn dist(p: Vector, g: Geo) -> Number {
    match g {
        Geo::Point(p1) => p.dist(p1),
        Geo::Ray (o,v) => p.dist(close_on_ray(o, v, p)),
        Geo::Circle (c,r) => p.dist(c) - r,
        // Geo::All => 0.0,
    }
}

pub(crate) fn choose(g: Geo) -> Vector {
    match g {
        Geo::Point(p) => p,
        Geo::Ray (o,v) => along_line(o, v, 1.0),
        Geo::Circle (c,r) => Vector::POSX * r + c,
        // Geo::All => Vector::ZERO,
    }
}
