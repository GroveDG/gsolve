use crate::math::{
    Number, Vector,
    geo::{Geo, OneD, TwoD, meet},
};

/// Internal point IDs.
///
/// Currently [`usize`].
pub type PID = usize;
/// Internal quantity IDs.
///
/// Currently [`usize`].
pub type QID = usize;

pub fn distance(position: Vector, value: Number) -> Geo {
    Geo::One(OneD::Circle {
        c: position,
        r: value,
    })
}
pub fn orientation(position: Vector, value: Number) -> Geo {
    Geo::One(OneD::Linear {
        o: position,
        v: Vector::from_angle(value),
        l: 0.,
    })
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct Quantity {
    quantity_fn: fn(Vector, Number) -> Geo,
    points: [PID; 2],
    value: Number,
}

#[derive(Debug, Clone, Default)]
pub struct Figure {
    quantities: Vec<Quantity>,
    points: Vec<Vec<QID>>,
}
impl Figure {
    pub fn new_point(&mut self) -> PID {
        let pid = self.points.len();
        self.points.push(Vec::new());
        pid
    }
    pub fn add_quantity(&mut self, quantity: Quantity) -> QID {
        let qid = self.quantities.len();
        self.quantities.push(quantity);
        for point in quantity.points {
            self.points[point].push(qid);
        }
        qid
    }
    pub fn solve(self) -> Result<Vec<Vector>, &'static str> {
        let mut geos = vec![vec![Geo::Two(TwoD::All)]; self.points.len()];
        geos[0] = vec![Geo::Zero(Vector::ZERO)];
        for quantity in self.quantities {
            let position = &geos[quantity.points[0]];
            if position.len() != 1 {
                return Err("Position not len 1");
            }
            let position = match position[0] {
                Geo::Zero(v) => v,
                _ => return Err("Not position"),
            };
            let geo = (quantity.quantity_fn)(position, quantity.value);
            geos[quantity.points[1]] = meet(geos[quantity.points[1]].clone(), vec![geo]);
        }
        let mut positions = Vec::new();
        for position in geos {
            if position.len() != 1 {
                return Err("Position not len 1 (final)");
            }
            let position = match position[0] {
                Geo::Zero(v) => v,
                _ => return Err("Not position (final)"),
            };
            positions.push(position);
        }
        Ok(positions)
    }
}

#[test]
fn rect() {
    let mut fig = Figure::default();
    let a = fig.new_point();
    let b = fig.new_point();
    // let c = fig.new_point();
    // let d = fig.new_point();
    fig.add_quantity(Quantity {
        quantity_fn: distance,
        points: [a, b],
        value: 10.,
    });
    fig.add_quantity(Quantity {
        quantity_fn: orientation,
        points: [a, b],
        value: 0.,
    });
    let result = fig.solve();
    println!("{:?}", result);
}
