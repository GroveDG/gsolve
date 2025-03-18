use crate::math::{
    Number, Vector,
    geo::{Geo, choose, meet},
};

/// Internal point IDs.
///
/// Currently [`usize`].
pub type PID = usize;

pub fn distance(position: Vector, value: Number) -> Vec<Geo> {
    vec![Geo::Circle(position, value)]
}
pub fn orientation(position: Vector, value: Number) -> Vec<Geo> {
    vec![Geo::Ray(position, Vector::from_angle(value))]
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct Quantity {
    func: fn(Vector, Number) -> Vec<Geo>,
    point: PID,
    value: Number,
}

#[derive(Debug, Clone, Default)]
pub struct Order {
    order: Vec<[Quantity; 2]>,
}
impl Order {
    pub fn add_point(&mut self, quantities: [Quantity; 2]) -> PID {
        self.order.push(quantities);
        let pid = self.order.len();
        pid
    }
    pub fn get(&self, i: usize) -> Option<&[Quantity; 2]> {
        if i == 0 {
            return None;
        }
        self.order.get(i - 1)
    }
    pub fn len(&self) -> usize {
        self.order.len() + 1
    }
    fn solve_iter(&self, i: usize, positions: &mut Vec<Vector>) -> Result<(), &'static str> {
        let Some(quantities) = self.get(i) else {
            return Ok(());
        };
        let result = quantities
            .iter()
            .map(|q| (q.func)(positions[q.point], q.value))
            .reduce(meet)
            .ok_or("Empty Quantities")?;
        for position in result {
            positions[i] = choose(position);
            if self.solve_iter(i + 1, positions).is_ok() {
                return Ok(());
            }
        }
        Err("Unsolved")
    }
    pub fn solve(self) -> Result<Vec<Vector>, &'static str> {
        let mut positions = vec![Vector::ZERO; self.len()];
        self.solve_iter(1, &mut positions)?;
        Ok(positions)
    }
}

#[test]
fn rect() {
    use std::f64::consts::PI;

    let mut fig = Order::default();
    let a = 0;
    let b = fig.add_point([
        Quantity {
            func: distance,
            point: a,
            value: 10.,
        },
        Quantity {
            func: orientation,
            point: a,
            value: 0.,
        }
    ]);
    let c = fig.add_point([
        Quantity {
            func: distance,
            point: b,
            value: 5.,
        },
        Quantity {
            func: orientation,
            point: b,
            value: PI/2.,
        }
    ]);
    let _d = fig.add_point([
        Quantity {
            func: distance,
            point: a,
            value: 5.,
        },
        Quantity {
            func: distance,
            point: c,
            value: 10.,
        }
    ]);
    let result = fig.solve().unwrap();
    for p in result {
        println!("{}", p);
    }
}
