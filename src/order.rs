use crate::math::{
    Number, Vector,
    geo::{Geo, choose, meet},
};

/// Internal point IDs.
///
/// Currently [`usize`].
pub type PID = usize;

pub fn distance(point: PID, value: Number) -> Quantity {
    let func = move |pos: Vec<Vector>| vec![Geo::Circle(pos[0], value)];
    Quantity {
        func: Box::new(func),
        points: vec![point],
    }
}
pub fn orientation(point: PID, value: Number) -> Quantity {
    let func = move |pos: Vec<Vector>| vec![Geo::Ray(pos[0], Vector::from_angle(value))];
    Quantity {
        func: Box::new(func),
        points: vec![point],
    }
}

pub struct Quantity {
    func: Box<dyn Fn(Vec<Vector>) -> Vec<Geo>>,
    points: Vec<PID>,
}
#[derive(Default)]
pub struct Order {
    order: Vec<Vec<Quantity>>,
}
impl Order {
    pub fn add_point(&mut self, quantities: Vec<Quantity>) -> PID {
        let pid = self.order.len();
        self.order.push(quantities);
        pid
    }
    fn solve_iter(&self, i: usize, positions: &mut Vec<Vector>) -> Result<(), &'static str> {
        let Some(quantities) = self.order.get(i) else {
            return Ok(());
        };
        let result = quantities
            .iter()
            .map(|q| (q.func)(q.points.iter().map(|p| positions[*p]).collect()))
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
        let mut positions = vec![Vector::ZERO; self.order.len()];
        self.solve_iter(1, &mut positions)?;
        Ok(positions)
    }
}

#[test]
fn rect() {
    use std::f64::consts::PI;

    let mut fig = Order::default();
    let a = fig.add_point(vec![]);
    let b = fig.add_point(vec![
        distance(a, 10.),
        orientation(a, 0.)
    ]);
    let c = fig.add_point(vec![
        distance(b, 5.),
        orientation(b, PI/2.)
    ]);
    let _d = fig.add_point(vec![
        distance(a, 5.),
        distance(c, 10.)
    ]);
    let result = fig.solve().unwrap();
    for p in result {
        println!("{}", p);
    }
}
