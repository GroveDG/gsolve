pub(crate) mod geo;
mod quantity;
mod vector;

pub(crate) use quantity::AboutEq;
pub use quantity::Number;
pub use vector::Vector;
pub(crate) use quantity::{Op, invert_ops, solve_for_left};
