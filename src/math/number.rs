/// Numerical type.
pub type Number = f64;

pub(crate) const EPSILON: Number = 1e-9;

pub(crate) trait AboutEq {
    fn about_eq(self, v: Self) -> bool;
    fn about_zero(self) -> bool;
}

impl AboutEq for Number {
    fn about_eq(self, v: Self) -> bool {
        (v - self).abs() <= EPSILON
    }
    fn about_zero(self) -> bool {
        self.abs() <= EPSILON
    }
}