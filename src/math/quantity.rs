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

#[derive(Debug, Clone, Copy)]
pub enum Op {
    Add(Number),
    Mul(Number),
    Pow(Number),
}
impl Op {
    pub fn inverse(self) -> Self {
        match self {
            Self::Add(n) => Self::Add(-n),
            Self::Mul(n) => Self::Mul(n.recip()),
            Self::Pow(n) => Self::Pow(n.recip()),
        }
    }
}

pub fn invert_ops(ops: &Vec<Op>) -> Vec<Op> {
    ops.iter().copied().rev().map(Op::inverse).collect()
}

pub fn solve_for_left(l_ops: &Vec<Op>, r_ops: &Vec<Op>) -> Vec<Op> {
    // Ops0(Q0) = Ops1(Q1)
    // Ops1⁻¹(Ops0(Q0)) = Q1
    let mut inv = r_ops.clone();
    inv.append(&mut invert_ops(l_ops));
    inv
    // Ops2(Q1) = ...
    // Ops2(Ops1⁻¹(Ops0(Q0))) = Ops2(Q1) = ...
}
