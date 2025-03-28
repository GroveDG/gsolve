use std::{
    fmt::Display, hash::Hash, ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign}
};

use super::{AboutEq, Number};

/// 2D Vector.
#[allow(missing_docs)]
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct Vector {
    pub x: Number,
    pub y: Number,
}
impl Hash for Vector {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.x.to_be_bytes().hash(state);
        self.y.to_be_bytes().hash(state);
    }
}

#[allow(missing_docs)]
impl Vector {
    pub const ZERO: Vector = Vector { x: 0.0, y: 0.0 };

    pub const POSY: Vector = Vector { x: 0.0, y: 1.0 };
    pub const NEGY: Vector = Vector { x: 0.0, y: -1.0 };
    pub const POSX: Vector = Vector { x: 1.0, y: 0.0 };
    pub const NEGX: Vector = Vector { x: -1.0, y: 0.0 };

    pub const POSINF: Vector = Vector {
        x: Number::INFINITY,
        y: Number::INFINITY,
    };
    pub const NEGINF: Vector = Vector {
        x: Number::NEG_INFINITY,
        y: Number::NEG_INFINITY,
    };
    /// Magnitude.
    pub fn mag(self) -> Number {
        Number::sqrt(self.x * self.x + self.y * self.y)
    }
    /// Distance.
    pub fn dist(self, rhs: Self) -> Number {
        (rhs - self).mag()
    }
    /// Normalized
    pub fn unit(self) -> Vector {
        let d = self.mag();
        self / d
    }
    /// Normalized and return magnitude.
    pub fn unit_mag(self) -> (Vector, Number) {
        let d = self.mag();
        (self / d, d)
    }
    /// Dot product.
    pub fn dot(self, rhs: Self) -> Number {
        self.x * rhs.x + self.y * rhs.y
    }
    /// Cross product.
    pub fn cross(self, rhs: Self) -> Number {
        self.x * rhs.y - self.y * rhs.x
    }
    /// Perpendicular with positive rotation.
    pub fn perp(self) -> Vector {
        Vector {
            x: -self.y,
            y: self.x,
        }
    }
    /// Rotate.
    pub fn rot(self, angle: Number) -> Vector {
        let v = Vector::from_angle(angle);
        Vector {
            x: self.x * v.x - self.y * v.y,
            y: self.x * v.y + self.y * v.x,
        }
    }
    /// Polar of length 1 to cartesian.
    pub fn from_angle(angle: Number) -> Vector {
        Vector {
            x: Number::cos(angle),
            y: Number::sin(angle),
        }
    }
    /// Convert to polar.
    pub fn angle(self) -> Number {
        self.y.atan2(self.x)
    }
    /// Component-wise absolute value.
    pub fn abs(self) -> Self {
        Self {
            x: self.x.abs(),
            y: self.y.abs()
        }
    }
}

impl Into<(Number, Number)> for Vector {
    fn into(self) -> (Number, Number) {
        (self.x, self.y)
    }
}

impl Display for Vector {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "({:.2}, {:.2})", self.x, self.y)
    }
}

impl AboutEq for Vector {
    fn about_eq(self, v: Self) -> bool {
        self.x.about_eq(v.x) && self.y.about_eq(v.y)
    }

    fn about_zero(self) -> bool {
        self.x.about_zero() && self.y.about_zero()
    }
}

impl Add for Vector {
    type Output = Vector;

    fn add(self, rhs: Self) -> Self::Output {
        Vector {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl AddAssign for Vector {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl Add<Number> for Vector {
    type Output = Vector;

    fn add(self, rhs: Number) -> Self::Output {
        Vector {
            x: self.x + rhs,
            y: self.y + rhs,
        }
    }
}

impl AddAssign<Number> for Vector {
    fn add_assign(&mut self, rhs: Number) {
        self.x += rhs;
        self.y += rhs;
    }
}

impl Sub for Vector {
    type Output = Vector;

    fn sub(self, rhs: Self) -> Self::Output {
        Vector {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl SubAssign for Vector {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl Sub<Number> for Vector {
    type Output = Vector;

    fn sub(self, rhs: Number) -> Self::Output {
        Vector {
            x: self.x - rhs,
            y: self.y - rhs,
        }
    }
}

impl SubAssign<Number> for Vector {
    fn sub_assign(&mut self, rhs: Number) {
        self.x -= rhs;
        self.y -= rhs;
    }
}

impl Mul for Vector {
    type Output = Vector;

    fn mul(self, rhs: Self) -> Self::Output {
        Vector {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
        }
    }
}

impl MulAssign for Vector {
    fn mul_assign(&mut self, rhs: Self) {
        self.x *= rhs.x;
        self.y *= rhs.y;
    }
}

impl Mul<Number> for Vector {
    type Output = Vector;

    fn mul(self, rhs: Number) -> Self::Output {
        Vector {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl MulAssign<Number> for Vector {
    fn mul_assign(&mut self, rhs: Number) {
        self.x *= rhs;
        self.y *= rhs;
    }
}

impl Div for Vector {
    type Output = Vector;

    fn div(self, rhs: Self) -> Self::Output {
        Vector {
            x: self.x / rhs.x,
            y: self.y / rhs.y,
        }
    }
}

impl DivAssign for Vector {
    fn div_assign(&mut self, rhs: Self) {
        self.x /= rhs.x;
        self.y /= rhs.y;
    }
}

impl Div<Number> for Vector {
    type Output = Vector;

    fn div(self, rhs: Number) -> Self::Output {
        Vector {
            x: self.x / rhs,
            y: self.y / rhs,
        }
    }
}

impl DivAssign<Number> for Vector {
    fn div_assign(&mut self, rhs: Number) {
        self.x /= rhs;
        self.y /= rhs;
    }
}

impl Neg for Vector {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }
}