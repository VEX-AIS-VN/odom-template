use core::ops::{Add, AddAssign, Mul, Neg, Sub, SubAssign};

use shrewnit::prelude::*;

#[derive(Clone, Debug)]
/// A translation vector corresponding with an (x, y) absolute position on a field.
pub struct Translation {
    pub x: Length,
    pub y: Length
}

impl Translation {
    pub fn new(x: Length, y: Length) -> Self {
        Self { x, y }
    }

    /// Instantiates a Translation vector entirely of 0 values.
    pub fn zero() -> Self {
        Self { x: Length::from_canonical(0f64), y: Length::from_canonical(0f64) }
    }

    /// Gets the Euclidean norm of the vector, being its magnitude from the (relative) origin.
    pub fn norm(&self) -> Length {
        Length::from_canonical((self.x * self.x + self.y * self.y).canonical().sqrt())
    }

    /// Gets the angle of the vector compared with the origin.
    pub fn heading(&self) -> Angle {
        self.y.canonical().atan2(self.x.canonical()) * Radians
    }

    /// Gets the distance between two Translations.
    pub fn distance(&self, other: Translation) -> Length {
        (self - &other).norm()
    }

    /// Gets the square of the Euclidean norm of the vector. This exists as calculating norm is
    /// more expensive, when it may only be necessary to have the square.
    pub fn norm_squared(&self) -> f64 {
        let x_meters = self.x.to::<Meters>();
        let y_meters = self.y.to::<Meters>();

        x_meters * x_meters + y_meters * y_meters
    }

    /// Rotates a translation around the origin by a certain angle.
    pub fn rotate(&self, angle: Angle) -> Translation {
        let (sin_theta, cos_theta) = angle.to::<Radians>().sin_cos();
        Translation::new(self.x * cos_theta - self.y * sin_theta, self.x * sin_theta + self.y * cos_theta)
    }
}

impl Into<evian::math::Vec2<f64>> for Translation {
    fn into(self) -> evian::math::Vec2<f64> {
        evian::math::Vec2::new(self.x.to::<Meters>(), self.y.to::<Meters>())
    }
}

impl From<Pose> for Translation {
    fn from(value: Pose) -> Self {
        value.translation
    }
}

impl Add<&Translation> for &Translation {
    type Output = Translation;

    fn add(self, rhs: &Translation) -> Self::Output {
        Translation {
            x: self.x + rhs.x,
            y: self.y + rhs.y
        }
    }
}

impl Sub<&Translation> for &Translation {
    type Output = Translation;

    fn sub(self, rhs: &Translation) -> Self::Output {
        self + &-rhs
    }
}

impl Neg for Translation {
    type Output = Translation;

    fn neg(self) -> Self::Output {
        Self { x: self.x * -1f64, y: self.y * -1f64 }
    }
}

impl Neg for &Translation {
    type Output = Translation;

    fn neg(self) -> Self::Output {
        Translation { x: self.x * -1f64, y: self.y * -1f64 }
    }
}

impl Mul<f64> for Translation {
    type Output = Translation;

    fn mul(self, rhs: f64) -> Self::Output {
        Self { x: self.x * rhs, y: self.y * rhs }
    }
}

impl PartialEq<Translation> for Translation {
    fn eq(&self, other: &Translation) -> bool {
        self.x == other.x && self.y == other.y
    }

    fn ne(&self, other: &Translation) -> bool {
        !self.eq(other)
    }
}

#[derive(Clone, Debug)]
/// A pose, being a translation with an angle.
pub struct Pose {
    pub translation: Translation,
    pub angle: Angle
}

impl Pose {
    pub fn new(translation: Translation, angle: Angle) -> Self {
        Self { translation, angle }
    }

    pub fn zero() -> Self {
        Self { translation: Translation::zero(), angle: Angle::from_canonical(0f64) }
    }

    pub fn heading(&self) -> Angle {
        self.translation.heading()
    }

    pub fn norm(&self) -> Length {
        self.translation.norm()
    }

    pub fn norm_squared(&self) -> f64 {
        self.translation.norm_squared()
    }
    
    pub fn rotate(&self, angle: Angle) -> Pose {
        Pose::new(self.translation.rotate(angle), self.angle + angle)
    }
}

impl From<Translation> for Pose {
    fn from(value: Translation) -> Self {
        Self {
            translation: value,
            angle: 0.0.rotations()
        }
    }
}

impl Add<&Pose> for &Pose {
    type Output = Pose;

    fn add(self, rhs: &Pose) -> Self::Output {
        Pose {
            translation: &self.translation + &rhs.translation,
            angle: self.angle + rhs.angle
        }
    }
}

impl Add<Translation> for Pose {
    type Output = Pose;

    fn add(self, rhs: Translation) -> Self::Output {
        self + Pose::from(rhs)
    }
}

impl Neg for Pose {
    type Output = Pose;

    fn neg(self) -> Self::Output {
        Self { translation: -self.translation, angle: self.angle * -1f64 }
    }
}

impl Neg for &Pose {
    type Output = Pose;

    fn neg(self) -> Self::Output {
        Pose { translation: -self.translation.clone(), angle: self.angle * -1f64 }
    }
}

impl Sub<&Pose> for &Pose {
    type Output = Pose;

    fn sub(self, rhs: &Pose) -> Self::Output {
        self + -rhs
    }
}

impl Into<evian::math::Vec2<f64>> for Pose {
    fn into(self) -> evian::math::Vec2<f64> {
        evian::math::Vec2::new(self.translation.x.to::<Meters>(), self.translation.y.to::<Meters>())
    }
}

macro_rules! impl_owned {
    ($x:ident, $op:ident, $y:ident) => {
        impl $x<$y> for $y {
            type Output = $y;

            fn $op(self, other: Self) -> Self::Output {
                (&self).$op(&other)
            }
        }

        impl $x<&$y> for $y {
            type Output = $y;

            fn $op(self, other: &$y) -> Self::Output {
                (&self).$op(other)
            }
        }

        impl $x<$y> for &$y {
            type Output = $y;

            fn $op(self, other: $y) -> Self::Output {
                self.$op(&other)
            }
        }
    };
}

macro_rules! impl_assign {
    ($x:ident, $x_lower:ident, $y:ident, $op:ident, $( $field:ident ),*) => {
        impl $x for $y {
            fn $x_lower(&mut self, other: Self) {
                *self = self.clone().$op(other);
            }
        }
    };
}

impl_owned!(Add, add, Pose);
impl_owned!(Add, add, Translation);
impl_owned!(Sub, sub, Translation);
impl_owned!(Sub, sub, Pose);

impl_assign!(AddAssign, add_assign, Translation, add, x, y);
impl_assign!(AddAssign, add_assign, Pose, add, translation, angle);
impl_assign!(SubAssign, sub_assign, Translation, sub, x, y);
impl_assign!(SubAssign, sub_assign, Pose, sub, translation, angle);
