use shrewnit::Length;
use vexide::prelude::*;

use crate::Odometry;

pub trait Drivetrain {
    fn tick_odometry(&mut self);
}

pub struct DrivetrainConstants {
    pub wheel_diameter: Length,
    pub wheel_distance: Length
}

impl DrivetrainConstants {
    fn new(wheel_diameter: Length, wheel_distance: Length) -> Self {
        Self { wheel_diameter, wheel_distance }
    }
}

pub struct DifferentialDrivetrain<O> {
    constants: DrivetrainConstants,
    odometry: O,
    left_motor: Motor,
    right_motor: Motor
}

impl<O: Odometry> DifferentialDrivetrain<O> {
    pub fn new(constants: DrivetrainConstants, odometry: O, left_motor: Motor, right_motor: Motor) -> Self {
        Self {
            constants, odometry, left_motor, right_motor
        }
    }
}
