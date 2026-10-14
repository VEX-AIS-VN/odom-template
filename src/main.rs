#![no_std]
use vexide::prelude::*;

mod units;
mod odometry;
mod drivetrain;
mod motion_control;

trait Odometry {
    fn track_position(&self) {}
}

struct Robot {
    left_motor: Motor,
    right_motor: Motor
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
}
