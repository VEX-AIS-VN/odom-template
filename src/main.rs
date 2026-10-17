use shrewnit::Length;
use vexide::prelude::*;

#[derive(Clone)]
struct DrivetrainConstants {
    wheel_diameter: Length,
    wheel_distance: Length
}

mod units;
mod odometry;

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
