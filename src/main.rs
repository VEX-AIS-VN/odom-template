use shrewnit::Centimeters;
use vexide::prelude::*;
use evian::{
    drivetrain::model::Differential, tracking::shared_motors,
    prelude::*
};

mod units;
mod odometry;

use odometry::{DifferentialDriveOdometry, DrivetrainConstants};

struct Robot {
    drivetrain: Drivetrain<Differential, DifferentialDriveOdometry>,
    controller: Controller
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        
    }

    async fn driver(&mut self) {
        loop {
            let state = self.controller.state().unwrap_or_default();

            _ = self
                .drivetrain
                .model
                .drive_arcade(state.left_stick.y(), state.left_stick.x());
            println!("{}", self.drivetrain.tracking.position());

            sleep(Motor::WRITE_INTERVAL).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let left_motors = shared_motors![
        Motor::new(peripherals.port_7, Gearset::Green, Direction::Forward)
    ];

    let right_motors = shared_motors![
        Motor::new(peripherals.port_17, Gearset::Green, Direction::Forward)
    ];

    let constants = DrivetrainConstants {
        wheel_diameter: 5.0 * Centimeters,
        wheel_distance: 30.0 * Centimeters
    };

    let drivetrain = Drivetrain::new(
        Differential::from_shared(left_motors.clone(), right_motors.clone()),
        DifferentialDriveOdometry::new(constants, None, left_motors, right_motors)
    );

    Robot {
        drivetrain,
        controller: peripherals.primary_controller
    };
}
