use core::time::Duration;

use shrewnit::Centimeters;
use vexide::prelude::*;
use evian::{
    control::loops::{AngularPid, Pid},
    drivetrain::model::Differential, tracking::shared_motors,
    prelude::*
};

mod units;
mod odometry;
mod claw;

use odometry::{DifferentialDriveOdometry, DrivetrainConstants};

use crate::claw::Arm;

struct Robot {
    drivetrain: Drivetrain<Differential, DifferentialDriveOdometry>,
    arm: Arm,
    controller: Controller
}

impl Robot {
    const LINEAR_PID: Pid = Pid::new(1.0, 0.0, 0.125, None);
    const ANGULAR_PID: AngularPid = AngularPid::new(16.0, 0.0, 1.0, None);
    const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(4.0)
        .velocity(0.25)
        .duration(Duration::from_millis(15));
    const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(f64::to_radians(8.0))
        .velocity(0.09)
        .duration(Duration::from_millis(15));
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

    Robot {
        drivetrain: Drivetrain::new(
            Differential::from_shared(left_motors.clone(), right_motors.clone()),
            DifferentialDriveOdometry::new(constants, None, left_motors, right_motors)
        ),

        arm: Arm::new(
            Motor::new(peripherals.port_6, Gearset::Green, Direction::Forward),
            AngularPid::new(1.0, 2.0, 0.125, None)
        ),

        controller: peripherals.primary_controller
    }
    .compete()
    .await;
}
