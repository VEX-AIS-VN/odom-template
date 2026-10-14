use crate::drivetrain::DrivetrainConstants;
use crate::units::{Translation, Pose};
use core::f64::consts::PI;
use shrewnit::{Angle, Length, Radians, Rotations};

/// A trait to implement Odometry for Drivetrains.
pub trait Odometry<const N: usize> {
    /// Updates the state of the odometry with new information. Measurements is an N-sized array
    /// (corresponding to number of drivetrain motors) of angles, corresponding with how much they
    /// have rotated.
    fn tick(&mut self, measurements: [Angle; N]);

    fn get_pose(&self) -> Pose;
}

pub struct DifferentialDriveOdometry {
    constants: DrivetrainConstants,
    pub pose: Pose,
    prev_left: Angle,
    prev_right: Angle
}
 
impl DifferentialDriveOdometry {
    pub fn new(constants: DrivetrainConstants) -> Self {
        Self { constants, pose: Pose::zero(), prev_left: Angle::from_canonical(0.0), prev_right: Angle::from_canonical(0.0) }
    }
}

impl Odometry<2> for DifferentialDriveOdometry {
    // based on https://medium.com/@nahmed3536/wheel-odometry-model-for-differential-drive-robotics-91b85a012299
    fn tick(&mut self, measurements: [Angle; 2]) {
        let left = measurements[0];
        let right = measurements[1];

        let dleft = left - self.prev_left;
        let dright = right - self.prev_right;

        // more like delta theta
        // canonical conversion here, but a * unit / (b * unit) = a/b
        let omega = (dleft - dright) * PI * self.constants.wheel_diameter.canonical() / self.constants.wheel_distance.canonical();

        let translation = if (dleft - dright).to::<Radians>() < 1e-5 {
            Translation::new(self.constants.wheel_diameter * dleft.to::<Rotations>() * PI, self.constants.wheel_diameter * dright.to::<Rotations>() * PI)
        } else {
            let displacement: Length = self.constants.wheel_diameter / 2.0 * (dleft + dright).to::<Rotations>() * PI;
            let (sin_theta, cos_theta) = (omega / 2.0 + self.pose.angle).to::<Radians>().sin_cos();
 
            Translation::new(
                displacement * cos_theta,
                displacement * sin_theta
            )
        };
 
        self.pose += Pose::new(translation, omega);
        self.prev_left = left;
        self.prev_right = right;
    }

    fn get_pose(&self) -> Pose {
        self.pose.clone()
    }
}
