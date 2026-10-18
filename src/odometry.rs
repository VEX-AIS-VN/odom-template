use crate::units::{Translation, Pose};

use core::f64::consts::{PI, TAU};
use std::cell::RefCell;
use std::rc::Rc;
use std::time::Instant;

use evian::prelude::{TracksHeading, TracksPosition, TracksVelocity,};

use evian::tracking::Tracking;
use vexide::prelude::Motor;
use vexide::smart::imu::InertialSensor;
use vexide::task::{Task, spawn};
use vexide::time::sleep;

use shrewnit::*;

type SharedMotors<const N: usize> = Rc<RefCell<[Motor; N]>>;

pub struct DrivetrainConstants {
    pub wheel_diameter: Length,
    pub wheel_distance: Length
}

pub struct Localization {
    pub pose: Pose,
    pub velocity: LinearVelocity,
    pub angular_velocity: AngularVelocity,
}

impl Localization {
    fn new() -> Self {
        Localization {
            pose: Pose::zero(),
            velocity: LinearVelocity::from_canonical(0f64),
            angular_velocity: AngularVelocity::from_canonical(0f64)
        }
    }
}

pub struct DifferentialDriveOdometry {
    data: Rc<RefCell<Localization>>,
    _task: Task<()>,
}
 
impl DifferentialDriveOdometry {
    pub fn new<const N: usize>(
        constants: DrivetrainConstants,
        imu: Option<InertialSensor>,
        left_motors: SharedMotors<N>, right_motors: SharedMotors<N>) -> Self {
        let data = Rc::new(RefCell::new(Localization::new()));

        Self {
            data: data.clone(), _task: spawn(Self::task(data, constants, imu, left_motors, right_motors))
        }
    }

    // I LOVE ASYNC (i dont I HATE ASYNC AAAAAAAAAAAAAAAAAAAAAAAA)
    async fn task<const N: usize>(
        data: Rc<RefCell<Localization>>,
        constants: DrivetrainConstants,
        imu: Option<InertialSensor>,
        left_motors: SharedMotors<N>,
        right_motors: SharedMotors<N>
    ) {
        let mut prev_time = Instant::now();
        
        let mut prev_left = Angle::from_canonical(0f64);
        let mut prev_right = Angle::from_canonical(0f64);

        loop {
            sleep(Motor::WRITE_INTERVAL).await;
            let dt = prev_time.elapsed();

            let left = if let Ok(position) = left_motors.borrow()[0].position() {
                position.as_radians() * Radians
            } else {
                continue;
            }; 
            let right = if let Ok(position) = right_motors.borrow()[0].position() {
                position.as_radians() * Radians
            } else {
                continue;
            }; 

            let dleft = left - prev_left;
            let dright = right - prev_right;

            // more like delta theta
            let omega = if let Some(gyro) = imu.as_ref() {
                if let Ok(heading) = gyro.heading() {
                    heading.as_radians() * Radians - data.borrow().pose.angle
                }
                else {
                    (dleft - dright) * PI * constants.wheel_diameter.canonical() / constants.wheel_distance.canonical()
                }
            } else {
                (dleft - dright) * PI * constants.wheel_diameter.canonical() / constants.wheel_distance.canonical()
            };

            let translation = if (dleft - dright).to::<Radians>() < 1e-5 {
                Translation::new(constants.wheel_diameter * dleft.to::<Rotations>() * PI, constants.wheel_diameter * dright.to::<Rotations>() * PI)
            } else {
                let displacement: Length = constants.wheel_diameter / 2.0 * (dleft + dright).to::<Rotations>() * PI;
                let (sin_theta, cos_theta) = (omega / 2.0 + data.borrow().pose.angle).to::<Radians>().sin_cos();
     
                Translation::new(
                    displacement * cos_theta,
                    displacement * sin_theta
                )
            };

            let norm = translation.norm();

            prev_left = left;
            prev_right = right;

            prev_time = Instant::now();

            // no, this is not a typo, it's sign(cos(theta))
            // always positive when cos(theta) is positive, always negative when cos(theta) is negative
            // graph mod(x - pi / 2, tau) - pi in desmos next to cos(theta) if u arent convinced
            let sign_costheta = (data.borrow().pose.angle.to::<Radians>() - PI / 2.0) % TAU - PI;

            let new = Localization {
                pose: data.borrow().pose.clone() + Pose::new(translation, omega),
                velocity: norm.to::<Meters>().copysign(sign_costheta) / dt.as_secs_f64() * MetersPerSecond,
                angular_velocity: omega / (dt.as_secs_f64() * Seconds)
            };

            *data.borrow_mut() = new;
        }
    }
}

impl Tracking for DifferentialDriveOdometry {}

impl TracksHeading for DifferentialDriveOdometry {
    fn heading(&self) -> evian::math::Angle {
        evian::math::Angle::from_radians(self.data.borrow().pose.angle.to::<Radians>())
    }
}

impl TracksPosition for DifferentialDriveOdometry {
    fn position(&self) -> evian::math::Vec2<f64> {
        self.data.borrow().pose.clone().into()
    }
}

impl TracksVelocity for DifferentialDriveOdometry {
    fn linear_velocity(&self) -> f64 {
        self.data.borrow().velocity.clone().to::<MetersPerSecond>()
    }

    fn angular_velocity(&self) -> f64 {
        self.data.borrow().angular_velocity.clone().to::<RadiansPerSecond>()
    }
}
