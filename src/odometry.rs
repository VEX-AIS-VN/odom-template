use crate::units::{Translation, Pose};

use core::f64::consts::PI;
use std::cell::RefCell;
use std::rc::Rc;
use std::time::Instant;

use evian::prelude::{TracksHeading, TracksPosition, TracksVelocity};

use evian::tracking::Tracking;
use vexide::prelude::Motor;
use vexide::smart::imu::InertialSensor;
use vexide::task::{Task, spawn};
use vexide::time::sleep;

use shrewnit::*;

type SharedMotors<const N: usize> = Rc<RefCell<[Motor; N]>>;

pub struct DrivetrainConstants {
    pub wheel_diameter: Length,
    pub wheel_from_center: Length
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

fn average_motor_position<const N: usize>(
    motors: &SharedMotors<N>,
) -> Angle {
    let motors = motors.borrow();

    if motors.is_empty() {
        return Angle::from_canonical(0.0);
    }

    let mut sum = Angle::from_canonical(0.0);

    let mut count = 0;

    for motor in motors.iter() {
        if let Ok(position) = motor.position() {
            sum += position.as_radians() * Radians;
            count += 1
        }
    }

    sum / count as f64
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

    fn calculate_translation(
        constants: &DrivetrainConstants,
        dleft: Angle,
        dright: Angle,
        omega: Angle,
        prev_heading: Angle,
        dt: f64
    ) -> (Translation, LinearVelocity) {
        let displacement: Length = constants.wheel_diameter / 2.0
            * (dleft + dright).to::<Rotations>()
            * PI;

        let half_angle = (omega / 2.0).to::<Radians>();
        let sinc = if half_angle.abs() < 1e-6 {
            1.0
        } else {
            half_angle.sin() / half_angle
        };

        let mid_heading =
            (prev_heading + omega / 2.0).to::<Radians>();
        let (sin_heading, cos_heading) = mid_heading.sin_cos();

        let translation = Translation::new(
            displacement * sinc * cos_heading,
            displacement * sinc * sin_heading,
        );

        let velocity = translation.clone() * (1.0 / dt);

        (
            translation,
            (velocity.x * cos_heading + velocity.y * cos_heading).to::<Meters>() * MetersPerSecond
        )
    }

    fn calculate_omega(
        constants: &DrivetrainConstants,
        imu: &Option<InertialSensor>,
        dleft: Angle,
        dright: Angle,
        prev_imu: Angle
    ) -> Angle {
        if let Some(gyro) = imu.as_ref() {
            if let Ok(heading) = gyro.rotation() {
                let measure = heading.as_radians() * Radians;
                measure - prev_imu
            }
            else {
                (dright - dleft) * constants.wheel_diameter.canonical() / (4.0 * constants.wheel_from_center.canonical())
            }
        } else {
            (dright - dleft) * constants.wheel_diameter.canonical() / (4.0 * constants.wheel_from_center.canonical())
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

        let mut prev_imu = Angle::from_canonical(0f64);

        for motor in left_motors.borrow_mut().iter_mut() {
            motor.reset_position().unwrap();
        }

        for motor in right_motors.borrow_mut().iter_mut() {
            motor.reset_position().unwrap();
        }

        if let Some(sensor) = imu.as_ref() {
            prev_imu = sensor.rotation().map(|x: evian::math::Angle| x.as_radians() * Radians).unwrap_or(prev_imu);
        }

        loop {
            sleep(Motor::WRITE_INTERVAL).await;
            let dt = prev_time.elapsed().as_secs_f64() * Seconds;

            let left = average_motor_position(&left_motors);
            let right = average_motor_position(&right_motors);

            let dleft = left - prev_left;
            let dright = right - prev_right;

            // more like delta theta
            let omega = Self::calculate_omega(&constants, &imu, dleft, dright, prev_imu);

            if imu.as_ref().is_some() {
                prev_imu += omega;
            }

            let (translation, velocity) = Self::calculate_translation(&constants, dleft, dright, omega, data.borrow().pose.angle, dt.to::<Seconds>());

            prev_left = left;
            prev_right = right;

            prev_time = Instant::now();

            let new = Localization {
                pose: &data.borrow().pose + &Pose::new(translation, omega),
                velocity: velocity,
                angular_velocity: omega / dt
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
        (&self.data.borrow().pose).into()
    }
}

impl TracksVelocity for DifferentialDriveOdometry {
    fn linear_velocity(&self) -> f64 {
        (&self.data.borrow().velocity).to::<MetersPerSecond>()
    }

    fn angular_velocity(&self) -> f64 {
        (&self.data.borrow().angular_velocity).to::<RadiansPerSecond>()
    }
}
