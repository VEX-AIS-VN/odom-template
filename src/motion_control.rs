use shrewnit::prelude::*;
use shrewnit::dimension;
use core::ops::Sub;


dimension!(
    /// Represents a frequency.
    ///
    /// Canonically represented in Hertz (s^-1).
    pub Frequency {
        canonical: Hertz,

        Hertz: 1.0 per canonical,
        Hz: 1.0 per canonical,
    } where {
        Self * Length => LinearVelocity in MetersPerSecond,
        Self * LinearVelocity => LinearAcceleration in MetersPerSecondSquared,
        Self * Angle => AngularVelocity in RadiansPerSecond,
        Self * AngularVelocity => AngularAcceleration in RadiansPerSecondSquared,
    }
);


// API modeled after evian v0.3.0-rc.1
trait MotionController<T> {
    /// Updates a motion controller with a current measurement and a setpoint.
    fn update(&mut self, measurement: T, setpoint: T) -> T;

    /// Resets the motion controller.
    fn reset(&mut self);
}

// based on https://derock.blog/post/vex-robotics/pid/
#[allow(non_snake_case)]
pub struct PIDController {
    /// A generic PID controller which (should) work for angles, distance, and velocity. All the
    /// values (kP, kI, and kD) have units, but I've chosen for them to not have units because it's
    /// difficult to allow the benefits of Rust's type system from messing things up.

    // unfortunately these fields stay unitless because rust's type system/shrewnit doesnt really
    // work with it

    /// The proportional constant of a PID controller.
    /// This is unitless.
    kP: f64,

    /// The integral constant of a PID controller. The unit of this is the integral of the unit being
    /// controlled over time.
    kI: f64,

    /// The derivative constant of a PID controller. The unit of this is the integral of the unit
    /// being controlled over time.
    kD: f64,

    previous_error: f64,
    integral: f64,
    max_integral: f64
}

impl PIDController {
    #[allow(non_snake_case)]
    pub fn new(kP: f64, kI: f64, kD: f64, max_integral: f64) -> Self {
        Self {
            kP,
            kI,
            kD,
            previous_error: 0.0,
            integral: 0.0,
            max_integral: max_integral
        }
    }
}

// angular PID controller logic
// It operates under the same principle, but units:tm:
impl<T> MotionController<T> for PIDController 
where
    T: Dimension + Sub<T, Output = T>
{
    fn update(&mut self, measurement: T, setpoint: T) -> T {
        let error = (measurement - setpoint).canonical();

        self.integral += error;

        // preventing integral windup
        if self.integral > self.max_integral { self.integral = self.max_integral }
        if self.previous_error.is_sign_positive() != error.is_sign_positive() {
            self.integral = 0.0;
        }

        T::from_canonical(error * self.kP
            + self.integral * self.kI
            + error * (error - self.previous_error))
    }

    fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_error = 0.0;
    }
}

// TODO: this and trapezoidal motion + PID
pub struct TrapezoidalMotionProfile {
    maximum_acceleration: MetersPerSecondSquared,
    maximum_velocity: MetersPerSecond,
}
