use std::{cell::Cell, f64::consts::TAU, rc::Rc, time::Instant};

use vexide::{prelude::Motor, task::{Task, spawn}, time::sleep};
use evian::{control::loops::{AngularPid, Feedback}, math::Angle};

pub struct Arm {
    setpoint: Rc<Cell<Option<Angle>>>,
    roller_motor: Motor,
    _task: Task<()>
}

impl Arm {
    pub fn new(motor: Motor, roller_motor: Motor, pid: AngularPid) -> Self {
        let setpoint = Rc::new(Cell::new(None));

        Arm {
            setpoint: setpoint.clone(), roller_motor, _task: spawn(Self::task(motor, setpoint, pid))
        }
    }

    pub fn set_target(&mut self, angle: shrewnit::Angle) {
        self.setpoint.set(Some(Angle::from_radians(angle.to::<shrewnit::Radians>())));
    }

    pub fn intake(&mut self) {
        let _ = self.roller_motor.set_voltage(8.0);
    }

    async fn task(mut motor: Motor, setpoint: Rc<Cell<Option<Angle>>>, mut pid: AngularPid) {
        let mut prev_time = Instant::now();
        loop {
            sleep(Motor::WRITE_INTERVAL).await;

            let Ok(position) = motor.position() else {
                continue;
            };

            let target = match setpoint.get() {
                Some(target) => target,
                None => position
            };

            let current_time = prev_time.elapsed();
            prev_time = Instant::now();

            let signal = pid.update(position, target, current_time);  // radians/sec
            let _ = motor.set_velocity((signal / TAU * 60.0) as i32);  // conversion to rpm
        }
    }
}
