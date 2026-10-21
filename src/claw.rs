use std::{cell::RefCell, f64::consts::TAU, rc::Rc, time::Instant};

use vexide::{prelude::Motor, task::{Task, spawn}, time::sleep};
use evian::{control::loops::{AngularPid, Feedback}, math::Angle};

pub struct Arm {
    setpoint: Rc<RefCell<Option<Angle>>>,
    _task: Task<()>
}

impl Arm {
    pub fn new(motor: Motor, pid: AngularPid) -> Self {
        let setpoint = Rc::new(RefCell::new(None));

        Arm {
            setpoint: setpoint.clone(), _task: spawn(Self::task(motor, setpoint, pid))
        }
    }

    fn set_target(&mut self, angle: shrewnit::Angle) {
        *self.setpoint.borrow_mut() = Some(Angle::from_radians(angle.to::<shrewnit::Radians>()));
    }

    async fn task(mut motor: Motor, setpoint: Rc<RefCell<Option<Angle>>>, mut pid: AngularPid) {
        let mut prev_time = Instant::now();
        loop {
            sleep(Motor::WRITE_INTERVAL).await;

            let Ok(position) = motor.position() else {
                continue;
            };

            if setpoint.borrow().is_none() {
                *setpoint.borrow_mut() = Some(position);
            }

            let current_time = prev_time.elapsed();
            prev_time = Instant::now();

            let signal = pid.update(position, setpoint.borrow().clone().unwrap(), current_time);  // radians/sec
            let _ = motor.set_velocity((signal / TAU * 60.0) as i32);  // conversion to rotations/min
        }
    }
}
