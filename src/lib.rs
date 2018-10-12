//! Software PID controller
//!
//! This crate implements a PID controller. It has seen some amount of
//! real-world usage driving 100+ kW electrical motors, but has not been tested
//! to death. Use with caution (but do use it and file bug reports!).
//!
//! Any change in behaviour that may make calculations behave differently will
//! result in a major version upgrade; your tunings are safe as long as you
//! stay on the same major version.
//!
//! Owes a great debt to:
//!
//! * https://en.wikipedia.org/wiki/PID_controller
//! * http://www.embedded.com/design/prototyping-and-development/4211211/PID-without-a-PhD
//! * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

// FIXME: it may be worth to explore http://de.mathworks.com/help/simulink/slref/pidcontroller.html
//        for additional features/inspiration
#![no_std]

pub mod util;

use core::f64;

/// A generic controller interface.
///
/// A controller is fed timestamped values and calculates an adjusted value
/// based on previous readings.
///
/// Many controllers possess a set of adjustable parameters as well as a set
/// of input-value dependant state variables.
pub trait Controller<T> {
    /// Record a measurement from the plant.
    ///
    /// Records a new values. `delta_t` is the time since the last update in
    /// seconds.
    fn update(&mut self, value: T, delta_t: T) -> T;

    /// Adjust set target for the plant.
    ///
    /// The controller will usually try to adjust its output (from `update`) in
    /// a way that results in the plant approaching `target`.
    fn set_target(&mut self, target: T);

    /// Retrieve target value.
    fn target(&self) -> T;

    /// Reset internal state.
    ///
    /// Resets the internal state of the controller; not to be confused with
    /// its parameters.
    fn reset(&mut self);
}


/// PID controller derivative modes.
///
/// Two different ways of calculating the derivative can be used with the PID
/// controller, allowing to avoid "derivative kick" if needed (see
/// http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
/// for details information on the implementation that inspired this one).
///
/// Choosing `OnMeasurement` will avoid large bumps in the controller output
/// when changing the setpoint using `set_target()`.
#[derive(Debug, Clone, Copy)]
pub enum DerivativeMode {
    /// Calculate derivative of error (classic PID-Controller)
    OnError,
    /// Calculate derivative of actual changes in value.
    OnMeasurement,
}

/// PID Controller.
///
/// A PID controller, supporting the `Controller` interface. Any public values
/// are safe to modify while in operation.
///
/// `p_gain`, `i_gain` and `d_gain` are the respective gain values. The
/// controlller internally stores an already adjusted integral, making it safe
/// to alter the `i_gain` - it will *not* result in an immediate large jump in
/// controller output.
///
/// `i_min` and `i_max` are the limits for the internal integral storage.
/// Similarly, `out_min` and `out_max` clip the output value to an acceptable
/// range of values. By default, all limits are set to +/- infinity.
///
/// `d_mode` The `DerivativeMode`, the default is `OnMeasurement`.
#[derive(Debug, Clone)]
pub struct PIDController<T> {
    /// Proportional gain
    pub p_gain: T,

    /// Integral gain
    pub i_gain: T,

    /// Differential gain,
    pub d_gain: T,

    target: T,

    // Integral range limits
    pub i_min: T,
    pub i_max: T,

    // Output range limits
    pub out_min: T,
    pub out_max: T,

    pub d_mode: DerivativeMode,

    // The PIDs internal state. All other attributes are configuration values
    err_sum: T,
    prev_value: T,
    prev_error: T,
    init : bool
}

impl PIDController<f64> {
    /// Creates a new PID Controller.
    pub fn new(p_gain: f64, i_gain: f64, d_gain: f64) -> PIDController<f64> {
        PIDController{
            p_gain: p_gain,
            i_gain: i_gain,
            d_gain: d_gain,

            target: 0.0,

            err_sum: 0.0,
            prev_value: f64::NAN,
            prev_error: f64::NAN,

            i_min: -f64::INFINITY,
            i_max: f64::INFINITY,

            out_min: -f64::INFINITY,
            out_max: f64::INFINITY,

            d_mode: DerivativeMode::OnMeasurement,
            init : true
        }
    }

    /// Convenience function to set `i_min`/`i_max` and `out_min`/`out_max`
    /// to the same values simultaneously.
    pub fn set_limits(&mut self, min: f64, max: f64) {
        self.i_min = min;
        self.i_max = max;
        self.out_min = min;
        self.out_max = max;
    }
}


impl PIDController<i64> {
    /// Creates a new PID Controller.
    pub fn new(p_gain: i64, i_gain: i64, d_gain: i64) -> PIDController<i64> {
        PIDController{
            p_gain: p_gain,
            i_gain: i_gain,
            d_gain: d_gain,

            target: 0,

            err_sum: 0,
            prev_value: 0,
            prev_error: 0,

            i_min: i64::min_value(),
            i_max: i64::max_value(),

            out_min: i64::min_value(),
            out_max: i64::max_value(),

            d_mode: DerivativeMode::OnMeasurement,
            init : true
        }
    }

    /// Convenience function to set `i_min`/`i_max` and `out_min`/`out_max`
    /// to the same values simultaneously.
    pub fn set_limits(&mut self, min: i64, max: i64) {
        self.i_min = min;
        self.i_max = max;
        self.out_min = min;
        self.out_max = max;
    }
}

impl Controller<f64> for PIDController<f64> {
    fn set_target(&mut self, target: f64) {
        self.target = target;
    }

    fn target(&self) -> f64 {
        self.target
    }

    fn update(&mut self, value: f64, delta_t: f64) -> f64 {
        let error = self.target - value;

        // PROPORTIONAL
        let p_term = self.p_gain * error;

        // INTEGRAL
        self.err_sum = util::limit_range(
            self.i_min, self.i_max,
            self.err_sum + self.i_gain * error * delta_t
        );
        let i_term = self.err_sum;

        // DIFFERENTIAL
        let d_term = if self.init {
            self.init = false;
            // we have no previous values, so skip the derivative calculation
            0.0
        } else {
            match self.d_mode {
                DerivativeMode::OnMeasurement => {
                    // we use -delta_v instead of delta_error to reduce "derivative kick",
                    // see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
                    self.d_gain * (self.prev_value - value) / delta_t
                },
                DerivativeMode::OnError => {
                    self.d_gain * (error - self.prev_error) / delta_t
                }
            }
        };

        // store previous values
        self.prev_value = value;
        self.prev_error = error;

        util::limit_range(
            self.out_min, self.out_max,
            p_term + d_term + i_term
        )
    }

    fn reset(&mut self) {
        self.prev_value = f64::NAN;
        self.prev_error = f64::NAN;

        // FIXME: http://brettbeauregard.com/blog/2011/04/improving-the-beginner
        //               %E2%80%99s-pid-initialization/
        //        suggests that this should not be there. however, it may miss
        //        the fact that input and output can be two completely
        //        different domains
        self.err_sum = 0.0;
    }
}


impl Controller<i64> for PIDController<i64> {
    fn set_target(&mut self, target: i64) {
        self.target = target;
    }

    fn target(&self) -> i64 {
        self.target
    }

    fn update(&mut self, value: i64, delta_t: i64) -> i64 {
        let error = self.target - value;

        // PROPORTIONAL
        let p_term = self.p_gain * error;

        // INTEGRAL
        self.err_sum = util::limit_range(
            self.i_min, self.i_max,
            self.err_sum + self.i_gain * error * delta_t
        );
        let i_term = self.err_sum;

        // DIFFERENTIAL
        let d_term=
        if self.init {
            self.init = false;
            0
        } else {
            match self.d_mode {
                DerivativeMode::OnMeasurement => {
                    // we use -delta_v instead of delta_error to reduce "derivative kick",
                    // see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
                    self.d_gain * (self.prev_value - value) / delta_t
                },
                DerivativeMode::OnError => {
                    self.d_gain * (error - self.prev_error) / delta_t
                }
            }

        };

        // store previous values
        self.prev_value = value;
        self.prev_error = error;

        util::limit_range(
            self.out_min, self.out_max,
            p_term + d_term + i_term
        )
    }

    fn reset(&mut self) {
        self.prev_value = 0;
        self.prev_error = 0;

        // FIXME: http://brettbeauregard.com/blog/2011/04/improving-the-beginner
        //               %E2%80%99s-pid-initialization/
        //        suggests that this should not be there. however, it may miss
        //        the fact that input and output can be two completely
        //        different domains
        self.err_sum = 0;
    }
}

#[cfg(test)]
mod test {
    use {Controller,PIDController};
    type PIDControllerf = PIDController<f64>;
    type PIDControlleru = PIDController<i64>;

    #[test]
    fn p_controller_f64() {
        let mut controller : PIDController<f64> = PIDControllerf::new(5.0,0.0,0.0);
        controller.set_target(27.03);
        assert_eq!(controller.update(15.0, 0.5), (27.03 - 15.0) * 5.0);
        controller.set_target(-512.57);
        assert_eq!(controller.update(65.0,0.5), (-512.57-65.0)*5.0);
        controller.set_target(0.0);
        assert_eq!(controller.update(5.0,0.5), -5.0*5.0);
        assert_eq!(controller.update(0.0,0.5), 0.0);
        assert_eq!(controller.update(-0.1,0.5), 0.5);
    }

    #[test]
    fn p_controller_i64() {
        let mut controller : PIDController<i64> = PIDControlleru::new(5,0,0);
        controller.set_target(27);
        assert_eq!(controller.update(15, 1), (27 - 15) * 5);
        controller.set_target(512);
        assert_eq!(controller.update(65,1), (512-65)*5);
        controller.set_target(0);
        assert_eq!(controller.update(5,1), -5*5);
        assert_eq!(controller.update(0,1), 0);
        assert_eq!(controller.update(-1,1), 5);
    }


}