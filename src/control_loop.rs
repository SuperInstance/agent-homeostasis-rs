//! PID-inspired control loop for agent parameters.
//!
//! Implements a Proportional-Integral-Derivative controller adapted
//! for agent homeostatic regulation.

/// A PID controller for a single parameter.
#[derive(Debug, Clone)]
pub struct PidController {
    /// Parameter name.
    pub name: String,
    /// Proportional gain.
    pub kp: f64,
    /// Integral gain.
    pub ki: f64,
    /// Derivative gain.
    pub kd: f64,
    /// Target value (setpoint).
    pub target: f64,
    /// Integral accumulator (sum of errors over time).
    pub integral: f64,
    /// Previous error (for derivative term).
    pub prev_error: Option<f64>,
    /// Maximum integral value (anti-windup).
    pub integral_limit: f64,
    /// Output limit (clamps the correction).
    pub output_limit: f64,
}

impl PidController {
    /// Create a new PID controller.
    pub fn new(name: &str, kp: f64, ki: f64, kd: f64, target: f64) -> Self {
        PidController {
            name: name.to_string(),
            kp,
            ki,
            kd,
            target,
            integral: 0.0,
            prev_error: None,
            integral_limit: 100.0,
            output_limit: f64::INFINITY,
        }
    }

    /// Set integral anti-windup limit.
    pub fn with_integral_limit(mut self, limit: f64) -> Self {
        self.integral_limit = limit;
        self
    }

    /// Set output clamping limit.
    pub fn with_output_limit(mut self, limit: f64) -> Self {
        self.output_limit = limit;
        self
    }

    /// Compute the PID output for a given measurement.
    /// Returns (output, error, p_term, i_term, d_term).
    pub fn update(&mut self, measurement: f64) -> PidResult {
        let error = self.target - measurement;

        // Proportional term
        let p_term = self.kp * error;

        // Integral term (with anti-windup)
        self.integral += error;
        self.integral = self.integral.clamp(-self.integral_limit, self.integral_limit);
        let i_term = self.ki * self.integral;

        // Derivative term
        let d_term = match self.prev_error {
            Some(pe) => self.kd * (error - pe),
            None => 0.0,
        };

        self.prev_error = Some(error);

        let output = (p_term + i_term + d_term).clamp(-self.output_limit, self.output_limit);

        PidResult {
            output,
            error,
            p_term,
            i_term,
            d_term,
        }
    }

    /// Reset the controller state (integral and derivative).
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = None;
    }

    /// Update the target setpoint.
    pub fn set_target(&mut self, target: f64) {
        self.target = target;
    }
}

/// Result of a PID update.
#[derive(Debug, Clone)]
pub struct PidResult {
    /// The correction output.
    pub output: f64,
    /// The current error.
    pub error: f64,
    /// Proportional term contribution.
    pub p_term: f64,
    /// Integral term contribution.
    pub i_term: f64,
    /// Derivative term contribution.
    pub d_term: f64,
}

/// A multi-parameter PID control loop.
#[derive(Debug, Clone)]
pub struct ControlLoop {
    /// PID controllers for each parameter.
    pub controllers: Vec<PidController>,
}

impl Default for ControlLoop {
    fn default() -> Self {
        Self::new()
    }
}

impl ControlLoop {
    /// Create a new control loop.
    pub fn new() -> Self {
        ControlLoop { controllers: Vec::new() }
    }

    /// Add a controller.
    pub fn add(&mut self, controller: PidController) {
        self.controllers.push(controller);
    }

    /// Run one tick of the control loop.
    pub fn tick(&mut self, measurements: &[f64]) -> Vec<PidResult> {
        assert_eq!(measurements.len(), self.controllers.len(),
            "number of measurements must match controllers");
        self.controllers
            .iter_mut()
            .zip(measurements.iter())
            .map(|(c, &m)| c.update(m))
            .collect()
    }

    /// Run a simulation for n steps, starting from initial values.
    /// Returns the final values after applying corrections each step.
    pub fn simulate(&mut self, initial: &[f64], steps: usize) -> Vec<Vec<f64>> {
        assert_eq!(initial.len(), self.controllers.len());
        let mut trajectory = vec![initial.to_vec()];
        let mut values = initial.to_vec();

        for _ in 0..steps {
            let results = self.tick(&values);
            for (i, r) in results.iter().enumerate() {
                values[i] += r.output;
            }
            trajectory.push(values.clone());
        }

        trajectory
    }

    /// Number of controllers.
    pub fn len(&self) -> usize {
        self.controllers.len()
    }

    /// True if no controllers.
    pub fn is_empty(&self) -> bool {
        self.controllers.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pid_proportional_only() {
        let mut pid = PidController::new("temp", 1.0, 0.0, 0.0, 100.0);
        let result = pid.update(90.0);
        // Error = 10, P term = 10
        assert!((result.output - 10.0).abs() < 1e-10);
        assert!((result.p_term - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_pid_integral_accumulates() {
        let mut pid = PidController::new("temp", 0.0, 0.1, 0.0, 100.0);
        pid.update(90.0); // integral = 10
        pid.update(90.0); // integral = 20
        let result = pid.update(90.0); // integral = 30
        assert!((result.i_term - 3.0).abs() < 1e-10); // 0.1 * 30
    }

    #[test]
    fn test_pid_derivative() {
        let mut pid = PidController::new("temp", 0.0, 0.0, 1.0, 100.0);
        pid.update(95.0); // error = 5, no prev → d = 0
        let result = pid.update(90.0); // error = 10, prev = 5 → d = 5
        assert!((result.d_term - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_pid_anti_windup() {
        let mut pid = PidController::new("temp", 0.0, 1.0, 0.0, 100.0)
            .with_integral_limit(5.0);
        pid.update(0.0); // error = 100, integral = 100, clamped to 5
        let result = pid.update(0.0);
        assert!((result.i_term - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_pid_output_limit() {
        let mut pid = PidController::new("temp", 10.0, 0.0, 0.0, 100.0)
            .with_output_limit(5.0);
        let result = pid.update(0.0); // error = 100, p = 1000, clamped to 5
        assert!((result.output - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_pid_reset() {
        let mut pid = PidController::new("temp", 1.0, 1.0, 1.0, 100.0);
        pid.update(50.0);
        pid.reset();
        assert!((pid.integral).abs() < 1e-10);
        assert!(pid.prev_error.is_none());
    }

    #[test]
    fn test_control_loop_simulation_converges() {
        let mut cl = ControlLoop::new();
        cl.add(PidController::new("temp", 0.5, 0.01, 0.1, 100.0));
        let trajectory = cl.simulate(&[50.0], 200);
        let final_val = trajectory.last().unwrap()[0];
        assert!((final_val - 100.0).abs() < 2.0);
    }

    #[test]
    fn test_control_loop_multi_param() {
        let mut cl = ControlLoop::new();
        cl.add(PidController::new("temp", 0.5, 0.01, 0.1, 100.0));
        cl.add(PidController::new("pressure", 0.3, 0.005, 0.05, 50.0));
        let trajectory = cl.simulate(&[80.0, 30.0], 100);
        assert!((trajectory.last().unwrap()[0] - 100.0).abs() < 5.0);
        assert!((trajectory.last().unwrap()[1] - 50.0).abs() < 5.0);
    }
}
