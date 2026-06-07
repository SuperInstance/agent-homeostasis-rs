//! Setpoint tracking and dynamic adjustment.
//!
//! A setpoint defines the target value for an agent parameter,
//! with bounds on acceptable deviation and dynamic adjustment over time.

/// A setpoint defining a target value and acceptable range.
#[derive(Debug, Clone)]
pub struct Setpoint {
    /// Name of the parameter.
    pub name: String,
    /// Target value.
    pub target: f64,
    /// Minimum acceptable value (deadband lower bound).
    pub min: f64,
    /// Maximum acceptable value (deadband upper bound).
    pub max: f64,
    /// Rate at which the setpoint drifts per tick (for simulation).
    pub drift_rate: f64,
    /// Maximum allowed target value.
    pub max_target: f64,
    /// Minimum allowed target value.
    pub min_target: f64,
}

impl Setpoint {
    /// Create a new setpoint with symmetric tolerance.
    pub fn new(name: &str, target: f64, tolerance: f64) -> Self {
        Setpoint {
            name: name.to_string(),
            target,
            min: target - tolerance,
            max: target + tolerance,
            drift_rate: 0.0,
            max_target: f64::INFINITY,
            min_target: f64::NEG_INFINITY,
        }
    }

    /// Create a setpoint with asymmetric bounds.
    pub fn with_bounds(name: &str, target: f64, min: f64, max: f64) -> Self {
        Setpoint {
            name: name.to_string(),
            target,
            min,
            max,
            drift_rate: 0.0,
            max_target: f64::INFINITY,
            min_target: f64::NEG_INFINITY,
        }
    }

    /// Set the drift rate for this setpoint.
    pub fn with_drift(mut self, rate: f64) -> Self {
        self.drift_rate = rate;
        self
    }

    /// Set bounds on the target value.
    pub fn with_target_bounds(mut self, min: f64, max: f64) -> Self {
        self.min_target = min;
        self.max_target = max;
        self
    }

    /// Check if a value is within the acceptable range.
    pub fn is_satisfied(&self, value: f64) -> bool {
        value >= self.min && value <= self.max
    }

    /// Compute the deviation from target (positive = above, negative = below).
    pub fn deviation(&self, value: f64) -> f64 {
        value - self.target
    }

    /// Compute the normalized deviation: -1 to 1 within bounds, can exceed.
    pub fn normalized_deviation(&self, value: f64) -> f64 {
        if self.target == self.min && self.target == self.max {
            return 0.0;
        }
        let half_range = (self.max - self.min) / 2.0;
        if half_range == 0.0 {
            return 0.0;
        }
        (value - self.target) / half_range
    }

    /// Apply drift for one tick.
    pub fn apply_drift(&mut self) {
        self.target += self.drift_rate;
        self.target = self.target.clamp(self.min_target, self.max_target);
        // Also shift bounds
        let tol = (self.max - self.min) / 2.0;
        self.min = self.target - tol;
        self.max = self.target + tol;
    }

    /// Adjust the target by a delta, respecting bounds.
    pub fn adjust_target(&mut self, delta: f64) {
        self.target = (self.target + delta).clamp(self.min_target, self.max_target);
        let tol = (self.max - self.min) / 2.0;
        self.min = self.target - tol;
        self.max = self.target + tol;
    }
}

/// A tracker that monitors setpoint satisfaction over time.
#[derive(Debug, Clone)]
pub struct SetpointTracker {
    /// The setpoint being tracked.
    pub setpoint: Setpoint,
    /// History of values observed.
    pub history: Vec<f64>,
    /// Maximum history length.
    pub max_history: usize,
}

impl SetpointTracker {
    /// Create a new tracker for a setpoint.
    pub fn new(setpoint: Setpoint) -> Self {
        SetpointTracker {
            setpoint,
            history: Vec::new(),
            max_history: 1000,
        }
    }

    /// Record a value observation.
    pub fn observe(&mut self, value: f64) {
        if self.history.len() >= self.max_history {
            self.history.remove(0);
        }
        self.history.push(value);
    }

    /// Fraction of observations that satisfy the setpoint.
    pub fn satisfaction_rate(&self) -> f64 {
        if self.history.is_empty() {
            return 1.0;
        }
        let satisfied = self.history.iter().filter(|&&v| self.setpoint.is_satisfied(v)).count();
        satisfied as f64 / self.history.len() as f64
    }

    /// Mean deviation over history.
    pub fn mean_deviation(&self) -> f64 {
        if self.history.is_empty() {
            return 0.0;
        }
        let sum: f64 = self.history.iter().map(|&v| self.setpoint.deviation(v)).sum();
        sum / self.history.len() as f64
    }

    /// Current value (last observed).
    pub fn current_value(&self) -> Option<f64> {
        self.history.last().copied()
    }

    /// Apply drift and return the new target.
    pub fn tick_drift(&mut self) -> f64 {
        self.setpoint.apply_drift();
        self.setpoint.target
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_setpoint_satisfied() {
        let sp = Setpoint::new("temperature", 37.0, 1.0);
        assert!(sp.is_satisfied(37.0));
        assert!(sp.is_satisfied(36.5));
        assert!(sp.is_satisfied(38.0));
        assert!(!sp.is_satisfied(35.0));
        assert!(!sp.is_satisfied(39.0));
    }

    #[test]
    fn test_setpoint_deviation() {
        let sp = Setpoint::new("temp", 100.0, 5.0);
        assert!((sp.deviation(105.0) - 5.0).abs() < 1e-10);
        assert!((sp.deviation(95.0) - (-5.0)).abs() < 1e-10);
    }

    #[test]
    fn test_setpoint_normalized_deviation() {
        let sp = Setpoint::new("temp", 100.0, 5.0);
        assert!((sp.normalized_deviation(100.0)).abs() < 1e-10);
        assert!((sp.normalized_deviation(105.0) - 1.0).abs() < 1e-10);
        assert!((sp.normalized_deviation(95.0) - (-1.0)).abs() < 1e-10);
    }

    #[test]
    fn test_setpoint_drift() {
        let sp = Setpoint::new("temp", 100.0, 5.0).with_drift(1.0);
        let mut tracker = SetpointTracker::new(sp);
        let new_target = tracker.tick_drift();
        assert!((new_target - 101.0).abs() < 1e-10);
    }

    #[test]
    fn test_setpoint_drift_clamped() {
        let sp = Setpoint::new("temp", 99.0, 1.0).with_drift(5.0).with_target_bounds(0.0, 100.0);
        let mut tracker = SetpointTracker::new(sp);
        tracker.tick_drift();
        assert!((tracker.setpoint.target - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_tracker_satisfaction_rate() {
        let sp = Setpoint::new("temp", 50.0, 5.0);
        let mut tracker = SetpointTracker::new(sp);
        tracker.observe(50.0); // satisfied
        tracker.observe(51.0); // satisfied
        tracker.observe(60.0); // not satisfied
        assert!((tracker.satisfaction_rate() - 2.0 / 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_tracker_mean_deviation() {
        let sp = Setpoint::new("temp", 50.0, 5.0);
        let mut tracker = SetpointTracker::new(sp);
        tracker.observe(52.0);
        tracker.observe(48.0);
        assert!((tracker.mean_deviation()).abs() < 1e-10);
    }

    #[test]
    fn test_setpoint_adjust_target() {
        let mut sp = Setpoint::new("temp", 50.0, 5.0).with_target_bounds(0.0, 100.0);
        sp.adjust_target(10.0);
        assert!((sp.target - 60.0).abs() < 1e-10);
        sp.adjust_target(50.0); // would go to 110, clamped to 100
        assert!((sp.target - 100.0).abs() < 1e-10);
    }
}
