//! Corrective actions to restore balance.
//!
//! Actuators take corrective actions to bring agent parameters
//! back toward their setpoints.

/// The type of corrective action an actuator can take.
#[derive(Debug, Clone, PartialEq)]
pub enum ActionType {
    /// Increase the parameter value.
    Increase,
    /// Decrease the parameter value.
    Decrease,
    /// No action needed (within tolerance).
    Hold,
}

/// A corrective action to be applied.
#[derive(Debug, Clone)]
pub struct Action {
    /// Name of the parameter being adjusted.
    pub parameter: String,
    /// The type of action.
    pub action_type: ActionType,
    /// The magnitude of the adjustment.
    pub magnitude: f64,
}

impl Action {
    /// Create a new action.
    pub fn new(parameter: &str, action_type: ActionType, magnitude: f64) -> Self {
        Action {
            parameter: parameter.to_string(),
            action_type,
            magnitude,
        }
    }

    /// Create a hold action (no change).
    pub fn hold(parameter: &str) -> Self {
        Action::new(parameter, ActionType::Hold, 0.0)
    }

    /// Apply this action to a value, returning the new value.
    pub fn apply(&self, current: f64) -> f64 {
        match self.action_type {
            ActionType::Increase => current + self.magnitude,
            ActionType::Decrease => current - self.magnitude,
            ActionType::Hold => current,
        }
    }
}

/// An actuator that produces corrective actions based on deviation.
#[derive(Debug, Clone)]
pub struct Actuator {
    /// Name of the parameter this actuator controls.
    pub parameter: String,
    /// Maximum adjustment magnitude per tick.
    pub max_magnitude: f64,
    /// Gain factor for computing action magnitude.
    pub gain: f64,
    /// Minimum deviation to trigger action (dead zone).
    pub dead_zone: f64,
}

impl Actuator {
    /// Create a new actuator.
    pub fn new(parameter: &str, max_magnitude: f64, gain: f64) -> Self {
        Actuator {
            parameter: parameter.to_string(),
            max_magnitude,
            gain,
            dead_zone: 0.0,
        }
    }

    /// Set the dead zone.
    pub fn with_dead_zone(mut self, dz: f64) -> Self {
        self.dead_zone = dz;
        self
    }

    /// Compute the corrective action for a given deviation.
    pub fn compute_action(&self, deviation: f64) -> Action {
        if deviation.abs() < self.dead_zone {
            return Action::hold(&self.parameter);
        }

        let raw_magnitude = deviation.abs() * self.gain;
        let magnitude = raw_magnitude.min(self.max_magnitude);

        let action_type = if deviation > 0.0 {
            ActionType::Decrease // value is above target, decrease
        } else {
            ActionType::Increase // value is below target, increase
        };

        Action::new(&self.parameter, action_type, magnitude)
    }

    /// Compute and apply action to a current value given a deviation.
    pub fn correct(&self, current: f64, deviation: f64) -> f64 {
        let action = self.compute_action(deviation);
        action.apply(current)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_action_increase() {
        let a = Action::new("temp", ActionType::Increase, 5.0);
        let result = a.apply(37.0);
        assert!((result - 42.0).abs() < 1e-10);
    }

    #[test]
    fn test_action_decrease() {
        let a = Action::new("temp", ActionType::Decrease, 3.0);
        let result = a.apply(40.0);
        assert!((result - 37.0).abs() < 1e-10);
    }

    #[test]
    fn test_action_hold() {
        let a = Action::hold("temp");
        let result = a.apply(37.0);
        assert!((result - 37.0).abs() < 1e-10);
    }

    #[test]
    fn test_actuator_positive_deviation() {
        let act = Actuator::new("temp", 10.0, 1.0);
        let action = act.compute_action(5.0); // 5 above target
        assert_eq!(action.action_type, ActionType::Decrease);
        assert!((action.magnitude - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_actuator_negative_deviation() {
        let act = Actuator::new("temp", 10.0, 1.0);
        let action = act.compute_action(-3.0);
        assert_eq!(action.action_type, ActionType::Increase);
        assert!((action.magnitude - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_actuator_max_magnitude() {
        let act = Actuator::new("temp", 2.0, 1.0);
        let action = act.compute_action(100.0); // huge deviation
        assert!((action.magnitude - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_actuator_dead_zone() {
        let act = Actuator::new("temp", 10.0, 1.0).with_dead_zone(1.0);
        let action = act.compute_action(0.5); // within dead zone
        assert_eq!(action.action_type, ActionType::Hold);
    }

    #[test]
    fn test_actuator_correct() {
        let act = Actuator::new("temp", 10.0, 0.5);
        let new_val = act.correct(40.0, 4.0); // 4 above, gain 0.5 → decrease by 2
        assert!((new_val - 38.0).abs() < 1e-10);
    }
}
