//! Homeostatic regulator with setpoints and feedback.
//!
//! The regulator monitors multiple parameters, compares them against
//! setpoints, and generates corrective actions.

use crate::actuator::{Action, Actuator};
use crate::sensor::SensorReading;
use crate::setpoint::Setpoint;

/// Status of a regulated parameter.
#[derive(Debug, Clone)]
pub struct ParameterStatus {
    /// Parameter name.
    pub name: String,
    /// Current value.
    pub current: f64,
    /// Target value.
    pub target: f64,
    /// Deviation from target.
    pub deviation: f64,
    /// Whether the value is within tolerance.
    pub is_stable: bool,
    /// Recommended action.
    pub action: Action,
}

/// The overall health status of the regulator.
#[derive(Debug, Clone, PartialEq)]
pub enum HealthStatus {
    /// All parameters within tolerance.
    Stable,
    /// Some parameters outside tolerance, correcting.
    Correcting,
    /// Critical parameters far outside tolerance.
    Critical,
}

/// A homeostatic regulator that monitors and corrects multiple parameters.
#[derive(Debug, Clone)]
pub struct HomeostaticRegulator {
    /// Setpoints for each parameter.
    pub setpoints: Vec<Setpoint>,
    /// Actuators for each parameter.
    pub actuators: Vec<Actuator>,
}

impl Default for HomeostaticRegulator {
    fn default() -> Self {
        Self::new()
    }
}

impl HomeostaticRegulator {
    /// Create a new empty regulator.
    pub fn new() -> Self {
        HomeostaticRegulator {
            setpoints: Vec::new(),
            actuators: Vec::new(),
        }
    }

    /// Add a regulated parameter.
    pub fn add_parameter(&mut self, setpoint: Setpoint, actuator: Actuator) {
        assert_eq!(setpoint.name, actuator.parameter, "setpoint and actuator must have same parameter name");
        self.setpoints.push(setpoint);
        self.actuators.push(actuator);
    }

    /// Get the number of regulated parameters.
    pub fn len(&self) -> usize {
        self.setpoints.len()
    }

    /// True if no parameters.
    pub fn is_empty(&self) -> bool {
        self.setpoints.is_empty()
    }

    /// Process sensor readings and compute corrective actions.
    pub fn regulate(&self, readings: &[SensorReading]) -> Vec<ParameterStatus> {
        let mut statuses = Vec::new();

        for (i, sp) in self.setpoints.iter().enumerate() {
            let reading = readings.iter().find(|r| r.sensor_name == sp.name);
            let current = reading.map(|r| r.value).unwrap_or(sp.target);
            let deviation = sp.deviation(current);
            let is_stable = sp.is_satisfied(current);
            let action = self.actuators[i].compute_action(deviation);

            statuses.push(ParameterStatus {
                name: sp.name.clone(),
                current,
                target: sp.target,
                deviation,
                is_stable,
                action,
            });
        }

        statuses
    }

    /// Get the overall health status.
    pub fn health_status(&self, statuses: &[ParameterStatus]) -> HealthStatus {
        let all_stable = statuses.iter().all(|s| s.is_stable);
        if all_stable {
            return HealthStatus::Stable;
        }

        let has_critical = statuses.iter().any(|s| {
            s.deviation.abs() > (s.action.magnitude * 10.0 + 10.0)
        });

        if has_critical {
            HealthStatus::Critical
        } else {
            HealthStatus::Correcting
        }
    }

    /// Apply corrections from a status vector to get new parameter values.
    pub fn apply_corrections(&self, statuses: &[ParameterStatus]) -> Vec<f64> {
        statuses.iter().map(|s| s.action.apply(s.current)).collect()
    }

    /// Run one full regulation cycle: readings → corrections → new values.
    pub fn cycle(&self, readings: &[SensorReading]) -> (Vec<ParameterStatus>, Vec<f64>) {
        let statuses = self.regulate(readings);
        let new_values = self.apply_corrections(&statuses);
        (statuses, new_values)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_regulator() -> HomeostaticRegulator {
        let mut reg = HomeostaticRegulator::new();
        reg.add_parameter(
            Setpoint::new("temperature", 37.0, 1.0),
            Actuator::new("temperature", 2.0, 0.5),
        );
        reg.add_parameter(
            Setpoint::new("energy", 100.0, 10.0),
            Actuator::new("energy", 5.0, 0.3),
        );
        reg
    }

    #[test]
    fn test_regulator_stable() {
        let reg = make_regulator();
        let readings = vec![
            SensorReading::perfect("temperature", 37.0),
            SensorReading::perfect("energy", 100.0),
        ];
        let statuses = reg.regulate(&readings);
        assert!(statuses.iter().all(|s| s.is_stable));
        assert_eq!(reg.health_status(&statuses), HealthStatus::Stable);
    }

    #[test]
    fn test_regulator_correcting() {
        let reg = make_regulator();
        let readings = vec![
            SensorReading::perfect("temperature", 39.0),
            SensorReading::perfect("energy", 100.0),
        ];
        let statuses = reg.regulate(&readings);
        assert!(!statuses[0].is_stable);
        assert_eq!(statuses[0].action.action_type, crate::actuator::ActionType::Decrease);
        assert_eq!(reg.health_status(&statuses), HealthStatus::Correcting);
    }

    #[test]
    fn test_regulator_apply_corrections() {
        let reg = make_regulator();
        let readings = vec![
            SensorReading::perfect("temperature", 39.0),
            SensorReading::perfect("energy", 90.0),
        ];
        let (_statuses, new_vals) = reg.cycle(&readings);
        // Temperature was 39, above 37 by 2, gain 0.5 → decrease by 1
        assert!(new_vals[0] < 39.0);
        // Energy was 90, below 100 by 10, gain 0.3 → increase by 3, capped at 5
        assert!(new_vals[1] > 90.0);
    }

    #[test]
    fn test_regulator_empty() {
        let reg = HomeostaticRegulator::new();
        assert!(reg.is_empty());
        let statuses = reg.regulate(&[]);
        assert!(statuses.is_empty());
    }

    #[test]
    fn test_regulator_multiple_cycles_converge() {
        let reg = make_regulator();
        let mut values = vec![42.0, 80.0];
        for _ in 0..100 {
            let readings: Vec<SensorReading> = ["temperature", "energy"]
                .iter()
                .zip(&values)
                .map(|(&name, &v)| SensorReading::perfect(name, v))
                .collect();
            let (_, new_vals) = reg.cycle(&readings);
            values = new_vals;
        }
        // Should have converged toward setpoints
        assert!((values[0] - 37.0).abs() < 5.0);
        assert!((values[1] - 100.0).abs() < 15.0);
    }

    #[test]
    fn test_health_status_stable() {
        let reg = make_regulator();
        let statuses = vec![ParameterStatus {
            name: "temp".to_string(),
            current: 37.0,
            target: 37.0,
            deviation: 0.0,
            is_stable: true,
            action: Action::hold("temp"),
        }];
        assert_eq!(reg.health_status(&statuses), HealthStatus::Stable);
    }
}
