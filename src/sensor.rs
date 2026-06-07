//! Sensor readings with noise modeling.
//!
//! Sensors provide readings of agent parameters, with configurable
//! noise characteristics.

/// A sensor that reads a parameter value with optional noise.
#[derive(Debug, Clone)]
pub struct Sensor {
    /// Name of the parameter being sensed.
    pub name: String,
    /// Noise scale (standard deviation of Gaussian-like noise).
    pub noise_scale: f64,
    /// Whether the sensor is active.
    pub active: bool,
    /// Last raw reading before noise.
    last_raw: Option<f64>,
}

impl Sensor {
    /// Create a new noiseless sensor.
    pub fn new(name: &str) -> Self {
        Sensor {
            name: name.to_string(),
            noise_scale: 0.0,
            active: true,
            last_raw: None,
        }
    }

    /// Create a sensor with noise.
    pub fn with_noise(name: &str, noise_scale: f64) -> Self {
        Sensor {
            name: name.to_string(),
            noise_scale,
            active: true,
            last_raw: None,
        }
    }

    /// Read a value. For deterministic noise, uses a simple hash-based perturbation.
    pub fn read(&mut self, true_value: f64) -> SensorReading {
        self.last_raw = Some(true_value);
        let noise = if self.noise_scale > 0.0 {
            // Simple deterministic noise using the value itself as seed
            let hash = (true_value * 1000.0).to_bits() as f64;
            let pseudo = (hash * 12.9898 + 78.233).sin() * 43758.5453;
            (pseudo - pseudo.floor() - 0.5) * 2.0 * self.noise_scale
        } else {
            0.0
        };
        SensorReading {
            sensor_name: self.name.clone(),
            value: true_value + noise,
            noise_applied: noise,
            raw_value: true_value,
        }
    }

    /// Deactivate the sensor.
    pub fn deactivate(&mut self) {
        self.active = false;
    }

    /// Activate the sensor.
    pub fn activate(&mut self) {
        self.active = true;
    }
}

/// A single sensor reading.
#[derive(Debug, Clone)]
pub struct SensorReading {
    /// Name of the sensor.
    pub sensor_name: String,
    /// The noisy reading value.
    pub value: f64,
    /// The noise that was applied.
    pub noise_applied: f64,
    /// The raw (true) value before noise.
    pub raw_value: f64,
}

impl SensorReading {
    /// Create a perfect (noiseless) reading.
    pub fn perfect(name: &str, value: f64) -> Self {
        SensorReading {
            sensor_name: name.to_string(),
            value,
            noise_applied: 0.0,
            raw_value: value,
        }
    }

    /// Signal-to-noise ratio (returns infinity if no noise).
    pub fn snr(&self) -> f64 {
        if self.noise_applied.abs() < 1e-15 {
            return f64::INFINITY;
        }
        self.raw_value.abs() / self.noise_applied.abs()
    }
}

/// A multi-sensor array that reads multiple parameters simultaneously.
#[derive(Debug, Clone)]
pub struct SensorArray {
    sensors: Vec<Sensor>,
}

impl Default for SensorArray {
    fn default() -> Self {
        Self::new()
    }
}

impl SensorArray {
    /// Create an empty sensor array.
    pub fn new() -> Self {
        SensorArray { sensors: Vec::new() }
    }

    /// Add a sensor to the array.
    pub fn add(&mut self, sensor: Sensor) {
        self.sensors.push(sensor);
    }

    /// Read all sensors with the given true values.
    pub fn read_all(&mut self, true_values: &[f64]) -> Vec<SensorReading> {
        assert_eq!(true_values.len(), self.sensors.len(),
            "number of values must match number of sensors");
        self.sensors
            .iter_mut()
            .zip(true_values.iter())
            .filter(|(s, _)| s.active)
            .map(|(s, &v)| s.read(v))
            .collect()
    }

    /// Number of sensors.
    pub fn len(&self) -> usize {
        self.sensors.len()
    }

    /// True if no sensors.
    pub fn is_empty(&self) -> bool {
        self.sensors.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sensor_no_noise() {
        let mut s = Sensor::new("temp");
        let r = s.read(37.0);
        assert!((r.value - 37.0).abs() < 1e-10);
        assert!((r.noise_applied).abs() < 1e-10);
    }

    #[test]
    fn test_sensor_with_noise() {
        let mut s = Sensor::with_noise("temp", 0.5);
        let r = s.read(37.0);
        // Value should be close to 37 but not exactly
        assert!((r.value - 37.0).abs() < 2.0); // generous bound
        assert!(r.noise_applied.abs() <= 0.5);
    }

    #[test]
    fn test_sensor_reading_perfect() {
        let r = SensorReading::perfect("temp", 42.0);
        assert!((r.value - 42.0).abs() < 1e-10);
        assert!(r.snr().is_infinite());
    }

    #[test]
    fn test_sensor_reading_snr() {
        let r = SensorReading {
            sensor_name: "test".to_string(),
            value: 10.5,
            noise_applied: 0.5,
            raw_value: 10.0,
        };
        assert!((r.snr() - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_sensor_deactivate() {
        let mut s = Sensor::new("temp");
        s.deactivate();
        assert!(!s.active);
        s.activate();
        assert!(s.active);
    }

    #[test]
    fn test_sensor_array() {
        let mut arr = SensorArray::new();
        arr.add(Sensor::new("temp"));
        arr.add(Sensor::new("pressure"));
        assert_eq!(arr.len(), 2);
        let readings = arr.read_all(&[37.0, 101.3]);
        assert_eq!(readings.len(), 2);
        assert!((readings[0].value - 37.0).abs() < 1e-10);
        assert!((readings[1].value - 101.3).abs() < 1e-10);
    }

    #[test]
    fn test_sensor_array_empty() {
        let arr = SensorArray::new();
        assert!(arr.is_empty());
    }
}
