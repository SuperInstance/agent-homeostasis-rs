# agent-homeostasis-rs

**Homeostatic regulation for agent systems — maintaining stable internal conditions with PID-inspired control loops.**

This crate implements biological homeostasis as a software engineering pattern: define setpoints with tolerances for agent parameters, sense current values with configurable noise models, compute corrective actions through gain-based actuators, and drive everything with PID controllers featuring anti-windup and output clamping. A top-level `HomeostaticRegulator` orchestrates the full sense→compare→act→adapt cycle. With 37 tests covering sensors, actuators, PID control, setpoint drift, and integrated regulation, it gives any Rust agent the ability to maintain itself.

## Why This Matters

Every living system maintains homeostasis — the dynamic equilibrium that keeps internal conditions viable despite environmental change. AGI systems need the same capability: memory usage must stay bounded, attention must be allocated proportionally, response latency must remain acceptable, and confidence calibration must track reality. This crate models each regulated parameter as a control system with setpoints, sensors, and actuators — the same feedback loop architecture that keeps your body at 37°C. The PID controller is industry-proven (it runs most of the world's industrial processes), and its adaptation to software agents means your system doesn't just detect problems — it automatically corrects them before they cascade.

## Quick Start

```toml
# Cargo.toml
[dependencies]
agent-homeostasis-rs = "0.1.0"
```

```rust
use agent_homeostasis_rs::regulator::HomeostaticRegulator;
use agent_homeostasis_rs::sensor::{Sensor, SensorReading};
use agent_homeostasis_rs::actuator::Actuator;
use agent_homeostasis_rs::setpoint::Setpoint;
use agent_homeostasis_rs::control_loop::PidController;

// Define what "healthy" looks like
let temperature_sp = Setpoint::new("cpu_temp", 65.0, 10.0); // 65°C ± 10
let memory_sp = Setpoint::new("memory", 80.0, 15.0);        // 80% ± 15

// Create actuators that can correct deviations
let temp_actuator = Actuator::new("cpu_temp", 5.0, 0.5)  // max 5.0 correction, gain 0.5
    .with_dead_zone(2.0);
let mem_actuator = Actuator::new("memory", 10.0, 0.3)
    .with_dead_zone(5.0);

// Wire up the regulator
let mut regulator = HomeostaticRegulator::new();
regulator.add_parameter(temperature_sp, temp_actuator);
regulator.add_parameter(memory_sp, mem_actuator);

// Sense current state
let mut temp_sensor = Sensor::with_noise("cpu_temp", 0.5);
let mut mem_sensor = Sensor::new("memory");

let readings = vec![
    temp_sensor.read(72.0),   // Reading: ~72°C (with noise)
    mem_sensor.read(91.0),    // Reading: 91% (exact)
];

// Regulate: get status and corrective actions
let statuses = regulator.regulate(&readings);
for status in &statuses {
    println!("{}: {:.1} (target {:.1}) — {}",
        status.name, status.current, status.target,
        if status.is_stable { "STABLE" } else { "CORRECTING" }
    );
}

// Standalone PID control
let mut pid = PidController::new("latency", 1.0, 0.1, 0.05, 200.0)
    .with_integral_limit(50.0)
    .with_output_limit(100.0);
let result = pid.update(250.0); // Measured 250ms, target 200ms
println!("Correction: {:.2} (P={}, I={}, D={})",
    result.output, result.p_term, result.i_term, result.d_term);
```

## Architecture

| Module | Purpose |
|---|---|
| `setpoint` | Target values, tolerances, drift modeling, satisfaction checks |
| `sensor` | Noisy readings, deterministic perturbation, activation control |
| `actuator` | Corrective action generation, dead zones, magnitude clamping |
| `control_loop` | PID controller with anti-windup, output clamping, term decomposition |
| `regulator` | Top-level orchestrator: setpoints + actuators + sensor integration |

## API Tour

### Setpoints (`setpoint`)

- **`Setpoint { name, target, min, max, drift_rate, min_target, max_target }`**
  - `::new(name, target, tolerance)` — Symmetric tolerance band
  - `::with_bounds(name, target, min, max)` — Asymmetric bounds
  - `.with_drift(rate)` — Simulate setpoint drift over time
  - `.with_target_bounds(min, max)` — Clamp target range
  - `.is_satisfied(value) → bool` — Within tolerance?
  - `.deviation(value) → f64` — Signed deviation from target
  - `.normalized_deviation(value) → f64` — Scaled -1 to +1

### Sensors (`sensor`)

- **`Sensor { name, noise_scale, active }`**
  - `::new(name)` — Perfect sensor
  - `::with_noise(name, noise_scale)` — Deterministic hash-based noise
  - `.read(true_value) → SensorReading` — Take a reading
  - `.activate()`, `.deactivate()` — Toggle
- **`SensorReading { sensor_name, value, noise_applied, raw_value }`**

### Actuators (`actuator`)

- **`Actuator { parameter, max_magnitude, gain, dead_zone }`**
  - `::new(parameter, max_magnitude, gain)` — Basic actuator
  - `.with_dead_zone(dz)` — Ignore small deviations
  - `.compute_action(setpoint, reading) → Action` — Generate correction
- **`Action { parameter, action_type, magnitude }`**
  - `.apply(current) → f64` — Apply to a value
- **`ActionType`** — `Increase`, `Decrease`, `Hold`

### PID Controller (`control_loop`)

- **`PidController { name, kp, ki, kd, target, integral, prev_error, ... }`**
  - `::new(name, kp, ki, kd, target)` — Full PID
  - `.with_integral_limit(limit)` — Anti-windup
  - `.with_output_limit(limit)` — Clamp corrections
  - `.update(measurement) → PidResult` — Compute output
- **`PidResult { output, error, p_term, i_term, d_term }`** — Full decomposition

### Regulator (`regulator`)

- **`HomeostaticRegulator { setpoints, actuators }`**
  - `::new()` — Empty regulator
  - `.add_parameter(setpoint, actuator)` — Register a parameter
  - `.regulate(readings) → Vec<ParameterStatus>` — Full regulation cycle
  - `.health_status() → HealthStatus` — `Stable`, `Correcting`, `Critical`
- **`ParameterStatus { name, current, target, deviation, is_stable, action }`**
- **`HealthStatus`** — `Stable`, `Correcting`, `Critical`

## Performance

- Sensor reading: O(1) — hash-based noise is constant time
- Actuator computation: O(1) per parameter
- PID update: O(1) — simple arithmetic
- Full regulation cycle: O(n) for n parameters
- Suitable for thousands of regulated parameters at kHz update rates

## Ecosystem

Part of the **SuperInstance** family:

- [`constraint-dynamics-rs`](https://github.com/SuperInstance/constraint-dynamics-rs) — Constraints as dynamic rules
- [`agent-operations`](https://github.com/SuperInstance/agent-operations) — Agent lifecycle management
- [`renormalization-group-rs`](https://github.com/SuperInstance/renormalization-group-rs) — Multi-scale parameter analysis
- [`conservation-law-rs`](https://github.com/SuperInstance/conservation-law-rs) — Conservation laws for agent systems
- [`categorical-agents-rs`](https://github.com/SuperInstance/categorical-agents-rs) — Categorical composition of agents

## Ideas for Improvement

- **Adaptive PID tuning** — Ziegler-Nichols or Twiddle for automatic gain adjustment
- **Cascade control** — Nested PID loops for hierarchical parameters
- **Model predictive control** — Forward-looking control based on system model
- **Fault detection** — Anomaly detection on sensor readings
- **Multi-variable coupling** — Handle interactions between regulated parameters
- **Serde support** — Persist and restore regulator configurations
- **Async integration** — `tokio`-compatible periodic regulation loops

## License

MIT
