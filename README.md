# agent-homeostasis-rs

Homeostatic regulation for agent systems — maintaining stable internal conditions with PID-inspired control loops, sensor arrays, actuator feedback, and setpoint tracking.

## What It Does

Biological systems maintain homeostasis: temperature, blood sugar, pH all stay within narrow ranges despite external perturbations. This crate gives AI agents the same capability. Each agent parameter (energy, attention, throughput, etc.) has a target setpoint, a sensor to measure it, and an actuator to correct deviations — all wired together through a PID control loop.

Core modules:

- **`regulator`** — `HomeostaticRegulator` monitors multiple parameters, compares against setpoints, generates corrective actions
- **`sensor`** — `Sensor` / `SensorArray` with configurable noise modeling
- **`actuator`** — `Actuator` with dead zones, gain, and magnitude limits
- **`control_loop`** — `PidController` with anti-windup, output clamping, multi-parameter simulation
- **`setpoint`** — `Setpoint` with tolerance bounds, drift, and tracking history

## Quick Start

```toml
[dependencies]
agent-homeostasis-rs = { git = "https://github.com/SuperInstance/agent-homeostasis-rs" }
```

```rust
use agent_homeostasis_rs::regulator::HomeostaticRegulator;
use agent_homeostasis_rs::sensor::SensorReading;
use agent_homeostasis_rs::setpoint::Setpoint;
use agent_homeostasis_rs::actuator::Actuator;

fn main() {
    // Create a regulator with two parameters
    let mut reg = HomeostaticRegulator::new();
    reg.add_parameter(
        Setpoint::new("temperature", 37.0, 1.0),      // target 37, ±1.0 tolerance
        Actuator::new("temperature", 2.0, 0.5),        // max correction 2.0, gain 0.5
    );
    reg.add_parameter(
        Setpoint::new("energy", 100.0, 10.0),          // target 100, ±10.0 tolerance
        Actuator::new("energy", 5.0, 0.3),             // max correction 5.0, gain 0.3
    );

    // Feed sensor readings
    let readings = vec![
        SensorReading::perfect("temperature", 39.0),   // 2 above target
        SensorReading::perfect("energy", 85.0),         // 15 below target
    ];

    // One regulation cycle
    let (statuses, new_values) = reg.cycle(&readings);

    for status in &statuses {
        println!("{}: current={:.1} target={:.1} deviation={:.1} stable={}",
            status.name, status.current, status.target, status.deviation, status.is_stable);
    }
}
```

## PID Controller

Proportional-Integral-Derivative controller adapted for agent parameters. Each term handles a different aspect of regulation:

- **P (proportional):** Corrects based on current error magnitude
- **I (integral):** Eliminates steady-state error by accumulating past errors
- **D (derivative):** Anticipates future error by tracking rate of change

```rust
use agent_homeostasis_rs::control_loop::PidController;

let mut pid = PidController::new("temperature", 0.5, 0.01, 0.1, 37.0);
//                                      kp      ki    kd   target

let result = pid.update(35.0);
println!("Output: {:.4}", result.output);     // correction to apply
println!("Error:  {:.4}", result.error);      // target - measurement = 2.0
println!("P term: {:.4}", result.p_term);     // 0.5 * 2.0 = 1.0
println!("I term: {:.4}", result.i_term);     // small, just started
println!("D term: {:.4}", result.d_term);     // 0 (first reading)
```

### Anti-Windup

The integral accumulator is clamped to prevent runaway accumulation:

```rust
use agent_homeostasis_rs::control_loop::PidController;

let mut pid = PidController::new("energy", 0.0, 1.0, 0.0, 100.0)
    .with_integral_limit(5.0);  // integral capped at ±5

// Large sustained error won't cause windup
for _ in 0..100 {
    pid.update(0.0);  // error = 100 each time
}
let result = pid.update(0.0);
println!("I term (clamped): {:.4}", result.i_term); // ≤ 5.0
```

### Output Clamping

```rust
let pid = PidController::new("attention", 10.0, 0.0, 0.0, 50.0)
    .with_output_limit(3.0);  // max correction per tick = 3.0
```

### Setpoint Changes

```rust
let mut pid = PidController::new("throughput", 1.0, 0.1, 0.5, 100.0);
pid.set_target(120.0);  // dynamically raise target
pid.reset();            // clear integral and derivative state
```

## Multi-Parameter Control Loop

Run multiple PID controllers in lockstep with a single `tick()` call.

```rust
use agent_homeostasis_rs::control_loop::{ControlLoop, PidController};

let mut cl = ControlLoop::new();
cl.add(PidController::new("temperature", 0.5, 0.01, 0.1, 37.0));
cl.add(PidController::new("pressure",    0.3, 0.005, 0.05, 101.3));
cl.add(PidController::new("energy",      0.8, 0.02, 0.2, 100.0));

// One tick
let measurements = vec![36.0, 102.0, 80.0];
let results = cl.tick(&measurements);
for r in &results {
    println!("Output: {:.4}, Error: {:.4}", r.output, r.error);
}
```

### Simulation

Run a closed-loop simulation for N steps. Starting values converge toward setpoints.

```rust
use agent_homeostasis_rs::control_loop::{ControlLoop, PidController};

let mut cl = ControlLoop::new();
cl.add(PidController::new("temp", 0.5, 0.01, 0.1, 100.0));
cl.add(PidController::new("pressure", 0.3, 0.005, 0.05, 50.0));

let initial = vec![80.0, 30.0];
let trajectory = cl.simulate(&initial, 200);

println!("Step 0:  temp={:.2}, pressure={:.2}", trajectory[0][0], trajectory[0][1]);
println!("Final:   temp={:.2}, pressure={:.2}", trajectory.last().unwrap()[0], trajectory.last().unwrap()[1]);
// Both converge close to setpoints
```

## Sensors

Sensors read parameter values with optional noise modeling.

```rust
use agent_homeostasis_rs::sensor::{Sensor, SensorReading};

// Perfect (noiseless) sensor
let mut s = Sensor::new("temperature");
let reading = s.read(37.0);
println!("Value: {:.2}, Noise: {:.4}", reading.value, reading.noise_applied);

// Sensor with noise (σ = 0.5)
let mut noisy = Sensor::with_noise("temperature", 0.5);
let noisy_reading = noisy.read(37.0);
println!("Noisy value: {:.2}", noisy_reading.value);
println!("Signal-to-noise ratio: {:.1}", noisy_reading.snr());
```

### Sensor Readings

```rust
use agent_homeostasis_rs::sensor::SensorReading;

// Create a perfect reading directly
let r = SensorReading::perfect("energy", 95.0);
assert_eq!(r.value, 95.0);
assert!(r.snr().is_infinite());  // no noise → infinite SNR
```

### Sensor Array

Read multiple parameters simultaneously:

```rust
use agent_homeostasis_rs::sensor::{SensorArray, Sensor};

let mut array = SensorArray::new();
array.add(Sensor::new("temperature"));
array.add(Sensor::with_noise("pressure", 0.1));
array.add(Sensor::new("energy"));

let readings = array.read_all(&[37.0, 101.3, 85.0]);
for r in &readings {
    println!("{}: {:.2}", r.sensor_name, r.value);
}
```

## Actuators

Actuators compute corrective actions based on deviation from setpoint.

```rust
use agent_homeostasis_rs::actuator::{Actuator, ActionType};

let act = Actuator::new("temperature", 2.0, 0.5); // max magnitude 2.0, gain 0.5

// Deviation = 5.0 (above target)
let action = act.compute_action(5.0);
assert_eq!(action.action_type, ActionType::Decrease);
println!("Magnitude: {:.2}", action.magnitude); // min(5.0 * 0.5, 2.0) = 2.0

// Apply action
let new_val = action.apply(42.0); // 42 - 2 = 40
```

### Dead Zone

Small deviations within the dead zone produce no action:

```rust
use agent_homeostasis_rs::actuator::{Actuator, ActionType};

let act = Actuator::new("energy", 5.0, 1.0).with_dead_zone(2.0);

let action = act.compute_action(1.5); // within dead zone
assert_eq!(action.action_type, ActionType::Hold);

let action = act.compute_action(3.0); // outside dead zone
assert_ne!(action.action_type, ActionType::Hold);
```

### Direct Correction

```rust
use agent_homeostasis_rs::actuator::Actuator;

let act = Actuator::new("temp", 10.0, 0.5);
let corrected = act.correct(40.0, 4.0); // current=40, deviation=4 → decrease by 2
println!("Corrected: {:.1}", corrected); // 38.0
```

## Setpoints

Define target values and acceptable ranges for agent parameters.

```rust
use agent_homeostasis_rs::setpoint::Setpoint;

let sp = Setpoint::new("temperature", 37.0, 1.0);
// target=37.0, acceptable range=[36.0, 38.0]

assert!(sp.is_satisfied(37.0));
assert!(sp.is_satisfied(36.5));
assert!(!sp.is_satisfied(39.0));

println!("Deviation: {:.1}", sp.deviation(39.0));         // +2.0
println!("Normalized: {:.2}", sp.normalized_deviation(38.0)); // +1.0 (at boundary)
```

### Asymmetric Bounds

```rust
use agent_homeostasis_rs::setpoint::Setpoint;

let sp = Setpoint::with_bounds("energy", 100.0, 50.0, 120.0);
// target=100, min=50, max=120
```

### Drifting Setpoints

Setpoints can drift over time (simulating changing targets):

```rust
use agent_homeostasis_rs::setpoint::Setpoint;

let sp = Setpoint::new("throughput", 100.0, 5.0)
    .with_drift(0.5)                     // target increases by 0.5 per tick
    .with_target_bounds(80.0, 120.0);    // clamped to [80, 120]
```

### Setpoint Tracker

Monitor satisfaction rate over time:

```rust
use agent_homeostasis_rs::setpoint::{Setpoint, SetpointTracker};

let sp = Setpoint::new("energy", 100.0, 10.0);
let mut tracker = SetpointTracker::new(sp);

tracker.observe(95.0);   // satisfied
tracker.observe(105.0);  // satisfied
tracker.observe(85.0);   // not satisfied (outside ±10)
tracker.observe(110.0);  // satisfied

println!("Satisfaction rate: {:.1}%", tracker.satisfaction_rate() * 100.0); // 75%
println!("Mean deviation: {:.1}", tracker.mean_deviation());
println!("Current value: {:?}", tracker.current_value()); // Some(110.0)

// Apply drift
let new_target = tracker.tick_drift();
```

## Homeostatic Regulator

The top-level regulator wires setpoints, actuators, and sensor readings together.

```rust
use agent_homeostasis_rs::regulator::{HomeostaticRegulator, HealthStatus};
use agent_homeostasis_rs::sensor::SensorReading;
use agent_homeostasis_rs::setpoint::Setpoint;
use agent_homeostasis_rs::actuator::Actuator;

let mut reg = HomeostaticRegulator::new();
reg.add_parameter(
    Setpoint::new("temperature", 37.0, 1.0),
    Actuator::new("temperature", 2.0, 0.5),
);
reg.add_parameter(
    Setpoint::new("energy", 100.0, 10.0),
    Actuator::new("energy", 5.0, 0.3),
);

// Check health
let readings = vec![
    SensorReading::perfect("temperature", 37.0),
    SensorReading::perfect("energy", 100.0),
];
let statuses = reg.regulate(&readings);
assert_eq!(reg.health_status(&statuses), HealthStatus::Stable);
```

### Convergence Over Multiple Cycles

```rust
use agent_homeostasis_rs::regulator::HomeostaticRegulator;
use agent_homeostasis_rs::sensor::SensorReading;
use agent_homeostasis_rs::setpoint::Setpoint;
use agent_homeostasis_rs::actuator::Actuator;

let mut reg = HomeostaticRegulator::new();
reg.add_parameter(
    Setpoint::new("temp", 37.0, 1.0),
    Actuator::new("temp", 2.0, 0.5),
);
reg.add_parameter(
    Setpoint::new("energy", 100.0, 10.0),
    Actuator::new("energy", 5.0, 0.3),
);

let mut values = vec![42.0, 80.0];
for cycle in 0..100 {
    let readings: Vec<SensorReading> = ["temp", "energy"]
        .iter()
        .zip(&values)
        .map(|(&name, &v)| SensorReading::perfect(name, v))
        .collect();
    let (_, new_vals) = reg.cycle(&readings);
    values = new_vals;
}
println!("After 100 cycles: temp={:.2}, energy={:.2}", values[0], values[1]);
// Should converge toward 37.0 and 100.0
```

## Conservation Budgets

Agent parameters under conservation law constraints use bounded setpoints and limited actuators. The combination of `with_target_bounds` and `Actuator::max_magnitude` ensures total resource consumption stays within budget.

```rust
use agent_homeostasis_rs::setpoint::Setpoint;
use agent_homeostasis_rs::actuator::Actuator;

// Energy budget: target 100, must stay in [0, 200], max correction 5 per tick
let energy_sp = Setpoint::new("energy", 100.0, 20.0)
    .with_target_bounds(0.0, 200.0);
let energy_act = Actuator::new("energy", 5.0, 0.3); // max 5, gain 0.3

// Conservation law: total corrections across all agents must not exceed fleet budget
let fleet_budget = 1000.0;
let agent_max_correction = 5.0;
let max_agents = (fleet_budget / agent_max_correction) as usize;
println!("Max agents under conservation budget: {}", max_agents);
```

## si-cli Integration

`si-cli` uses agent-homeostasis-rs to manage agent regulation policies:

```bash
# Set agent parameters
si agent setpoint --name energy --target 100 --tolerance 10
si agent setpoint --name attention --target 0.8 --tolerance 0.1

# Check agent health
si agent health
# → STABLE: all parameters within tolerance

# Run regulation cycle
si agent regulate --cycles 100
```

## si-fleet-api Integration

The fleet API exposes homeostasis data over HTTP:

```
GET /v1/agents/{agent_id}/homeostasis
→ {
    "parameters": [
        {"name": "energy", "current": 85.0, "target": 100.0, "stable": false},
        {"name": "attention", "current": 0.82, "target": 0.8, "stable": true}
    ],
    "health": "Correcting"
}
```

## Supabase Integration

Agent homeostasis state is stored in Supabase:

```sql
CREATE TABLE agent_setpoints (
    agent_id UUID REFERENCES agents(id),
    parameter TEXT NOT NULL,
    target FLOAT NOT NULL,
    tolerance FLOAT NOT NULL,
    min_bound FLOAT,
    max_bound FLOAT,
    PRIMARY KEY (agent_id, parameter)
);

CREATE TABLE agent_regulation_log (
    id UUID DEFAULT gen_random_uuid() PRIMARY KEY,
    agent_id UUID REFERENCES agents(id),
    parameter TEXT NOT NULL,
    value FLOAT NOT NULL,
    target FLOAT NOT NULL,
    deviation FLOAT NOT NULL,
    correction FLOAT NOT NULL,
    created_at TIMESTAMPTZ DEFAULT NOW()
);
```

## Architecture

```
src/
├── lib.rs           — Module declarations
├── regulator.rs     — HomeostaticRegulator, ParameterStatus, HealthStatus
├── sensor.rs        — Sensor, SensorReading, SensorArray
├── actuator.rs      — Actuator, Action, ActionType
├── control_loop.rs  — PidController, PidResult, ControlLoop
└── setpoint.rs      — Setpoint, SetpointTracker
```

## Testing

```bash
cargo test
```

Tests cover:
- PID proportional, integral, derivative terms
- Anti-windup and output clamping
- Control loop convergence simulation
- Sensor noise and SNR
- Actuator dead zone and magnitude limits
- Setpoint satisfaction, deviation, drift
- Regulator health status transitions
- Multi-cycle convergence

## License

MIT
