# agent-homeostasis-rs

Homeostatic regulation for agent systems. PID-inspired control loops, sensor arrays with noise, actuators with dead zones, and setpoint tracking — the machinery that keeps agents stable.

## The Core Idea

Biological systems maintain homeostasis: temperature, blood sugar, hydration stay within bounds despite perturbations. Agent systems need the same thing. Compute budgets, error rates, latency, memory — all must converge to setpoints or the agent degrades.

This crate provides the building blocks:

- **Setpoint**: target value + tolerance band + drift
- **Sensor**: reads parameters with configurable noise
- **Actuator**: computes corrective actions with gain and dead zones
- **PID Controller**: proportional + integral + derivative with anti-windup
- **Regulator**: ties setpoints, sensors, and actuators together
- **Control Loop**: multi-parameter PID simulation

## Architecture

```
agent-homeostasis-rs
├── lib.rs          — Module declarations
├── setpoint.rs     — Setpoint, SetpointTracker
├── sensor.rs       — Sensor, SensorReading, SensorArray
├── actuator.rs     — Actuator, Action, ActionType
├── control_loop.rs — PidController, PidResult, ControlLoop
└── regulator.rs    — HomeostaticRegulator, ParameterStatus, HealthStatus
```

## PID Controller Internals

The PID controller is the heart of the crate. It computes three terms:

```
output = Kp * error + Ki * ∫error dt + Kd * d(error)/dt
```

```rust
use agent_homeostasis_rs::control_loop::PidController;

fn pid_basics() {
    let mut pid = PidController::new("temperature", 2.0, 0.5, 0.1, 37.0);

    // First measurement: 30.0°C (too cold)
    let result = pid.update(30.0);
    println!("Error: {:.1}", result.error);          // 7.0
    println!("P term: {:.2}", result.p_term);         // 14.0
    println!("I term: {:.2}", result.i_term);         // 3.5
    println!("D term: {:.2}", result.d_term);         // 0.0 (first tick)
    println!("Output: {:.2}", result.output);         // 17.5

    // Second measurement: 35.0°C (getting closer)
    let result = pid.update(35.0);
    println!("Error: {:.1}", result.error);           // 2.0
    println!("D term: {:.2}", result.d_term);         // 0.1 * (2.0 - 7.0) = -0.5

    // Third measurement: 37.0°C (at setpoint)
    let result = pid.update(37.0);
    println!("Error: {:.1}", result.error);           // 0.0
    println!("Output: {:.2}", result.output);         // small residual from I
}
```

### Anti-Windup

The integral term can accumulate to extreme values during sustained errors. Anti-windup clamps it:

```rust
use agent_homeostasis_rs::control_loop::PidController;

fn anti_windup() {
    let mut pid = PidController::new("budget", 0.0, 1.0, 0.0, 100.0)
        .with_integral_limit(5.0);

    // Sustained large error
    pid.update(0.0); // error = 100, integral would be 100 but clamped to 5
    let result = pid.update(0.0);
    println!("I term (clamped): {:.1}", result.i_term); // 5.0

    // Compare without clamping:
    let mut pid_unclamped = PidController::new("budget", 0.0, 1.0, 0.0, 100.0);
    pid_unclamped.update(0.0);
    pid_unclamped.update(0.0);
    // integral = 200, i_term = 200 — way too aggressive
}
```

### Output Limiting

```rust
use agent_homeostasis_rs::control_loop::PidController;

fn output_limit() {
    let mut pid = PidController::new("rate", 10.0, 0.0, 0.0, 100.0)
        .with_output_limit(5.0);

    // Huge error, but output is clamped
    let result = pid.update(0.0);
    println!("Output (clamped): {:.1}", result.output); // 5.0
    println!("Raw P term: {:.1}", result.p_term);      // 1000.0
}
```

## Feedback Loops Converging

The `ControlLoop::simulate` method runs a multi-step simulation showing convergence:

```rust
use agent_homeostasis_rs::control_loop::{PidController, ControlLoop};

fn convergence_demo() {
    let mut cl = ControlLoop::new();
    cl.add(PidController::new("temperature", 0.5, 0.01, 0.1, 37.0));

    // Start at 20°C, simulate 200 ticks
    let trajectory = cl.simulate(&[20.0], 200);

    println!("Step  0: {:.2}°C", trajectory[0][0]);
    println!("Step 10: {:.2}°C", trajectory[10][0]);
    println!("Step 50: {:.2}°C", trajectory[50][0]);
    println!("Step200: {:.2}°C", trajectory[200][0]); // ~37.0

    // Verify convergence
    let final_temp = trajectory.last().unwrap()[0];
    assert!((final_temp - 37.0).abs() < 2.0, "Should converge to 37°C");
}
```

### Multi-Parameter Convergence

```rust
use agent_homeostasis_rs::control_loop::{PidController, ControlLoop};

fn multi_param_convergence() {
    let mut cl = ControlLoop::new();
    cl.add(PidController::new("temperature", 0.5, 0.01, 0.1, 37.0));
    cl.add(PidController::new("pressure", 0.3, 0.005, 0.05, 50.0));
    cl.add(PidController::new("humidity", 0.4, 0.008, 0.08, 60.0));

    // Start far from all setpoints
    let trajectory = cl.simulate(&[20.0, 80.0, 30.0], 300);

    let final = trajectory.last().unwrap();
    println!("Temperature: {:.2}°C (target: 37.0)", final[0]);
    println!("Pressure:    {:.2}    (target: 50.0)", final[1]);
    println!("Humidity:    {:.2}%   (target: 60.0)", final[2]);

    assert!((final[0] - 37.0).abs() < 5.0);
    assert!((final[1] - 50.0).abs() < 5.0);
    assert!((final[2] - 60.0).abs() < 5.0);
}
```

## Sensor Arrays with Noise

Sensors read true values but add noise. The `SensorArray` reads multiple parameters simultaneously:

```rust
use agent_homeostasis_rs::sensor::{Sensor, SensorReading, SensorArray};

fn sensor_demo() {
    // Noiseless sensor
    let mut temp_sensor = Sensor::new("temperature");
    let reading = temp_sensor.read(37.0);
    println!("Noiseless: value={:.1}, noise={:.4}",
        reading.value, reading.noise_applied);

    // Noisy sensor
    let mut noisy_sensor = Sensor::with_noise("temperature", 0.5);
    let reading = noisy_sensor.read(37.0);
    println!("Noisy: value={:.2}, noise={:.4}, SNR={:.1}",
        reading.value, reading.noise_applied, reading.snr());

    // Perfect reading (no sensor)
    let perfect = SensorReading::perfect("temperature", 37.0);
    assert!(perfect.snr().is_infinite());
}
```

### Multi-Sensor Array

```rust
use agent_homeostasis_rs::sensor::{Sensor, SensorArray};

fn sensor_array() {
    let mut arr = SensorArray::new();
    arr.add(Sensor::new("temperature"));        // noiseless
    arr.add(Sensor::with_noise("pressure", 2.0)); // ±2.0 noise
    arr.add(Sensor::with_noise("humidity", 3.0)); // ±3.0 noise

    let true_values = [37.0, 101.3, 55.0];
    let readings = arr.read_all(&true_values);

    for r in &readings {
        println!("{}: {:.2} (raw: {:.2}, noise: {:.2})",
            r.sensor_name, r.value, r.raw_value, r.noise_applied);
    }
}
```

## Actuators and Corrective Actions

Actuators compute what to do about deviations:

```rust
use agent_homeostasis_rs::actuator::{Actuator, ActionType, Action};

fn actuator_demo() {
    let actuator = Actuator::new("temperature", 2.0, 0.5);
    // max_correction=2.0, gain=0.5

    // Positive deviation: value is above target, decrease
    let action = actuator.compute_action(5.0);
    assert_eq!(action.action_type, ActionType::Decrease);
    println!("Deviation +5: {:?}, magnitude={:.2}", action.action_type, action.magnitude);
    // magnitude = min(5.0 * 0.5, 2.0) = 2.0 (capped)

    // Negative deviation: value is below target, increase
    let action = actuator.compute_action(-3.0);
    assert_eq!(action.action_type, ActionType::Increase);
    println!("Deviation -3: {:?}, magnitude={:.2}", action.action_type, action.magnitude);
    // magnitude = min(3.0 * 0.5, 2.0) = 1.5

    // Apply correction
    let new_val = action.apply(34.0); // 34 + 1.5 = 35.5
    println!("34.0 → {:.1}", new_val);
}
```

### Dead Zone

```rust
use agent_homeostasis_rs::actuator::Actuator;

fn dead_zone() {
    let actuator = Actuator::new("temperature", 2.0, 0.5)
        .with_dead_zone(1.0); // ignore deviations < 1.0

    let action = actuator.compute_action(0.5); // within dead zone
    assert_eq!(action.action_type, agent_homeostasis_rs::actuator::ActionType::Hold);
}
```

## Setpoint Tracking

Setpoints define targets, tolerance bands, and optional drift:

```rust
use agent_homeostasis_rs::setpoint::{Setpoint, SetpointTracker};

fn setpoint_demo() {
    let sp = Setpoint::new("temperature", 37.0, 1.0);
    // Target 37.0, tolerance ±1.0 → acceptable range [36.0, 38.0]

    assert!(sp.is_satisfied(36.5));  // within range
    assert!(sp.is_satisfied(38.0));  // at boundary
    assert!(!sp.is_satisfied(39.0)); // outside

    println!("Deviation at 35: {:.1}", sp.deviation(35.0));   // -2.0
    println!("Normalized at 38: {:.1}", sp.normalized_deviation(38.0)); // +1.0
}
```

### Drifting Setpoints

Setpoints can drift over time (simulating changing targets):

```rust
use agent_homeostasis_rs::setpoint::{Setpoint, SetpointTracker};

fn drifting_setpoint() {
    let sp = Setpoint::new("load", 50.0, 5.0)
        .with_drift(2.0)             // target increases by 2 per tick
        .with_target_bounds(0.0, 100.0); // but never exceeds 100

    let mut tracker = SetpointTracker::new(sp);
    for tick in 0..10 {
        let target = tracker.tick_drift();
        tracker.observe(50.0 + tick as f64); // observations track the drift
        println!("Tick {}: target={:.0}, satisfaction={:.0}%",
            tick, target, tracker.satisfaction_rate() * 100.0);
    }
}
```

### Tracker Statistics

```rust
use agent_homeostasis_rs::setpoint::{Setpoint, SetpointTracker};

fn tracker_stats() {
    let sp = Setpoint::new("temperature", 37.0, 2.0);
    let mut tracker = SetpointTracker::new(sp);

    tracker.observe(36.0); // satisfied
    tracker.observe(37.5); // satisfied
    tracker.observe(38.5); // satisfied
    tracker.observe(40.0); // not satisfied
    tracker.observe(35.0); // satisfied

    println!("Satisfaction rate: {:.0}%", tracker.satisfaction_rate() * 100.0); // 80%
    println!("Mean deviation:    {:.2}", tracker.mean_deviation());
    println!("Current value:     {:?}", tracker.current_value());
}
```

## The Regulator: Full Feedback Loop

The `HomeostaticRegulator` ties everything together:

```rust
use agent_homeostasis_rs::regulator::HomeostaticRegulator;
use agent_homeostasis_rs::setpoint::Setpoint;
use agent_homeostasis_rs::actuator::Actuator;
use agent_homeostasis_rs::sensor::SensorReading;

fn regulated_agent() {
    let mut reg = HomeostaticRegulator::new();
    reg.add_parameter(
        Setpoint::new("temperature", 37.0, 1.0),
        Actuator::new("temperature", 2.0, 0.5),
    );
    reg.add_parameter(
        Setpoint::new("energy", 100.0, 10.0),
        Actuator::new("energy", 5.0, 0.3),
    );

    // Readings: temperature is high, energy is low
    let readings = vec![
        SensorReading::perfect("temperature", 39.0), // 2 above target
        SensorReading::perfect("energy", 90.0),       // 10 below target
    ];

    let (statuses, new_values) = reg.cycle(&readings);

    for s in &statuses {
        println!("{}: current={:.1} target={:.1} deviation={:.1} stable={}",
            s.name, s.current, s.target, s.deviation, s.is_stable);
        println!("  action: {:?}, magnitude={:.2}", s.action.action_type, s.action.magnitude);
    }

    println!("New temperature: {:.2}", new_values[0]); // < 39.0
    println!("New energy:      {:.2}", new_values[1]); // > 90.0
}
```

### Multi-Agent Ecology Self-Regulating

```rust
use agent_homeostasis_rs::regulator::HomeostaticRegulator;
use agent_homeostasis_rs::setpoint::Setpoint;
use agent_homeostasis_rs::actuator::Actuator;
use agent_homeostasis_rs::sensor::SensorReading;
use agent_homeostasis_rs::control_loop::HealthStatus;

fn ecology() {
    let mut reg = HomeostaticRegulator::new();
    reg.add_parameter(
        Setpoint::new("cpu_load", 0.6, 0.1),
        Actuator::new("cpu_load", 0.2, 0.5),
    );
    reg.add_parameter(
        Setpoint::new("memory", 0.7, 0.15),
        Actuator::new("memory", 0.1, 0.3),
    );
    reg.add_parameter(
        Setpoint::new("error_rate", 0.01, 0.02),
        Actuator::new("error_rate", 0.05, 0.8),
    );

    // Simulate perturbation: CPU spike, memory leak
    let mut values = vec![0.9, 0.95, 0.05];

    for step in 0..50 {
        let readings: Vec<SensorReading> = ["cpu_load", "memory", "error_rate"]
            .iter()
            .zip(&values)
            .map(|(&name, &v)| SensorReading::perfect(name, v))
            .collect();

        let (statuses, new_values) = reg.cycle(&readings);
        values = new_values;

        if step % 10 == 0 {
            let health = reg.health_status(&statuses);
            println!("Step {:3}: cpu={:.2} mem={:.2} err={:.3} [{:?}]",
                step, values[0], values[1], values[2], health);
        }
    }

    // After 50 cycles, system should be converging toward setpoints
    assert!(values[0] < 0.9); // CPU load decreased
    assert!(values[1] < 0.95); // memory decreased
}
```

### Health Status

```rust
use agent_homeostasis_rs::regulator::{HomeostaticRegulator, HealthStatus, ParameterStatus};
use agent_homeostasis_rs::actuator::{Action, ActionType};

fn health_check() {
    let reg = HomeostaticRegulator::new();

    // All stable
    let statuses = vec![ParameterStatus {
        name: "temp".into(),
        current: 37.0,
        target: 37.0,
        deviation: 0.0,
        is_stable: true,
        action: Action::hold("temp"),
    }];
    assert_eq!(reg.health_status(&statuses), HealthStatus::Stable);

    // Some correcting
    let statuses = vec![ParameterStatus {
        name: "temp".into(),
        current: 39.0,
        target: 37.0,
        deviation: 2.0,
        is_stable: false,
        action: Action::new("temp", ActionType::Decrease, 1.0),
    }];
    assert_eq!(reg.health_status(&statuses), HealthStatus::Correcting);
}
```

## API Reference

### `control_loop` Module

| Type/Function | Description |
|---|---|
| `PidController` | PID controller for a single parameter |
| `PidResult` | Output + error + P/I/D term breakdown |
| `ControlLoop` | Multi-parameter PID control loop |
| `PidController::new(name, kp, ki, kd, target)` | Create a PID controller |
| `.with_integral_limit(limit)` | Set anti-windup clamp |
| `.with_output_limit(limit)` | Set output clamp |
| `controller.update(measurement)` | Compute one PID step |
| `control_loop.simulate(initial, steps)` | Run full trajectory |
| `control_loop.tick(measurements)` | One step for all controllers |

### `sensor` Module

| Type/Function | Description |
|---|---|
| `Sensor` | Single parameter sensor with optional noise |
| `SensorReading` | Reading with value, noise, raw, and SNR |
| `SensorArray` | Multi-sensor simultaneous reading |
| `Sensor::new(name)` | Noiseless sensor |
| `Sensor::with_noise(name, scale)` | Noisy sensor |
| `SensorReading::perfect(name, value)` | Noiseless reading |
| `reading.snr()` | Signal-to-noise ratio |

### `actuator` Module

| Type/Function | Description |
|---|---|
| `Actuator` | Corrective action computer |
| `Action` | A specific correction (type + magnitude) |
| `ActionType` | Increase / Decrease / Hold |
| `Actuator::new(param, max_mag, gain)` | Create actuator |
| `.with_dead_zone(dz)` | Ignore small deviations |
| `actuator.compute_action(deviation)` | Get correction |
| `action.apply(current)` | Apply to get new value |

### `setpoint` Module

| Type/Function | Description |
|---|---|
| `Setpoint` | Target + tolerance + drift config |
| `SetpointTracker` | Track satisfaction rate over time |
| `Setpoint::new(name, target, tolerance)` | Symmetric tolerance |
| `.with_drift(rate)` | Enable target drift per tick |
| `.with_target_bounds(min, max)` | Clamp target drift |
| `sp.is_satisfied(value)` | Within tolerance? |
| `sp.deviation(value)` | Signed deviation |
| `sp.normalized_deviation(value)` | -1 to +1 |

### `regulator` Module

| Type/Function | Description |
|---|---|
| `HomeostaticRegulator` | Ties setpoints + actuators |
| `ParameterStatus` | Per-parameter status after regulation |
| `HealthStatus` | Stable / Correcting / Critical |
| `regulator.cycle(readings)` | Full read → correct → new values |
| `regulator.regulate(readings)` | Get statuses only |
| `regulator.health_status(statuses)` | Overall health |

## Building and Testing

```bash
cargo build
cargo test
```

## License

MIT
