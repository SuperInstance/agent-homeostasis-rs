# INTEGRATION.md — agent-homeostasis-rs × conservation-law-rs × fleet-warden-rs

**Agent homeostasis** provides PID control loops, sensors, actuators, and
regulators to maintain stable internal conditions. It connects to
Lagrangian mechanics for energy-based setpoints and to fleet-warden for
infrastructure health regulation.

## Synergy Map

```
conservation-law-rs            agent-homeostasis-rs           fleet-warden-rs
┌──────────────────┐          ┌──────────────────────┐       ┌─────────────────┐
│ AgentState        │◄────────►│ HomeostaticRegulator │◄─────►│ DiskBudget      │
│ total_energy      │          │ PidController        │       │ full_scan       │
│ SymplecticIntegr  │          │ SensorArray          │       │ disk_budget     │
│ verify_noether    │          │ Actuator             │       │ BudgetSample    │
└──────────────────┘          │ Setpoint             │       └─────────────────┘
                              │ ControlLoop          │
                              └──────────────────────┘
```

## Key Insight

An agent's internal state (energy, temperature, load) must stay within
bounds. Homeostatic regulation treats these as setpoints with feedback
correction. Conservation-law provides the physical model (energy as a
conserved quantity), and fleet-warden extends this to infrastructure
(disk, memory) as regulated parameters.

## Example 1: Energy-Based Setpoint Regulation

Use total energy from conservation-law as the regulated parameter.

```rust
use conservation_law::lagrangian::{AgentState, MechanicalLagrangian, total_energy};
use agent_homeostasis::{PidController, Setpoint, SensorReading};

fn regulate_energy() {
    let lagrangian = MechanicalLagrangian {
        mass: 1.0,
        potential_fn: |q: &[f64; 1]| 0.5 * q[0] * q[0],
    };
    let state = AgentState::new([2.0], [0.0]);
    let e0 = total_energy(&lagrangian, &state);

    let mut pid = PidController::new("energy", 0.5, 0.05, 0.1, e0);
    let measurement = total_energy(&lagrangian, &AgentState::new([2.1], [0.0]));

    let result = pid.update(measurement);
    println!("Energy error: {:.4}, correction: {:.4}", result.error, result.output);
}
```

## Example 2: Multi-Parameter Agent Regulation

Regulate temperature and energy simultaneously with a ControlLoop.

```rust
use agent_homeostasis::{ControlLoop, PidController, SensorArray, Sensor};

fn regulate_agent() {
    let mut loop_ctrl = ControlLoop::new();
    loop_ctrl.add(PidController::new("temp", 0.3, 0.02, 0.1, 37.0));
    loop_ctrl.add(PidController::new("energy", 0.2, 0.01, 0.05, 100.0));

    let mut sensors = SensorArray::new();
    sensors.add(Sensor::new("temp"));
    sensors.add(Sensor::new("energy"));

    let mut values = vec![39.0, 85.0];
    for _ in 0..50 {
        let readings = sensors.read_all(&values);
        let results = loop_ctrl.tick(&readings.iter().map(|r| r.value).collect::<Vec<_>>());
        for (i, r) in results.iter().enumerate() {
            values[i] += r.output;
        }
    }
    println!("Final: temp={:.1}, energy={:.1}", values[0], values[1]);
}
```

## Example 3: HomeostaticRegulator with Fleet-Warden Health

Use the high-level regulator to monitor disk, memory, and CPU.

```rust
use agent_homeostasis::{HomeostaticRegulator, Setpoint, Actuator, SensorReading};
use fleet_warden::budget::disk_budget;

fn fleet_health_regulator() {
    let mut reg = HomeostaticRegulator::new();

    let disk_budget = disk_budget().unwrap();
    let disk_used_pct = disk_budget.used_pct;

    reg.add_parameter(
        Setpoint::new("disk_usage", 70.0, 10.0),
        Actuator::new("disk_usage", 5.0, 0.5),
    );

    let readings = vec![
        SensorReading::perfect("disk_usage", disk_used_pct),
    ];

    let (statuses, new_values) = reg.cycle(&readings);
    for s in &statuses {
        println!("{}: {:.1} -> target {:.1}, action: {:?}",
            s.name, s.current, s.target, s.action.action_type);
    }
}
```

## Cargo.toml Wiring

```toml
[dependencies]
agent-homeostasis = { git = "https://github.com/SuperInstance/agent-homeostasis-rs" }
conservation-law = { git = "https://github.com/SuperInstance/conservation-law-rs" }
fleet-warden = { git = "https://github.com/SuperInstance/fleet-warden-rs" }
```

## Design Patterns

### Pattern: Cascading Setpoint Drift

Model seasonal load changes by drifting setpoints gradually:

```rust
use agent_homeostasis::setpoint::SetpointTracker;
use agent_homeostasis::Setpoint;

fn seasonal_load_adjustment(base_load: f64) -> Vec<f64> {
    let mut tracker = SetpointTracker::new(
        Setpoint::new("load", base_load, 10.0).with_drift(0.5)
    );

    let mut targets = vec![];
    for _ in 0..24 {
        targets.push(tracker.tick_drift());
    }
    targets
}
```

### Pattern: Sensor Redundancy

Combine multiple sensors to tolerate individual failures:

```rust
use agent_homeostasis::sensor::SensorArray;

fn fault_tolerant_reading(sensors: &mut SensorArray) -> f64 {
    let readings = sensors.read_all();
    let median = readings.iter().cloned()
        .collect::<Vec<_>>()
        .sort_by(|a, b| a.partial_cmp(b).unwrap());
    readings[readings.len() / 2]
}
```
