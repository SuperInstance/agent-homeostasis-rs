# Integration Guide: agent-homeostasis-rs

## What This Crate Provides

Homeostatic regulation for agent systems — maintaining stable internal conditions through sensor feedback, PID control, and actuator correction.

- **`regulator::HomeostaticRegulator`** — Monitors multiple parameters against setpoints and generates corrective actions. Methods: `new()`, `add_parameter()`, `regulate()`, `len()`, `is_empty()`.
- **`regulator::ParameterStatus`** — Per-parameter status: `name`, `current`, `target`, `deviation`, `is_stable`, `action`.
- **`regulator::HealthStatus`** — Fleet-wide health: `Stable`, `Correcting`, `Critical`.
- **`sensor::Sensor`** — Reads parameter values with configurable deterministic noise. Methods: `new()`, `with_noise()`, `read()`, `activate()`, `deactivate()`.
- **`sensor::SensorReading`** — A single reading: `sensor_name`, `value`, `noise_applied`, `raw_value`.
- **`actuator::Actuator`** — Produces corrective actions with gain, max magnitude, and dead zone. Methods: `new()`, `with_dead_zone()`, `compute_action()`.
- **`actuator::Action`** — Corrective action: `parameter`, `action_type` (`Increase`/`Decrease`/`Hold`), `magnitude`. Methods: `new()`, `hold()`, `apply()`.
- **`control_loop::PidController`** — PID controller with anti-windup and output clamping. Methods: `new()`, `with_integral_limit()`, `with_output_limit()`, `update()`.
- **`control_loop::PidResult`** — PID output breakdown: `output`, `error`, `p_term`, `i_term`, `d_term`.
- **`control_loop::ControlLoop`** — Multi-parameter control loop simulation over ticks.
- **`setpoint::Setpoint`** — Target value with symmetric/asymmetric tolerance, drift rate, and bounds. Methods: `new()`, `with_bounds()`, `with_drift()`, `with_target_bounds()`, `is_satisfied()`, `deviation()`, `normalized_deviation()`.
- **`setpoint::SetpointTracker`** — Tracks setpoint drift over time with `tick_drift()`.

## How to Add This Crate

```bash
cargo add agent-homeostasis
```

```rust
use agent_homeostasis::{
    HomeostaticRegulator, Sensor, Actuator, PidController, Setpoint,
};
```

## Cross-Repo Connections

### With `conservation-law-rs`: Energy-Conserved Homeostasis

Enforce that the total energy spent on regulation does not exceed a conservation budget, using Lagrangian mechanics to model the agent as a damped harmonic oscillator:

```rust
use agent_homeostasis::{HomeostaticRegulator, Sensor, Actuator, Setpoint};
use conservation_law::lagrangian::{AgentState, total_energy};

fn regulate_with_energy_budget(
    regulator: &HomeostaticRegulator,
    readings: &[agent_homeostasis::SensorReading],
    max_energy: f64,
) -> Vec<agent_homeostasis::ParameterStatus> {
    let mut status = regulator.regulate(readings);
    
    // Compute total actuator energy
    let total_force: f64 = status.iter()
        .map(|s| s.action.magnitude.abs())
        .sum();
    
    if total_force > max_energy {
        // Scale all actions proportionally to conserve energy
        let scale = max_energy / total_force;
        for s in status.iter_mut() {
            s.action.magnitude *= scale;
        }
        println!("Energy budget enforced: scaled by {:.3}", scale);
    }
    
    status
}
```

### With `si-cli`: Interactive Health Check

The si-cli exposes homeostasis status as an interactive command:

```rust
use agent_homeostasis::{HomeostaticRegulator, HealthStatus};

fn cli_health_check(regulator: &HomeostaticRegulator, readings: &[agent_homeostasis::SensorReading]) {
    let status = regulator.regulate(readings);
    
    let all_stable = status.iter().all(|s| s.is_stable);
    let health = if all_stable {
        HealthStatus::Stable
    } else if status.iter().any(|s| s.deviation.abs() > 50.0) {
        HealthStatus::Critical
    } else {
        HealthStatus::Correcting
    };
    
    println!("Fleet health: {:?}", health);
    for s in &status {
        println!("  {}: {:.2} / {:.2} (deviation: {:.2})",
            s.name, s.current, s.target, s.deviation);
    }
}
```

### With `si-fleet-api`: REST Health Monitoring

Expose homeostatic parameters via REST for remote fleet monitoring:

```rust
use agent_homeostasis::{HomeostaticRegulator, Sensor, SensorReading, HealthStatus};
use si_fleet_api::{HttpRequest, HttpResponse};

fn get_health_status(req: HttpRequest) -> HttpResponse {
    let regulator: HomeostaticRegulator = req.state().get().unwrap();
    let readings: Vec<SensorReading> = req.state().get().unwrap();
    let status = regulator.regulate(&readings);
    
    let health = if status.iter().all(|s| s.is_stable) {
        "stable"
    } else if status.iter().any(|s| s.deviation.abs() > 50.0) {
        "critical"
    } else {
        "correcting"
    };
    
    HttpResponse::json(json!({
        "health": health,
        "parameters": status.iter().map(|s| json!({
            "name": s.name,
            "current": s.current,
            "target": s.target,
            "deviation": s.deviation,
            "stable": s.is_stable,
            "action": format!("{:?}", s.action.action_type),
        })).collect::<Vec<_>>(),
    }))
}

fn post_pid_tune(req: HttpRequest) -> HttpResponse {
    let body: serde_json::Value = req.json().unwrap();
    let name = body["parameter"].as_str().unwrap();
    let kp = body["kp"].as_f64().unwrap();
    let ki = body["ki"].as_f64().unwrap();
    let kd = body["kd"].as_f64().unwrap();
    
    let mut pid = agent_homeostasis::PidController::new(name, kp, ki, kd, 0.0);
    pid = pid.with_integral_limit(100.0).with_output_limit(50.0);
    
    HttpResponse::json(json!({ "status": "pid_updated", "parameter": name }))
}
```

### With Supabase: Time-Series Sensor Storage

Persist sensor readings and regulation history in Supabase for longitudinal health analytics:

```rust
use agent_homeostasis::{SensorReading, ParameterStatus};
use supabase_rs::SupabaseClient;

async fn persist_sensor_readings(
    client: &SupabaseClient,
    agent_id: &str,
    readings: &[SensorReading],
) {
    for reading in readings {
        client.from("sensor_readings")
            .insert(json!({
                "agent_id": agent_id,
                "sensor_name": reading.sensor_name,
                "value": reading.value,
                "raw_value": reading.raw_value,
                "noise_applied": reading.noise_applied,
                "timestamp": chrono::Utc::now().to_rfc3339(),
            }))
            .execute()
            .await
            .unwrap();
    }
}

async fn get_health_trend(
    client: &SupabaseClient,
    agent_id: &str,
    sensor_name: &str,
    hours: i64,
) -> Vec<(f64, f64)> {
    let since = chrono::Utc::now() - chrono::Duration::hours(hours);
    let rows = client.from("sensor_readings")
        .select("value,timestamp")
        .eq("agent_id", agent_id)
        .eq("sensor_name", sensor_name)
        .gte("timestamp", since.to_rfc3339())
        .order("timestamp.asc")
        .execute()
        .await
        .unwrap();
    
    rows.into_iter()
        .map(|r| (r.get("value").unwrap().parse().unwrap(), 0.0))
        .collect()
}
```

## Design Patterns

### Pattern: Cascading Setpoint Drift

Model seasonal load changes by drifting setpoints gradually:

```rust
use agent_homeostasis::setpoint::{SetpointTracker, Setpoint};

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
    let mut sorted = readings.clone();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    sorted[sorted.len() / 2]
}
```

### Pattern: PID Anti-Windup for Thermal Regulation

Use integral clamping to prevent runaway heating/cooling:

```rust
use agent_homeostasis::PidController;

fn thermal_pid(target_temp: f64) -> PidController {
    PidController::new("temperature", 2.0, 0.5, 1.0, target_temp)
        .with_integral_limit(20.0)
        .with_output_limit(100.0)
}
```
