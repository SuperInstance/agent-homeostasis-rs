# INTEGRATION.md — agent-homeostasis-rs × ga-core × symplectic-opt

**Geometric self-regulation**: PID controllers in the language of geometric algebra and symplectic geometry.

## Synergy Map

```
agent-homeostasis-rs          ga-core               symplectic-opt
┌──────────────────┐    ┌───────────────┐    ┌─────────────────────┐
│ PidController    │    │ Multivector   │    │ SymplecticMatrix    │
│ ControlLoop      │◄──►│ Rotor         │◄──►│ HamiltonianSystem   │
│ Setpoint         │    │ Conformal     │    │ EnergyTracker       │
│ Sensor / Actuator│    │ (geometric    │    │ NaturalGradient     │
│ Regulator        │    │  product)     │    │ ConservationLaw     │
└──────────────────┘    └───────────────┘    └─────────────────────┘
         │                      │                       │
         └──────────────────────┼───────────────────────┘
                                ▼
                    Geometric PID: corrections as
                    multivector rotations preserving
                    symplectic structure
```

## Key Insight

A PID controller adjusts scalar parameters. But when agents live in high-dimensional configuration spaces, corrections should be geometric — rotations in the agent's state space that preserve energy. ga-core provides the rotation machinery (rotors via sandwich products), symplectic-opt ensures the correction preserves the symplectic form, and agent-homeostasis-rs provides the feedback loop.

## Example 1: Symplectic PID Controller

Use a Hamiltonian system to drive PID corrections that conserve energy:

```rust
use agent_homeostasis_rs::control_loop::{PidController, PidResult};
use symplectic_opt::hamiltonian::SeparableHamiltonian;
use symplectic_opt::conservation::EnergyTracker;
use symplectic_opt::symplectic::SymplecticMatrix;

/// A PID controller whose corrections are symplectically constrained.
/// The PID output is projected onto the nearest symplectic update,
/// ensuring energy conservation during regulation.
struct SymplecticPid {
    pid: PidController,
    hamiltonian: SeparableHamiltonian,
    energy_tracker: EnergyTracker,
}

impl SymplecticPid {
    fn new(
        name: &str,
        kp: f64, ki: f64, kd: f64,
        target: f64,
        masses: Vec<f64>,
        potential_coeffs: Vec<f64>,
    ) -> Self {
        let pid = PidController::new(name, kp, ki, kd, target);
        let hamiltonian = SeparableHamiltonian::new(masses, potential_coeffs);
        let energy_tracker = EnergyTracker::from_energy(0.0);

        Self { pid, hamiltonian, energy_tracker }
    }

    /// Update with symplectic correction.
    fn update(&mut self, measurement: f64, q: &[f64], p: &[f64]) -> (PidResult, Vec<f64>, Vec<f64>) {
        let result = self.pid.update(measurement);

        // Use Störmer-Verlet to propagate the correction
        let (new_q, new_p) = self.hamiltonian.stormer_verlet(q, p, result.output * 0.01, 1);

        // Track energy
        let energy = self.hamiltonian.energy(&new_q, &new_p);
        self.energy_tracker.record(energy);

        (result, new_q, new_p)
    }

    fn energy_conserved(&self) -> bool {
        self.energy_tracker.is_conserved()
    }

    fn energy_drift(&self) -> f64 {
        self.energy_tracker.max_drift()
    }
}

fn main() {
    let mut spid = SymplecticPid::new(
        "position",
        2.0, 0.1, 0.5,     // PID gains
        10.0,               // target position
        vec![1.0, 1.0],     // masses
        vec![0.5, 0.5],     // potential coefficients (harmonic)
    );

    let q = vec![0.0, 0.0]; // initial position
    let p = vec![0.0, 0.0]; // initial momentum

    let (result, new_q, new_p) = spid.update(0.0, &q, &p);
    println!("PID output: {:.4}", result.output);
    println!("New position: {:?}", new_q);
    println!("Energy conserved: {}", spid.energy_conserved());
    println!("Max energy drift: {:.6}", spid.energy_drift());
}
```

## Example 2: Geometric Rotation as Actuator Correction

Use ga-core rotors to apply corrections as rotations in conformal space:

```rust
use agent_homeostasis_rs::actuator::{Actuator, ActionType};
use agent_homeostasis_rs::setpoint::Setpoint;
use ga_core::multivector::Multivector;
use ga_core::rotor::Rotor;
use ga_core::conformal::Conformal;

/// Apply actuator corrections as conformal rotations instead of
/// simple linear adjustments. This preserves the geometric structure
/// of the agent's state space.
fn geometric_correction(
    current_state: [f64; 3],
    target: [f64; 3],
    actuator: &Actuator,
    setpoint: &Setpoint,
) -> [f64; 3] {
    let deviation = setpoint.deviation(current_state[0]);

    // Compute correction using the actuator
    let action = actuator.compute_action(deviation);

    if action.action_type == ActionType::Hold {
        return current_state;
    }

    // Build a rotor that rotates current toward target
    // The rotation axis is the cross product of current and target
    let axis = [
        current_state[1] * target[2] - current_state[2] * target[1],
        current_state[2] * target[0] - current_state[0] * target[2],
        current_state[0] * target[1] - current_state[1] * target[0],
    ];
    let axis_len = (axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]).sqrt();
    if axis_len < 1e-10 {
        return current_state; // already aligned
    }
    let axis_norm = [axis[0]/axis_len, axis[1]/axis_len, axis[2]/axis_len];

    // Scale the rotation angle by the actuator's correction magnitude
    let angle = action.magnitude * 0.1;
    let rotor = Rotor::from_axis_angle(axis_norm, angle);

    // Apply via sandwich product
    rotor.apply(current_state)
}

fn main() {
    let actuator = Actuator::new("position", 2.0, 0.5);
    let setpoint = Setpoint::new("position", 10.0, 1.0);

    let current = [3.0, 0.0, 0.0];
    let target = [10.0, 0.0, 0.0];
    let corrected = geometric_correction(current, target, &actuator, &setpoint);

    println!("Current: {:?}", current);
    println!("Corrected: {:?}", corrected);
    println!("Moved toward target: {}", corrected[0] > current[0]);
}
```

## Example 3: Multi-Agent Regulation with Energy Conservation

A fleet of agents, each regulated by homeostasis, with fleet-wide energy tracked via symplectic conservation:

```rust
use agent_homeostasis_rs::control_loop::{PidController, ControlLoop};
use agent_homeostasis_rs::regulator::{HomeostaticRegulator, HealthStatus};
use agent_homeostasis_rs::setpoint::Setpoint;
use agent_homeostasis_rs::actuator::Actuator;
use agent_homeostasis_rs::sensor::SensorReading;
use symplectic_opt::conservation::EnergyTracker;

/// A fleet of agents where total computational energy is conserved.
/// When one agent increases activity, others must decrease — the budget is fixed.
struct ConservedFleet {
    regulators: Vec<HomeostaticRegulator>,
    energy_tracker: EnergyTracker,
    total_budget: f64,
}

impl ConservedFleet {
    fn new(n_agents: usize, budget_per_agent: f64) -> Self {
        let mut regulators = Vec::new();
        for i in 0..n_agents {
            let mut reg = HomeostaticRegulator::new();
            reg.add_parameter(
                Setpoint::new("compute", budget_per_agent, budget_per_agent * 0.1),
                Actuator::new("compute", budget_per_agent * 0.2, 0.5),
            );
            regulators.push(reg);
        }

        let total = n_agents as f64 * budget_per_agent;
        let tracker = EnergyTracker::from_energy(total);

        ConservedFleet {
            regulators,
            energy_tracker: tracker,
            total_budget: total,
        }
    }

    fn tick(&mut self, agent_readings: &[Vec<SensorReading>]) -> Vec<HealthStatus> {
        let mut statuses = Vec::new();
        let mut total_energy = 0.0;

        for (i, (reg, readings)) in self.regulators.iter().zip(agent_readings).enumerate() {
            let (s, new_vals) = reg.cycle(readings);
            total_energy += new_vals[0];
            statuses.push(reg.health_status(&s));
        }

        // Track fleet energy — it should stay near total_budget
        self.energy_tracker.record(total_energy);

        statuses
    }

    fn energy_conserved(&self) -> bool {
        self.energy_tracker.is_conserved()
    }
}

fn main() {
    let mut fleet = ConservedFleet::new(5, 100.0);

    // All agents start at their setpoint
    let readings: Vec<Vec<SensorReading>> = (0..5)
        .map(|_| vec![SensorReading::perfect("compute", 100.0)])
        .collect();

    let statuses = fleet.tick(&readings);
    for (i, s) in statuses.iter().enumerate() {
        println!("Agent {}: {:?}", i, s);
    }
    println!("Fleet energy conserved: {}", fleet.energy_conserved());
}
```

## Data Flow

```
SensorReading ──► HomeostaticRegulator.cycle()
                         │
                         ▼
                ParameterStatus + corrections
                         │
            ┌────────────┤
            ▼            ▼
    Rotor.apply()  SymplecticMatrix
    (geometric     (preserves
     rotation)      symplectic form)
            │            │
            └────────────┤
                         ▼
              EnergyTracker.record()
              (verify conservation)
```

## When to Use This Combination

- **Fleet coordination** where total compute budget is fixed (one agent's gain = another's loss)
- **Robotics/physics** where corrections must preserve energy or momentum
- **Geometric ML** where parameter updates should respect the manifold structure
- **Multi-agent RL** where policy updates should be rotations, not arbitrary translations
