//! # agent-homeostasis-rs
//!
//! Homeostatic regulation for agent systems — maintaining stable internal conditions.
//!
//! Provides:
//! - **regulator**: HomeostaticRegulator with setpoints and feedback
//! - **sensor**: Sensor readings with noise modeling
//! - **actuator**: Corrective actions to restore balance
//! - **control_loop**: PID-inspired control loop for agent parameters
//! - **setpoint**: Setpoint tracking and dynamic adjustment

pub mod regulator;
pub mod sensor;
pub mod actuator;
pub mod control_loop;
pub mod setpoint;
