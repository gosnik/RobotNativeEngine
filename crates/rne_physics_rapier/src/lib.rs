//! Rapier physics backend for Robot Native Engine.
//!
//! Enable `simd` for Rapier's stable SIMD contact solver. This is opt-in because
//! floating-point results can differ from scalar builds and it cannot be
//! combined with Rapier's `enhanced-determinism` feature. Replay comparisons
//! should use the same feature configuration and target environment.

#![deny(missing_docs)]

pub mod backend;
mod convert;

pub use backend::{step_physics, RapierBackend};
#[cfg(feature = "motor-stage-diagnostics")]
pub use backend::{
    MotorStageCheckpointError, MotorStageIntegrationParameters, MotorStageJointBinding,
    MotorStageNativeCheckpoint, MotorStageNativeRecord, MotorStageNativeRecordKind,
};
#[cfg(feature = "motor-stage-diagnostics")]
pub use rapier3d::pipeline::{
    MultibodyMotorStage, MultibodyMotorStageEvent, MultibodyMotorStageRow,
    MultibodyMotorStageTelemetry,
};
