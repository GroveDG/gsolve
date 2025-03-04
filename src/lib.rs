#![deny(missing_docs)]
//! A geometric constraint solver based on intersecting geometry.
//! 
//! Solving is done in steps:
//! 
//! 1. Create a new [Figure].
//! 2. Add [Constraints][constraints::Constraint].
//! 3. [Order][order] the figure.
//! 4. [Solve][solve] the ordering.

#[allow(missing_docs)]
pub mod math;
#[allow(missing_docs)]
pub mod constraints;
mod figure;
pub mod order;
pub mod solve;

pub use figure::{Figure, CID, PID};