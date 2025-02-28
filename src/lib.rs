pub mod math;
pub mod constraints;
mod figure;
mod order;
mod solve;

pub use order::order_bfs;
pub use solve::solve_brute;
pub use figure::{Figure, CID, PID};