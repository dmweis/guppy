mod interpolating_motion_planner;
mod trajectory_motion_planner;

pub use interpolating_motion_planner::{ContinuousMotionController, LinearMotion};
pub use trajectory_motion_planner::MotionController;
