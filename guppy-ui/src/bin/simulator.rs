use anyhow::Result;
use guppy_controller::arm_config;
use guppy_controller::arm_controller::{EndEffectorPose, KinematicSolver};
use guppy_controller::motion_planner::LinearMotion;
use guppy_ui::visualizer::VisualizerInterface;
use nalgebra as na;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::time::Duration;
use tokio::time::sleep;

#[tokio::main]
async fn main() -> Result<()> {
    let running = Arc::new(AtomicBool::new(true));
    let running_handle = running.clone();

    ctrlc::set_handler(move || {
        running_handle.store(false, Ordering::Release);
        println!("Caught interrupt\nExiting...");
    })?;

    let mut visualizer = VisualizerInterface::default();
    let config = arm_config::ArmConfig::included();
    let kinematic_solver = KinematicSolver::new(config);

    while running.load(Ordering::Acquire) {
        let start = na::Vector3::new(0.24, 0.07, 0.12);
        let end = na::Vector3::new(0.16, -0.07, 0.28);

        for position in LinearMotion::new(start, end, 0.001) {
            if !running.load(Ordering::Acquire) {
                break;
            }
            let joint_pose = kinematic_solver.calculate_ik(EndEffectorPose::new(position, 0.0))?;
            let arm_pose = kinematic_solver.calculate_fk(joint_pose)?;
            visualizer.set_position(arm_pose);
            sleep(Duration::from_millis(10)).await;
        }
        for position in LinearMotion::new(end, start, 0.001) {
            if !running.load(Ordering::Acquire) {
                break;
            }
            let joint_pose = kinematic_solver.calculate_ik(EndEffectorPose::new(position, 0.0))?;
            let arm_pose = kinematic_solver.calculate_fk(joint_pose)?;
            visualizer.set_position(arm_pose);
            sleep(Duration::from_millis(10)).await;
        }
    }
    Ok(())
}
