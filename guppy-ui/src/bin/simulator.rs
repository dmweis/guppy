use anyhow::Result;
use guppy_controller::arm_config;
use guppy_controller::arm_controller::{EndEffectorPose, KinematicSolver};
use guppy_ui::visualizer::VisualizerInterface;
use nalgebra as na;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::time::{Duration, Instant};
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
    let start = Instant::now();

    while running.load(Ordering::Acquire) {
        let temporal = (start.elapsed().as_secs_f32() * 2.).sin();
        let z = temporal * 0.07;
        let y = temporal * 0.07;
        let position = na::Vector3::new(0.2, 0.0 + y, 0.2 + z);
        let joint_pose = kinematic_solver.calculate_ik(EndEffectorPose::new(position, 0.0))?;
        let arm_pose = kinematic_solver.calculate_fk(joint_pose)?;
        visualizer.set_position(arm_pose);
        sleep(Duration::from_millis(10)).await;
    }
    Ok(())
}
