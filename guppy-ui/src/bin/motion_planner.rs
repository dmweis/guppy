use anyhow::Result;
use clap::Clap;
use guppy_controller::arm_controller;
use guppy_controller::arm_driver::{self, ArmDriver, LedColor};
use guppy_controller::motion_planner;
use guppy_controller::{arm_config, motion_planner::MotionController};
use guppy_ui::arm_controller::EndEffectorPose;
use nalgebra as na;
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
};
use tokio::time::sleep;

#[derive(Clap)]
struct Args {
    #[clap(about = "Serial port to use")]
    port: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Args = Args::parse();
    let running = Arc::new(AtomicBool::new(true));
    let running_handle = running.clone();
    let collision_handler =
        motion_planner::CollisionHandler::new(arm_config::ArmConfig::included());

    ctrlc::set_handler(move || {
        running_handle.store(false, Ordering::Release);
        println!("Caught interrupt\nExiting...");
    })?;

    let mut driver =
        arm_driver::SerialArmDriver::new(&args.port, arm_config::ArmConfig::included()).await?;
    driver.set_color(LedColor::Magenta).await?;
    let arm_controller =
        arm_controller::LssArmController::new(driver, arm_config::ArmConfig::included());
    let mut motion_planner =
        MotionController::new(arm_controller, collision_handler, 0.15, 10.0).await?;

    while running.load(Ordering::Acquire) {
        motion_planner.set_target(EndEffectorPose::new(
            na::Vector3::new(0.18, 0.04, 0.22),
            0.0,
        ));
        sleep(Duration::from_secs(2)).await;

        motion_planner.set_target(EndEffectorPose::new(
            na::Vector3::new(0.28, -0.04, 0.18),
            0.0,
        ));
        sleep(Duration::from_secs(2)).await;
    }

    sleep(Duration::from_secs_f32(0.5)).await;
    Ok(())
}
