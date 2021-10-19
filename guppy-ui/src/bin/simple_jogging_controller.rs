use anyhow::Result;
use clap::Clap;
use guppy_controller::arm_controller;
use guppy_controller::arm_driver::{self, ArmDriver, LedColor};
use guppy_controller::collision_handler;
use guppy_controller::motion_planner::MotionController;
use guppy_controller::{arm_config, motion_planner::LssMotionController};
use guppy_ui::{arm_driver::ArmControlSettings, visualizer::VisualizerInterface};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use tokio::time::sleep;
use tracing_subscriber::filter::LevelFilter;

#[derive(Clap)]
#[clap()]

struct Args {
    #[clap(about = "Serial port to use")]
    port: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Args = Args::parse();
    tracing_subscriber::fmt()
        .pretty()
        .with_max_level(LevelFilter::INFO)
        .init();
    move_run(args).await?;
    Ok(())
}

async fn move_run(args: Args) -> Result<()> {
    let running = Arc::new(AtomicBool::new(true));
    let mut visualizer = VisualizerInterface::sensible_default();
    let collision_handler =
        collision_handler::CollisionHandler::new(arm_config::ArmConfig::included());

    ctrlc::set_handler({
        let running = Arc::clone(&running);
        move || {
            running.store(false, Ordering::Release);
            println!("Caught interrupt\nExiting...");
        }
    })?;

    let mut driver = arm_driver::SerialArmDriver::new(
        &args.port,
        arm_config::ArmConfig::included(),
        &ArmControlSettings::included_continuous(),
    )
    .await?;
    driver.set_color(LedColor::Magenta).await?;
    let arm_controller =
        arm_controller::LssArmController::new(driver, arm_config::ArmConfig::included());

    let mut motion_planner =
        LssMotionController::new(arm_controller, collision_handler, 0.15, 10.0).await?;

    motion_planner.open_gripper(false).await?;
    // smoothly transition to home
    let initial_state = visualizer.get_desired_state();
    motion_planner
        .move_to_trajectory(initial_state.pose())
        .await?;

    sleep(Duration::from_secs(2)).await;

    let mut gripper_opened = false;

    motion_planner.apply_continuous_settings().await?;
    while running.load(Ordering::Acquire) && visualizer.window_opened() {
        let desired_state = visualizer.get_desired_state().clone();
        if let Ok(arm_positions) = motion_planner.move_to_jogging(desired_state.pose()).await {
            visualizer.set_position(arm_positions);
        }
        if gripper_opened != desired_state.gripper_state() {
            gripper_opened = desired_state.gripper_state();
            if gripper_opened {
                motion_planner.close_gripper(true).await?;
            } else {
                motion_planner.open_gripper(false).await?;
            }
        }

        sleep(Duration::from_millis(20)).await;
    }

    motion_planner.halt().await?;
    motion_planner.apply_trajectory_settings().await?;
    motion_planner.home().await?;
    sleep(Duration::from_secs(2)).await;
    Ok(())
}
