use anyhow::Result;
use clap::Clap;
use guppy_controller::arm_controller;
use guppy_controller::arm_controller::ArmController;
use guppy_controller::arm_driver::{self, ArmDriver, LedColor};
use guppy_controller::collision_handler;
use guppy_controller::{arm_config, motion_planner::LssMotionController};
use guppy_ui::{arm_driver::ArmControlSettings, visualizer::VisualizerInterface};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use tokio::time::sleep;

#[derive(Clap)]
#[clap()]

struct Args {
    #[clap(about = "Serial port to use")]
    port: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Args = Args::parse();
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
    let mut arm_controller =
        arm_controller::LssArmController::new(driver, arm_config::ArmConfig::included());

    arm_controller.move_gripper(1.0).await?;
    let mut gripper_opened = false;

    while running.load(Ordering::Acquire) {
        let desired_state = visualizer.get_desired_state().clone();
        if let Ok((pose, joints)) = arm_controller.calculate_full_poses(desired_state.pose()) {
            // check collisions
            if !collision_handler.pose_collision_free(&pose) {
                arm_controller.set_color(LedColor::Yellow).await?;
                continue;
            }

            arm_controller.set_color(LedColor::Magenta).await?;

            if let Ok(_arm_positions) = arm_controller.move_joints_to(&joints).await {
                visualizer.set_position(pose);
            } else {
                eprintln!("Message error");
            }

            if gripper_opened != desired_state.gripper_state() {
                gripper_opened = desired_state.gripper_state();
                if gripper_opened {
                    arm_controller.move_gripper(0.0).await?;
                } else {
                    arm_controller.move_gripper(1.0).await?;
                }
            }
        } else {
            arm_controller.set_color(LedColor::Red).await?;
        }
        sleep(Duration::from_millis(20)).await;
    }
    arm_controller.halt().await?;
    let mut motion_planner =
        LssMotionController::new(arm_controller, collision_handler, 0.15, 10.0).await?;
    motion_planner.home().await?;
    sleep(Duration::from_secs_f32(0.5)).await;
    Ok(())
}
