use std::time::Duration;

use anyhow::Result;
use clap::Parser;
use guppy_controller::{
    arm_config::ArmConfig,
    arm_controller::{ArmController, EndEffectorPose, LssArmController},
    arm_driver::{ArmControlSettings, SerialArmDriver},
    collision_handler::CollisionHandler,
    motion_planner::MotionController,
};
use nalgebra as na;
use tokio::time::sleep;

#[derive(Parser)]
#[command(author, version)]
struct Args {
    /// Serial port to use
    #[arg()]
    port: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    let config = ArmConfig::included();
    let driver = SerialArmDriver::new(
        &args.port,
        config.clone(),
        ArmControlSettings::included_trajectory(),
    )
    .await?;
    let mut controller = LssArmController::new(driver, config.clone());
    let okay = controller.check_motors_okay().await?;
    println!("Motors okay: {}", okay);
    controller.set_color(lss_driver::LedColor::Magenta).await?;

    let collision_handler = CollisionHandler::new(config);

    let mut motion_controller =
        MotionController::new(controller, collision_handler, 0.15, 10.0).await?;

    loop {
        motion_controller
            .move_to_blocking(EndEffectorPose::new(
                na::Vector3::new(0.18, 0.06, 0.22),
                0.0,
            ))
            .await?;
        motion_controller.open_gripper().await?;
        sleep(Duration::from_secs(4)).await;

        motion_controller
            .move_to_blocking(EndEffectorPose::new(
                na::Vector3::new(0.28, -0.03, 0.10),
                0.0,
            ))
            .await?;
        motion_controller.close_gripper().await?;
        sleep(Duration::from_secs(4)).await;
    }

    // motion_controller.home().await?;

    // motion_controller.open_gripper().await?;

    // tokio::time::sleep(std::time::Duration::from_secs(3)).await;

    // motion_controller.close_gripper().await?;

    // motion_controller
    //     .move_to_blocking(EndEffectorPose::new(na::Vector3::new(0.18, 0.0, 0.0), 0.0))
    //     .await?;

    // tokio::time::sleep(std::time::Duration::from_secs(20)).await;

    // motion_controller.home().await?;

    // motion_controller.limp().await?;

    Ok(())
}
