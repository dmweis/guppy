use anyhow::Result;
use clap::Parser;
use guppy_controller::{
    arm_config::ArmConfig,
    arm_controller::{ArmController, EndEffectorPose, LssArmController},
    arm_driver::{ArmControlSettings, SerialArmDriver},
    collision_handler::CollisionHandler,
    motion_planner::MotionController,
};
use guppy_zenoh::logging;
use nalgebra as na;
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
};
use tokio::time::sleep;

#[derive(Parser)]
#[command(author, version)]
struct Args {
    /// Serial port to use
    #[arg()]
    port: String,

    /// Sets the level of verbosity
    #[arg(short, long, action = clap::ArgAction::Count)]
    verbose: u8,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    logging::setup_tracing(args.verbose);

    let config = ArmConfig::included();
    let driver = SerialArmDriver::new(
        &args.port,
        config.clone(),
        ArmControlSettings::included_trajectory(),
    )
    .await?;
    let mut controller = LssArmController::new(driver, config.clone());
    let okay = controller.check_motors_okay().await?;
    tracing::info!("Motors okay: {}", okay);
    controller.set_color(lss_driver::LedColor::Magenta).await?;

    let collision_handler = CollisionHandler::new(config);

    let mut motion_controller =
        MotionController::new(controller, collision_handler, 0.15, 10.0).await?;

    let keep_running = Arc::new(AtomicBool::new(true));

    tokio::spawn({
        let keep_running = keep_running.clone();
        async move {
            tokio::signal::ctrl_c()
                .await
                .expect("Failed to wait for Ctrl+c");
            tracing::info!("Detected Ctrl+c");
            keep_running.store(false, Ordering::Relaxed);
        }
    });

    while keep_running.load(Ordering::Relaxed) {
        motion_controller
            .move_to_blocking(EndEffectorPose::new(
                na::Vector3::new(0.18, 0.06, 0.22),
                0.0,
            ))
            .await?;
        motion_controller.open_gripper().await?;

        if !keep_running.load(Ordering::Relaxed) {
            continue;
        }

        sleep(Duration::from_secs(4)).await;

        motion_controller
            .move_to_blocking(EndEffectorPose::new(
                na::Vector3::new(0.28, -0.03, 0.10),
                0.0,
            ))
            .await?;
        motion_controller.close_gripper().await?;

        if !keep_running.load(Ordering::Relaxed) {
            continue;
        }

        sleep(Duration::from_secs(4)).await;
    }

    tracing::info!("Moving to home");
    motion_controller.home().await?;

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
