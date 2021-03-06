use anyhow::Result;
use async_std::io;
use clap::Clap;
use guppy_controller::arm_config;
use guppy_controller::arm_controller;
use guppy_controller::arm_controller::ArmController;
use guppy_controller::arm_driver::{self, ArmDriver, LedColor};
use guppy_controller::collision_handler;
use guppy_ui::{arm_driver::ArmControlSettings, visualizer::VisualizerInterface};
use nalgebra as na;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use tokio::time::sleep;

#[derive(Clap)]
#[clap()]
struct Args {
    #[clap(subcommand)]
    command: SubCommand,
}

#[derive(Clap)]
enum SubCommand {
    DisplayPositions(GenericArgs),
    Ik(GenericArgs),
    Move(GenericArgs),
    TeachPendent(GenericArgs),
    Viz,
}

#[derive(Clap)]
struct GenericArgs {
    #[clap(about = "Serial port to use")]
    port: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Args = Args::parse();
    match args.command {
        SubCommand::DisplayPositions(args) => {
            display_positions(args).await?;
        }
        SubCommand::Ik(args) => {
            ik_run(args).await?;
        }
        SubCommand::TeachPendent(args) => {
            teach_pendent(args).await?;
        }
        SubCommand::Move(args) => {
            move_run(args).await?;
        }
        SubCommand::Viz => test_visualizer().await?,
    }
    Ok(())
}

async fn test_visualizer() -> Result<()> {
    let running = Arc::new(AtomicBool::new(true));
    let running_handle = running.clone();

    let mut visualizer = VisualizerInterface::sensible_default();

    ctrlc::set_handler(move || {
        running_handle.store(false, Ordering::Release);
        println!("Caught interrupt\nExiting...");
    })?;

    while running.load(Ordering::Acquire) {
        let positions = crate::arm_controller::ArmPositions::new(
            na::Vector3::new(0.0, 0.0, 0.0),
            na::Vector3::new(0.1, 0.01, 0.1),
            na::Vector3::new(0.0, 0.0, 0.2),
            na::Vector3::new(0.1, 0.01, 0.3),
            na::Vector3::new(0.0, 0.0, 0.5),
            0.0,
        );
        let positions_2 = crate::arm_controller::ArmPositions::new(
            na::Vector3::new(0.2, 0.2, 0.0),
            na::Vector3::new(0.3, 0.21, 0.1),
            na::Vector3::new(0.2, 0.2, 0.2),
            na::Vector3::new(0.3, 0.21, 0.3),
            na::Vector3::new(0.2, 0.2, 0.5),
            0.0,
        );
        visualizer.set_position(positions.clone());
        visualizer.set_motion_plan(Some(vec![positions_2]));
        println!("{:?}", positions.end_effector);
        sleep(Duration::from_secs_f32(0.02)).await;
    }
    Ok(())
}

/// Monitor pose in loop and display
async fn ik_run(args: GenericArgs) -> Result<()> {
    let running = Arc::new(AtomicBool::new(true));
    let running_handle = running.clone();

    let mut visualizer = VisualizerInterface::sensible_default();

    ctrlc::set_handler(move || {
        running_handle.store(false, Ordering::Release);
        println!("Caught interrupt\nExiting...");
    })?;

    let mut driver = arm_driver::SerialArmDriver::new(
        &args.port,
        arm_config::ArmConfig::included(),
        ArmControlSettings::included_continuous(),
    )
    .await?;
    driver.limp().await?;
    driver.set_color(LedColor::Cyan).await?;
    let mut arm_controller =
        arm_controller::LssArmController::new(driver, arm_config::ArmConfig::included());

    while running.load(Ordering::Acquire) {
        if let Ok(positions) = arm_controller.read_position().await {
            println!(
                "end_effector:\n   x: {:.3}\n   y: {:.3}\n   z: {:.3}\n   angle: {:.3}\n",
                positions.end_effector.x,
                positions.end_effector.y,
                positions.end_effector.z,
                positions.end_effector_angle
            );
            visualizer.set_position(positions.clone());
            let calculated_ik = arm_controller.calculate_ik(positions.get_end_effector_pose())?;
            let translated_fk = arm_controller.calculate_fk(calculated_ik)?;
            visualizer.set_motion_plan(Some(vec![translated_fk]));
            sleep(Duration::from_millis(20)).await;
        } else {
            eprintln!("Message error");
        }
    }
    sleep(Duration::from_secs_f32(0.5)).await;
    Ok(())
}

async fn move_run(args: GenericArgs) -> Result<()> {
    let running = Arc::new(AtomicBool::new(true));
    let running_handle = running.clone();
    let mut visualizer = VisualizerInterface::sensible_default();
    let collision_handler =
        collision_handler::CollisionHandler::new(arm_config::ArmConfig::included());

    ctrlc::set_handler(move || {
        running_handle.store(false, Ordering::Release);
        println!("Caught interrupt\nExiting...");
    })?;

    let mut driver = arm_driver::SerialArmDriver::new(
        &args.port,
        arm_config::ArmConfig::included(),
        ArmControlSettings::included_continuous(),
    )
    .await?;
    driver.set_color(LedColor::Magenta).await?;
    let mut arm_controller =
        arm_controller::LssArmController::new(driver, arm_config::ArmConfig::included());

    while running.load(Ordering::Acquire) {
        let desired_pose = visualizer.get_desired_state().pose().clone();
        let (pose, joints) = arm_controller.calculate_full_poses(desired_pose)?;
        // check collisions
        if !collision_handler.pose_collision_free(&pose) {
            arm_controller.set_color(LedColor::Red).await?;
            continue;
        }

        arm_controller.set_color(LedColor::Magenta).await?;

        if let Ok(_arm_positions) = arm_controller.move_joints_to(joints).await {
            visualizer.set_position(pose);
        } else {
            eprintln!("Message error");
        }
        sleep(Duration::from_millis(20)).await;
    }
    arm_controller.halt().await?;
    arm_controller.set_color(LedColor::Yellow).await?;
    sleep(Duration::from_secs_f32(0.5)).await;
    Ok(())
}

async fn wait_for_enter() -> Result<()> {
    let stdin = io::stdin();
    let mut line = String::new();
    stdin.read_line(&mut line).await?;
    Ok(())
}

/// read pose for a bit and then repeat poses
async fn teach_pendent(args: GenericArgs) -> Result<()> {
    let running = Arc::new(AtomicBool::new(true));
    let running_handle = running.clone();

    let mut visualizer = VisualizerInterface::sensible_default();

    ctrlc::set_handler(move || {
        running_handle.store(false, Ordering::SeqCst);
        println!("Caught interrupt\nExiting...");
    })?;

    let driver = arm_driver::SerialArmDriver::new(
        &args.port,
        arm_config::ArmConfig::included(),
        ArmControlSettings::included_continuous(),
    )
    .await?;
    let mut arm_controller =
        arm_controller::LssArmController::new(driver, arm_config::ArmConfig::included());

    let mut history = vec![];

    arm_controller.set_color(LedColor::Magenta).await?;
    arm_controller.limp().await?;

    println!("Starting recording\npress Ctrl+C to exit record mode");
    while running.load(Ordering::SeqCst) {
        println!("Press enter to record position");
        wait_for_enter().await?;
        let current_position = arm_controller.read_position().await?;
        history.push(current_position.clone());
        visualizer.set_position(current_position);
    }

    println!("Entering replay mode\npress Ctrl+c to exit");
    arm_controller.set_color(LedColor::Cyan).await?;
    running.store(true, Ordering::SeqCst);

    while running.load(Ordering::SeqCst) {
        for arm_pose in &history {
            if let Ok(_arm_positions) = arm_controller
                .move_to(arm_pose.get_end_effector_pose())
                .await
            {
                let joint_positions = arm_controller.calculate_fk(_arm_positions)?;
                visualizer.set_position(joint_positions);
            } else {
                eprintln!("Message error");
            }
            sleep(Duration::from_secs(1)).await;
        }
        println!("Sequence done\nPress enter to repeat");
        wait_for_enter().await?;
        sleep(Duration::from_millis(500)).await;
        if !running.load(Ordering::SeqCst) {
            break;
        }
    }
    arm_controller.set_color(LedColor::Red).await?;
    arm_controller.limp().await?;
    sleep(Duration::from_secs_f32(0.5)).await;
    Ok(())
}

async fn display_positions(args: GenericArgs) -> Result<()> {
    let running = Arc::new(AtomicBool::new(true));
    let running_handle = running.clone();

    ctrlc::set_handler(move || {
        running_handle.store(false, Ordering::Release);
        println!("Caught interrupt\nExiting...");
    })?;

    let mut driver = arm_driver::SerialArmDriver::new(
        &args.port,
        arm_config::ArmConfig::default(),
        ArmControlSettings::included_continuous(),
    )
    .await?;
    driver.set_color(LedColor::Cyan).await?;
    driver.limp().await?;
    driver.set_color(LedColor::White).await?;
    while running.load(Ordering::Acquire) {
        let positions = driver.read_position().await?;
        println!("{:?}", positions);
        sleep(Duration::from_secs_f32(0.2)).await;
    }
    driver.set_color(LedColor::Magenta).await?;
    sleep(Duration::from_secs_f32(0.2)).await;
    Ok(())
}
