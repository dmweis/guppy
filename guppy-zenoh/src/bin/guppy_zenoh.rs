use anyhow::{Context, Result};
use clap::Parser;
use guppy_controller::{
    arm_config::ArmConfig,
    arm_controller::{ArmController, EndEffectorPose, LssArmController},
    arm_driver::{ArmControlSettings, SerialArmDriver},
    collision_handler::CollisionHandler,
    motion_planner::MotionController,
};
use guppy_zenoh::{error::WrapperError, logging};
use lss_driver::LedColor;
use nalgebra as na;
use std::{path::PathBuf, time::Duration};
use tracing::*;
use zenoh::config::Config as ZenohConfig;
use zenoh::prelude::r#async::*;

#[derive(Parser)]
#[command(author, version)]
struct Args {
    /// Serial port to use
    #[arg(short, long)]
    port: String,

    /// Zenoh config
    #[arg(short, long)]
    zenoh_config: Option<PathBuf>,

    /// Sets the level of verbosity
    #[arg(short, long, action = clap::ArgAction::Count)]
    verbose: u8,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    logging::setup_tracing(args.verbose);

    // setup zenoh
    let zenoh_config = if let Some(conf_file) = &args.zenoh_config {
        ZenohConfig::from_file(conf_file).map_err(WrapperError::ZenohError)?
    } else {
        ZenohConfig::default()
    };
    let zenoh_session = zenoh::open(zenoh_config)
        .res()
        .await
        .map_err(WrapperError::ZenohError)?
        .into_arc();

    tracing::info!("Moving to home");

    let (command_sender, mut command_receiver) = tokio::sync::mpsc::channel(10);

    let control_loop_future = tokio::spawn(async move {
        while let Err(err) = guppy_control_loop(&mut command_receiver, &args.port).await {
            tracing::error!("Error in control loop {:?}", err);
        }
    });

    let home_command_subscriber = zenoh_session
        .declare_subscriber("guppy/command/home")
        .res()
        .await
        .map_err(WrapperError::ZenohError)?;

    let attention_command_subscriber = zenoh_session
        .declare_subscriber("guppy/command/attention")
        .res()
        .await
        .map_err(WrapperError::ZenohError)?;

    let open_gripper_subscriber = zenoh_session
        .declare_subscriber("guppy/command/gripper/open")
        .res()
        .await
        .map_err(WrapperError::ZenohError)?;

    let close_gripper_subscriber = zenoh_session
        .declare_subscriber("guppy/command/gripper/close")
        .res()
        .await
        .map_err(WrapperError::ZenohError)?;

    let ping_subscriber = zenoh_session
        .declare_subscriber("guppy/ping")
        .res()
        .await
        .map_err(WrapperError::ZenohError)?;

    loop {
        tokio::select! {
            _ = home_command_subscriber.recv_async() => {
                info!("Got home command");
                command_sender.send(GuppyCommand::Home).await?;
            }
            _ = attention_command_subscriber.recv_async() => {
                info!("Got attention command");
                command_sender.send(GuppyCommand::Attention).await?;
            }
            _ = open_gripper_subscriber.recv_async() => {
                info!("Got open gripper command");
                command_sender.send(GuppyCommand::OpenGripper).await?;
            }
            _ = close_gripper_subscriber.recv_async() => {
                info!("Got close gripper command");
                command_sender.send(GuppyCommand::CloseGripper).await?;
            }
            _ = ping_subscriber.recv_async() => {
                command_sender.send(GuppyCommand::Ping).await?;
            }
            _ = tokio::signal::ctrl_c() => {
                tracing::info!("Detected Ctrl+C");
                break;
            }
        }
    }

    tracing::info!("Sending guppy exit command");
    command_sender.send(GuppyCommand::Exit).await?;
    control_loop_future.await?;

    Ok(())
}

enum GuppyCommand {
    Color(LedColor),
    MoveTo(EndEffectorPose),
    Ping,
    Home,
    Attention,
    OpenGripper,
    CloseGripper,
    Exit,
}

const GUPPY_OPERATION_TIMEOUT: Duration = Duration::from_secs(30);

async fn guppy_control_loop(
    command_receiver: &mut tokio::sync::mpsc::Receiver<GuppyCommand>,
    serial_port: &str,
) -> anyhow::Result<()> {
    let mut controller = build_motion_controller(serial_port).await?;

    tracing::info!("Initialized guppy controller");
    tracing::info!("Homing arm");
    controller.home().await?;

    let mut homed = true;

    loop {
        let timeout_command =
            tokio::time::timeout(GUPPY_OPERATION_TIMEOUT, command_receiver.recv()).await;

        let command = match timeout_command {
            Err(_elapsed) => {
                // timeout
                if !homed {
                    tracing::info!("Guppy command timed out. Homing");
                    GuppyCommand::Home
                } else {
                    tracing::debug!("Already hommed on timeout. Skipping");
                    continue;
                }
            }
            Ok(command) => {
                // channel can return None in case it's closed
                command.context("Guppy command channel closed")?
            }
        };

        match command {
            GuppyCommand::Ping => {
                // pinging exists to prevent timeouts
                // otherwise should be ignored
            }
            GuppyCommand::MoveTo(pose) => {
                if homed {
                    info!("Reapplying settings");
                    controller.apply_settings().await?;
                    controller.update_last_known_pose().await?;
                    homed = false;
                }

                controller.move_to(pose).await?
            }
            GuppyCommand::Attention => {
                // attention is a hardcoded pose
                if homed {
                    info!("Reapplying settings");
                    controller.apply_settings().await?;
                    controller.update_last_known_pose().await?;
                    homed = false;
                }
                let pose = EndEffectorPose::new(na::Vector3::new(0.25, 0.0, 0.2), 0.0);

                controller.move_to(pose).await?
            }
            GuppyCommand::Home => {
                controller.home().await?;
                homed = true;
            }
            GuppyCommand::OpenGripper => controller.open_gripper().await?,
            GuppyCommand::CloseGripper => controller.close_gripper().await?,
            GuppyCommand::Color(color) => controller.set_color(color).await?,
            GuppyCommand::Exit => break,
        }
    }

    tracing::info!("Command loop exited");

    if !homed {
        info!("Homing");
        controller.home().await?;
    }

    Ok(())
}

async fn build_motion_controller(serial_port: &str) -> anyhow::Result<MotionController> {
    let config = ArmConfig::included();
    let driver = SerialArmDriver::new(
        serial_port,
        config.clone(),
        ArmControlSettings::included_trajectory(),
    )
    .await?;
    let mut controller = LssArmController::new(driver, config.clone());
    let motors_okay = controller.check_motors_okay().await?;

    if !motors_okay {
        anyhow::bail!("Motors not okay");
    }

    controller.set_color(lss_driver::LedColor::Magenta).await?;

    let collision_handler = CollisionHandler::new(config);

    let motion_controller =
        MotionController::new(controller, collision_handler, 0.10, 30.0).await?;
    Ok(motion_controller)
}
