mod arm_config;
mod arm_controller;
mod arm_driver;
mod speech_service;
#[cfg(feature = "visualiser")]
mod visualizer;

use async_std::task::sleep;
use std::time::Duration;
#[cfg(feature = "visualiser")]
use visualizer::VisualizerInterface;

use crate::arm_controller::EndEffectorPose;
use clap::Clap;
use nalgebra as na;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

#[derive(Clap)]
#[clap()]
struct Args {
    #[clap(subcommand)]
    command: SubCommand,
}

#[derive(Clap)]
enum SubCommand {
    DisplayPositions(DisplayPositionsArgs),
    Ik(GenericArgs),
    Move(GenericArgs),
    Config(ConfigArgs),
    #[cfg(feature = "visualiser")]
    Viz,
}

#[derive(Clap)]
struct ConfigArgs {
    #[clap(short, long)]
    input: Option<String>,
    #[clap(short, long)]
    output: Option<String>,
}

#[derive(Clap)]
struct DisplayPositionsArgs {
    #[clap(about = "Serial port to use")]
    port: String,
    #[clap(short, long)]
    speak: bool,
}

#[derive(Clap)]
struct GenericArgs {
    #[clap(about = "Serial port to use")]
    port: String,
    #[clap(short, long)]
    speak: bool,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Args = Args::parse();
    match args.command {
        SubCommand::DisplayPositions(args) => {
            display_positions(args).await?;
        }
        SubCommand::Config(args) => {
            config(args)?;
        }
        SubCommand::Ik(args) => {
            ik_run(args).await?;
        }
        SubCommand::Move(args) => {
            move_run(args).await?;
        }
        #[cfg(feature = "visualiser")]
        SubCommand::Viz => test_visualizer().await?,
    }
    Ok(())
}

#[cfg(feature = "visualiser")]
async fn test_visualizer() -> Result<(), Box<dyn std::error::Error>> {
    let running = Arc::new(AtomicBool::new(true));
    let running_handle = running.clone();
    let desired_point = Arc::new(Mutex::new(EndEffectorPose::new(
        na::Vector3::new(0., 0., 0.),
        0.,
    )));
    let mut visualizer = VisualizerInterface::new(desired_point.clone());

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

async fn ik_run(args: GenericArgs) -> Result<(), Box<dyn std::error::Error>> {
    let running = Arc::new(AtomicBool::new(true));
    let running_handle = running.clone();
    #[cfg(feature = "visualiser")]
    let desired_point = Arc::new(Mutex::new(EndEffectorPose::new(
        na::Vector3::new(0., 0., 0.),
        0.,
    )));
    #[cfg(feature = "visualiser")]
    let mut visualizer = VisualizerInterface::new(desired_point.clone());

    ctrlc::set_handler(move || {
        running_handle.store(false, Ordering::Release);
        println!("Caught interrupt\nExiting...");
    })?;

    if args.speak {
        speech_service::say(format!("Waking up arm. Connecting to {}", args.port)).await?;
    }
    let mut driver =
        arm_driver::SerialArmDriver::new(&args.port, arm_config::ArmConfig::included()).await?;
    driver.limp().await?;
    driver.set_color(lss_driver::LedColor::Cyan).await?;
    let mut arm_controller =
        arm_controller::LssArmController::new(driver, arm_config::ArmConfig::included());

    if args.speak {
        speech_service::say("Connected successfully!".to_owned()).await?;
    }
    while running.load(Ordering::Acquire) {
        if let Ok(positions) = arm_controller.read_position().await {
            println!(
                "end_effector:\n   x: {:.3}\n   y: {:.3}\n   z: {:.3}\n   angle: {:.3}\n",
                positions.end_effector.x,
                positions.end_effector.y,
                positions.end_effector.z,
                positions.end_effector_angle
            );
            #[cfg(feature = "visualiser")]
            {
                visualizer.set_position(positions.clone());
                let calculated_ik = arm_controller
                    .calculate_ik(positions.end_effector, positions.end_effector_angle)
                    .await?;
                let translated_fk = arm_controller.calculate_fk(calculated_ik).await?;
                visualizer.set_motion_plan(Some(vec![translated_fk]));
            }
            sleep(Duration::from_micros(20)).await;
        } else {
            eprintln!("Message error");
        }
    }
    sleep(Duration::from_secs_f32(0.5)).await;
    Ok(())
}

async fn move_run(args: GenericArgs) -> Result<(), Box<dyn std::error::Error>> {
    let running = Arc::new(AtomicBool::new(true));
    let running_handle = running.clone();
    let desired_point = Arc::new(Mutex::new(EndEffectorPose::new(
        na::Vector3::new(0.2, 0., 0.2),
        0.,
    )));
    #[cfg(feature = "visualiser")]
    let mut visualizer = VisualizerInterface::new(desired_point.clone());

    ctrlc::set_handler(move || {
        running_handle.store(false, Ordering::Release);
        println!("Caught interrupt\nExiting...");
    })?;

    if args.speak {
        speech_service::say(format!("Waking up arm. Connecting to {}", args.port)).await?;
    }
    let mut driver =
        arm_driver::SerialArmDriver::new(&args.port, arm_config::ArmConfig::included()).await?;
    driver.set_color(lss_driver::LedColor::Magenta).await?;
    let mut arm_controller =
        arm_controller::LssArmController::new(driver, arm_config::ArmConfig::included());

    if args.speak {
        speech_service::say("Connected successfully!".to_owned()).await?;
    }
    // let start = Instant::now();
    while running.load(Ordering::Acquire) {
        // let temporal = (start.elapsed().as_secs_f32() * 2.).sin();
        // let z = temporal * 0.07;
        // let y = temporal * 0.07;
        // let position = na::Vector3::new(0.2, 0.0 + y, 0.2 + z);
        let pose = desired_point.lock().unwrap().clone();
        if let Ok(_arm_positions) = arm_controller
            .move_to(pose.position, pose.end_effector_angle)
            .await
        {
            #[cfg(feature = "visualiser")]
            {
                let joint_positions = arm_controller.calculate_fk(_arm_positions).await?;
                visualizer.set_position(joint_positions);
            }
        } else {
            eprintln!("Message error");
        }
        sleep(Duration::from_micros(20)).await;
    }
    arm_controller.limp().await?;
    arm_controller
        .set_color(lss_driver::LedColor::Yellow)
        .await?;
    sleep(Duration::from_secs_f32(0.5)).await;
    Ok(())
}

async fn display_positions(args: DisplayPositionsArgs) -> Result<(), Box<dyn std::error::Error>> {
    let running = Arc::new(AtomicBool::new(true));
    let running_handle = running.clone();

    ctrlc::set_handler(move || {
        running_handle.store(false, Ordering::Release);
        println!("Caught interrupt\nExiting...");
    })?;

    if args.speak {
        speech_service::say(format!("Waking up arm. Connecting to {}", args.port)).await?;
    }
    let mut driver =
        arm_driver::SerialArmDriver::new(&args.port, arm_config::ArmConfig::default()).await?;
    driver.set_color(lss_driver::LedColor::Cyan).await?;
    if args.speak {
        speech_service::say("Connected successfully!".to_owned()).await?;
    }
    driver.limp().await?;
    driver.set_color(lss_driver::LedColor::White).await?;
    while running.load(Ordering::Acquire) {
        let positions = driver.read_position().await?;
        println!("{:?}", positions);
        sleep(Duration::from_secs_f32(0.2)).await;
    }
    driver.set_color(lss_driver::LedColor::Magenta).await?;
    sleep(Duration::from_secs_f32(0.2)).await;
    Ok(())
}

fn config(args: ConfigArgs) -> Result<(), Box<dyn std::error::Error>> {
    if let Some(input) = args.input {
        let input_file_ext = Path::new(&input)
            .extension()
            .ok_or("Failed to extract extension")?
            .to_str()
            .ok_or("Failed to convert to string")?
            .to_ascii_lowercase();
        match &input_file_ext[..] {
            "json" => {
                let config = arm_config::ArmConfig::load_json(&input)?;
                println!("Config parsed successfully\n\n{:?}", config);
                if let Some(output) = args.output {
                    config.save_yaml(&output)?;
                    println!("Config written as yaml");
                }
            }
            "yaml" => {
                let config = arm_config::ArmConfig::load_yaml(&input)?;
                println!("Config parsed successfully\n\n{:?}", config);
                if let Some(output) = args.output {
                    config.save_json(&output)?;
                    println!("Config written as json");
                }
            }
            _ => {
                println!(
                    "Unknown config extension {}\nParsing as yaml",
                    input_file_ext
                );
                let config = arm_config::ArmConfig::load_yaml(&input)?;
                println!("Config parsed successfully\n\n{:?}", config);
            }
        }
    } else if let Some(output) = args.output {
        println!("Writing included config");
        let input_file_ext = Path::new(&output)
            .extension()
            .ok_or("Failed to extract extension")?
            .to_str()
            .ok_or("Failed to convert to string")?
            .to_ascii_lowercase();
        let included = arm_config::ArmConfig::included();
        match &input_file_ext[..] {
            "json" => {
                println!("Saving as json");
                included.save_json(&output)?;
            }
            _ => {
                println!("Saving as yaml");
                included.save_yaml(&output)?;
            }
        }
    }
    Ok(())
}
