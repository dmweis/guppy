mod arm_config;
mod arm_controller;
mod arm_driver;
mod speech_service;
use async_std::task::sleep;
use std::time::Duration;

use clap::Clap;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

#[derive(Clap)]
#[clap()]
struct Args {
    #[clap(subcommand)]
    command: SubCommand,
}

#[derive(Clap)]
enum SubCommand {
    DisplayPositions(DisplayPositionsArgs),
    Config(ConfigArgs),
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
    }
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
