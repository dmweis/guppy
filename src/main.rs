mod arm_config;
mod arm_controller;
mod arm_driver;
mod speech_service;
use ctrlc;
use std::time::Duration;
use async_std::task::sleep;


use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use clap::Clap;

#[derive(Clap)]
#[clap()]
struct Args {
    #[clap(about = "Serial port to use")]
    port: String,
    #[clap(short, long)]
    speak: bool,
    #[clap(short, long)]
    config_path: Option<String>,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Args = Args::parse();
    let running = Arc::new(AtomicBool::new(true));
    let running_handle = running.clone();

    ctrlc::set_handler(move || {
        running_handle.store(false, Ordering::Release);
        println!("Caught interrupt");
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
    driver.halt().await?;
    driver.set_color(lss_driver::LedColor::Magenta).await?;
    Ok(())
}
