mod arm_config;
mod arm_driver;
mod speech_service;

use clap::Clap;
use lss_driver;

#[derive(Clap)]
#[clap()]
struct Args {
    #[clap(about = "Serial port to use")]
    port: String,
    #[clap(short, long)]
    speak: bool,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Args = Args::parse();
    if args.speak {
        speech_service::say(format!("Waking up arm. Connecting to {}", args.port)).await?;
    }
    let mut driver = arm_driver::SerialArmDriver::new(&args.port)?;
    driver.set_color(lss_driver::LedColor::Cyan).await?;
    if args.speak {
        speech_service::say("Connected successfully!".to_owned()).await?;
    }
    Ok(())
}
