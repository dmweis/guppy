mod speech_service;
mod arm_config;

use lss_driver;
use clap::Clap;


#[derive(Clap)]
#[clap()]
struct Args {
    #[clap(about = "Serial port to use")]
    port: String,
}


#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Args = Args::parse();
    speech_service::say(format!("Waking up arm. Connecting to {}", args.port)).await?;

    let mut driver = lss_driver::LSSDriver::new(&args.port).unwrap();
    driver.set_color(lss_driver::BROADCAST_ID, lss_driver::LedColor::Cyan).await?;
    speech_service::say("Connected successfully!".to_owned()).await?;
    Ok(())
}
