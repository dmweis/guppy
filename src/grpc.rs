mod arm_config;
mod arm_controller;
mod arm_driver;

use clap::Clap;

use tonic::{transport::Server, Request, Response, Status};

use guppy_service::guppy_configure_server::{GuppyConfigure, GuppyConfigureServer};
use std::sync::Arc;
use tokio::sync::Mutex;

pub mod guppy_service {
    tonic::include_proto!("guppy_service");
}

#[derive(Clap)]
struct Args {
    #[clap(about = "Serial port to use")]
    port: String,
}

struct GuppyConfigHandler {
    driver: Arc<Mutex<std::boxed::Box<dyn arm_controller::ArmController>>>,
}

impl GuppyConfigHandler {
    async fn new(port: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let config = arm_config::ArmConfig::included();
        let driver =
            arm_driver::SerialArmDriver::new(port, arm_config::ArmConfig::included()).await?;
        let controller = arm_controller::LssArmController::new(driver, config);
        Ok(GuppyConfigHandler {
            driver: Arc::new(Mutex::new(controller)),
        })
    }
}

#[tonic::async_trait]
impl GuppyConfigure for GuppyConfigHandler {
    async fn set_led(
        &self,
        request: Request<guppy_service::SetLedRequest>,
    ) -> Result<Response<guppy_service::SetLedResponse>, Status> {
        let inner_request = request.into_inner();
        let mut driver = self.driver.lock().await;
        if let Some(color_id) = inner_request.led_color {
            let color = match color_id {
                1 => lss_driver::LedColor::Red,
                2 => lss_driver::LedColor::Green,
                3 => lss_driver::LedColor::Blue,
                4 => lss_driver::LedColor::Yellow,
                5 => lss_driver::LedColor::Cyan,
                6 => lss_driver::LedColor::Magenta,
                7 => lss_driver::LedColor::White,
                _ => lss_driver::LedColor::Off,
            };
            driver
                .set_color(color)
                .await
                .map_err(|_| Status::internal("failed to set color"))?;
            Ok(Response::new(guppy_service::SetLedResponse {}))
        } else {
            Err(Status::unimplemented("message"))
        }
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let address = "127.0.0.1:5002".parse().unwrap();
    let handler = GuppyConfigHandler::new(&args.port).await?;

    println!("Starting service at {}", address);

    Server::builder()
        .add_service(GuppyConfigureServer::new(handler))
        .serve(address)
        .await?;

    Ok(())
}
