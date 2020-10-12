mod arm_config;
mod arm_controller;
mod arm_driver;

use tonic::{transport::Server, Request, Response, Status};

use tokio::sync::Mutex;
use std::sync::Arc;
use guppy_service::guppy_configure_server::{GuppyConfigure, GuppyConfigureServer};

pub mod guppy_service {
    tonic::include_proto!("guppy_service");
}

struct GuppyConfigHandler {
    driver: Arc<Mutex<std::boxed::Box<dyn arm_controller::ArmController>>>
}

impl GuppyConfigHandler {
    async fn new(port: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let config = arm_config::ArmConfig::included();
        let driver = arm_driver::SerialArmDriver::new(port, arm_config::ArmConfig::included()).await?;
        let controller = arm_controller::LssArmController::new(driver, config);
        Ok(GuppyConfigHandler {
            driver: Arc::new(Mutex::new(controller)),
        })
    }
}

#[tonic::async_trait]
impl GuppyConfigure for GuppyConfigHandler {
    async fn set_led(request: Request<guppy_service::SetLedRequest>) -> Result<Response<guppy_service::SetLedResponse>, Status> {
        Err(Status::aborted("lol"))
    }
}

fn main() {
    println!("Hello world");
}
