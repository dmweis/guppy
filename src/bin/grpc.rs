use guppy::arm_config;
use guppy::arm_controller;
use guppy::arm_driver;

use clap::Clap;

use tonic::{transport::Server, Request, Response, Status};

use guppy_service::guppy_configure_server::{GuppyConfigure, GuppyConfigureServer};
use guppy_service::guppy_controller_server::{GuppyController, GuppyControllerServer};
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

type ControllerWrapper = Arc<Mutex<std::boxed::Box<dyn arm_controller::ArmController>>>;

async fn connect_to_arm(port: &str) -> Result<ControllerWrapper, Box<dyn std::error::Error>> {
    let config = arm_config::ArmConfig::included();
    let driver = arm_driver::SerialArmDriver::new(port, arm_config::ArmConfig::included()).await?;
    let controller = arm_controller::LssArmController::new(driver, config);
    Ok(Arc::new(Mutex::new(controller)))
}

struct GuppyConfigHandler {
    driver: ControllerWrapper,
}

impl GuppyConfigHandler {
    fn new(driver: ControllerWrapper) -> Self {
        GuppyConfigHandler { driver }
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

    async fn set_configuration(
        &self,
        request: Request<guppy_service::ArmControlSettings>,
    ) -> Result<Response<guppy_service::ConfigurationResponse>, Status> {
        let config_proto = request.into_inner();
        let mut driver = self.driver.lock().await;
        driver
            .setup_motors(config_proto.into())
            .await
            .map_err(|_| Status::unimplemented("message"))?;
        Ok(Response::new(guppy_service::ConfigurationResponse {}))
    }

    async fn get_default_configuration(
        &self,
        _: Request<guppy_service::ConfigurationRequest>,
    ) -> Result<Response<guppy_service::ArmControlSettings>, Status> {
        let default_config = arm_driver::ArmControlSettings::default().into();
        Ok(Response::new(default_config))
    }

    async fn get_arm_configuration(
        &self,
        _: Request<guppy_service::ConfigurationRequest>,
    ) -> Result<Response<guppy_service::ArmControlSettings>, Status> {
        let mut driver = self.driver.lock().await;
        let arm_settings = driver
            .load_motor_settings()
            .await
            .map_err(|_| Status::internal("failed to read config from arm"))?;
        Ok(Response::new(arm_settings.into()))
    }
}

struct GuppyControllerHandler {
    driver: ControllerWrapper,
}

impl GuppyControllerHandler {
    fn new(driver: ControllerWrapper) -> Self {
        GuppyControllerHandler { driver }
    }
}

#[tonic::async_trait]
impl GuppyController for GuppyControllerHandler {
    async fn move_to(
        &self,
        request: Request<guppy_service::MoveToCommand>,
    ) -> Result<Response<guppy_service::ArmPositions>, Status> {
        let inner = request.into_inner();
        let position = inner
            .position
            .ok_or_else(|| Status::invalid_argument("missing position"))?;
        let mut driver = self.driver.lock().await;
        let joint_positions = driver
            .move_to(position.into(), inner.effector_angle)
            .await
            .map_err(|_| Status::internal("Failed to move"))?;
        let arm_positions = driver
            .calculate_fk(joint_positions)
            .await
            .map_err(|_| Status::internal("Failed to calculate FK"))?;
        Ok(Response::new(arm_positions.into()))
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let address = "0.0.0.0:5002".parse().unwrap();
    let driver = connect_to_arm(&args.port).await?;
    let config_handler = GuppyConfigHandler::new(Arc::clone(&driver));
    let controller_handler = GuppyControllerHandler::new(driver);

    println!("Starting service at {}", address);

    Server::builder()
        .add_service(GuppyConfigureServer::new(config_handler))
        .add_service(GuppyControllerServer::new(controller_handler))
        .serve(address)
        .await?;

    Ok(())
}

impl From<guppy_service::ArmControlSettings> for arm_driver::ArmControlSettings {
    fn from(source: guppy_service::ArmControlSettings) -> Self {
        arm_driver::ArmControlSettings {
            base: source.base.map(|config| config.into()),
            shoulder: source.shoulder.map(|config| config.into()),
            elbow: source.elbow.map(|config| config.into()),
            wrist: source.wrist.map(|config| config.into()),
        }
    }
}

impl From<guppy_service::ServoControlSettings> for arm_driver::ServoControlSettings {
    fn from(source: guppy_service::ServoControlSettings) -> Self {
        arm_driver::ServoControlSettings {
            motion_profile: source.motion_profile,
            angular_holding_stiffness: source.angular_holding_stiffness,
            angular_stiffness: source.angular_stiffness,
            filter_position_count: source.filter_position_count.map(|val| val as u8),
            maximum_motor_duty: source.maximum_motor_duty,
            angular_acceleration: source.angular_acceleration,
            angular_deceleration: source.angular_deceleration,
            maximum_speed_degrees: source.maximum_speed_degrees,
        }
    }
}

impl From<arm_driver::ArmControlSettings> for guppy_service::ArmControlSettings {
    fn from(source: arm_driver::ArmControlSettings) -> Self {
        guppy_service::ArmControlSettings {
            base: source.base.map(|config| config.into()),
            shoulder: source.shoulder.map(|config| config.into()),
            elbow: source.elbow.map(|config| config.into()),
            wrist: source.wrist.map(|config| config.into()),
        }
    }
}

impl From<arm_driver::ServoControlSettings> for guppy_service::ServoControlSettings {
    fn from(source: arm_driver::ServoControlSettings) -> Self {
        guppy_service::ServoControlSettings {
            motion_profile: source.motion_profile,
            angular_holding_stiffness: source.angular_holding_stiffness,
            angular_stiffness: source.angular_stiffness,
            filter_position_count: source.filter_position_count.map(|val| val as u32),
            maximum_motor_duty: source.maximum_motor_duty,
            angular_acceleration: source.angular_acceleration,
            angular_deceleration: source.angular_deceleration,
            maximum_speed_degrees: source.maximum_speed_degrees,
        }
    }
}

impl From<nalgebra::Vector3<f32>> for guppy_service::Vector {
    fn from(source: nalgebra::Vector3<f32>) -> Self {
        guppy_service::Vector {
            x: source.x,
            y: source.y,
            z: source.z,
        }
    }
}

impl Into<nalgebra::Vector3<f32>> for guppy_service::Vector {
    fn into(self) -> nalgebra::Vector3<f32> {
        nalgebra::Vector3::new(self.x, self.y, self.z)
    }
}

impl From<arm_driver::JointPositions> for guppy_service::JointPositions {
    fn from(source: arm_driver::JointPositions) -> Self {
        guppy_service::JointPositions {
            base: source.base,
            shoulder: source.shoulder,
            elbow: source.elbow,
            wrist: source.wrist,
        }
    }
}

impl From<arm_controller::ArmPositions> for guppy_service::ArmPositions {
    fn from(source: arm_controller::ArmPositions) -> Self {
        guppy_service::ArmPositions {
            base: Some(source.base.into()),
            shoulder: Some(source.shoulder.into()),
            elbow: Some(source.elbow.into()),
            wrist: Some(source.wrist.into()),
            end_effector: Some(source.end_effector.into()),
            end_effector_angle: source.end_effector_angle,
        }
    }
}
