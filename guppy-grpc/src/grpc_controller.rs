use anyhow::Result;
use guppy_controller::arm_controller;
use guppy_controller::arm_driver;
use guppy_controller::{arm_config, arm_driver::ArmControlSettings};
pub use guppy_service::guppy_configure_server::{GuppyConfigure, GuppyConfigureServer};
pub use guppy_service::guppy_controller_server::{GuppyController, GuppyControllerServer};
use std::sync::Arc;
use tokio::sync::Mutex;
use tonic::{Request, Response, Status};

pub mod guppy_service {
    tonic::include_proto!("guppy_service");
}

pub type ControllerWrapper = Arc<Mutex<std::boxed::Box<dyn arm_controller::ArmController>>>;

pub async fn connect_to_arm(port: &str) -> Result<ControllerWrapper> {
    let config = arm_config::ArmConfig::included();
    let driver = arm_driver::SerialArmDriver::new(
        port,
        config.clone(),
        ArmControlSettings::included_continuous(),
    )
    .await?;
    let controller = arm_controller::LssArmController::new(driver, config);
    Ok(Arc::new(Mutex::new(controller)))
}

pub struct GuppyConfigHandler {
    driver: ControllerWrapper,
}

impl GuppyConfigHandler {
    pub fn new(driver: ControllerWrapper) -> Self {
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
                1 => arm_driver::LedColor::Red,
                2 => arm_driver::LedColor::Green,
                3 => arm_driver::LedColor::Blue,
                4 => arm_driver::LedColor::Yellow,
                5 => arm_driver::LedColor::Cyan,
                6 => arm_driver::LedColor::Magenta,
                7 => arm_driver::LedColor::White,
                _ => arm_driver::LedColor::Off,
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
        let default_config = arm_driver::ArmControlSettings::included_continuous().into();
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

pub struct GuppyControllerHandler {
    driver: ControllerWrapper,
}

impl GuppyControllerHandler {
    pub fn new(driver: ControllerWrapper) -> Self {
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
            .move_to(arm_controller::EndEffectorPose::new(
                position.into(),
                inner.effector_angle,
            ))
            .await
            .map_err(|_| Status::internal("Failed to move"))?;
        let arm_positions = driver.calculate_fk(joint_positions);
        Ok(Response::new(arm_positions.into()))
    }

    async fn set_gripper(
        &self,
        request: Request<guppy_service::SetGripperRequest>,
    ) -> Result<Response<guppy_service::SetGripperResponse>, Status> {
        let gripper_position = request.into_inner();
        let mut driver = self.driver.lock().await;
        driver
            .move_gripper(gripper_position.gripper)
            .await
            .map_err(|_| Status::internal("failed to move gripper"))?;
        Ok(Response::new(guppy_service::SetGripperResponse {}))
    }
}

impl From<guppy_service::ArmControlSettings> for arm_driver::ArmControlSettings {
    fn from(source: guppy_service::ArmControlSettings) -> Self {
        arm_driver::ArmControlSettings {
            base: source.base.map(|config| config.into()),
            shoulder: source.shoulder.map(|config| config.into()),
            elbow: source.elbow.map(|config| config.into()),
            wrist: source.wrist.map(|config| config.into()),
            gripper: source.gripper.map(|config| config.into()),
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
            gripper: source.gripper.map(|config| config.into()),
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

impl From<guppy_service::Vector> for nalgebra::Vector3<f32> {
    fn from(vector: guppy_service::Vector) -> Self {
        nalgebra::Vector3::new(vector.x, vector.y, vector.z)
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
