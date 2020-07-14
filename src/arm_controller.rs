use crate::arm_config;
use crate::arm_driver;
use async_trait::async_trait;
use nalgebra as na;
use std::error::Error;

#[derive(Debug, Clone, PartialEq)]
pub struct ArmPositions {
    pub base: na::Vector3<f32>,
    pub shoulder: na::Vector3<f32>,
    pub elbow: na::Vector3<f32>,
    pub wrist: na::Vector3<f32>,
    pub end_effector: na::Vector3<f32>,
}

impl ArmPositions {
    pub fn new(
        base: na::Vector3<f32>,
        shoulder: na::Vector3<f32>,
        elbow: na::Vector3<f32>,
        wrist: na::Vector3<f32>,
        end_effector: na::Vector3<f32>,
    ) -> ArmPositions {
        ArmPositions {
            base,
            shoulder,
            elbow,
            wrist,
            end_effector,
        }
    }
}

#[async_trait(?Send)]
pub trait ArmController {
    async fn set_color(&mut self, color: lss_driver::LedColor) -> Result<(), Box<dyn Error>>;
    async fn move_to(&mut self, position: na::Vector3<f32>) -> Result<(), Box<dyn Error>>;
    async fn read_position(&mut self) -> Result<ArmPositions, Box<dyn Error>>;
}

pub struct LssArmController {
    driver: Box<dyn arm_driver::ArmDriver>,
    config: arm_config::ArmConfig,
}

impl LssArmController {
    pub fn new(
        driver: Box<dyn arm_driver::ArmDriver>,
        config: arm_config::ArmConfig,
    ) -> Box<dyn ArmController> {
        Box::new(LssArmController { driver, config })
    }
}

#[async_trait(?Send)]
impl ArmController for LssArmController {
    async fn set_color(&mut self, color: lss_driver::LedColor) -> Result<(), Box<dyn Error>> {
        self.driver.set_color(color).await?;
        Ok(())
    }

    async fn move_to(&mut self, position: na::Vector3<f32>) -> Result<(), Box<dyn Error>> {
        unimplemented!("Not yet!");
    }

    async fn read_position(&mut self) -> Result<ArmPositions, Box<dyn Error>> {
        let motor_positions = self.driver.read_position().await?;
        let base = na::Vector3::new(0.0, 0.0, 0.0);
        let base_rotation = na::Rotation3::from_axis_angle(
            &na::Vector3::z_axis(),
            motor_positions.base.to_radians(),
        );
        let shoulder = base + self.config.shoulder;
        let shoulder_rotation = na::Rotation3::from_axis_angle(
            &na::Vector3::y_axis(),
            motor_positions.shoulder.to_radians(),
        );
        let elbow = shoulder + base_rotation * shoulder_rotation * self.config.elbow;
        let elbow_rotation = na::Rotation3::from_axis_angle(
            &na::Vector3::y_axis(),
            motor_positions.elbow.to_radians(),
        );
        let wrist = elbow + base_rotation * shoulder_rotation * elbow_rotation * self.config.wrist;
        let wrist_rotation = na::Rotation3::from_axis_angle(
            &na::Vector3::y_axis(),
            motor_positions.wrist.to_radians(),
        );
        let end_effector: na::Vector3<f32> = wrist
            + base_rotation
                * shoulder_rotation
                * elbow_rotation
                * wrist_rotation
                * self.config.end_effector;
        let arm_positions = ArmPositions::new(base, shoulder, elbow, wrist, end_effector);
        Ok(arm_positions)
    }
}
