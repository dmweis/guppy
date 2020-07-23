use crate::arm_config;
use async_trait::async_trait;
use std::error::Error;

#[derive(Debug, Clone, PartialEq)]
pub struct JointPositions {
    pub base: f32,
    pub shoulder: f32,
    pub elbow: f32,
    pub wrist: f32,
}

impl JointPositions {
    pub fn new(base: f32, shoulder: f32, elbow: f32, wrist: f32) -> JointPositions {
        JointPositions {
            base,
            shoulder,
            elbow,
            wrist,
        }
    }
}

#[async_trait(?Send)]
pub trait ArmDriver {
    async fn set_color(&mut self, color: lss_driver::LedColor) -> Result<(), Box<dyn Error>>;
    async fn setup_motors(&mut self) -> Result<(), Box<dyn Error>>;
    async fn halt(&mut self) -> Result<(), Box<dyn Error>>;
    async fn limp(&mut self) -> Result<(), Box<dyn Error>>;
    async fn move_to(&mut self, position: JointPositions) -> Result<(), Box<dyn Error>>;
    async fn read_position(&mut self) -> Result<JointPositions, Box<dyn Error>>;
}

pub struct SerialArmDriver {
    driver: lss_driver::LSSDriver,
    config: arm_config::ArmConfig,
}

impl SerialArmDriver {
    #[allow(clippy::new_ret_no_self)]
    pub async fn new(
        port: &str,
        config: arm_config::ArmConfig,
    ) -> Result<Box<dyn ArmDriver>, Box<dyn Error>> {
        let driver = lss_driver::LSSDriver::new(port)?;
        let mut arm_driver = SerialArmDriver { driver, config };
        arm_driver.setup_motors().await?;
        Ok(Box::new(arm_driver))
    }
}

#[async_trait(?Send)]
impl ArmDriver for SerialArmDriver {
    async fn set_color(&mut self, color: lss_driver::LedColor) -> Result<(), Box<dyn Error>> {
        self.driver
            .set_color(lss_driver::BROADCAST_ID, color)
            .await?;
        Ok(())
    }

    async fn setup_motors(&mut self) -> Result<(), Box<dyn Error>> {
        self.driver
            .set_motion_profile(lss_driver::BROADCAST_ID, false)
            .await?;
        self.driver
            .set_angular_holding_stiffness(lss_driver::BROADCAST_ID, -4)
            .await?;
        self.driver
            .set_angular_stiffness(lss_driver::BROADCAST_ID, -4)
            .await?;
        self.driver
            .set_filter_position_count(lss_driver::BROADCAST_ID, 0)
            .await?;
        Ok(())
    }

    async fn halt(&mut self) -> Result<(), Box<dyn Error>> {
        self.driver.halt_hold(lss_driver::BROADCAST_ID).await?;
        Ok(())
    }

    async fn limp(&mut self) -> Result<(), Box<dyn Error>> {
        self.driver.limp(lss_driver::BROADCAST_ID).await?;
        Ok(())
    }

    async fn move_to(&mut self, position: JointPositions) -> Result<(), Box<dyn Error>> {
        self.driver
            .move_to_position(self.config.base_id, position.base)
            .await?;
        self.driver
            .move_to_position(self.config.shoulder_id, position.shoulder)
            .await?;
        self.driver
            .move_to_position(self.config.elbow_id, position.elbow)
            .await?;
        self.driver
            .move_to_position(self.config.wrist_id, position.wrist)
            .await?;
        Ok(())
    }

    async fn read_position(&mut self) -> Result<JointPositions, Box<dyn Error>> {
        let base_position = self.driver.query_position(self.config.base_id).await?;
        let shoulder_position = self.driver.query_position(self.config.shoulder_id).await?;
        let elbow_position = self.driver.query_position(self.config.elbow_id).await?;
        let wrist_position = self.driver.query_position(self.config.wrist_id).await?;
        Ok(JointPositions::new(
            base_position,
            shoulder_position,
            elbow_position,
            wrist_position,
        ))
    }
}
