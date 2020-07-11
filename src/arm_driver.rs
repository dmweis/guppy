use crate::arm_config;
use async_trait::async_trait;
use std::error::Error;

pub struct JointPositions {
    base: f32,
    shoulder: f32,
    elbow: f32,
    wrist: f32,
}

#[async_trait(?Send)]
pub trait ArmDriver {
    async fn set_color(&mut self, color: lss_driver::LedColor) -> Result<(), Box<dyn Error>>;
    async fn setup_motors(&mut self) -> Result<(), Box<dyn Error>>;
    async fn halt(&mut self) -> Result<(), Box<dyn Error>>;
    async fn limp(&mut self) -> Result<(), Box<dyn Error>>;
    async fn move_to(&mut self, position: JointPositions) -> Result<(), Box<dyn Error>>;
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
            .set_angular_holding_stiffness(lss_driver::BROADCAST_ID, -2)
            .await?;
        self.driver
            .set_angular_stiffness(lss_driver::BROADCAST_ID, -2)
            .await?;
        self.driver
            .set_filter_position_count(lss_driver::BROADCAST_ID, 4)
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
}
