use crate::arm_config;
use async_trait::async_trait;
use std::error::Error;

#[async_trait(?Send)]
pub trait ArmDriver {
    async fn set_color(&mut self, color: lss_driver::LedColor) -> Result<(), Box<dyn Error>>;
}

pub struct SerialArmDriver {
    driver: lss_driver::LSSDriver,
}

impl SerialArmDriver {
    #[allow(clippy::new_ret_no_self)]
    pub fn new(port: &str) -> Result<Box<dyn ArmDriver>, Box<dyn Error>> {
        let driver = lss_driver::LSSDriver::new(port)?;
        Ok(Box::new(SerialArmDriver { driver }))
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
}
