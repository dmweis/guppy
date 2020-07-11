use crate::arm_config;
use crate::arm_driver;
use async_trait::async_trait;
use nalgebra::Point3;
use std::error::Error;

#[async_trait(?Send)]
pub trait ArmController {
    async fn set_color(&mut self, color: lss_driver::LedColor) -> Result<(), Box<dyn Error>>;
    async fn move_to(&mut self, position: Point3<f32>) -> Result<(), Box<dyn Error>>;
    async fn read_position(&mut self) -> Result<Point3<f32>, Box<dyn Error>>;
}

pub struct LssArmController {
    driver: Box<dyn arm_driver::ArmDriver>,
    config: arm_config::ArmConfig,
}

impl LssArmController {
    pub fn new(
        driver: Box<dyn arm_driver::ArmDriver>,
        config: arm_config::ArmConfig,
    ) -> LssArmController {
        LssArmController { driver, config }
    }
}

#[async_trait(?Send)]
impl ArmController for LssArmController {
    async fn set_color(&mut self, color: lss_driver::LedColor) -> Result<(), Box<dyn Error>> {
        self.driver.set_color(color).await?;
        Ok(())
    }

    async fn move_to(&mut self, position: Point3<f32>) -> Result<(), Box<dyn Error>> {
        unimplemented!("Not yet!");
    }

    async fn read_position(&mut self) -> Result<Point3<f32>, Box<dyn Error>> {
        unimplemented!("Not yet!");
    }
}
