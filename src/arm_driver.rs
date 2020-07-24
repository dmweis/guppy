use crate::arm_config;
use async_trait::async_trait;
use std::error::Error;

#[derive(Debug, Clone, PartialEq)]
pub struct ServoControlSettings {
    /// Settings for servo in arm
    pub motion_profile: Option<bool>,
    /// useful for non interpolated mode
    pub angular_holding_stiffness: Option<i32>,
    /// -10 to 10, usually -4 to 4
    pub angular_stiffness: Option<i32>,
    /// -10 to 10, usually -4 to 4
    pub filter_position_count: Option<u8>,
    /// I don't know how this works
    pub maximum_motor_duty: Option<u32>,
    /// useful for compliance, 255 to 1023
    pub angular_acceleration: Option<i32>,
    /// only when motion_profile is on
    pub angular_deceleration: Option<i32>,
    /// only when motion_profile is on
    pub maximum_speed_degrees: Option<u32>,
}

impl Default for ServoControlSettings {
    fn default() -> Self {
        ServoControlSettings {
            motion_profile: Some(false),
            angular_holding_stiffness: Some(-4),
            angular_stiffness: Some(-4),
            filter_position_count: Some(0),
            maximum_motor_duty: None,
            angular_acceleration: None,
            angular_deceleration: None,
            maximum_speed_degrees: None,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ArmControlSettings {
    /// Settings for each servo in the arm
    pub base: Option<ServoControlSettings>,
    pub shoulder: Option<ServoControlSettings>,
    pub elbow: Option<ServoControlSettings>,
    pub wrist: Option<ServoControlSettings>,
}

impl Default for ArmControlSettings {
    fn default() -> Self {
        ArmControlSettings {
            base: Some(ServoControlSettings::default()),
            shoulder: Some(ServoControlSettings::default()),
            elbow: Some(ServoControlSettings::default()),
            wrist: Some(ServoControlSettings::default()),
        }
    }
}

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
    async fn setup_motors(&mut self, settings: ArmControlSettings) -> Result<(), Box<dyn Error>>;
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
        arm_driver
            .setup_motors(ArmControlSettings::default())
            .await?;
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

    async fn setup_motors(&mut self, settings: ArmControlSettings) -> Result<(), Box<dyn Error>> {
        async fn set_motor(
            driver: &mut lss_driver::LSSDriver,
            motor_id: u8,
            settings: &ServoControlSettings,
        ) -> Result<(), Box<dyn Error>> {
            if let Some(motion_profile) = settings.motion_profile {
                driver.set_motion_profile(motor_id, motion_profile).await?;
            }
            if let Some(angular_holding_stiffness) = settings.angular_holding_stiffness {
                driver
                    .set_angular_holding_stiffness(motor_id, angular_holding_stiffness)
                    .await?;
            }
            if let Some(angular_stiffness) = settings.angular_stiffness {
                driver
                    .set_angular_stiffness(motor_id, angular_stiffness)
                    .await?;
            }
            if let Some(filter_position_count) = settings.filter_position_count {
                driver
                    .set_filter_position_count(motor_id, filter_position_count)
                    .await?;
            }
            if let Some(_) = settings.maximum_motor_duty {
                unimplemented!(
                    "maximum_motor_duty is not yet supported in this version of the driver"
                );
                // driver.set_maximum_motor_duty(motor_id, maximum_motor_duty).await?;
            }
            if let Some(angular_acceleration) = settings.angular_acceleration {
                driver
                    .set_angular_acceleration(motor_id, angular_acceleration)
                    .await?;
            }
            if let Some(angular_deceleration) = settings.angular_deceleration {
                driver
                    .set_angular_deceleration(motor_id, angular_deceleration)
                    .await?;
            }
            if let Some(_) = settings.maximum_speed_degrees {
                unimplemented!(
                    "maximum_speed_degrees is not yet supported in this version of the driver"
                );
                // driver.set_maximum_speed_degrees(motor_id, maximum_speed_degrees).await?;
            }
            Ok(())
        }
        if let Some(base_settings) = &settings.base {
            set_motor(&mut self.driver, self.config.base_id, base_settings).await?;
        }
        if let Some(shoulder_settings) = &settings.shoulder {
            set_motor(&mut self.driver, self.config.shoulder_id, shoulder_settings).await?;
        }
        if let Some(elbow_settings) = &settings.elbow {
            set_motor(&mut self.driver, self.config.elbow_id, elbow_settings).await?;
        }
        if let Some(wrist_settings) = &settings.wrist {
            set_motor(&mut self.driver, self.config.wrist_id, wrist_settings).await?;
        }
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
