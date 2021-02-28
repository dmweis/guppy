use crate::arm_config;
use anyhow::Result;
use async_trait::async_trait;
use lss_driver::LSSDriver;
pub use lss_driver::LedColor;
use serde::{Deserialize, Serialize};
use std::{fs, include_bytes, str, sync::Arc};
use tokio::sync::Mutex;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ServoControlSettings {
    /// Settings for servo in arm
    #[serde(skip_serializing_if = "Option::is_none")]
    pub motion_profile: Option<bool>,
    /// useful for non interpolated mode
    #[serde(skip_serializing_if = "Option::is_none")]
    pub angular_holding_stiffness: Option<i32>,
    /// -10 to 10, usually -4 to 4
    #[serde(skip_serializing_if = "Option::is_none")]
    pub angular_stiffness: Option<i32>,
    /// -10 to 10, usually -4 to 4
    #[serde(skip_serializing_if = "Option::is_none")]
    pub filter_position_count: Option<u8>,
    /// I don't know how this works
    #[serde(skip_serializing_if = "Option::is_none")]
    pub maximum_motor_duty: Option<u32>,
    /// useful for compliance, 255 to 1023
    #[serde(skip_serializing_if = "Option::is_none")]
    pub angular_acceleration: Option<i32>,
    /// only when motion_profile is on
    #[serde(skip_serializing_if = "Option::is_none")]
    pub angular_deceleration: Option<i32>,
    /// only when motion_profile is on
    #[serde(skip_serializing_if = "Option::is_none")]
    pub maximum_speed_degrees: Option<u32>,
}

impl ServoControlSettings {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        motion_profile: Option<bool>,
        angular_holding_stiffness: Option<i32>,
        angular_stiffness: Option<i32>,
        filter_position_count: Option<u8>,
        maximum_motor_duty: Option<u32>,
        angular_acceleration: Option<i32>,
        angular_deceleration: Option<i32>,
        maximum_speed_degrees: Option<u32>,
    ) -> ServoControlSettings {
        ServoControlSettings {
            motion_profile,
            angular_holding_stiffness,
            angular_stiffness,
            filter_position_count,
            maximum_motor_duty,
            angular_acceleration,
            angular_deceleration,
            maximum_speed_degrees,
        }
    }
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

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ArmControlSettings {
    /// Settings for each servo in the arm
    #[serde(skip_serializing_if = "Option::is_none")]
    pub base: Option<ServoControlSettings>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub shoulder: Option<ServoControlSettings>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub elbow: Option<ServoControlSettings>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wrist: Option<ServoControlSettings>,
}

impl ArmControlSettings {
    fn parse_json(json: &str) -> Result<ArmControlSettings> {
        let config = serde_json::from_str(json)?;
        Ok(config)
    }

    pub fn load_json(path: &str) -> Result<ArmControlSettings> {
        let text = fs::read_to_string(path)?;
        let config = ArmControlSettings::parse_json(&text)?;
        Ok(config)
    }

    pub fn save_json(&self, path: &str) -> Result<()> {
        let json = serde_json::to_string_pretty(self)?;
        fs::write(path, &json)?;
        Ok(())
    }

    /// Guppy comes with an included config file.
    ///
    /// This file is packaged with the binary
    /// This method retrieves this included version
    pub fn included() -> ArmControlSettings {
        let json = str::from_utf8(include_bytes!("../config/motor_settings.json")).unwrap();
        ArmControlSettings::parse_json(json).unwrap()
    }
}

impl Default for ArmControlSettings {
    fn default() -> Self {
        ArmControlSettings::included()
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

#[async_trait]
pub trait ArmDriver: Send + Sync {
    async fn set_color(&mut self, color: lss_driver::LedColor) -> Result<()>;
    async fn setup_motors(&mut self, settings: ArmControlSettings) -> Result<()>;
    async fn load_motor_settings(&mut self) -> Result<ArmControlSettings>;
    async fn halt(&mut self) -> Result<()>;
    async fn limp(&mut self) -> Result<()>;
    async fn move_to(&mut self, position: JointPositions) -> Result<()>;
    async fn read_position(&mut self) -> Result<JointPositions>;
    async fn move_gripper(&mut self, closed: f32) -> Result<()>;
}

pub struct SerialArmDriver {
    driver: lss_driver::LSSDriver,
    config: arm_config::ArmConfig,
}

impl SerialArmDriver {
    pub async fn new(port: &str, config: arm_config::ArmConfig) -> Result<Box<Self>> {
        let driver = lss_driver::LSSDriver::new(port)?;
        let mut arm_driver = SerialArmDriver { driver, config };
        arm_driver
            .setup_motors(ArmControlSettings::default())
            .await?;
        Ok(Box::new(arm_driver))
    }
}

#[async_trait]
impl ArmDriver for SerialArmDriver {
    async fn set_color(&mut self, color: lss_driver::LedColor) -> Result<()> {
        for id in self.config.get_ids().iter() {
            self.driver.set_color(*id, color).await?;
        }
        Ok(())
    }

    async fn setup_motors(&mut self, settings: ArmControlSettings) -> Result<()> {
        async fn set_motor(
            driver: &mut lss_driver::LSSDriver,
            motor_id: u8,
            settings: &ServoControlSettings,
        ) -> Result<()> {
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
            if let Some(maximum_motor_duty) = settings.maximum_motor_duty {
                driver
                    .set_maximum_motor_duty(motor_id, maximum_motor_duty as i32)
                    .await?;
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
            if let Some(maximum_speed_degrees) = settings.maximum_speed_degrees {
                driver
                    .set_maximum_speed(motor_id, maximum_speed_degrees as f32)
                    .await?;
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

    async fn load_motor_settings(&mut self) -> Result<ArmControlSettings> {
        async fn read_motor_config(
            driver: &mut lss_driver::LSSDriver,
            motor_id: u8,
        ) -> Result<ServoControlSettings> {
            let motion_profile = driver.query_motion_profile(motor_id).await?;
            let angular_holding_stiffness =
                driver.query_angular_holding_stiffness(motor_id).await?;
            let angular_stiffness = driver.query_angular_stiffness(motor_id).await?;
            let filter_position_count = driver.query_filter_position_count(motor_id).await?;
            let maximum_motor_duty = driver.query_maximum_motor_duty(motor_id).await? as u32;
            let angular_acceleration = driver.query_angular_acceleration(motor_id).await?;
            let angular_deceleration = driver.query_angular_deceleration(motor_id).await?;
            let maximum_speed_degrees = driver.query_maximum_speed(motor_id).await? as u32;
            Ok(ServoControlSettings::new(
                Some(motion_profile),
                Some(angular_holding_stiffness),
                Some(angular_stiffness),
                Some(filter_position_count),
                Some(maximum_motor_duty),
                Some(angular_acceleration),
                Some(angular_deceleration),
                Some(maximum_speed_degrees),
            ))
        }
        let base = Some(read_motor_config(&mut self.driver, self.config.base_id).await?);
        let shoulder = Some(read_motor_config(&mut self.driver, self.config.shoulder_id).await?);
        let elbow = Some(read_motor_config(&mut self.driver, self.config.elbow_id).await?);
        let wrist = Some(read_motor_config(&mut self.driver, self.config.wrist_id).await?);
        Ok(ArmControlSettings {
            base,
            shoulder,
            elbow,
            wrist,
        })
    }

    async fn halt(&mut self) -> Result<()> {
        for id in self.config.get_ids().iter() {
            self.driver.halt_hold(*id).await?;
        }
        Ok(())
    }

    async fn limp(&mut self) -> Result<()> {
        for id in self.config.get_ids().iter() {
            self.driver.limp(*id).await?;
        }
        Ok(())
    }

    async fn move_to(&mut self, position: JointPositions) -> Result<()> {
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

    async fn read_position(&mut self) -> Result<JointPositions> {
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

    /// Set how open the gripper is
    /// 0.0 is fully open
    /// 1.0 is fully closed
    async fn move_gripper(&mut self, closed: f32) -> Result<()> {
        let desired_position = calc_gripper(closed);
        self.driver
            .move_to_position(self.config.gripper_id, desired_position)
            .await?;
        Ok(())
    }
}

pub struct SharedSerialArmDriver {
    driver: Arc<Mutex<LSSDriver>>,
    config: arm_config::ArmConfig,
}

impl SharedSerialArmDriver {
    pub async fn new(
        driver: Arc<Mutex<LSSDriver>>,
        config: arm_config::ArmConfig,
    ) -> Result<Box<Self>> {
        let mut arm_driver = Self { driver, config };
        arm_driver
            .setup_motors(ArmControlSettings::default())
            .await?;
        Ok(Box::new(arm_driver))
    }
}

#[async_trait]
impl ArmDriver for SharedSerialArmDriver {
    async fn set_color(&mut self, color: lss_driver::LedColor) -> Result<()> {
        let mut driver = self.driver.lock().await;
        for id in self.config.get_ids().iter() {
            driver.set_color(*id, color).await?;
        }
        Ok(())
    }

    async fn setup_motors(&mut self, settings: ArmControlSettings) -> Result<()> {
        async fn set_motor(
            driver: &mut lss_driver::LSSDriver,
            motor_id: u8,
            settings: &ServoControlSettings,
        ) -> Result<()> {
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
            if let Some(maximum_motor_duty) = settings.maximum_motor_duty {
                driver
                    .set_maximum_motor_duty(motor_id, maximum_motor_duty as i32)
                    .await?;
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
            if let Some(maximum_speed_degrees) = settings.maximum_speed_degrees {
                driver
                    .set_maximum_speed(motor_id, maximum_speed_degrees as f32)
                    .await?;
            }
            Ok(())
        }
        let mut driver = self.driver.lock().await;
        if let Some(base_settings) = &settings.base {
            set_motor(&mut driver, self.config.base_id, base_settings).await?;
        }
        if let Some(shoulder_settings) = &settings.shoulder {
            set_motor(&mut driver, self.config.shoulder_id, shoulder_settings).await?;
        }
        if let Some(elbow_settings) = &settings.elbow {
            set_motor(&mut driver, self.config.elbow_id, elbow_settings).await?;
        }
        if let Some(wrist_settings) = &settings.wrist {
            set_motor(&mut driver, self.config.wrist_id, wrist_settings).await?;
        }
        Ok(())
    }

    async fn load_motor_settings(&mut self) -> Result<ArmControlSettings> {
        async fn read_motor_config(
            driver: &mut lss_driver::LSSDriver,
            motor_id: u8,
        ) -> Result<ServoControlSettings> {
            let motion_profile = driver.query_motion_profile(motor_id).await?;
            let angular_holding_stiffness =
                driver.query_angular_holding_stiffness(motor_id).await?;
            let angular_stiffness = driver.query_angular_stiffness(motor_id).await?;
            let filter_position_count = driver.query_filter_position_count(motor_id).await?;
            let maximum_motor_duty = driver.query_maximum_motor_duty(motor_id).await? as u32;
            let angular_acceleration = driver.query_angular_acceleration(motor_id).await?;
            let angular_deceleration = driver.query_angular_deceleration(motor_id).await?;
            let maximum_speed_degrees = driver.query_maximum_speed(motor_id).await? as u32;
            Ok(ServoControlSettings::new(
                Some(motion_profile),
                Some(angular_holding_stiffness),
                Some(angular_stiffness),
                Some(filter_position_count),
                Some(maximum_motor_duty),
                Some(angular_acceleration),
                Some(angular_deceleration),
                Some(maximum_speed_degrees),
            ))
        }
        let mut driver = self.driver.lock().await;
        let base = Some(read_motor_config(&mut driver, self.config.base_id).await?);
        let shoulder = Some(read_motor_config(&mut driver, self.config.shoulder_id).await?);
        let elbow = Some(read_motor_config(&mut driver, self.config.elbow_id).await?);
        let wrist = Some(read_motor_config(&mut driver, self.config.wrist_id).await?);
        Ok(ArmControlSettings {
            base,
            shoulder,
            elbow,
            wrist,
        })
    }

    async fn halt(&mut self) -> Result<()> {
        let mut driver = self.driver.lock().await;
        for id in self.config.get_ids().iter() {
            driver.halt_hold(*id).await?;
        }
        Ok(())
    }

    async fn limp(&mut self) -> Result<()> {
        let mut driver = self.driver.lock().await;
        for id in self.config.get_ids().iter() {
            driver.limp(*id).await?;
        }
        Ok(())
    }

    async fn move_to(&mut self, position: JointPositions) -> Result<()> {
        let mut driver = self.driver.lock().await;
        driver
            .move_to_position(self.config.base_id, position.base)
            .await?;
        driver
            .move_to_position(self.config.shoulder_id, position.shoulder)
            .await?;
        driver
            .move_to_position(self.config.elbow_id, position.elbow)
            .await?;
        driver
            .move_to_position(self.config.wrist_id, position.wrist)
            .await?;
        Ok(())
    }

    async fn read_position(&mut self) -> Result<JointPositions> {
        let mut driver = self.driver.lock().await;
        let base_position = driver.query_position(self.config.base_id).await?;
        let shoulder_position = driver.query_position(self.config.shoulder_id).await?;
        let elbow_position = driver.query_position(self.config.elbow_id).await?;
        let wrist_position = driver.query_position(self.config.wrist_id).await?;
        Ok(JointPositions::new(
            base_position,
            shoulder_position,
            elbow_position,
            wrist_position,
        ))
    }

    /// Set how open the gripper is
    /// 0.0 is fully open
    /// 1.0 is fully closed
    async fn move_gripper(&mut self, closed: f32) -> Result<()> {
        let desired_position = calc_gripper(closed);
        self.driver
            .lock()
            .await
            .move_to_position(self.config.gripper_id, desired_position)
            .await?;
        Ok(())
    }
}

fn calc_gripper(open: f32) -> f32 {
    let val = open.min(1.0).max(0.0);
    -1.0 * (1.0 - val) * 90.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn arm_driver_includes_default_config() {
        ArmControlSettings::default();
    }

    #[test]
    fn test_closed_gripper() {
        let res = calc_gripper(1.0);
        assert_relative_eq!(res, 0.0)
    }

    #[test]
    fn test_opened_gripper() {
        let res = calc_gripper(0.0);
        assert_relative_eq!(res, -90.0)
    }

    #[test]
    fn gripper_clamps_opened() {
        let res = calc_gripper(-2.0);
        assert_relative_eq!(res, -90.0)
    }

    #[test]
    fn gripper_clamps_closed() {
        let res = calc_gripper(20.0);
        assert_relative_eq!(res, 0.0)
    }
}
