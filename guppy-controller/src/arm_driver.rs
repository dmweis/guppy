use crate::arm_config;
use async_trait::async_trait;
use lazy_static::lazy_static;
pub use lss_driver::LedColor;
use lss_driver::{CommandModifier, LSSDriver};
use serde::{Deserialize, Serialize};
use std::{fs, str, sync::Arc, time::Duration};
use thiserror::Error;
use tokio::sync::Mutex;

#[derive(Error, Debug)]
pub enum DriverError {
    #[error("error while accessing configuration")]
    IoError(#[from] std::io::Error),
    #[error("error while parsing json")]
    DeserializationError(#[from] serde_json::error::Error),
    #[error("failed when talking to arm")]
    LssDriverError(#[from] lss_driver::LssDriverError),
}

type Result<T> = std::result::Result<T, DriverError>;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
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

lazy_static! {
    static ref INCLUDED_TRAJECTORY: ArmControlSettings = {
        let json = include_str!("../config/motor_settings_trajectory.json");
        ArmControlSettings::parse_json(json).unwrap()
    };
    static ref INCLUDED_CONTINUOUS: ArmControlSettings = {
        let json = include_str!("../config/motor_settings_continuous.json");
        ArmControlSettings::parse_json(json).unwrap()
    };
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
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
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gripper: Option<ServoControlSettings>,
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
    pub fn included_continuous() -> ArmControlSettings {
        // TODO (David): These don't need to be clones
        INCLUDED_CONTINUOUS.clone()
    }

    /// Guppy comes with an included config file.
    ///
    /// This file is packaged with the binary
    /// This method retrieves this included version
    pub fn included_trajectory() -> ArmControlSettings {
        // TODO (David): These don't need to be clones
        INCLUDED_TRAJECTORY.clone()
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

#[derive(Debug, PartialEq, Eq)]
pub enum MotorStatus {
    Ok,
    SafeMode,
    NotDetected,
}

impl MotorStatus {
    fn from_safety_mode_query_result(
        response: std::result::Result<lss_driver::SafeModeStatus, lss_driver::LssDriverError>,
    ) -> Self {
        // This is some pretty awkward logic
        match response {
            Ok(lss_driver::SafeModeStatus::NoLimits) => MotorStatus::Ok,
            Ok(_) => MotorStatus::SafeMode,
            Err(_) => MotorStatus::NotDetected,
        }
    }
}

#[derive(Debug)]
pub struct ArmMotorStatus {
    base: MotorStatus,
    shoulder: MotorStatus,
    elbow: MotorStatus,
    wrist: MotorStatus,
    gripper: MotorStatus,
}

impl ArmMotorStatus {
    pub fn is_arm_okay(&self) -> bool {
        self.base == MotorStatus::Ok
            && self.shoulder == MotorStatus::Ok
            && self.elbow == MotorStatus::Ok
            && self.wrist == MotorStatus::Ok
            && self.gripper == MotorStatus::Ok
    }
}

#[async_trait]
pub trait ArmDriver: Send + Sync {
    async fn set_color(&mut self, color: lss_driver::LedColor) -> Result<()>;
    async fn setup_motors(&mut self, settings: &ArmControlSettings) -> Result<()>;
    async fn load_motor_settings(&mut self) -> Result<ArmControlSettings>;
    async fn halt(&mut self) -> Result<()>;
    async fn limp(&mut self) -> Result<()>;
    async fn move_to(&mut self, position: &JointPositions) -> Result<()>;
    async fn move_to_timed(&mut self, position: &JointPositions, duration: Duration) -> Result<()>;
    async fn read_position(&mut self) -> Result<JointPositions>;
    /// 0.0 is fully open
    /// 1.0 is fully closed
    async fn move_gripper(
        &mut self,
        closed: f32,
        current_limit: u32,
        duration: Duration,
    ) -> Result<()>;
    async fn query_motor_status(&mut self) -> Result<ArmMotorStatus>;
    async fn restart_motors(&mut self) -> Result<()>;
}

pub struct SerialArmDriver {
    driver: lss_driver::LSSDriver,
    config: arm_config::ArmConfig,
}

impl SerialArmDriver {
    pub async fn new(
        port: &str,
        config: arm_config::ArmConfig,
        control_settings: &ArmControlSettings,
    ) -> Result<Box<Self>> {
        let driver = lss_driver::LSSDriver::new(port)?;
        let mut arm_driver = SerialArmDriver { driver, config };
        arm_driver.setup_motors(control_settings).await?;
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

    async fn setup_motors(&mut self, settings: &ArmControlSettings) -> Result<()> {
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
        if let Some(gripper_settings) = &settings.gripper {
            set_motor(&mut self.driver, self.config.gripper_id, gripper_settings).await?;
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
        let gripper = Some(read_motor_config(&mut self.driver, self.config.gripper_id).await?);
        Ok(ArmControlSettings {
            base,
            shoulder,
            elbow,
            wrist,
            gripper,
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

    async fn move_to(&mut self, position: &JointPositions) -> Result<()> {
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

    async fn move_to_timed(&mut self, position: &JointPositions, duration: Duration) -> Result<()> {
        self.driver
            .move_to_position_with_modifier(
                self.config.base_id,
                position.base,
                CommandModifier::TimedDuration(duration),
            )
            .await?;
        self.driver
            .move_to_position_with_modifier(
                self.config.shoulder_id,
                position.shoulder,
                CommandModifier::TimedDuration(duration),
            )
            .await?;
        self.driver
            .move_to_position_with_modifier(
                self.config.elbow_id,
                position.elbow,
                CommandModifier::TimedDuration(duration),
            )
            .await?;
        self.driver
            .move_to_position_with_modifier(
                self.config.wrist_id,
                position.wrist,
                CommandModifier::TimedDuration(duration),
            )
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
    async fn move_gripper(
        &mut self,
        closed: f32,
        current_limit: u32,
        duration: Duration,
    ) -> Result<()> {
        let desired_position = calc_gripper(closed);
        self.driver
            .move_to_position_with_modifiers(
                self.config.gripper_id,
                desired_position,
                &[
                    CommandModifier::CurrentHold(current_limit),
                    CommandModifier::TimedDuration(duration),
                ],
            )
            .await?;
        Ok(())
    }

    async fn query_motor_status(&mut self) -> Result<ArmMotorStatus> {
        let base = MotorStatus::from_safety_mode_query_result(
            self.driver.query_safety_status(self.config.base_id).await,
        );
        let shoulder = MotorStatus::from_safety_mode_query_result(
            self.driver
                .query_safety_status(self.config.shoulder_id)
                .await,
        );
        let elbow = MotorStatus::from_safety_mode_query_result(
            self.driver.query_safety_status(self.config.elbow_id).await,
        );
        let wrist = MotorStatus::from_safety_mode_query_result(
            self.driver.query_safety_status(self.config.wrist_id).await,
        );
        let gripper = MotorStatus::from_safety_mode_query_result(
            self.driver
                .query_safety_status(self.config.gripper_id)
                .await,
        );
        Ok(ArmMotorStatus {
            base,
            shoulder,
            elbow,
            wrist,
            gripper,
        })
    }

    async fn restart_motors(&mut self) -> Result<()> {
        for id in self.config.get_ids() {
            self.driver.reset(id).await?;
        }
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
        control_settings: &ArmControlSettings,
    ) -> Result<Box<Self>> {
        let mut arm_driver = Self { driver, config };
        arm_driver.setup_motors(control_settings).await?;
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

    async fn setup_motors(&mut self, settings: &ArmControlSettings) -> Result<()> {
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
        if let Some(gripper_settings) = &settings.gripper {
            set_motor(&mut driver, self.config.gripper_id, gripper_settings).await?;
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
        let gripper = Some(read_motor_config(&mut driver, self.config.gripper_id).await?);
        Ok(ArmControlSettings {
            base,
            shoulder,
            elbow,
            wrist,
            gripper,
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

    async fn move_to(&mut self, position: &JointPositions) -> Result<()> {
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

    async fn move_to_timed(&mut self, position: &JointPositions, duration: Duration) -> Result<()> {
        let mut driver = self.driver.lock().await;
        driver
            .move_to_position_with_modifier(
                self.config.base_id,
                position.base,
                CommandModifier::TimedDuration(duration),
            )
            .await?;
        driver
            .move_to_position_with_modifier(
                self.config.shoulder_id,
                position.shoulder,
                CommandModifier::TimedDuration(duration),
            )
            .await?;
        driver
            .move_to_position_with_modifier(
                self.config.elbow_id,
                position.elbow,
                CommandModifier::TimedDuration(duration),
            )
            .await?;
        driver
            .move_to_position_with_modifier(
                self.config.wrist_id,
                position.wrist,
                CommandModifier::TimedDuration(duration),
            )
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
    async fn move_gripper(
        &mut self,
        closed: f32,
        current_limit: u32,
        duration: Duration,
    ) -> Result<()> {
        let desired_position = calc_gripper(closed);
        self.driver
            .lock()
            .await
            .move_to_position_with_modifiers(
                self.config.gripper_id,
                desired_position,
                &[
                    CommandModifier::TimedDuration(duration),
                    CommandModifier::CurrentHold(current_limit),
                ],
            )
            .await?;
        Ok(())
    }

    async fn query_motor_status(&mut self) -> Result<ArmMotorStatus> {
        let mut driver = self.driver.lock().await;
        let base = MotorStatus::from_safety_mode_query_result(
            driver.query_safety_status(self.config.base_id).await,
        );
        let shoulder = MotorStatus::from_safety_mode_query_result(
            driver.query_safety_status(self.config.shoulder_id).await,
        );
        let elbow = MotorStatus::from_safety_mode_query_result(
            driver.query_safety_status(self.config.elbow_id).await,
        );
        let wrist = MotorStatus::from_safety_mode_query_result(
            driver.query_safety_status(self.config.wrist_id).await,
        );
        let gripper = MotorStatus::from_safety_mode_query_result(
            driver.query_safety_status(self.config.gripper_id).await,
        );
        Ok(ArmMotorStatus {
            base,
            shoulder,
            elbow,
            wrist,
            gripper,
        })
    }

    async fn restart_motors(&mut self) -> Result<()> {
        let mut driver = self.driver.lock().await;
        for id in self.config.get_ids() {
            driver.reset(id).await?;
        }
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
    fn arm_driver_includes_continuous_config() {
        ArmControlSettings::included_continuous();
    }

    #[test]
    fn arm_driver_includes_trajectory_config() {
        ArmControlSettings::included_trajectory();
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
