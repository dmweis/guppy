use crate::arm_config;
use crate::arm_driver;
use async_trait::async_trait;
use nalgebra as na;
use std::error::Error;
use crate::arm_driver::JointPositions;

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
    async fn calculate_ik(&self, position: na::Vector3<f32>, effector_angle: f32) -> Result<JointPositions, Box<dyn Error>>;
    async fn calculate_fk(&self, joints: JointPositions) -> Result<ArmPositions, Box<dyn Error>>;
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

    /// calculate Ik
    /// This method expects a point translated from arm base
    async fn calculate_ik(&self, position: na::Vector3<f32>, effector_angle: f32) -> Result<JointPositions, Box<dyn Error>> {
        let effector_angle = effector_angle.to_radians();
        let base_angle = position.y.atan2(position.x);
        let horizontal_distance = (position.x.powi(2) + position.y.powi(2)).sqrt();
        let height = position.z - self.config.shoulder.z;
        let end_effector_len = self.config.end_effector.magnitude();
        // assuming end always at 0°
        let effector_horizontal = effector_angle.cos() * end_effector_len;
        let effector_vertical = effector_angle.sin() * end_effector_len;
        let reduced_horizontal_distance = horizontal_distance - 0.0;
        let reduced_height = height - 0.0;
        let shoulder_length = self.config.shoulder.magnitude();
        let forearm_length = self.config.elbow.magnitude();
        let arm_distance = (reduced_height.powi(2) + reduced_horizontal_distance.powi(2)).sqrt();
        let shoulder_plane_target_angle = (reduced_height / reduced_horizontal_distance).atan();
        
        let upper_shoulder_angle = ((arm_distance.powi(2) + shoulder_length.powi(2) - forearm_length.powi(2)) / 2.0 * arm_distance * shoulder_length).acos();
        let shoulder_angle = shoulder_plane_target_angle + upper_shoulder_angle;
        println!("{}", reduced_horizontal_distance);

        let elbow_angle = ((forearm_length.powi(2) + shoulder_length.powi(2) - arm_distance.powi(2)) / 2.0 * forearm_length * shoulder_length).acos();


        Ok(JointPositions::new(
            base_angle.to_degrees(),
            90.0 - shoulder_angle.to_degrees(),
            -90.0 + elbow_angle.to_degrees(),
            0.0
        ))
    }

    async fn calculate_fk(&self, joints: JointPositions) -> Result<ArmPositions, Box<dyn Error>> {
        let base = na::Vector3::new(0.0, 0.0, 0.0);
        let base_rotation = na::Rotation3::from_axis_angle(
            &na::Vector3::z_axis(),
            joints.base.to_radians(),
        );
        let shoulder = base + self.config.shoulder;
        let shoulder_rotation = na::Rotation3::from_axis_angle(
            &na::Vector3::y_axis(),
            joints.shoulder.to_radians(),
        );
        let elbow = shoulder + base_rotation * shoulder_rotation * self.config.elbow;
        let elbow_rotation = na::Rotation3::from_axis_angle(
            &na::Vector3::y_axis(),
            joints.elbow.to_radians(),
        );
        let wrist = elbow + base_rotation * shoulder_rotation * elbow_rotation * self.config.wrist;
        let wrist_rotation = na::Rotation3::from_axis_angle(
            &na::Vector3::y_axis(),
            joints.wrist.to_radians(),
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

    async fn read_position(&mut self) -> Result<ArmPositions, Box<dyn Error>> {
        let motor_positions = self.driver.read_position().await?;
        let arm_positions = self.calculate_fk(motor_positions).await?;
        Ok(arm_positions)
    }
}
