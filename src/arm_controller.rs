use crate::arm_config;
use crate::arm_driver;
use crate::arm_driver::JointPositions;
use async_trait::async_trait;
use nalgebra as na;
use std::error::Error;

#[derive(Debug, Clone, PartialEq)]
pub struct EndEffectorPose {
    pub position: na::Vector3<f32>,
    pub end_effector_angle: f32,
}

impl EndEffectorPose {
    pub fn new(position: na::Vector3<f32>, end_effector_angle: f32) -> EndEffectorPose {
        EndEffectorPose {
            position,
            end_effector_angle,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ArmPositions {
    pub base: na::Vector3<f32>,
    pub shoulder: na::Vector3<f32>,
    pub elbow: na::Vector3<f32>,
    pub wrist: na::Vector3<f32>,
    pub end_effector: na::Vector3<f32>,
    pub end_effector_angle: f32,
}

impl ArmPositions {
    pub fn new(
        base: na::Vector3<f32>,
        shoulder: na::Vector3<f32>,
        elbow: na::Vector3<f32>,
        wrist: na::Vector3<f32>,
        end_effector: na::Vector3<f32>,
        end_effector_angle: f32,
    ) -> ArmPositions {
        ArmPositions {
            base,
            shoulder,
            elbow,
            wrist,
            end_effector,
            end_effector_angle,
        }
    }
}

#[async_trait(?Send)]
pub trait ArmController {
    async fn set_color(&mut self, color: lss_driver::LedColor) -> Result<(), Box<dyn Error>>;
    async fn calculate_ik(
        &self,
        position: na::Vector3<f32>,
        effector_angle: f32,
    ) -> Result<JointPositions, Box<dyn Error>>;
    async fn calculate_fk(&self, joints: JointPositions) -> Result<ArmPositions, Box<dyn Error>>;
    async fn read_position(&mut self) -> Result<ArmPositions, Box<dyn Error>>;
    async fn move_to(
        &mut self,
        position: na::Vector3<f32>,
        effector_angle: f32,
    ) -> Result<JointPositions, Box<dyn Error>>;
    async fn halt(&mut self) -> Result<(), Box<dyn Error>>;
    async fn limp(&mut self) -> Result<(), Box<dyn Error>>;
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
    async fn calculate_ik(
        &self,
        position: na::Vector3<f32>,
        effector_angle: f32,
    ) -> Result<JointPositions, Box<dyn Error>> {
        let effector_angle = effector_angle.to_radians();
        let base_angle = position.y.atan2(position.x);
        let horizontal_distance = (position.x.powi(2) + position.y.powi(2)).sqrt();
        let height = position.z - self.config.shoulder.z;
        // end effector calc
        let end_effector_len = self.config.end_effector.magnitude();
        let effector_horizontal = effector_angle.cos() * end_effector_len;
        let effector_vertical = -(effector_angle.sin() * end_effector_len);
        let reduced_horizontal_distance = horizontal_distance - effector_horizontal;
        let reduced_height = height - effector_vertical;

        // TODO: make sure these are always positive
        let shoulder_length = self.config.elbow.magnitude();
        let forearm_length = self.config.wrist.magnitude();
        let reduced_distance =
            (reduced_height.powi(2) + reduced_horizontal_distance.powi(2)).sqrt();
        // correct until here I think
        // this is angle between horizontal plane and shoulder/forearm triangle
        let shoulder_plane_target_angle = reduced_height.atan2(reduced_horizontal_distance);

        let upper_shoulder_angle = ((reduced_distance.powi(2) + shoulder_length.powi(2)
            - forearm_length.powi(2))
            / (2.0 * reduced_distance * shoulder_length))
            .acos();
        // don't forget about shoulder not being a straight line!
        let shoulder_offset_angle = (self.config.elbow.x / self.config.elbow.z).atan();
        let shoulder_angle =
            shoulder_plane_target_angle + upper_shoulder_angle + shoulder_offset_angle;

        let elbow_angle = ((forearm_length.powi(2) + shoulder_length.powi(2)
            - reduced_distance.powi(2))
            / (2.0 * forearm_length * shoulder_length))
            .acos();
        let _wrist_angle = ((forearm_length.powi(2) + reduced_distance.powi(2)
            - shoulder_length.powi(2))
            / (2.0 * forearm_length * reduced_distance))
            .acos();

        let offset_shoulder = 90_f32.to_radians() - shoulder_angle;
        let offset_elbow_angle = 90_f32.to_radians() - elbow_angle + shoulder_offset_angle;
        Ok(JointPositions::new(
            base_angle.to_degrees(),
            offset_shoulder.to_degrees(),
            offset_elbow_angle.to_degrees(),
            (-offset_elbow_angle - offset_shoulder + effector_angle).to_degrees(),
        ))
    }

    async fn calculate_fk(&self, joints: JointPositions) -> Result<ArmPositions, Box<dyn Error>> {
        let base = na::Vector3::new(0.0, 0.0, 0.0);
        let base_rotation =
            na::Rotation3::from_axis_angle(&na::Vector3::z_axis(), joints.base.to_radians());
        let shoulder = base + self.config.shoulder;
        let shoulder_rotation =
            na::Rotation3::from_axis_angle(&na::Vector3::y_axis(), joints.shoulder.to_radians());
        let elbow = shoulder + base_rotation * shoulder_rotation * self.config.elbow;
        let elbow_rotation =
            na::Rotation3::from_axis_angle(&na::Vector3::y_axis(), joints.elbow.to_radians());
        let wrist = elbow + base_rotation * shoulder_rotation * elbow_rotation * self.config.wrist;
        let wrist_rotation =
            na::Rotation3::from_axis_angle(&na::Vector3::y_axis(), joints.wrist.to_radians());
        let end_effector: na::Vector3<f32> = wrist
            + base_rotation
                * shoulder_rotation
                * elbow_rotation
                * wrist_rotation
                * self.config.end_effector;
        let arm_positions = ArmPositions::new(
            base,
            shoulder,
            elbow,
            wrist,
            end_effector,
            joints.elbow + joints.shoulder + joints.wrist,
        );
        Ok(arm_positions)
    }

    async fn read_position(&mut self) -> Result<ArmPositions, Box<dyn Error>> {
        let motor_positions = self.driver.read_position().await?;
        let arm_positions = self.calculate_fk(motor_positions).await?;
        Ok(arm_positions)
    }

    async fn move_to(
        &mut self,
        position: na::Vector3<f32>,
        effector_angle: f32,
    ) -> Result<JointPositions, Box<dyn Error>> {
        let joint_positions = self.calculate_ik(position, effector_angle).await?;
        self.driver.move_to(joint_positions.clone()).await?;
        Ok(joint_positions)
    }

    async fn halt(&mut self) -> Result<(), Box<dyn Error>> {
        self.driver.halt().await?;
        Ok(())
    }

    async fn limp(&mut self) -> Result<(), Box<dyn Error>> {
        self.driver.limp().await?;
        Ok(())
    }
}
