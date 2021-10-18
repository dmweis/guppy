use crate::arm_config;
use crate::arm_driver;
use crate::arm_driver::JointPositions;
use async_trait::async_trait;
use nalgebra as na;
use std::time::Duration;
use thiserror::Error;

/// Error generated during IK calculation
#[derive(Error, Debug)]
pub enum IkError {
    #[error("IK resolution produced results with NaN")]
    NanFound(&'static str),
    #[error("point is out of reach of current chain")]
    OutOfReach,
    #[error("error originating from driver")]
    DriverError(#[from] arm_driver::DriverError),
}

type Result<T> = std::result::Result<T, IkError>;

#[derive(Debug, Clone, PartialEq, Default)]
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

    pub fn get_end_effector_pose(&self) -> EndEffectorPose {
        EndEffectorPose::new(self.end_effector, self.end_effector_angle)
    }
}

#[async_trait]
pub trait ArmController: Send + Sync {
    async fn set_color(&mut self, color: lss_driver::LedColor) -> Result<()>;
    /// 0.0 is fully open
    /// 1.0 is fully closed
    async fn move_gripper(&mut self, closed: f32) -> Result<()>;
    fn calculate_ik(&self, pose: EndEffectorPose) -> Result<JointPositions>;
    fn calculate_fk(&self, joints: JointPositions) -> ArmPositions;
    fn calculate_full_poses(
        &self,
        target: EndEffectorPose,
    ) -> Result<(ArmPositions, JointPositions)>;
    async fn read_position(&mut self) -> Result<ArmPositions>;
    async fn move_to(&mut self, pose: EndEffectorPose) -> Result<JointPositions>;
    async fn move_joints_to(&mut self, joints: JointPositions) -> Result<()>;
    async fn move_joints_timed(
        &mut self,
        joints: &JointPositions,
        duration: Duration,
    ) -> Result<()>;
    async fn halt(&mut self) -> Result<()>;
    async fn limp(&mut self) -> Result<()>;
    async fn setup_motors(&mut self, settings: arm_driver::ArmControlSettings) -> Result<()>;
    async fn load_motor_settings(&mut self) -> Result<arm_driver::ArmControlSettings>;
    async fn check_motors_okay(&mut self) -> Result<bool>;
}

pub struct LssArmController {
    driver: Box<dyn arm_driver::ArmDriver>,
    kinematic_solver: KinematicSolver,
}

impl LssArmController {
    pub fn new(driver: Box<dyn arm_driver::ArmDriver>, config: arm_config::ArmConfig) -> Box<Self> {
        let kinematic_solver = KinematicSolver::new(config);
        Box::new(LssArmController {
            driver,
            kinematic_solver,
        })
    }
}

#[async_trait]
impl ArmController for LssArmController {
    async fn set_color(&mut self, color: lss_driver::LedColor) -> Result<()> {
        self.driver.set_color(color).await?;
        Ok(())
    }

    /// 0.0 is fully open
    /// 1.0 is fully closed
    async fn move_gripper(&mut self, closed: f32) -> Result<()> {
        self.driver
            .move_gripper(closed, 400, Duration::from_millis(800))
            .await?;
        Ok(())
    }

    /// calculate Ik
    /// This method expects a point translated from arm base
    fn calculate_ik(&self, pose: EndEffectorPose) -> Result<JointPositions> {
        self.kinematic_solver.calculate_ik(pose)
    }

    fn calculate_fk(&self, joints: JointPositions) -> ArmPositions {
        self.kinematic_solver.calculate_fk(joints)
    }

    fn calculate_full_poses(
        &self,
        target: EndEffectorPose,
    ) -> Result<(ArmPositions, JointPositions)> {
        // This should really be one call.
        // I don't know why I solve the poses so awkwardly
        let joints = self.kinematic_solver.calculate_ik(target)?;
        let pose = self.kinematic_solver.calculate_fk(joints.clone());
        Ok((pose, joints))
    }

    async fn read_position(&mut self) -> Result<ArmPositions> {
        let motor_positions = self.driver.read_position().await?;
        let arm_positions = self.calculate_fk(motor_positions);
        Ok(arm_positions)
    }

    async fn move_to(&mut self, pose: EndEffectorPose) -> Result<JointPositions> {
        let joint_positions = self.calculate_ik(pose)?;
        self.driver.move_to(joint_positions.clone()).await?;
        Ok(joint_positions)
    }

    async fn move_joints_to(&mut self, joints: JointPositions) -> Result<()> {
        self.driver.move_to(joints).await?;
        Ok(())
    }

    async fn move_joints_timed(
        &mut self,
        joints: &JointPositions,
        duration: Duration,
    ) -> Result<()> {
        self.driver.move_to_timed(joints, duration).await?;
        Ok(())
    }

    async fn halt(&mut self) -> Result<()> {
        self.driver.halt().await?;
        Ok(())
    }

    async fn limp(&mut self) -> Result<()> {
        self.driver.limp().await?;
        Ok(())
    }

    async fn setup_motors(&mut self, settings: arm_driver::ArmControlSettings) -> Result<()> {
        self.driver.setup_motors(settings).await?;
        Ok(())
    }

    async fn load_motor_settings(&mut self) -> Result<arm_driver::ArmControlSettings> {
        Ok(self.driver.load_motor_settings().await?)
    }

    async fn check_motors_okay(&mut self) -> Result<bool> {
        let arm_status = self.driver.query_motor_status().await?;
        Ok(arm_status.is_arm_okay())
    }
}

pub struct KinematicSolver {
    config: arm_config::ArmConfig,
}

impl KinematicSolver {
    pub fn new(config: arm_config::ArmConfig) -> Self {
        KinematicSolver { config }
    }

    pub fn calculate_ik(&self, pose: EndEffectorPose) -> Result<JointPositions> {
        let effector_angle = pose.end_effector_angle.to_radians();
        let base_angle = (-pose.position.y).atan2(pose.position.x);
        let horizontal_distance = (pose.position.x.powi(2) + pose.position.y.powi(2)).sqrt();
        let height = pose.position.z - self.config.shoulder.z;
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
        if reduced_distance > (shoulder_length + forearm_length) {
            return Err(IkError::OutOfReach);
        }
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
        let wrist_angle = -offset_elbow_angle - offset_shoulder + effector_angle;
        if !base_angle.is_finite() {
            return Err(IkError::NanFound("base_angle"));
        }
        if !offset_shoulder.is_finite() {
            return Err(IkError::NanFound("offset_shoulder"));
        }
        if !offset_elbow_angle.is_finite() {
            return Err(IkError::NanFound("offset_elbow_angle"));
        }
        if !wrist_angle.is_finite() {
            return Err(IkError::NanFound("wrist_angle"));
        }
        Ok(JointPositions::new(
            base_angle.to_degrees(),
            offset_shoulder.to_degrees(),
            offset_elbow_angle.to_degrees(),
            wrist_angle.to_degrees(),
        ))
    }

    pub fn calculate_fk(&self, joints: JointPositions) -> ArmPositions {
        let base = na::Vector3::new(0.0, 0.0, 0.0);
        let base_rotation =
            na::Rotation3::from_axis_angle(&na::Vector3::z_axis(), -joints.base.to_radians());
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
        ArmPositions::new(
            base,
            shoulder,
            elbow,
            wrist,
            end_effector,
            joints.elbow + joints.shoulder + joints.wrist,
        )
    }
}
