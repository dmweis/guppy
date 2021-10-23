use crate::{
    arm_controller::{self, ArmController, ArmPositions, EndEffectorPose},
    arm_driver::{ArmControlSettings, JointPositions, ServoControlSettings},
    collision_handler::CollisionHandler,
};
use async_trait::async_trait;
use nalgebra as na;
use std::time::Duration;
use thiserror::Error;
use tokio::time::sleep;
use tracing::*;

#[derive(Error, Debug)]
pub enum PlannerError {
    #[error("error from arm controller")]
    ControllerError(#[from] arm_controller::IkError),
    #[error("motion would produce collision")]
    CollisionError,
}

type Result<T> = std::result::Result<T, PlannerError>;

#[async_trait]
pub trait MotionController: Send + Sync {
    fn calculate_full_poses(
        &self,
        target: &EndEffectorPose,
    ) -> Result<(ArmPositions, JointPositions)>;
    async fn read_position(&mut self) -> Result<ArmPositions>;
    async fn move_to_jogging(&mut self, pose: &EndEffectorPose) -> Result<ArmPositions>;
    async fn move_to_trajectory(
        &mut self,
        pose: &EndEffectorPose,
    ) -> Result<(ArmPositions, Duration)>;
    async fn open_gripper(&mut self, current_limit: bool) -> Result<()>;
    async fn close_gripper(&mut self, current_limit: bool) -> Result<()>;
    async fn halt(&mut self) -> Result<()>;
    async fn limp(&mut self) -> Result<()>;
    async fn check_motors_okay(&mut self) -> Result<bool>;
    async fn restart_motors(&mut self) -> Result<()>;
}

pub struct LssMotionController {
    arm_controller: Box<dyn ArmController>,
    collision_handler: CollisionHandler,
    /// move speed in m/s
    translation_speed: f32,
    /// degree per second
    // should be rad really
    rotational_speed: f32,
    last_pose: EndEffectorPose,
}

impl LssMotionController {
    pub async fn new(
        mut arm_controller: Box<dyn ArmController>,
        collision_handler: CollisionHandler,
        translation_speed: f32,
        rotational_speed: f32,
    ) -> Result<Self> {
        let last_pose = arm_controller
            .read_position()
            .await?
            .get_end_effector_pose();
        let mut motion_planner = Self {
            arm_controller,
            collision_handler,
            translation_speed,
            rotational_speed,
            last_pose,
        };
        motion_planner.apply_trajectory_settings().await?;
        Ok(motion_planner)
    }

    pub async fn apply_trajectory_settings(&mut self) -> Result<()> {
        info!("Applying trajectory settings");
        self.arm_controller
            .setup_motors(&ArmControlSettings::included_trajectory())
            .await?;
        Ok(())
    }

    pub async fn apply_continuous_settings(&mut self) -> Result<()> {
        info!("Applying continuous settings");
        self.arm_controller
            .setup_motors(&ArmControlSettings::included_continuous())
            .await?;
        Ok(())
    }

    pub async fn read_settings(&mut self) -> Result<ArmControlSettings> {
        Ok(self.arm_controller.load_motor_settings().await?)
    }

    pub async fn home(&mut self) -> Result<()> {
        self.apply_trajectory_settings().await?;
        let lifted_home = JointPositions::new(0.0, -80.0, 90.0, 15.0);
        self.arm_controller
            .move_joints_timed(&lifted_home, Duration::from_millis(900))
            .await?;
        sleep(Duration::from_millis(900)).await;
        let relaxed_arm_settings = ArmControlSettings {
            shoulder: Some(ServoControlSettings {
                angular_holding_stiffness: Some(-40),
                angular_stiffness: Some(-40),
                ..Default::default()
            }),
            elbow: Some(ServoControlSettings {
                angular_holding_stiffness: Some(-40),
                angular_stiffness: Some(-40),
                ..Default::default()
            }),
            wrist: Some(ServoControlSettings {
                angular_holding_stiffness: Some(-40),
                angular_stiffness: Some(-40),
                ..Default::default()
            }),
            ..Default::default()
        };
        self.arm_controller
            .setup_motors(&relaxed_arm_settings)
            .await?;
        self.arm_controller
            .set_color(lss_driver::LedColor::Red)
            .await?;
        sleep(Duration::from_secs(4)).await;
        self.arm_controller
            .set_color(lss_driver::LedColor::Off)
            .await?;
        self.arm_controller.limp().await?;
        Ok(())
    }
}

#[async_trait]
impl MotionController for LssMotionController {
    fn calculate_full_poses(
        &self,
        target: &EndEffectorPose,
    ) -> Result<(ArmPositions, JointPositions)> {
        Ok(self.arm_controller.calculate_full_poses(target)?)
    }

    async fn read_position(&mut self) -> Result<ArmPositions> {
        Ok(self.arm_controller.read_position().await?)
    }

    async fn move_to_jogging(&mut self, target: &EndEffectorPose) -> Result<ArmPositions> {
        match self.arm_controller.calculate_full_poses(target) {
            Ok((positions, joints)) => {
                if self.collision_handler.pose_collision_free(&positions) {
                    self.arm_controller.move_joints_to(&joints).await?;
                    self.last_pose = target.clone();
                    Ok(positions)
                } else {
                    self.arm_controller
                        .set_color(lss_driver::LedColor::Yellow)
                        .await?;
                    Err(PlannerError::CollisionError)
                }
            }
            Err(error) => {
                self.arm_controller
                    .set_color(lss_driver::LedColor::Red)
                    .await?;
                Err(PlannerError::ControllerError(error))
            }
        }
    }

    async fn move_to_trajectory(
        &mut self,
        target: &EndEffectorPose,
    ) -> Result<(ArmPositions, Duration)> {
        let duration = estimate_time(
            &self.last_pose,
            target,
            self.translation_speed,
            self.rotational_speed,
        );

        match self.arm_controller.calculate_full_poses(target) {
            Ok((positions, joints)) => {
                if self.collision_handler.pose_collision_free(&positions) {
                    self.arm_controller
                        .move_joints_timed(&joints, duration)
                        .await?;
                    self.last_pose = target.clone();
                    Ok((positions, duration))
                } else {
                    self.arm_controller
                        .set_color(lss_driver::LedColor::Yellow)
                        .await?;
                    Err(PlannerError::CollisionError)
                }
            }
            Err(error) => {
                self.arm_controller
                    .set_color(lss_driver::LedColor::Red)
                    .await?;
                Err(PlannerError::ControllerError(error))
            }
        }
    }

    async fn open_gripper(&mut self, current_limit: bool) -> Result<()> {
        let current_limit = if current_limit { Some(400) } else { None };
        self.arm_controller.move_gripper(0.0, current_limit).await?;
        Ok(())
    }

    async fn close_gripper(&mut self, current_limit: bool) -> Result<()> {
        let current_limit = if current_limit { Some(400) } else { None };
        self.arm_controller.move_gripper(1.0, current_limit).await?;
        Ok(())
    }

    async fn halt(&mut self) -> Result<()> {
        Ok(self.arm_controller.halt().await?)
    }

    async fn limp(&mut self) -> Result<()> {
        Ok(self.arm_controller.limp().await?)
    }

    async fn check_motors_okay(&mut self) -> Result<bool> {
        Ok(self.arm_controller.check_motors_okay().await?)
    }

    async fn restart_motors(&mut self) -> Result<()> {
        Ok(self.arm_controller.restart_motors().await?)
    }
}

/// Selects time it would take to take this trajectory at given speed
/// Picks the longer time for motion wether it is the rotation or translation
fn estimate_time(
    start: &EndEffectorPose,
    target: &EndEffectorPose,
    translation_speed: f32,
    rotational_speed: f32,
) -> Duration {
    let linear_distance = na::distance(&start.position.into(), &target.position.into());
    let angular_distance = (start.end_effector_angle - target.end_effector_angle).abs();

    let translation_time_sec = linear_distance / translation_speed;
    let angular_time_sec = angular_distance / rotational_speed;

    let bigger_time = translation_time_sec.max(angular_time_sec);

    Duration::from_secs_f32(bigger_time)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn pick_longer_time_translation() {
        let start = EndEffectorPose::new(na::Vector3::new(0.0, 0.0, 0.0), 0.0);
        let target = EndEffectorPose::new(na::Vector3::new(10.0, 0.0, 0.0), 2.0);
        let duration = estimate_time(&start, &target, 1.0, 1.0);
        assert_relative_eq!(duration.as_secs_f32(), 10.0);
    }

    #[test]
    fn pick_longer_time_rotation() {
        let start = EndEffectorPose::new(na::Vector3::new(0.0, 0.0, 0.0), 0.0);
        let target = EndEffectorPose::new(na::Vector3::new(10.0, 0.0, 0.0), 20.0);
        let duration = estimate_time(&start, &target, 1.0, 1.0);
        assert_relative_eq!(duration.as_secs_f32(), 20.0);
    }
}
