use crate::{
    arm_controller::{ArmController, EndEffectorPose},
    arm_driver::{ArmControlSettings, JointPositions, ServoControlSettings},
    collision_handler::CollisionHandler,
};
use anyhow::Result;
use nalgebra as na;
use std::time::Duration;
use tokio::time::sleep;

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

pub struct MotionController {
    arm_controller: Box<dyn ArmController>,
    collision_handler: CollisionHandler,
    /// move speed in m/s
    translation_speed: f32,
    /// degree per second
    // should be rad really
    rotational_speed: f32,
    last_pose: EndEffectorPose,
}

impl MotionController {
    pub async fn new(
        mut arm_controller: Box<dyn ArmController>,
        collision_handler: CollisionHandler,
        translation_speed: f32,
        rotational_speed: f32,
    ) -> Result<Self> {
        arm_controller
            .setup_motors(ArmControlSettings::included_trajectory())
            .await?;
        let last_pose = arm_controller
            .read_position()
            .await?
            .get_end_effector_pose();
        Ok(Self {
            arm_controller,
            collision_handler,
            translation_speed,
            rotational_speed,
            last_pose,
        })
    }

    pub async fn open_gripper(&mut self) -> Result<()> {
        self.arm_controller.move_gripper(0.0).await
    }

    pub async fn close_gripper(&mut self) -> Result<()> {
        self.arm_controller.move_gripper(1.0).await
    }

    pub async fn move_to(&mut self, target: EndEffectorPose) -> Result<()> {
        let duration = estimate_time(
            &self.last_pose,
            &target,
            self.translation_speed,
            self.rotational_speed,
        );
        if let Ok((positions, joints)) = self.arm_controller.calculate_full_poses(target.clone()) {
            if self.collision_handler.pose_collision_free(&positions) {
                self.arm_controller
                    .move_joints_timed(joints, duration)
                    .await?;
                self.last_pose = target;
            } else {
                self.arm_controller
                    .set_color(lss_driver::LedColor::Yellow)
                    .await?;
            }
        } else {
            self.arm_controller
                .set_color(lss_driver::LedColor::Red)
                .await?;
        }
        Ok(())
    }

    pub async fn move_to_blocking(&mut self, target: EndEffectorPose) -> Result<()> {
        let duration = estimate_time(
            &self.last_pose,
            &target,
            self.translation_speed,
            self.rotational_speed,
        );
        if let Ok((positions, joints)) = self.arm_controller.calculate_full_poses(target.clone()) {
            if self.collision_handler.pose_collision_free(&positions) {
                self.arm_controller
                    .set_color(lss_driver::LedColor::Cyan)
                    .await?;
                self.arm_controller
                    .move_joints_timed(joints, duration)
                    .await?;
                self.last_pose = target;
                sleep(duration).await;
                self.arm_controller
                    .set_color(lss_driver::LedColor::Magenta)
                    .await?;
            } else {
                self.arm_controller
                    .set_color(lss_driver::LedColor::Yellow)
                    .await?;
            }
        } else {
            self.arm_controller
                .set_color(lss_driver::LedColor::Red)
                .await?;
        }
        Ok(())
    }

    pub async fn apply_settings(&mut self) -> Result<()> {
        self.arm_controller
            .setup_motors(ArmControlSettings::included_trajectory())
            .await?;
        Ok(())
    }

    pub async fn home(&mut self) -> Result<()> {
        let lifted_home = JointPositions::new(0.0, -80.0, 82.0, 15.0);
        self.arm_controller
            .move_joints_timed(lifted_home, Duration::from_millis(900))
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
            .setup_motors(relaxed_arm_settings)
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
