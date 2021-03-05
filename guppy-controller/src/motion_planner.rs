use crate::arm_controller::{ArmController, ArmPositions, EndEffectorPose};
use crate::{arm_config::ArmConfig, arm_controller};
use anyhow::{Context, Result};
use nalgebra as na;
use parry3d::{query::PointQuery, shape};
use std::f32;
use std::sync::mpsc;
use std::time;
use tokio::task;

pub struct CollisionHandler {
    workspace_sphere: shape::Ball,
    base_collider: shape::Cylinder,
    config: ArmConfig,
}

impl CollisionHandler {
    pub fn new(config: ArmConfig) -> Self {
        Self {
            workspace_sphere: shape::Ball::new(0.28),
            base_collider: shape::Cylinder::new(0.18, 0.09),
            config,
        }
    }

    pub fn point_in_workspace(&self, point: &na::Point3<f32>) -> bool {
        self.workspace_sphere.contains_point(
            &na::Isometry3::translation(
                self.config.shoulder.x,
                self.config.shoulder.y,
                self.config.shoulder.z,
            ),
            point,
        )
    }

    pub fn check_self_collision(&self, point: &na::Point3<f32>) -> bool {
        let axisangle = na::Vector3::x() * f32::consts::FRAC_PI_2;

        !self
            .base_collider
            .contains_point(&na::Isometry3::new(na::zero(), axisangle), point)
    }

    pub fn pose_collision_free(&self, arm_pose: &ArmPositions) -> bool {
        self.point_in_workspace(&arm_pose.wrist.into())
            && self.check_self_collision(&arm_pose.wrist.into())
            && self.check_self_collision(&arm_pose.end_effector.into())
    }
}

pub struct LinearMotion {
    target: na::Vector3<f32>,
    current: na::Vector3<f32>,
    max_step: f32,
}

impl LinearMotion {
    pub fn new(start: na::Vector3<f32>, target: na::Vector3<f32>, max_step: f32) -> Self {
        LinearMotion {
            target,
            current: start,
            max_step,
        }
    }
}

impl Iterator for LinearMotion {
    type Item = na::Vector3<f32>;

    fn next(&mut self) -> Option<Self::Item> {
        match self.current.move_towards(&self.target, self.max_step) {
            Some(next) => {
                self.current = next;
                Some(next)
            }
            None => None,
        }
    }
}

trait MoveTowards: Sized {
    fn move_towards(&self, target: &Self, max_step: f32) -> Option<Self>;
}

impl MoveTowards for na::Vector3<f32> {
    fn move_towards(&self, target: &Self, max_step: f32) -> Option<Self> {
        if self == target {
            return None;
        }
        let distance = na::distance(&na::Point3::from(*self), &na::Point3::from(*target));
        if distance <= max_step {
            return Some(*target);
        }
        let translation = target - self;
        let next = self + (translation.normalize() * max_step);
        Some(next)
    }
}

impl MoveTowards for f32 {
    fn move_towards(&self, target: &Self, max_step: f32) -> Option<Self> {
        // we actually want to compare identity
        #[allow(clippy::float_cmp)]
        if self == target {
            return None;
        }
        let distance = (target - self).abs();
        if distance <= max_step {
            return Some(*target);
        }
        let next = self + max_step.copysign(target - self);
        Some(next)
    }
}

trait MovePoseTowards: Sized {
    fn move_towards(
        &self,
        target: &Self,
        max_translation: f32,
        max_angle_move: f32,
    ) -> Option<Self>;
}

impl MovePoseTowards for EndEffectorPose {
    fn move_towards(
        &self,
        target: &Self,
        max_translation: f32,
        max_angle_move: f32,
    ) -> Option<Self> {
        let pose = (
            self.position
                .move_towards(&target.position, max_translation),
            self.end_effector_angle
                .move_towards(&target.end_effector_angle, max_angle_move),
        );

        match pose {
            (None, None) => None,
            (Some(position), None) => {
                Some(EndEffectorPose::new(position, target.end_effector_angle))
            }
            (None, Some(angle)) => Some(EndEffectorPose::new(target.position, angle)),
            (Some(position), Some(angle)) => Some(EndEffectorPose::new(position, angle)),
        }
    }
}

enum MotionCommand {
    MoveTo(EndEffectorPose),
}

pub struct MotionController {
    sender: mpsc::Sender<MotionCommand>,
    _join_handle: task::JoinHandle<()>,
}

impl MotionController {
    pub async fn new(
        arm_controller: Box<dyn ArmController>,
        collision_handler: CollisionHandler,
        translation_speed: f32,
        rotational_speed: f32,
    ) -> Result<Self> {
        let (sender, receiver) = mpsc::channel();
        let internal = MotionControllerInternal::new(
            arm_controller,
            collision_handler,
            translation_speed,
            rotational_speed,
            receiver,
        )
        .await?;
        let join_handle = internal.start();
        Ok(Self {
            sender,
            _join_handle: join_handle,
        })
    }

    pub fn set_target(&mut self, target: EndEffectorPose) {
        self.sender
            .send(MotionCommand::MoveTo(target))
            .context("Failed to send motion command")
            .unwrap();
    }
}

struct MotionControllerInternal {
    arm_controller: Box<dyn ArmController>,
    collision_handler: CollisionHandler,
    /// move speed in m/s
    translation_speed: f32,
    /// angel per second
    // should be rad really
    rotational_speed: f32,
    last_command_ts: time::Instant,
    last_pose: arm_controller::EndEffectorPose,
    desired_position: Option<arm_controller::EndEffectorPose>,
    receiver: mpsc::Receiver<MotionCommand>,
}

impl MotionControllerInternal {
    async fn new(
        mut arm_controller: Box<dyn ArmController>,
        collision_handler: CollisionHandler,
        translation_speed: f32,
        rotational_speed: f32,
        receiver: mpsc::Receiver<MotionCommand>,
    ) -> Result<Self> {
        let last_pose = arm_controller
            .read_position()
            .await?
            .get_end_effector_pose();
        Ok(Self {
            arm_controller,
            collision_handler,
            translation_speed,
            rotational_speed,
            last_command_ts: time::Instant::now(),
            last_pose,
            desired_position: None,
            receiver,
        })
    }

    fn start(mut self) -> task::JoinHandle<()> {
        tokio::spawn(async move {
            self.last_command_ts = time::Instant::now();
            loop {
                tokio::time::sleep(time::Duration::from_millis(20)).await;
                if self.check_messaged().is_err() {
                    // return error if sender is closed
                    // we want to end if that's the case
                    return;
                }
                self.tick().await.unwrap();
                self.last_command_ts = time::Instant::now();
            }
        })
    }

    fn check_messaged(&mut self) -> Result<()> {
        loop {
            match self.receiver.try_recv() {
                Ok(message) => match message {
                    MotionCommand::MoveTo(pose) => self.desired_position = Some(pose),
                },
                Err(mpsc::TryRecvError::Empty) => break,
                Err(mpsc::TryRecvError::Disconnected) => {
                    return Err(anyhow::anyhow!("Sender closed"))
                }
            }
        }
        Ok(())
    }

    async fn tick(&mut self) -> Result<()> {
        let elapsed = self.last_command_ts.elapsed();
        if let Some(desired_pose) = &self.desired_position {
            let next = self.last_pose.move_towards(
                &desired_pose,
                (self.translation_speed * elapsed.as_secs_f32()).abs(),
                (self.rotational_speed * elapsed.as_secs_f32()).abs(),
            );
            if let Some(next) = next {
                let (positions, joints) = self.arm_controller.calculate_full_poses(next.clone())?;
                if self.collision_handler.pose_collision_free(&positions) {
                    self.last_pose = next;
                    self.arm_controller
                        .set_color(lss_driver::LedColor::Cyan)
                        .await?;
                    self.arm_controller.move_joints_to(joints).await?;
                } else {
                    self.arm_controller
                        .set_color(lss_driver::LedColor::Red)
                        .await?;
                    self.desired_position = None;
                }
            }
        } else {
            self.arm_controller
                .set_color(lss_driver::LedColor::Blue)
                .await?;
            self.desired_position = None;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn linear_motion_end_is_identical_to_target() {
        let end = na::Vector3::new(1.0, 0.0, 0.0);
        let motion = LinearMotion::new(na::Vector3::new(0.0, 0.0, 0.0), end, 0.1);
        assert_relative_eq!(end, motion.last().unwrap());
    }

    #[test]
    fn linear_motion_is_long_axis() {
        let start = na::Vector3::new(0.0, 0.0, 0.0);
        let target = na::Vector3::new(1.0, 0.0, 0.0);
        let motion = LinearMotion::new(start, target, 0.1);
        for point in motion {
            assert_relative_eq!(point.yz(), na::Vector2::new(0.0, 0.0));
        }
    }

    #[test]
    fn test_workspace_collider() {
        let config = ArmConfig::included();
        let root_point = config.shoulder;
        let collision_handler = CollisionHandler::new(config);
        assert!(collision_handler.point_in_workspace(&na::Point3::new(0.0, 0.0, 0.0)));
        assert!(
            collision_handler.point_in_workspace(&(na::Point3::new(0.27, 0.0, 0.0) + root_point))
        );
        assert!(
            collision_handler.point_in_workspace(&(na::Point3::new(0.0, 0.27, 0.0) + root_point))
        );
        assert!(
            collision_handler.point_in_workspace(&(na::Point3::new(0.0, 0.0, 0.27) + root_point))
        );
        assert!(
            !collision_handler.point_in_workspace(&(na::Point3::new(0.29, 0.0, 0.0) + root_point))
        );
    }

    #[test]
    fn test_body_collider() {
        let config = ArmConfig::included();
        let collision_handler = CollisionHandler::new(config);
        assert!(!collision_handler.check_self_collision(&(na::Point3::new(0.0, 0.0, 0.0))));
        assert!(
            !collision_handler.check_self_collision(&(na::Point3::new(0.0, 0.0, 0.1))),
            "z up point is in"
        );
        assert!(
            collision_handler.check_self_collision(&(na::Point3::new(0.0, 0.0, 0.19))),
            "z up 19 point is not in"
        );
        assert!(
            collision_handler.check_self_collision(&(na::Point3::new(0.1, 0.0, 0.0))),
            "x away point is not in"
        );
        assert!(
            collision_handler.check_self_collision(&(na::Point3::new(0.0, 0.1, 0.0))),
            "y away point is not in"
        );
    }

    #[test]
    fn test_move_towards_float() {
        let start = 1.0;
        let mut current = start;
        let target = 2.0;
        let mut counter = 0;
        while let Some(next) = current.move_towards(&target, 0.1) {
            assert!(next > start);
            assert!(next <= target);
            current = next;
            counter += 1;
        }
        assert_eq!(counter, 10);
        assert_relative_eq!(target, current);
    }

    #[test]
    fn test_move_towards_float_target_negative() {
        let start = 1.0;
        let mut current = start;
        let target = -2.0;
        while let Some(next) = current.move_towards(&target, 0.1) {
            current = next;
        }
        assert_relative_eq!(target, current);
    }

    #[test]
    fn test_move_towards_float_neg_to_neg() {
        let start = -1.0;
        let mut current = start;
        let target = -2.0;
        while let Some(next) = current.move_towards(&target, 0.1) {
            current = next;
        }
        assert_relative_eq!(target, current);
    }

    #[test]
    fn move_end_effector() {
        let start = EndEffectorPose::new(na::Vector3::new(0.0, 0.0, 0.0), 0.0);
        let target = EndEffectorPose::new(na::Vector3::new(1.0, 0.0, 0.0), 10.0);
        let mut current = start;
        let mut counter = 0;
        while let Some(pose) = current.move_towards(&target, 0.1, 1.0) {
            current = pose;
            counter += 1;
        }
        assert_eq!(counter, 10);
        assert_relative_eq!(current.position, target.position);
        assert_relative_eq!(current.end_effector_angle, target.end_effector_angle);
    }
}
