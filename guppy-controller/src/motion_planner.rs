use crate::arm_controller::{ArmController, EndEffectorPose};
use crate::{arm_config::ArmConfig, arm_controller};
use anyhow::Result;
use nalgebra as na;
use parry3d::{query::PointQuery, shape};
use std::f32;

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

    pub fn colliding_with_self(&self, point: &na::Point3<f32>) -> bool {
        let axisangle = na::Vector3::x() * f32::consts::FRAC_PI_2;

        self.base_collider
            .contains_point(&na::Isometry3::new(na::zero(), axisangle), point)
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
        let distance = (self - target).abs();
        if distance <= max_step {
            return Some(*target);
        }
        let next = self + max_step;
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
        assert!(collision_handler.colliding_with_self(&(na::Point3::new(0.0, 0.0, 0.0))));
        assert!(
            collision_handler.colliding_with_self(&(na::Point3::new(0.0, 0.0, 0.1))),
            "z up point is in"
        );
        assert!(
            !collision_handler.colliding_with_self(&(na::Point3::new(0.0, 0.0, 0.19))),
            "z up 19 point is not in"
        );
        assert!(
            !collision_handler.colliding_with_self(&(na::Point3::new(0.1, 0.0, 0.0))),
            "x away point is not in"
        );
        assert!(
            !collision_handler.colliding_with_self(&(na::Point3::new(0.0, 0.1, 0.0))),
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
