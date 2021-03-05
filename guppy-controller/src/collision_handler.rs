use crate::arm_config::ArmConfig;
use crate::arm_controller::ArmPositions;
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

#[cfg(test)]
mod tests {
    use super::*;

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
}
