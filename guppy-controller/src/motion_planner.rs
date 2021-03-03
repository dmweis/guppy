use crate::arm_config::ArmConfig;
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
        if self.current == self.target {
            return None;
        }
        let distance = na::distance(&self.current.into(), &self.target.into());
        if distance <= self.max_step {
            self.current = self.target;
            return Some(self.target);
        }
        let translation = self.target - self.current;
        let next = self.current + (translation.normalize() * self.max_step);
        self.current = next;
        Some(next)
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
            collision_handler.point_in_workspace(&(na::Point3::new(0.35, 0.0, 0.0) + root_point))
        );
        assert!(
            collision_handler.point_in_workspace(&(na::Point3::new(0.0, 0.35, 0.0) + root_point))
        );
        assert!(
            collision_handler.point_in_workspace(&(na::Point3::new(0.0, 0.0, 0.35) + root_point))
        );
        assert!(
            !collision_handler.point_in_workspace(&(na::Point3::new(0.37, 0.0, 0.0) + root_point))
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
}
