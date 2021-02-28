use nalgebra as na;

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
}
