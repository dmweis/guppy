#![doc = include_str!("../../doc_include.md")]

pub mod visualizer;
pub use guppy_controller::arm_config;
pub use guppy_controller::arm_controller;
pub use guppy_controller::arm_driver;

use nalgebra::Vector3;
use nalgebra_new as na_new;

pub fn new_vec_to_old(new_type: na_new::Vector3<f32>) -> Vector3<f32> {
    Vector3::new(new_type.x, new_type.y, new_type.z)
}
