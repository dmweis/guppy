use guppy_controller::arm_controller;
pub use guppy_service::guppy_controller_client::GuppyControllerClient;
pub use tonic::Request;

pub mod guppy_service {
    tonic::include_proto!("guppy_service");
}

impl From<nalgebra::Vector3<f32>> for guppy_service::Vector {
    fn from(source: nalgebra::Vector3<f32>) -> Self {
        guppy_service::Vector {
            x: source.x,
            y: source.y,
            z: source.z,
        }
    }
}

impl From<guppy_service::Vector> for nalgebra::Vector3<f32> {
    fn from(vector: guppy_service::Vector) -> Self {
        nalgebra::Vector3::new(vector.x, vector.y, vector.z)
    }
}

impl From<guppy_service::ArmPositions> for arm_controller::ArmPositions {
    fn from(source: guppy_service::ArmPositions) -> Self {
        arm_controller::ArmPositions {
            base: source.base.unwrap().into(),
            shoulder: source.shoulder.unwrap().into(),
            elbow: source.elbow.unwrap().into(),
            wrist: source.wrist.unwrap().into(),
            end_effector: source.end_effector.unwrap().into(),
            end_effector_angle: source.end_effector_angle,
        }
    }
}
