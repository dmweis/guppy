use guppy::arm_controller;
use guppy::visualizer;

use guppy_service::guppy_controller_client::GuppyControllerClient;

use tonic::Request;

pub mod guppy_service {
    tonic::include_proto!("guppy_service");
}

use async_std::task::sleep;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use visualizer::VisualizerInterface;

impl From<nalgebra::Vector3<f32>> for guppy_service::Vector {
    fn from(source: nalgebra::Vector3<f32>) -> Self {
        guppy_service::Vector {
            x: source.x,
            y: source.y,
            z: source.z,
        }
    }
}

impl Into<nalgebra::Vector3<f32>> for guppy_service::Vector {
    fn into(self) -> nalgebra::Vector3<f32> {
        nalgebra::Vector3::new(self.x, self.y, self.z)
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

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    move_run().await?;
    Ok(())
}

async fn move_run() -> Result<(), Box<dyn std::error::Error>> {
    let running = Arc::new(AtomicBool::new(true));
    let running_handle = running.clone();

    let mut visualizer = VisualizerInterface::default();

    ctrlc::set_handler(move || {
        running_handle.store(false, Ordering::Release);
        println!("Caught interrupt\nExiting...");
    })?;

    let mut client = GuppyControllerClient::connect("http://pi42.local:5001").await?;

    // let start = Instant::now();
    while running.load(Ordering::Acquire) {
        // let temporal = (start.elapsed().as_secs_f32() * 2.).sin();
        // let z = temporal * 0.07;
        // let y = temporal * 0.07;
        // let position = na::Vector3::new(0.2, 0.0 + y, 0.2 + z);
        let pose = visualizer.get_desired_state().pose().clone();
        if let Ok(joint_positions) = client
            .move_to(Request::new(guppy_service::MoveToCommand {
                position: Some(pose.position.into()),
                effector_angle: pose.end_effector_angle,
            }))
            .await
        {
            visualizer.set_position(joint_positions.into_inner().into());
        } else {
            eprintln!("Message error");
        }
        let gripper_closed = visualizer.get_desired_state().gripper_state();
        if gripper_closed {
            client
                .set_gripper(Request::new(guppy_service::SetGripperRequest {
                    gripper: 1.0,
                }))
                .await?;
        } else {
            client
                .set_gripper(Request::new(guppy_service::SetGripperRequest {
                    gripper: 0.0,
                }))
                .await?;
        }
        sleep(Duration::from_millis(20)).await;
    }
    // arm_controller.limp().await?;
    // arm_controller
    //     .set_color(lss_driver::LedColor::Yellow)
    //     .await?;
    sleep(Duration::from_secs_f32(0.5)).await;
    Ok(())
}
