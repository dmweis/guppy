mod arm_config;
mod arm_controller;
mod arm_driver;
mod speech_service;
#[cfg(feature = "visualiser")]
mod visualizer;

use guppy_service::guppy_controller_client::GuppyControllerClient;

use tonic::Request;

pub mod guppy_service {
    tonic::include_proto!("guppy_service");
}

use async_std::io;
use async_std::task::sleep;
use std::time::Duration;
#[cfg(feature = "visualiser")]
use visualizer::VisualizerInterface;

use crate::arm_controller::EndEffectorPose;
use clap::Clap;
use nalgebra as na;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;

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
    let desired_point = Arc::new(Mutex::new(EndEffectorPose::new(
        na::Vector3::new(0.2, 0., 0.2),
        0.,
    )));
    #[cfg(feature = "visualiser")]
    let mut visualizer = VisualizerInterface::new(desired_point.clone());

    ctrlc::set_handler(move || {
        running_handle.store(false, Ordering::Release);
        println!("Caught interrupt\nExiting...");
    })?;

    let mut client = GuppyControllerClient::connect("http://127.0.0.1:5002").await?;

    // let start = Instant::now();
    while running.load(Ordering::Acquire) {
        // let temporal = (start.elapsed().as_secs_f32() * 2.).sin();
        // let z = temporal * 0.07;
        // let y = temporal * 0.07;
        // let position = na::Vector3::new(0.2, 0.0 + y, 0.2 + z);
        let pose = desired_point.lock().unwrap().clone();
        if let Ok(joint_positions) = client
            .move_to(Request::new(guppy_service::MoveToCommand {
                position: Some(pose.position.into()),
                effector_angle: pose.end_effector_angle,
            }))
            .await
        {
            #[cfg(feature = "visualiser")]
            {
                visualizer.set_position(joint_positions.into_inner().into());
            }
        } else {
            eprintln!("Message error");
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
