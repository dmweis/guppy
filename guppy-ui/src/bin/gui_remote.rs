use anyhow::Result;
use guppy_grpc::grpc_client::{guppy_service, GuppyControllerClient, Request};
use guppy_ui::visualizer::VisualizerInterface;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use tokio::time::sleep;

#[tokio::main]
async fn main() -> Result<()> {
    move_run().await?;
    Ok(())
}

async fn move_run() -> Result<()> {
    let running = Arc::new(AtomicBool::new(true));
    let running_handle = running.clone();

    let mut visualizer = VisualizerInterface::sensible_default();

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
