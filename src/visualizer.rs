use crate::arm_controller::ArmPositions;
use kiss3d::window::Window;
use nalgebra as na;
use nalgebra::{Isometry3, Point2, Point3, Translation3, UnitQuaternion, Vector3};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex,
};
use std::time::Instant;

fn add_ground_plane(window: &mut Window) {
    let size = 0.5;
    for i in 0..4 {
        for j in 0..4 {
            let mut cube = window.add_cube(size, size, 0.001);
            if (i + j) % 2 == 0 {
                cube.set_color(1.0, 0.3, 0.2);
            } else {
                cube.set_color(0.5, 0.04, 0.17);
            }
            let distance = (1_f32.powi(2) + 1_f32.powi(2)).sqrt();
            let x_ind = j as f32 - distance;
            let y_ind = i as f32 - distance;
            let trans = Isometry3::from_parts(
                Translation3::new(size * x_ind, 0.0, size * y_ind),
                UnitQuaternion::from_euler_angles(0.0, -1.57, -1.57),
            );
            cube.set_local_transformation(trans);
        }
    }
}

pub struct VisualizerInterface {
    current_arm_pose: Arc<Mutex<Option<ArmPositions>>>,
    keep_running: Arc<AtomicBool>,
    thread_handle: Option<std::thread::JoinHandle<()>>,
}

impl VisualizerInterface {
    pub fn new() -> VisualizerInterface {
        let current_arm_pose = Arc::new(Mutex::new(None));
        let keep_running = Arc::new(AtomicBool::new(true));

        let keep_running_clone = keep_running.clone();
        let current_arm_pose_clone = current_arm_pose.clone();
        let thread_handle = std::thread::spawn(move || {
            render_loop(current_arm_pose_clone, keep_running_clone);
        });
        VisualizerInterface {
            current_arm_pose,
            keep_running,
            thread_handle: Some(thread_handle),
        }
    }

    pub fn set_position(&mut self, arm_position: ArmPositions) {
        self.current_arm_pose.lock().unwrap().replace(arm_position);
    }
}

impl Drop for VisualizerInterface {
    fn drop(&mut self) {
        self.keep_running.store(false, Ordering::Release);
        if let Some(thread_handle) = self.thread_handle.take() {
            thread_handle.join().unwrap();
        }
    }
}

fn convert_coordinates(position: Vector3<f32>) -> Point3<f32> {
    Point3::new(position.y, position.z, -position.x)
}

fn render_loop(current_arm_pose: Arc<Mutex<Option<ArmPositions>>>, keep_running: Arc<AtomicBool>) {
    let blue = Point3::new(0.0, 0.0, 1.0);
    let white = Point3::new(1.0, 1.0, 1.0);
    let mut window = Window::new("Guppy");
    let mut frame_counter = Instant::now();

    let mut camera =
        kiss3d::camera::ArcBall::new(Point3::new(1.0, 1.0, 1.0), Point3::new(0.0, 0.0, 0.0));
    camera.set_zoom_modifier(10.0);

    window.set_background_color(0.5, 0.5, 0.5);
    window.set_framerate_limit(Some(60));

    let mut base_sphere = window.add_sphere(0.01);
    base_sphere.set_color(1.0, 0.0, 1.0);
    let mut shoulder_sphere = window.add_sphere(0.01);
    shoulder_sphere.set_color(1.0, 0.0, 1.0);
    let mut elbow_sphere = window.add_sphere(0.01);
    elbow_sphere.set_color(1.0, 0.0, 1.0);
    let mut wrist_sphere = window.add_sphere(0.01);
    wrist_sphere.set_color(1.0, 0.0, 1.0);
    let mut end_effector_sphere = window.add_sphere(0.01);
    end_effector_sphere.set_color(0.0, 1.0, 1.0);

    add_ground_plane(&mut window);

    while window.render_with_camera(&mut camera) && keep_running.load(Ordering::Acquire) {
        let arm_pose = current_arm_pose.lock().unwrap().clone();
        if let Some(arm_pose) = arm_pose {
            let base = convert_coordinates(arm_pose.base);
            base_sphere.set_local_translation(na::Translation3::new(base.x, base.y, base.z));

            let shoulder = convert_coordinates(arm_pose.shoulder);
            window.draw_line(&base, &shoulder, &blue);
            shoulder_sphere
                .set_local_translation(na::Translation3::new(shoulder.x, shoulder.y, shoulder.z));

            let elbow = convert_coordinates(arm_pose.elbow);
            window.draw_line(&shoulder, &elbow, &blue);
            elbow_sphere.set_local_translation(na::Translation3::new(elbow.x, elbow.y, elbow.z));

            let wrist = convert_coordinates(arm_pose.wrist);
            window.draw_line(&elbow, &wrist, &blue);
            wrist_sphere.set_local_translation(na::Translation3::new(wrist.x, wrist.y, wrist.z));

            let end_effector = convert_coordinates(arm_pose.end_effector);
            end_effector_sphere.set_local_translation(na::Translation3::new(
                end_effector.x,
                end_effector.y,
                end_effector.z,
            ));
            window.draw_line(&wrist, &end_effector, &blue);
            window.draw_text(
                &format!(
                    "End effector:\n   x: {}\n   y: {}\n   z: {}\nCamera dist: {}\nframe time: {}ms",
                    arm_pose.end_effector.x,
                    arm_pose.end_effector.y,
                    arm_pose.end_effector.z,
                    camera.dist(),
                    frame_counter.elapsed().as_millis(),
                ),
                &Point2::new(1.0, 1.0),
                50.0,
                &kiss3d::text::Font::default(),
                &white,
            );
            frame_counter = Instant::now();
        }
    }
    window.close()
}
