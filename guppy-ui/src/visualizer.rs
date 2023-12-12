use guppy_controller::arm_controller::{ArmPositions, EndEffectorPose};
use kiss3d::{
    event::{Action, Key},
    scene::SceneNode,
    window::Window,
};
use nalgebra::{Isometry3, Point2, Point3, Translation3, UnitQuaternion, Vector3};

use nalgebra_new as na_new;

use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    time::Instant,
};

use crate::new_vec_to_old;

fn add_ground_plane(window: &mut Window) {
    let size = 0.5;
    for i in 0..4 {
        for j in 0..4 {
            let mut cube = window.add_cube(size, size, 0.001);
            if (i + j) % 2 == 0 {
                // cube.set_color(1.0, 0.3, 0.2);
                cube.set_color(0.0, 0.0, 0.0);
            } else {
                // cube.set_color(0.5, 0.04, 0.17);
                cube.set_color(1.0, 1.0, 1.0);
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

#[derive(Default, Clone)]
pub struct DesiredState {
    pose: EndEffectorPose,
    gripper_state: bool,
}

impl DesiredState {
    pub fn new(pose: EndEffectorPose, gripper_state: bool) -> Self {
        DesiredState {
            pose,
            gripper_state,
        }
    }
    pub fn pose(&self) -> &EndEffectorPose {
        &self.pose
    }

    pub fn gripper_state(&self) -> bool {
        self.gripper_state
    }
}

pub struct VisualizerInterface {
    current_arm_pose: Arc<Mutex<Option<ArmPositions>>>,
    motion_plan: Arc<Mutex<Option<Vec<ArmPositions>>>>,
    desired_state: Arc<Mutex<DesiredState>>,
    keep_running: Arc<AtomicBool>,
    thread_handle: Option<std::thread::JoinHandle<()>>,
}

impl VisualizerInterface {
    pub fn new(desired_state: DesiredState) -> Self {
        let current_arm_pose = Arc::new(Mutex::new(None));
        let motion_plan = Arc::new(Mutex::new(None));
        let desired_state = Arc::new(Mutex::new(desired_state));
        let keep_running = Arc::new(AtomicBool::new(true));

        let keep_running_clone = keep_running.clone();
        let current_arm_pose_clone = current_arm_pose.clone();
        let desired_state_clone = desired_state.clone();
        let motion_plan_clone = motion_plan.clone();
        let thread_handle = std::thread::spawn(move || {
            render_loop(
                current_arm_pose_clone,
                motion_plan_clone,
                keep_running_clone,
                desired_state_clone,
            );
        });
        VisualizerInterface {
            current_arm_pose,
            motion_plan,
            keep_running,
            desired_state,
            thread_handle: Some(thread_handle),
        }
    }

    pub fn sensible_default() -> Self {
        let desired_state = DesiredState::new(
            EndEffectorPose::new(na_new::Vector3::new(0.2, 0., 0.2), 0.0),
            false,
        );
        Self::new(desired_state)
    }

    pub fn set_position(&mut self, arm_position: ArmPositions) {
        self.current_arm_pose
            .lock()
            .expect("Failed lock in set arm position")
            .replace(arm_position);
    }

    pub fn get_desired_state(&self) -> DesiredState {
        self.desired_state
            .lock()
            .expect("Failed lock desired_state")
            .clone()
    }

    pub fn set_motion_plan(&mut self, motion_plan: Option<Vec<ArmPositions>>) {
        match motion_plan {
            Some(plan) => {
                self.motion_plan
                    .lock()
                    .expect("Failed lock in set_motion_plan")
                    .replace(plan);
            }
            None => {
                self.motion_plan
                    .lock()
                    .expect("Failed lock in set_motion_plan")
                    .take();
            }
        }
    }
}

impl Drop for VisualizerInterface {
    fn drop(&mut self) {
        self.keep_running.store(false, Ordering::Release);
        if let Some(thread_handle) = self.thread_handle.take() {
            thread_handle
                .join()
                .expect("Failed drop for VisualizerInterface");
        }
    }
}

fn convert_coordinates(position: Vector3<f32>) -> Point3<f32> {
    Point3::new(position.y, position.z, position.x)
}

struct ArmRenderer {
    base_sphere: SceneNode,
    shoulder_sphere: SceneNode,
    elbow_sphere: SceneNode,
    wrist_sphere: SceneNode,
    end_effector_sphere: SceneNode,
    color: Point3<f32>,
}

impl ArmRenderer {
    fn new(
        window: &mut Window,
        color: Point3<f32>,
        end_effector_color: Point3<f32>,
    ) -> ArmRenderer {
        let mut base_sphere = window.add_sphere(0.01);
        base_sphere.set_color(color.x, color.y, color.z);
        let mut shoulder_sphere = window.add_sphere(0.01);
        shoulder_sphere.set_color(color.x, color.y, color.z);
        let mut elbow_sphere = window.add_sphere(0.01);
        elbow_sphere.set_color(color.x, color.y, color.z);
        let mut wrist_sphere = window.add_sphere(0.01);
        wrist_sphere.set_color(color.x, color.y, color.z);
        let mut end_effector_sphere = window.add_sphere(0.01);
        end_effector_sphere.set_color(
            end_effector_color.x,
            end_effector_color.y,
            end_effector_color.z,
        );
        ArmRenderer {
            base_sphere,
            shoulder_sphere,
            elbow_sphere,
            wrist_sphere,
            end_effector_sphere,
            color,
        }
    }

    fn step(&mut self, window: &mut Window, arm_pose: &ArmPositions) {
        let base = convert_coordinates(new_vec_to_old(arm_pose.base));
        self.base_sphere
            .set_local_translation(Translation3::new(base.x, base.y, base.z));

        let shoulder = convert_coordinates(new_vec_to_old(arm_pose.shoulder));
        window.draw_line(&base, &shoulder, &self.color);
        self.shoulder_sphere
            .set_local_translation(Translation3::new(shoulder.x, shoulder.y, shoulder.z));

        let elbow = convert_coordinates(new_vec_to_old(arm_pose.elbow));
        window.draw_line(&shoulder, &elbow, &self.color);
        self.elbow_sphere
            .set_local_translation(Translation3::new(elbow.x, elbow.y, elbow.z));

        let wrist = convert_coordinates(new_vec_to_old(arm_pose.wrist));
        window.draw_line(&elbow, &wrist, &self.color);
        self.wrist_sphere
            .set_local_translation(Translation3::new(wrist.x, wrist.y, wrist.z));

        let end_effector = convert_coordinates(new_vec_to_old(arm_pose.end_effector));
        self.end_effector_sphere
            .set_local_translation(Translation3::new(
                end_effector.x,
                end_effector.y,
                end_effector.z,
            ));
        window.draw_line(&wrist, &end_effector, &self.color);
    }
}

impl Drop for ArmRenderer {
    fn drop(&mut self) {
        self.base_sphere.unlink();
        self.shoulder_sphere.unlink();
        self.elbow_sphere.unlink();
        self.wrist_sphere.unlink();
        self.end_effector_sphere.unlink();
    }
}

fn render_loop(
    current_arm_pose: Arc<Mutex<Option<ArmPositions>>>,
    motion_plan: Arc<Mutex<Option<Vec<ArmPositions>>>>,
    keep_running: Arc<AtomicBool>,
    desired_state: Arc<Mutex<DesiredState>>,
) {
    let white = Point3::new(1.0, 1.0, 1.0);
    let mut window = Window::new("Guppy");
    let mut frame_counter = Instant::now();

    let mut camera =
        kiss3d::camera::ArcBall::new(Point3::new(1.0, 1.0, 1.0), Point3::new(0.0, 0.0, 0.0));
    camera.set_dist_step(10.0);

    window.set_background_color(0.5, 0.5, 0.5);
    window.set_framerate_limit(Some(240));

    let mut primary_arm = ArmRenderer::new(
        &mut window,
        Point3::new(1.0, 0.0, 1.0),
        Point3::new(0.0, 1.0, 1.0),
    );

    add_ground_plane(&mut window);

    while !window.should_close() && keep_running.load(Ordering::Acquire) {
        let arm_pose = current_arm_pose.lock().expect("Render failed").clone();
        let motion_plan_poses = motion_plan.lock().expect("Render failed").clone();
        if let Some(arm_pose) = arm_pose {
            primary_arm.step(&mut window, &arm_pose);
            window.draw_text(
                &format!(
                    "End effector:\n   x: {}\n   y: {}\n   z: {}\n   angle: {}\nCamera dist: {}\nframe time: {}ms",
                    arm_pose.end_effector.x,
                    arm_pose.end_effector.y,
                    arm_pose.end_effector.z,
                    arm_pose.end_effector_angle,
                    camera.dist(),
                    frame_counter.elapsed().as_millis(),
                ),
                &Point2::new(1.0, 1.0),
                50.0,
                &kiss3d::text::Font::default(),
                &white,
            );
        }
        let mut arm_renders = vec![];
        if let Some(motion_plan) = motion_plan_poses {
            for step in motion_plan {
                let mut arm_render = ArmRenderer::new(
                    &mut window,
                    Point3::new(1.0, 0.0, 0.0),
                    Point3::new(0.0, 1.0, 0.0),
                );
                arm_render.step(&mut window, &step);
                arm_renders.push(arm_render);
            }
        }
        let mut pos_copy = desired_state.lock().unwrap().pose.clone();
        if window.get_key(Key::D) == Action::Press {
            pos_copy.position.y -= frame_counter.elapsed().as_secs_f32() * 0.1;
        }
        if window.get_key(Key::A) == Action::Press {
            pos_copy.position.y += frame_counter.elapsed().as_secs_f32() * 0.1;
        }
        if window.get_key(Key::W) == Action::Press {
            pos_copy.position.x += frame_counter.elapsed().as_secs_f32() * 0.1;
        }
        if window.get_key(Key::S) == Action::Press {
            pos_copy.position.x -= frame_counter.elapsed().as_secs_f32() * 0.1;
        }
        if window.get_key(Key::E) == Action::Press {
            pos_copy.position.z += frame_counter.elapsed().as_secs_f32() * 0.1;
        }
        if window.get_key(Key::Q) == Action::Press {
            pos_copy.position.z -= frame_counter.elapsed().as_secs_f32() * 0.1;
        }
        if window.get_key(Key::R) == Action::Press {
            pos_copy.end_effector_angle -= frame_counter.elapsed().as_secs_f32() * 20.;
        }
        if window.get_key(Key::F) == Action::Press {
            pos_copy.end_effector_angle += frame_counter.elapsed().as_secs_f32() * 20.;
        }
        if window.get_key(Key::B) == Action::Press {
            desired_state.lock().unwrap().gripper_state = true;
        }
        if window.get_key(Key::V) == Action::Press {
            desired_state.lock().unwrap().gripper_state = false;
        }
        if window.get_key(Key::Return) == Action::Press {
            pos_copy.end_effector_angle = 0.0;
            pos_copy.position.x = 0.2;
            pos_copy.position.y = 0.0;
            pos_copy.position.z = 0.2;
        }
        desired_state.lock().unwrap().pose = pos_copy;
        frame_counter = Instant::now();
        window.render_with_camera(&mut camera);
    }
    window.close()
}
