use guppy_controller::arm_controller::{ArmPositions, EndEffectorPose};
use kiss3d::{
    event::{Action, Key},
    scene::SceneNode,
    window::Window,
};
use nalgebra::{Isometry3, Point2, Point3, Translation3, UnitQuaternion, Vector3};
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    time::{Duration, Instant},
};

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

    pub fn pose_mut(&mut self) -> &mut EndEffectorPose {
        &mut self.pose
    }

    pub fn gripper_state(&self) -> bool {
        self.gripper_state
    }

    pub fn set_gripper_state(&mut self, gripper_state: bool) {
        self.gripper_state = gripper_state;
    }
}

pub struct VisualizerInterface {
    state: Arc<VisualizerInterfaceInternal>,
    thread_handle: Option<std::thread::JoinHandle<()>>,
}

impl VisualizerInterface {
    pub fn set_position(&mut self, arm_position: ArmPositions) {
        self.state.set_current_pose(arm_position);
    }

    pub fn get_latest_arm_command(&self) -> ArmMotionCommand {
        self.state.get_latest_arm_command()
    }

    pub fn window_opened(&self) -> bool {
        self.state.window_opened()
    }
}

impl Default for VisualizerInterface {
    fn default() -> Self {
        let state = Arc::new(VisualizerInterfaceInternal::new());
        let thread_handle = std::thread::spawn({
            let internal_state = Arc::clone(&state);
            move || render_loop(internal_state)
        });
        VisualizerInterface {
            state,
            thread_handle: Some(thread_handle),
        }
    }
}

impl Drop for VisualizerInterface {
    fn drop(&mut self) {
        self.state.stop_running();
        if let Some(thread_handle) = self.thread_handle.take() {
            thread_handle
                .join()
                .expect("Failed drop for VisualizerInterface");
        }
    }
}

pub enum ArmMotionCommand {
    None,
    Trajectory(DesiredState),
    Jogging(DesiredState),
}

impl ArmMotionCommand {
    pub fn take(&mut self) -> Self {
        std::mem::take(self)
    }

    pub fn is_some(&self) -> bool {
        !matches!(self, ArmMotionCommand::None)
    }
}

impl Default for ArmMotionCommand {
    fn default() -> Self {
        ArmMotionCommand::None
    }
}

struct VisualizerInterfaceInternal {
    current_arm_pose: Arc<Mutex<Option<ArmPositions>>>,
    keep_running: Arc<AtomicBool>,
    desired_state: Arc<Mutex<ArmMotionCommand>>,
    window_opened: Arc<AtomicBool>,
}

impl VisualizerInterfaceInternal {
    fn new() -> Self {
        Self {
            current_arm_pose: Arc::default(),
            keep_running: Arc::new(AtomicBool::new(true)),
            desired_state: Arc::default(),
            window_opened: Arc::new(AtomicBool::new(true)),
        }
    }

    fn send_new_arm_command(&self, command: ArmMotionCommand) {
        *self.desired_state.lock().unwrap() = command;
    }

    fn get_latest_arm_command(&self) -> ArmMotionCommand {
        self.desired_state.lock().unwrap().take()
    }

    fn set_current_pose(&self, pose: ArmPositions) {
        *self.current_arm_pose.lock().unwrap() = Some(pose);
    }

    fn get_current_pose(&self) -> Option<ArmPositions> {
        self.current_arm_pose.lock().unwrap().clone()
    }

    fn keep_running(&self) -> bool {
        self.keep_running.load(Ordering::SeqCst)
    }

    fn stop_running(&self) {
        self.keep_running.store(false, Ordering::SeqCst);
    }

    fn window_opened(&self) -> bool {
        self.window_opened.load(Ordering::SeqCst)
    }

    fn set_window_closed(&self) {
        self.window_opened.store(false, Ordering::SeqCst);
    }
}

fn convert_coordinates(position: Vector3<f32>) -> Point3<f32> {
    Point3::new(position.y, position.z, position.x)
}

fn render_loop(state: Arc<VisualizerInterfaceInternal>) {
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

    let mut desired_state =
        DesiredState::new(EndEffectorPose::new(Vector3::new(0.2, 0., 0.2), 0.0), false);

    state.send_new_arm_command(ArmMotionCommand::Trajectory(desired_state.clone()));

    while !window.should_close() && state.keep_running() {
        let arm_pose = state.get_current_pose();
        if let Some(arm_pose) = arm_pose {
            primary_arm.update_pose(&mut window, &arm_pose);
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

        let command = process_keyboard_input(&window, &mut desired_state, frame_counter.elapsed());
        if command.is_some() {
            state.send_new_arm_command(command);
        }
        frame_counter = Instant::now();
        window.render_with_camera(&mut camera);
    }
    window.close();
    state.set_window_closed();
}

fn process_keyboard_input(
    window: &Window,
    desired_state: &mut DesiredState,
    frame_time: Duration,
) -> ArmMotionCommand {
    if window.get_key(Key::Return) == Action::Press {
        desired_state.pose_mut().end_effector_angle = 0.0;
        desired_state.pose_mut().position.x = 0.2;
        desired_state.pose_mut().position.y = 0.0;
        desired_state.pose_mut().position.z = 0.2;
        return ArmMotionCommand::Trajectory(desired_state.clone());
    }
    let elapsed_seconds = frame_time.as_secs_f32();
    let xy = desired_state.pose().position.xy();
    let (mut distance, mut angle) = cartesian_to_polar((xy.x, xy.y));
    let mut input_detected = false;
    if window.get_key(Key::D) == Action::Press {
        angle -= elapsed_seconds * 0.8;
        input_detected = true;
    }
    if window.get_key(Key::A) == Action::Press {
        angle += elapsed_seconds * 0.8;
        input_detected = true;
    }
    if window.get_key(Key::W) == Action::Press {
        distance += elapsed_seconds * 0.1;
        input_detected = true;
    }
    if window.get_key(Key::S) == Action::Press {
        distance -= elapsed_seconds * 0.1;
        input_detected = true;
    }
    if window.get_key(Key::E) == Action::Press {
        desired_state.pose_mut().position.z += elapsed_seconds * 0.1;
        input_detected = true;
    }
    if window.get_key(Key::Q) == Action::Press {
        desired_state.pose_mut().position.z -= elapsed_seconds * 0.1;
        input_detected = true;
    }
    if window.get_key(Key::R) == Action::Press {
        desired_state.pose_mut().end_effector_angle -= elapsed_seconds * 20.;
        input_detected = true;
    }
    if window.get_key(Key::F) == Action::Press {
        desired_state.pose_mut().end_effector_angle += elapsed_seconds * 20.;
        input_detected = true;
    }
    if window.get_key(Key::B) == Action::Press {
        desired_state.set_gripper_state(true);
        input_detected = true;
    }
    if window.get_key(Key::V) == Action::Press {
        desired_state.set_gripper_state(false);
        input_detected = true;
    }
    if input_detected {
        // add small parts to clamp to prevent overflowing
        // TODO (David): Find a smarter way to do this
        angle = angle.clamp(
            -std::f32::consts::PI + 0.0001,
            std::f32::consts::PI - 0.0001,
        );
        let (x, y) = polar_to_cartesian((distance, angle));
        desired_state.pose_mut().position.x = x;
        desired_state.pose_mut().position.y = y;
        ArmMotionCommand::Jogging(desired_state.clone())
    } else {
        ArmMotionCommand::None
    }
}

fn cartesian_to_polar((x, y): (f32, f32)) -> (f32, f32) {
    let distance = (x.powi(2) + y.powi(2)).sqrt();
    let angle = y.atan2(x);
    (distance, angle)
}

fn polar_to_cartesian((distance, angle): (f32, f32)) -> (f32, f32) {
    let x = distance * angle.cos();
    let y = distance * angle.sin();
    (x, y)
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

    fn update_pose(&mut self, window: &mut Window, arm_pose: &ArmPositions) {
        let base = convert_coordinates(arm_pose.base);
        self.base_sphere.set_local_translation(base.into());

        let shoulder = convert_coordinates(arm_pose.shoulder);
        window.draw_line(&base, &shoulder, &self.color);
        self.shoulder_sphere.set_local_translation(shoulder.into());

        let elbow = convert_coordinates(arm_pose.elbow);
        window.draw_line(&shoulder, &elbow, &self.color);
        self.elbow_sphere.set_local_translation(elbow.into());

        let wrist = convert_coordinates(arm_pose.wrist);
        window.draw_line(&elbow, &wrist, &self.color);
        self.wrist_sphere.set_local_translation(wrist.into());

        let end_effector = convert_coordinates(arm_pose.end_effector);
        self.end_effector_sphere
            .set_local_translation(end_effector.into());
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

fn add_ground_plane(window: &mut Window) {
    let size = 0.5;
    for i in 0..4 {
        for j in 0..4 {
            let mut cube = window.add_cube(size, size, 0.001);
            if (i + j) % 2 == 0 {
                cube.set_color(0.0, 0.0, 0.0);
            } else {
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

#[cfg(test)]
mod test {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn cartesian_to_polar_zero() {
        let (distance, angle) = cartesian_to_polar((0.0, 0.0));
        assert_relative_eq!(distance, 0.0);
        assert_relative_eq!(angle, 0.0);
    }

    #[test]
    fn cartesian_to_polar_front() {
        let (distance, angle) = cartesian_to_polar((0.1, 0.0));
        assert_relative_eq!(distance, 0.1);
        assert_relative_eq!(angle, 0.0);
    }

    #[test]
    fn cartesian_to_polar_left() {
        let (distance, angle) = cartesian_to_polar((0.0, 0.1));
        assert_relative_eq!(distance, 0.1);
        assert_relative_eq!(angle, std::f32::consts::FRAC_PI_2);
    }

    #[test]
    fn cartesian_to_polar_right() {
        let (distance, angle) = cartesian_to_polar((0.0, -0.1));
        assert_relative_eq!(distance, 0.1);
        assert_relative_eq!(angle, -std::f32::consts::FRAC_PI_2);
    }

    #[test]
    fn polar_to_cartesian_zero() {
        let (x, y) = polar_to_cartesian((0.0, 0.0));
        assert_relative_eq!(x, 0.0);
        assert_relative_eq!(y, 0.0);
    }

    #[test]
    fn polar_to_cartesian_front() {
        let (x, y) = polar_to_cartesian((0.1, 0.0));
        assert_relative_eq!(x, 0.1);
        assert_relative_eq!(y, 0.0);
    }

    #[test]
    fn polar_to_cartesian_left() {
        let (x, y) = polar_to_cartesian((0.1, std::f32::consts::FRAC_PI_2));
        assert_relative_eq!(x, 0.0);
        assert_relative_eq!(y, 0.1);
    }

    #[test]
    fn polar_to_cartesian_right_negative() {
        let (x, y) = polar_to_cartesian((0.1, -std::f32::consts::FRAC_PI_2));
        assert_relative_eq!(x, 0.0);
        assert_relative_eq!(y, -0.1);
    }

    #[test]
    fn polar_to_cartesian_right_wrap() {
        let (x, y) = polar_to_cartesian((0.1, std::f32::consts::PI + std::f32::consts::FRAC_PI_2));
        assert_relative_eq!(x, 0.0);
        assert_relative_eq!(y, -0.1);
    }

    #[test]
    fn arm_motion_command_is_some_() {
        let command = ArmMotionCommand::Trajectory(DesiredState::new(
            EndEffectorPose::new(na::Vector3::zeros(), 0.0),
            true,
        ));
        assert!(command.is_some());
        let command = ArmMotionCommand::None;
        assert!(!command.is_some());
    }
}
