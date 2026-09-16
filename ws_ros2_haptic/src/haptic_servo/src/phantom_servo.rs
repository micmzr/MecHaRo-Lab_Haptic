mod quaternions;

use rclrs::*;
use tf2_rs::{
    BufferCore, TransformListener, TransformStamped, LookupTime,
};
use std::{thread, time::Duration};
use std::sync::{Arc, Mutex};
use ros_env::geometry_msgs::msg::PoseStamped as PoseStampedMsg;
use ros_env::omni_msgs::msg::OmniButtonEvent as OmniButtonEventMsg;
use ros_env::omni_msgs::msg::OmniFeedback as OmniFeedbackMsg; // todo

struct ServoPublisherNode {
    #[allow(unused)]
    publisher: Publisher<PoseStampedMsg>,
    node: Node,
    tf: Option<TransformStamped>,
    ready: bool,
}

impl ServoPublisherNode {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("hs_publisher").unwrap();
        let publisher = node
            .create_publisher("/servo_node/pose_target_cmds")
            .unwrap();
        Ok(Self { publisher, node, tf: None, ready: false})
    }

    fn set_transform(&mut self, tf: TransformStamped) {
        self.tf = Some(tf);
    }

    fn publish_data(&self, phantom_msg: PoseStampedMsg) -> Result<(), RclrsError> {
        if let Some(transform) = &self.tf {
            log_debug!(self.node.logger(), "Got transform : {} to {}", transform.parent_frame.as_str(), transform.child_frame.as_str());
        } else {
            log_warn!(self.node.logger(), "No transform available, cannot publish servo command");
            thread::sleep(Duration::from_millis(1000));
            return Err(RclrsError::UnknownRclError { code: -1, msg: None });
        }

        let stamp_ = Clock::system().now().to_ros_msg().unwrap();

        let transform = self.tf.as_ref().unwrap();

        // axis transform
        let q_rob = quaternions::Quaternion {
            x: transform.rotation[0],
            y: transform.rotation[1],
            z: transform.rotation[2],
            w: transform.rotation[3],
        };

        let q_inkwell = quaternions::Quaternion {
            x: -0.2072416477963069, 
            y: -0.003910005128970363, 
            z: -0.002516783525819699, 
            w: 0.9782787317933714,
        };
 
        let q_phantom = quaternions::Quaternion::from(phantom_msg.pose.orientation);
        
        // println!("q_pahntom: {}",q_phantom);

        let q_phantom = q_inkwell.delta(&q_phantom);

        // limit the rotation to 45 degrees in each axis to avoid singularities in the robot
        let (mut roll, mut pitch, mut yaw) = q_phantom.to_rpy();

        log_debug!(self.node.logger(), "Got roll: {} pitch: {} yaw: {}", roll, pitch, yaw);

        if roll.abs() > std::f64::consts::FRAC_PI_8 {
            roll = roll.signum() * std::f64::consts::FRAC_PI_8;
        }

        if pitch.abs() > std::f64::consts::FRAC_PI_8 {
            pitch = pitch.signum() * std::f64::consts::FRAC_PI_8;
        }

        if yaw.abs() > std::f64::consts::FRAC_PI_8 {
            yaw = yaw.signum() * std::f64::consts::FRAC_PI_8;
        }

        // let q_phantom = quaternions::Quaternion::from_rpy(roll, pitch, yaw);
        // roll -> x - axis,
        // pitch -> y - axis 
        // yaw -> z - axis 
        let q_phantom = quaternions::Quaternion::from_rpy(roll, -yaw, -pitch);

        let q_new = q_phantom.multiply(&q_rob);

        // x_phantom => -x_robot
        // y_phantom =>  z_robot
        // z_phantom =>  y_robot

        let msg: PoseStampedMsg = PoseStampedMsg {
            header: ros_env::std_msgs::msg::Header {
                stamp: ros_env::builtin_interfaces::msg::Time {
                    sec: stamp_.sec,
                    nanosec: stamp_.nanosec,
                },
                frame_id: transform.parent_frame.clone(),
            },

            pose: ros_env::geometry_msgs::msg::Pose {
                position: ros_env::geometry_msgs::msg::Point {
                    x: transform.translation[0]  - (phantom_msg.pose.position.x) as f64,
                    y: transform.translation[1]  + (phantom_msg.pose.position.z) as f64,
                    z: transform.translation[2]  + (phantom_msg.pose.position.y) as f64,
                },
                // orientation: q_rob.to_ros_msg(), // without rotations 
                orientation: q_new.to_ros_msg(),
            },
        };
        self.publisher.publish(msg).unwrap();
        
        Ok(())
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env().unwrap().create_basic_executor();
    let serv_pub = ServoPublisherNode::new(&executor).unwrap();

    let ref_frame: OptionalParameter<Arc<str>> = serv_pub.node.clone()
                            .declare_parameter( "ref_frame")
                            .default("base_link".into())
                            .optional()?;

    let eef_frame: OptionalParameter<Arc<str>> = serv_pub.node.clone()
                            .declare_parameter( "eef_frame")
                            .default("tool0".into())
                            .optional()?;

    let gripper_cmd: OptionalParameter<Arc<[bool]>> = serv_pub.node.clone()
                            .declare_parameter( "gripper_cmd")
                            .default( Arc::from( vec![false]) )
                            .optional()?;
    #[allow(unused)]
    let gripper_joints: OptionalParameter<Arc<[Arc<str>]>> = serv_pub.node.clone()
                            .declare_parameter("grippr_joints")
                            .default( Arc::from(vec![Arc::from("finger1")]) )
                            .optional()?;

    let gripper_open_vals: OptionalParameter<Arc<[f64]>> = serv_pub.node.clone()
                            .declare_parameter("gripper_open")
                            .default( Arc::from(vec![0.0]) )
                            .optional()?;

    let gripper_close_vals: OptionalParameter<Arc<[f64]>> = serv_pub.node.clone()
                            .declare_parameter("gripper_close")
                            .default( Arc::from(vec![0.025]) )
                            .optional()?;

    let gripper_cmd = gripper_cmd.get().unwrap()[0];

    log!(
        serv_pub.node.logger().once(),
        "Phantom servo integrating node started with reference frame: {} eef frame: {}",
        ref_frame.get().unwrap().as_ref(),
        eef_frame.get().unwrap().as_ref()
    );
    
    let node_name = serv_pub.node.name().to_string();
    let node_binding = node_name.clone();

    let buffer = BufferCore::new(Duration::from_secs(60));
    let _listener = TransformListener::new(&serv_pub.node, buffer.clone())?;

    let command_type = serv_pub
        .node
        .clone()
        .create_client::<ros_env::moveit_msgs::srv::ServoCommandType>("/servo_node/switch_command_type")?;

    let action_gripper_cmd = serv_pub
        .node
        .clone()
        .create_action_client::<ros_env::control_msgs::action::GripperCommand>("/panda_hand_controller/gripper_cmd")?;

    let action_gripper_fjt = serv_pub
        .node
        .clone()
        .create_action_client::<ros_env::control_msgs::action::FollowJointTrajectory>("/gripper_controller/follow_joint_trajectory")?; // todo
    
    let ref_frame_str = ref_frame.get().clone().unwrap().as_ref().to_string();
    let eef_frame_str = eef_frame.get().clone().unwrap().as_ref().to_string();

    let serv_pub_ptr = Arc::new(Mutex::new(serv_pub));
    let serv_pub_ptr_clone = Arc::clone(&serv_pub_ptr);

    let _ = executor.spin(SpinOptions::timeout(SpinOptions::new(), Duration::from_millis(5000))).first_error(); // need to collect some data by tf listener 

    let _promise = executor.commands().run(async move {
        log_info!(
            node_binding.as_str(),
            "Haptic servo is waiting for MoveIt servo node to be ready",
        );
        command_type.notify_on_service_ready().await.unwrap();

        let request = ros_env::moveit_msgs::srv::ServoCommandType_Request {
            command_type: ros_env::moveit_msgs::srv::ServoCommandType_Request::POSE,
        };
        // let request = ros_env::moveit_msgs::srv::ServoCommandType_Request::TWIST;

        log_info!(node_binding.as_str(), "Waiting for response");
        let response: ros_env::moveit_msgs::srv::ServoCommandType_Response =
            command_type.call(&request).unwrap().await.unwrap();

        let tf = match buffer.lookup_transform(ref_frame_str.as_str(), eef_frame_str.as_str(), LookupTime::Latest){
            Ok(t) => Some(t),
            Err(e) => {
                log_error!(node_binding.as_str(), "Error in lookup transform: {}", e);
                return;
            }
        }; 

        // println!("transform: {tf:?}");
        serv_pub_ptr.lock().unwrap().set_transform(tf.clone().unwrap());

        if response.success {
            log_info!(node_binding.as_str(), "Switched to input type: Pose",);
            serv_pub_ptr.lock().unwrap().ready = true;
        } else {
            log!(node_binding.as_str(), "Could not switch input to: Pose",);
        }
    
    });

    if serv_pub_ptr_clone.lock().unwrap().ready {
        let _ = executor.spin(SpinOptions::timeout(SpinOptions::new(), Duration::from_millis(1000))).first_error();
        return Ok(())
    }
    
    let msg_ = Arc::new(Mutex::new(None::<PoseStampedMsg>));

    let msg_sub = Arc::clone(&msg_);

    let _sub = serv_pub_ptr_clone.try_lock().unwrap().node.clone()
        .create_subscription(
            "/phantom/pose",
            move |msg: PoseStampedMsg| {
                *msg_sub.lock().unwrap() = Some(msg);
            },
        ).unwrap();

    let n_name = node_name.as_str().to_string();

    let _sub_button = serv_pub_ptr_clone.try_lock().unwrap().node.clone()
        .create_subscription(
            "/phantom/button",
            move |msg: OmniButtonEventMsg| {
                log_debug!(&n_name, "Received button event: grey button {} white button {}", msg.grey_button, msg.white_button);
                if msg.white_button == 1 && msg.grey_button == 0 {
                    log_info!(&n_name, "Sending close gripper command");
                    if gripper_cmd {
                        let goal = ros_env::control_msgs::action::GripperCommand_Goal {
                            command: ros_env::control_msgs::msg::GripperCommand {
                                position: gripper_close_vals.get().unwrap()[0], 
                                max_effort: 0.0,
                            }
                        };
                        if let Ok(_reqeust) = action_gripper_cmd.try_request_goal(goal) {
                            log_info!(&n_name, "Gripper close command sent successfully");
                        } else {
                            log_warn!(&n_name, "Failed to send gripper open command");
                        }
                    }
                    else {
                        let goal = ros_env::control_msgs::action::FollowJointTrajectory_Goal {
                            trajectory: ros_env::trajectory_msgs::msg::JointTrajectory {
                                joint_names: gripper_joints.get().unwrap().iter().map(|s| s.to_string()).collect(), 
                                points: vec![ros_env::trajectory_msgs::msg::JointTrajectoryPoint {
                                    positions: gripper_close_vals.get().unwrap().to_vec(),
                                    time_from_start: ros_env::builtin_interfaces::msg::Duration { sec: 1, nanosec: 0 },
                                    ..Default::default()
                                }],
                                ..Default::default()
                            },
                            ..Default::default()
                        };
                        if let Ok(_reqeust) = action_gripper_fjt.try_request_goal(goal) {
                            log_info!(&n_name, "Gripper close command sent successfully");
                        } else {
                            log_warn!(&n_name, "Failed to send gripper close command");
                        }
                    }
                }
                else if msg.white_button == 0 && msg.grey_button == 1 {
                    log_info!(&n_name, "Sending open gripper command");
                    if gripper_cmd {
                        let goal = ros_env::control_msgs::action::GripperCommand_Goal {
                            command: ros_env::control_msgs::msg::GripperCommand {
                                position: gripper_open_vals.get().unwrap()[0],
                                max_effort: 0.0,
                            }
                        };
                        if let Ok(_reqeust) = action_gripper_cmd.try_request_goal(goal) {
                            log_info!(&n_name, "Gripper open command sent successfully");
                        } else {
                            log_warn!(&n_name, "Failed to send gripper open command");
                        }
                    }
                    else {
                        let goal = ros_env::control_msgs::action::FollowJointTrajectory_Goal {
                            trajectory: ros_env::trajectory_msgs::msg::JointTrajectory {
                                joint_names: gripper_joints.get().unwrap().iter().map(|s| s.to_string()).collect(),
                                points: vec![ros_env::trajectory_msgs::msg::JointTrajectoryPoint {
                                    positions: gripper_open_vals.get().unwrap().to_vec(), 
                                    time_from_start: ros_env::builtin_interfaces::msg::Duration { sec: 1, nanosec: 0 },
                                    ..Default::default()
                                }],
                                ..Default::default()
                            },
                            ..Default::default()
                        };
                        if let Ok(_reqeust) = action_gripper_fjt.try_request_goal(goal) {
                            log_info!(&n_name, "Gripper open command sent successfully");
                        } else {
                            log_warn!(&n_name, "Failed to send gripper open command");
                        }
                    }
                }
            },
        ).unwrap();

    let msg_pub = Arc::clone(&msg_);

    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(10));
        if let Some(msg) = msg_pub.lock().unwrap().clone() {
            if let Ok(()) = serv_pub_ptr_clone.try_lock().unwrap().publish_data(msg) {
                // Do nothing
            } else {
                log_debug!(node_name.as_str(), "Error in publishing servo command");
                // break;
            }
        }       
    });

    executor.spin(SpinOptions::default()).first_error()
}
