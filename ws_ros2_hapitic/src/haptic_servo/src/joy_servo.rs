use rclrs::*;
use tf2_rs::*;
use std::{thread, time::Duration};
use std::sync::{Arc, Mutex};
use geometry_msgs::msg::PoseStamped as PoseStampedMsg;

struct ServoPublisherNode {
    #[allow(unused)]
    publisher: Publisher<PoseStampedMsg>,
    node: Node,
    tf: Option<TransformStamped>,
}

impl ServoPublisherNode {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("js_publisher").unwrap();
        let publisher = node
            .create_publisher("/servo_node/pose_target_cmds")
            .unwrap();
        Ok(Self { publisher, node, tf: None})
    }

    fn set_transform(&mut self, tf: TransformStamped) {
        self.tf = Some(tf);
    }

    fn publish_data(&self, joy_msg: sensor_msgs::msg::Joy) -> Result<(), RclrsError> {
        if let Some(transform) = &self.tf {
            log_debug!(self.node.logger(), "Got transform : {} to {}", transform.parent_frame.as_str(), transform.child_frame.as_str());
        } else {
            log_error!(self.node.logger(), "No transform available, cannot publish servo command");
            return Err(RclrsError::UnknownRclError { code: -1, msg: None });
        }

        let stamp_ = Clock::system().now().to_ros_msg().unwrap();

        let get_axis = |index: usize| {
            if joy_msg.axes[index].abs() >= 0.001 {
                joy_msg.axes[index] / 10.0
            } else {
                0.0
            }
        };

        let transform = self.tf.as_ref().unwrap();

        let msg: PoseStampedMsg = PoseStampedMsg {
            header: std_msgs::msg::Header {
                stamp: builtin_interfaces::msg::Time {
                    sec: stamp_.sec,
                    nanosec: stamp_.nanosec,
                },
                frame_id: transform.parent_frame.clone(),
            },
            
            pose: geometry_msgs::msg::Pose {
                position: geometry_msgs::msg::Point {
                    x: transform.translation[0]  + (get_axis(3)) as f64,
                    y: transform.translation[1]  + (get_axis(4)) as f64,
                    z: transform.translation[2]  + (get_axis(1)) as f64,
                },
                orientation: geometry_msgs::msg::Quaternion {
                    x: transform.rotation[0],
                    y: transform.rotation[1],
                    z: transform.rotation[2],
                    w: transform.rotation[3],
                },
            },

            // twist: geometry_msgs::msg::Twist {
            //     linear: geometry_msgs::msg::Vector3 {
            //         x: get_axis(3) as f64,
            //         y: get_axis(4) as f64,
            //         z: get_axis(1) as f64,
            //     },
            //     angular: geometry_msgs::msg::Vector3 {
            //         x: 0.0,
            //         y: 0.0,
            //         z: 0.0,
            //     },
            // },
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
                            .default("panda_link0".into())
                            .optional()?;

    let eef_frame: OptionalParameter<Arc<str>> = serv_pub.node.clone()
                            .declare_parameter( "eef_frame")
                            .default("panda_link8".into())
                            .optional()?;

    log!(
        serv_pub.node.logger().once(),
        "Joy servo integrating node started with reference frame: {} eef frame: {}",
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
        .create_client::<moveit_msgs::srv::ServoCommandType>("/servo_node/switch_command_type")?;


    let ref_frame_str = ref_frame.get().clone().unwrap().as_ref().to_string();
    let eef_frame_str = eef_frame.get().clone().unwrap().as_ref().to_string();

    let serv_pub_ptr = Arc::new(Mutex::new(serv_pub));
    let serv_pub_ptr_clone = Arc::clone(&serv_pub_ptr);

    let _promise = executor.commands().run(async move {
        log_info!(
            node_binding.as_str(),
            "Haptic servo is waiting for MoveIt servo node to be ready",
        );
        command_type.notify_on_service_ready().await.unwrap();

        let request = moveit_msgs::srv::ServoCommandType_Request {
            command_type: moveit_msgs::srv::ServoCommandType_Request::POSE,
        };
        // let request = moveit_msgs::srv::ServoCommandType_Request::TWIST;

        log_info!(node_binding.as_str(), "Waiting for response");
        let response: moveit_msgs::srv::ServoCommandType_Response =
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
        } else {
            log!(node_binding.as_str(), "Could not switch input to: Pose",);
        }
    
    });
    
    let msg_ = Arc::new(Mutex::new(None::<sensor_msgs::msg::Joy>));

    let msg_sub = Arc::clone(&msg_);

    let _sub = serv_pub_ptr_clone.try_lock().unwrap().node.clone()
        .create_subscription(
            "/joy",
            move |msg: sensor_msgs::msg::Joy| {
                *msg_sub.lock().unwrap() = Some(msg);
            },
        ).unwrap();

    let msg_pub = Arc::clone(&msg_);

    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(10));
        if let Some(msg) = msg_pub.lock().unwrap().clone() {
            if let Ok(()) = serv_pub_ptr_clone.try_lock().unwrap().publish_data(msg) {
                // Do nothing
            } else {
                log_error!(node_name.as_str(), "Error in publishing servo command");
                // break;
            }
        }       
    });

    executor.spin(SpinOptions::default()).first_error()
}
