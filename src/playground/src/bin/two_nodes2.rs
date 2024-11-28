use anyhow::{Error, Result};
use rclrs;
use std::env;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::Arc;
use std::thread;
use std::time::Duration;
use std_msgs::msg::String as RosString;

// Enhanced Publisher Component
struct Publisher {
    name: String,
    channel_sender: Sender<i32>,
    ros_publisher: Arc<rclrs::Publisher<RosString>>, // Changed to Arc
    counter: i32,
}

impl Publisher {
    fn new(
        name: &str,
        channel_sender: Sender<i32>,
        ros_publisher: Arc<rclrs::Publisher<RosString>>, // Changed to Arc
    ) -> Self {
        Self {
            name: name.to_string(),
            channel_sender,
            ros_publisher,
            counter: 0,
        }
    }

    fn start(&mut self) -> Result<(), Error> {
        println!("{} started", self.name);
        let mut ros_msg = RosString::default();

        loop {
            self.counter += 1;

            // Publish to channel
            println!("{} sending to channel: {}", self.name, self.counter);
            self.channel_sender.send(self.counter).unwrap();

            // Publish to ROS topic
            ros_msg.data = format!("Hello from {}: count {}", self.name, self.counter);
            println!("Publishing to ROS: [{}]", ros_msg.data);
            self.ros_publisher.publish(&ros_msg)?;

            thread::sleep(Duration::from_secs(1));
        }
    }
}

// Listener remains the same
struct Listener {
    name: String,
    receiver: Receiver<i32>,
}

impl Listener {
    fn new(name: &str, receiver: Receiver<i32>) -> Self {
        Self {
            name: name.to_string(),
            receiver,
        }
    }

    fn start(&self) {
        println!("{} started", self.name);
        loop {
            match self.receiver.recv() {
                Ok(msg) => println!("{} received: {}", self.name, msg),
                Err(_) => break,
            }
        }
    }
}

fn main() -> Result<(), Error> {
    // Initialize ROS2
    let context = rclrs::Context::new(env::args())?;
    let node = rclrs::create_node(&context, "dual_publisher")?;
    let ros_publisher = node.create_publisher("publish_hello", rclrs::QOS_PROFILE_DEFAULT)?;

    // Create channel communication
    let (sender, receiver) = channel();

    // Create components
    let mut publisher = Publisher::new("DualTalker", sender, ros_publisher);
    let listener = Listener::new("Listener", receiver);

    // Start listener in separate thread
    let listener_handle = thread::spawn(move || {
        listener.start();
    });

    // Start publisher in main thread
    publisher.start()?;

    // Wait for listener (though we'll never reach this due to infinite loop)
    listener_handle.join().unwrap();

    Ok(())
}
