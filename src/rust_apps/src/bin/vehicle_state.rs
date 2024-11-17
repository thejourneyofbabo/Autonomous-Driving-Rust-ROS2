use ad_msgs::msg::VehicleCommand;
use anyhow::{Error, Result};
use rclrs::{Context, Node, Subscription};
use std::{env, sync::Arc}; // Assuming the VehicleCommand message is in this package

fn main() -> Result<(), Error> {
    // Create the ROS 2 context and node
    let context = rclrs::Context::new(env::args())?;
    let node = rclrs::create_node(&context, "vehicle_command_subscriber")?;

    // Create the subscriber for the vehicle command topic
    let subscription: Arc<Subscription<VehicleCommand>> = node
        .create_subscription::<VehicleCommand, _>(
            "/ego/vehicle_command",
            rclrs::QOS_PROFILE_DEFAULT,
            move |msg: VehicleCommand| {
                // Print the received message
                println!(
                    "Received VehicleCommand: Accel: {}, Steering: {}, Brake: {}",
                    msg.accel, msg.steering, msg.brake
                );
            },
        )?;

    // Spin to keep the subscriber active
    rclrs::spin(node).map_err(|err| err.into())
}
