use ad_msgs::msg::VehicleCommand;
use anyhow::{Error, Result};
use rclrs::Context;
use std::{env, thread, time::Duration};

fn main() -> Result<(), Error> {
    // Create the ROS 2 context and node
    let context = Context::new(env::args())?;
    let node = rclrs::create_node(&context, "vehicle_command_publisher")?;

    // Create the publisher
    let publisher = node
        .create_publisher::<VehicleCommand>("/ego/vehicle_command", rclrs::QOS_PROFILE_DEFAULT)?;

    let mut msg = VehicleCommand::default();

    println!("Starting to publish vehicle commands...");
    println!("Will accelerate at 0.8 for 5 seconds");

    // Get the start time
    let start_time = std::time::Instant::now();
    let duration = Duration::from_secs(5);

    while start_time.elapsed() < duration {
        // Set strong acceleration command
        msg.steering = 0.0;
        msg.accel = 0.8; // 80% throttle
        msg.brake = 0.0;

        println!(
            "Publishing - Accel: {:.1}, Brake: {:.1}, Steering: {:.1} (Time elapsed: {:.1}s)",
            msg.accel,
            msg.brake,
            msg.steering,
            start_time.elapsed().as_secs_f64()
        );

        publisher.publish(&msg)?;
        thread::sleep(Duration::from_millis(100)); // Publish at 10Hz
    }

    // After 5 seconds, send zero command
    msg.accel = 0.0;
    println!("\nStopping - Setting acceleration to 0");
    publisher.publish(&msg)?;

    println!("Command sequence completed!");
    Ok(())
}
