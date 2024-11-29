use ad_msgs::msg::VehicleCommand;
use anyhow::{Error, Result};
use rclrs::Context;
use std::{env, thread, time::Duration};

fn main() -> Result<(), Error> {
    let context = Context::new(env::args())?;
    let node = rclrs::create_node(&context, "vehicle_command_publisher")?;
    let publisher = node.create_publisher("/ego/vehicle_command", rclrs::QOS_PROFILE_DEFAULT)?;

    let mut msg = VehicleCommand::default();

    //Demo sequence
    println!("String vehicle command sequence...");

    // 1. Accelerate forward
    println!("1. Accelerating forward...");
    for _ in 0..30 {
        // 3 seconds
        msg.steering = 0.0;
        msg.accel = 0.5;
        msg.brake = 0.0;
        publisher.publish(&msg)?;
        println!(
            "Forward - Accel: {:.1}, Steering: {:.1}",
            msg.accel, msg.steering
        );
        thread::sleep(Duration::from_millis(100));
    }

    // 2. Turn left while moving
    println!("\n2. Turning left..");
    for _ in 0..20 {
        // 2seconds
        msg.steering = 0.1;
        msg.accel = 0.3;
        msg.brake = 0.0;
        publisher.publish(&msg)?;
        println!(
            "Left turn - Accel: {:.1}, Steering: {:.1}",
            msg.accel, msg.steering
        );
        thread::sleep(Duration::from_millis(100));
    }

    // 3. Turn right while moving
    println!("\n3. Turning right...");
    for _ in 0..20 {
        // 2 seconds
        msg.steering = -0.1;
        msg.accel = 0.3;
        msg.brake = 0.0;
        publisher.publish(&msg)?;
        println!(
            "Right turn - Accel: {:.1}, Steering: {:.1}",
            msg.accel, msg.steering
        );
        thread::sleep(Duration::from_millis(100));
    }

    // 4. Straighten and brake
    println!("\n4. Braking...");
    for _ in 0..20 {
        // 2 seconds
        msg.steering = 0.0;
        msg.accel = 0.0;
        msg.brake = 0.7;
        publisher.publish(&msg)?;
        println!("Braking - Brake: {:.1}", msg.brake);
        thread::sleep(Duration::from_millis(100));
    }

    // 5. Full stop
    println!("\n5. Stopping...");
    msg.steering = 0.0;
    msg.accel = 0.0;
    msg.brake = 0.0;
    publisher.publish(&msg)?;
    println!("Stopped - All controls at 0.0");

    println!("\nSequence completed!");
    Ok(())
}
