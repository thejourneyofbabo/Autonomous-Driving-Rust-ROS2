use ad_msgs::msg::VehicleCommand;
use anyhow::{Error, Result};
use rclrs::{Context, Node, Publisher};
use std::env;
use tokio::time::{sleep, Duration};

// Function to publish acceleration forward commands
async fn accelerate_forward(publisher: &Publisher<VehicleCommand>) -> Result<(), Error> {
    println!("1. Accelerating forward...");
    let mut msg = VehicleCommand::default();
    for _ in 0..30 {
        msg.steering = 0.0;
        msg.accel = 0.5; // 50% throttle
        msg.brake = 0.0;
        publisher.publish(&msg)?;
        println!(
            "Forward - Accel: {:.1}, Steering: {:.1}",
            msg.accel, msg.steering
        );
        sleep(Duration::from_millis(100)).await;
    }
    Ok(())
}

// Function to publish left turn commands
async fn turn_left(publisher: &Publisher<VehicleCommand>) -> Result<(), Error> {
    println!("\n2. Turning left...");
    let mut msg = VehicleCommand::default();
    for _ in 0..20 {
        msg.steering = 0.05; // Turn left
        msg.accel = 0.3; // Maintain some speed
        msg.brake = 0.0;
        publisher.publish(&msg)?;
        println!(
            "Left turn - Accel: {:.1}, Steering: {:.1}",
            msg.accel, msg.steering
        );
        sleep(Duration::from_millis(100)).await;
    }
    Ok(())
}

// Function to publish right turn commands
async fn turn_right(publisher: &Publisher<VehicleCommand>) -> Result<(), Error> {
    println!("\n3. Turning right...");
    let mut msg = VehicleCommand::default();
    for _ in 0..20 {
        msg.steering = -0.05; // Turn right
        msg.accel = 0.3; // Maintain some speed
        msg.brake = 0.0;
        publisher.publish(&msg)?;
        println!(
            "Right turn - Accel: {:.1}, Steering: {:.1}",
            msg.accel, msg.steering
        );
        sleep(Duration::from_millis(100)).await;
    }
    Ok(())
}

// Function to publish brake commands
async fn brake(publisher: &Publisher<VehicleCommand>) -> Result<(), Error> {
    println!("\n4. Braking...");
    let mut msg = VehicleCommand::default();
    for _ in 0..20 {
        msg.steering = 0.0;
        msg.accel = 0.0;
        msg.brake = 0.7; // Apply brakes
        publisher.publish(&msg)?;
        println!("Braking - Brake: {:.1}", msg.brake);
        sleep(Duration::from_millis(100)).await;
    }
    Ok(())
}

// Function to publish stop command
async fn stop(publisher: &Publisher<VehicleCommand>) -> Result<(), Error> {
    println!("\n5. Stopping...");
    let mut msg = VehicleCommand::default();
    msg.steering = 0.0;
    msg.accel = 0.0;
    msg.brake = 0.0;
    publisher.publish(&msg)?;
    println!("Stopped - All controls at 0.0");
    Ok(())
}

#[tokio::main]
async fn main() -> Result<(), Error> {
    let context = Context::new(env::args())?;
    let node = rclrs::create_node(&context, "vehicle_command_publisher")?;
    let publisher = node
        .create_publisher::<VehicleCommand>("/ego/vehicle_command", rclrs::QOS_PROFILE_DEFAULT)?;

    println!("Starting vehicle command sequence...");

    accelerate_forward(&publisher).await?;
    turn_right(&publisher).await?;
    turn_left(&publisher).await?;
    brake(&publisher).await?;
    stop(&publisher).await?;

    println!("\nSequence completed!");
    Ok(())
}
