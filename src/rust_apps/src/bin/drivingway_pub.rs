use ad_msgs::msg::{LanePointData, PolyfitLaneData};
use anyhow::{Error, Result};
use rclrs::Context;
use std::env;

fn main() -> Result<(), Error> {
    let context = Context::new(env::args())?;
    let node = rclrs::create_node(&context, "lane_points_subscriber")?;

    let _subscription = node.create_subscription(
        "/ego/lane_points",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: LanePointData| {
            for point in msg.point.iter() {
                println!("x: {}, y: {}, z: {}", point.x, point.y, point.z);
            }
            println!("---");
        },
    )?;

    rclrs::spin(node).map_err(|err| err.into())
}
