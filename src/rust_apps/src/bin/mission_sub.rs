use ad_msgs::msg::Mission;
use anyhow::{Error, Result};
use rclrs::{Context, Node, Subscription};
use std::{env, sync::Arc};

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;

    let node = rclrs::create_node(&context, "mission_subscriber")?;

    let mut num_messages: usize = 0;

    // Change the expected type to Arc<Subscription<Mission>> to handle the returned type
    let subscription: Arc<Subscription<Mission>> = node.create_subscription::<Mission, _>(
        "/ego/mission",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: Mission| {
            num_messages += 1;
            println!("Received message #{}: {:?}", num_messages, msg);
        },
    )?;

    rclrs::spin(node).map_err(|err| err.into())
}
