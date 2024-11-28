use std::sync::mpsc::{channel, Receiver, Sender};
use std::thread;
use std::time::Duration;

// Simple Publisher Component
struct Publisher {
    name: String,
    sender: Sender<i32>,
    counter: i32,
}

impl Publisher {
    fn new(name: &str, sender: Sender<i32>) -> Self {
        Self {
            name: name.to_string(),
            sender,
            counter: 0,
        }
    }

    fn start(&mut self) {
        println!("{} started", self.name);
        loop {
            self.counter += 1;
            println!("{} sending: {}", self.name, self.counter);
            self.sender.send(self.counter).unwrap();
            thread::sleep(Duration::from_secs(1));
        }
    }
}

// Simple Listener Component
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

fn main() {
    // Create communication channel
    let (sender, receiver) = channel();

    // Create components
    let mut publisher = Publisher::new("Talker", sender);
    let listener = Listener::new("Listener", receiver);

    // Start listener in separate thread
    let _listener_handle = thread::spawn(move || {
        listener.start();
    });

    // Start publisher in main thread
    publisher.start();
}
