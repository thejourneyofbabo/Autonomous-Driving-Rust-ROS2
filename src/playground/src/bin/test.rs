use std::sync::mpsc::channel;
use std::thread;
use std::time::Duration;

fn main() {
    let (tx, rx) = channel();

    let handle = thread::spawn(move || {
        while let Ok(number) = rx.recv() {
            println!("Thread received: {}", number);
        }
    });

    // Add small delay to see the back-and-forth
    for i in 1..=5 {
        tx.send(i).unwrap();
        println!("Main sent: {}", i);
        thread::sleep(Duration::from_millis(100)); // Add delay
    }

    drop(tx);
    handle.join().unwrap();
}
