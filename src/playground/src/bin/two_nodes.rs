use std::any::Any;
use std::collections::HashMap;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

pub trait Component: Send + Sync + 'static {
    fn start(&mut self) -> Result<(), String>;
    fn stop(&mut self) -> Result<(), String>;
    fn get_name(&self) -> &str;
    fn as_any(&self) -> &dyn Any;
}

pub struct PublisherComponent {
    name: String,
    running: Arc<Mutex<bool>>,
    counter: Arc<Mutex<i32>>,
    sender: Option<Sender<i32>>,
}

impl PublisherComponent {
    pub fn new(name: &str, sender: Sender<i32>) -> Self {
        Self {
            name: name.to_string(),
            running: Arc::new(Mutex::new(false)),
            counter: Arc::new(Mutex::new(0)),
            sender: Some(sender),
        }
    }
}

impl Component for PublisherComponent {
    fn start(&mut self) -> Result<(), String> {
        let mut running = self.running.lock().unwrap();
        if *running {
            return Err("Component already running".to_string());
        }
        *running = true;

        let running_clone = Arc::clone(&self.running);
        let counter_clone = Arc::clone(&self.counter);
        let name_clone = self.name.clone();
        let sender = self.sender.clone().unwrap();

        thread::spawn(move || {
            while *running_clone.lock().unwrap() {
                let mut counter = counter_clone.lock().unwrap();
                *counter += 1;
                println!("Publisher '{}' sending: {}", name_clone, *counter);
                sender
                    .send(*counter)
                    .unwrap_or_else(|_| println!("Failed to send"));
                drop(counter);
                thread::sleep(Duration::from_secs(1));
            }
        });

        println!("Publisher '{}' started", self.name);
        Ok(())
    }

    fn stop(&mut self) -> Result<(), String> {
        let mut running = self.running.lock().unwrap();
        if !*running {
            return Err("Component not running".to_string());
        }
        *running = false;
        println!("Publisher '{}' stopped", self.name);
        Ok(())
    }

    fn get_name(&self) -> &str {
        &self.name
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

pub struct ListenerComponent {
    name: String,
    running: Arc<Mutex<bool>>,
    receiver: Arc<Mutex<Option<Receiver<i32>>>>, // Wrapped in Arc<Mutex>
}

impl ListenerComponent {
    pub fn new(name: &str, receiver: Receiver<i32>) -> Self {
        Self {
            name: name.to_string(),
            running: Arc::new(Mutex::new(false)),
            receiver: Arc::new(Mutex::new(Some(receiver))),
        }
    }
}

impl Component for ListenerComponent {
    fn start(&mut self) -> Result<(), String> {
        let mut running = self.running.lock().unwrap();
        if *running {
            return Err("Component already running".to_string());
        }
        *running = true;

        let running_clone = Arc::clone(&self.running);
        let name_clone = self.name.clone();
        let receiver_clone = Arc::clone(&self.receiver);

        thread::spawn(move || {
            // Take the receiver out of the Option
            let receiver = receiver_clone.lock().unwrap().take().unwrap();

            while *running_clone.lock().unwrap() {
                match receiver.recv_timeout(Duration::from_secs(1)) {
                    Ok(value) => println!("Listener '{}' received: {}", name_clone, value),
                    Err(_) => continue,
                }
            }
        });

        println!("Listener '{}' started", self.name);
        Ok(())
    }

    fn stop(&mut self) -> Result<(), String> {
        let mut running = self.running.lock().unwrap();
        if !*running {
            return Err("Component not running".to_string());
        }
        *running = false;
        println!("Listener '{}' stopped", self.name);
        Ok(())
    }

    fn get_name(&self) -> &str {
        &self.name
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

pub struct ComponentContainer {
    components: Arc<Mutex<HashMap<String, Box<dyn Component>>>>,
}

impl ComponentContainer {
    pub fn new() -> Self {
        Self {
            components: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    pub fn load_component(&self, component: Box<dyn Component>) -> Result<(), String> {
        let name = component.get_name().to_string();
        let mut components = self.components.lock().unwrap();

        if components.contains_key(&name) {
            return Err(format!("Component '{}' already exists", name));
        }

        components.insert(name.clone(), component);
        println!("Loaded component: {}", name);
        Ok(())
    }

    pub fn unload_component(&self, name: &str) -> Result<(), String> {
        let mut components = self.components.lock().unwrap();

        if let Some(mut component) = components.remove(name) {
            component.stop()?;
            println!("Unloaded component: {}", name);
            Ok(())
        } else {
            Err(format!("Component '{}' not found", name))
        }
    }

    pub fn start_component(&self, name: &str) -> Result<(), String> {
        let mut components = self.components.lock().unwrap();

        if let Some(component) = components.get_mut(name) {
            component.start()?;
            Ok(())
        } else {
            Err(format!("Component '{}' not found", name))
        }
    }

    pub fn list_components(&self) -> Vec<String> {
        let components = self.components.lock().unwrap();
        components.keys().cloned().collect()
    }
}

fn main() {
    let container = ComponentContainer::new();

    let (sender, receiver) = channel();

    let publisher = PublisherComponent::new("talker", sender);
    let listener = ListenerComponent::new("listener", receiver);

    container.load_component(Box::new(publisher)).unwrap();
    container.load_component(Box::new(listener)).unwrap();

    container.start_component("talker").unwrap();
    container.start_component("listener").unwrap();

    println!("Loaded components: {:?}", container.list_components());

    thread::sleep(Duration::from_secs(10));

    container.unload_component("listener").unwrap();
    container.unload_component("talker").unwrap();
}
