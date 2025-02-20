use crossterm::{
    event::{self, Event, KeyCode},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode},
};
use std::collections::HashSet;
use std::io::{self, Write};

fn main() {
    enable_raw_mode().unwrap();
    let mut stdout = io::stdout();
    println!("Press ESC to exit.");
    loop {
        let mut pressed_keys = HashSet::new();
        if event::poll(std::time::Duration::from_millis(100)).unwrap() {
            if let Event::Key(key_event) = event::read().unwrap() {
                match key_event.code {
                    KeyCode::Esc => break,
                    _ => {
                        pressed_keys.insert(key_event.code);
                    }
                }
            }
        }
    }
    disable_raw_mode().unwrap();
    println!("Exiting.");
}
