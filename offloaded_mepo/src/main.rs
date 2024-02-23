use std::io::{Read, Write};
use std::process::{Command, Stdio};

fn main() {
    let mepo = Command::new("mepo")
        .arg("-i")
        .stdin(Stdio::piped())
        .stdout(Stdio::null())
        .spawn()
        .unwrap();
    let mut mepo_input = mepo.stdin.as_ref().unwrap();

    mepo_input
        .write_all(b"prefset_n lat 70.0100; prefset_n lon -70.0020; prefset_n zoom 14;\n")
        .unwrap();

    let mut mcu = serialport::new("/dev/ttyACM0", 9600).open();
    while mcu.is_err() {
        mcu = serialport::new("/dev/ttyACM0", 9600).open();
    }
    println!("opened mcu");
    let mut mcu = mcu.unwrap();
    let mut command = [0; 255];

    loop {
        if let Ok(len) = mcu.read(&mut command) {
            let command_str = std::str::from_utf8(&command[..len]).unwrap();
            println!("received command: {}", command_str);

            mepo_input
                .write_fmt(format_args!("{command_str}\n"))
                .unwrap();

            command = [0; 255];
        }
    }
}
