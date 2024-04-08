use serial2::SerialPort;
use std::time::Duration;

fn main() {
    let mut butler = SerialPort::open("/dev/ttyACM0", 9600);
    let mut buffer = [0u8; 255];
    while butler.is_err() {
        butler = SerialPort::open("/dev/ttyACM0", 9600);
    }
    println!("opened mcu");
    let butler = butler.unwrap();
    butler.write_all(b"ready").unwrap();

    // Wait for the butler to respond
    loop {
        match butler.read(&mut buffer) {
            Ok(bytes) => {
                let received = core::str::from_utf8(&buffer[..bytes]).unwrap().trim();
                if received == "ready" {
                    break;
                }
            }
            Err(_) => {}
        }
    }
    butler.write_all(b"flash").unwrap();

    // Idle for some time
    std::thread::sleep(Duration::from_millis(500));
    butler.write_all(b"flash").unwrap();

    // Then do matmul on 1024x1024
    const SIZE: usize = 1024;
    const A: [u8; SIZE * SIZE] = [1u8; SIZE * SIZE];
    const B: [u8; SIZE * SIZE] = [2u8; SIZE * SIZE];
    let mut c = [0u8; SIZE * SIZE];

    for i in 0..SIZE {
        for k in 0..SIZE {
            for j in 0..SIZE {
                c[i * SIZE + j] =
                    c[i * SIZE + j].wrapping_add(A[i * SIZE + k].wrapping_mul(B[k * SIZE + j]));
            }
        }
    }
    butler.write_all(b"flash").unwrap();

    std::thread::sleep(Duration::from_millis(500));
    butler.write_all(b"exit").unwrap();
}
