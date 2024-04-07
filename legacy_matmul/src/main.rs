use std::time::Duration;

fn main() {
    let mut mcu = serialport::new("/dev/ttyACM0", 9600).open();
    while mcu.is_err() {
        mcu = serialport::new("/dev/ttyACM0", 9600).open();
    }
    println!("opened mcu");
    let mut mcu = mcu.unwrap();
    mcu.write_all(b"ready").unwrap();

    // Idle for some time
    std::thread::sleep(Duration::from_millis(100));
    mcu.write_all(b"flash").unwrap();

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
    mcu.write_all(b"flash").unwrap();
    std::thread::sleep(Duration::from_millis(100));
    mcu.write_all(b"exit").unwrap();
}
