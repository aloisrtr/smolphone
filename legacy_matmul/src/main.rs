use clap::Parser;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    pub size: usize,
}

fn main() {
    let Args { size } = Args::parse();
    println!("Simple matrix multiplication on legacy");
    println!("matrices size: {size}");
    let a = vec![1f32; size * size];
    let b = vec![2f32; size * size];
    let mut c = vec![0f32; size * size];

    for i in 0..size {
        for k in 0..size {
            for j in 0..size {
                c[i * size + j] += a[i * size + k] * b[k * size + j];
            }
        }
    }
}
