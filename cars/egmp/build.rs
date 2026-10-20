use std::io::Result;

fn main() -> Result<()> {
    build_utils::compile_car_contracts("egmp");
    Ok(())
}
