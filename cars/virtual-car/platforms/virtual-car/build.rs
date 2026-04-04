use std::io::Result;

fn main() -> Result<()> {
    let proto_path = "../../../../contracts/opencar/cars/virtual_car/v1/virtual_car.proto";
    let include_path = "../../../../contracts/";

    println!("cargo:rerun-if-changed={}", proto_path);
    
    prost_build::compile_protos(&[proto_path], &[include_path])?;
    Ok(())
}
