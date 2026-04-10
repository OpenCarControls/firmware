use std::env;
use std::io::Result;
use std::path::PathBuf;

fn main() -> Result<()> {
    // CARGO_MANIFEST_DIR is cars/virtual-car/platforms/virtual-car — go up 4 levels to workspace root
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let workspace_root = manifest_dir.ancestors().nth(4).unwrap();

    let contracts = workspace_root.join("contracts");
    let proto = contracts.join("opencar/cars/virtual_car/v1/virtual_car.proto");
    let meta = contracts.join("opencar/cars/virtual_car/v1/meta.toml");

    println!("cargo:rerun-if-changed={}", proto.display());
    println!("cargo:rerun-if-changed={}", meta.display());

    prost_build::compile_protos(&[&proto], &[&contracts])?;
    Ok(())
}
