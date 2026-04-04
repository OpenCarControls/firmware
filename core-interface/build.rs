use std::io::Result;

fn main() -> Result<()> {
    // Tell cargo to re-run this script if the proto file changes
    println!("cargo:rerun-if-changed=../contracts/opencar/core/v1/core.proto");
    
    // Compile the core.proto file
    prost_build::compile_protos(&["../contracts/opencar/core/v1/core.proto"], &["../contracts/"])?;
    Ok(())
}
