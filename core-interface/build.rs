use std::io::Result;

fn main() -> Result<()> {
    println!("cargo:rerun-if-changed=../contracts/opencar/core/v1/core.proto");
    println!("cargo:rerun-if-changed=../contracts/opencar/core/v1/system.proto");

    prost_build::compile_protos(
        &["../contracts/opencar/core/v1/core.proto"],
        &["../contracts/"],
    )?;
    Ok(())
}
