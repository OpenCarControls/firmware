use std::env;
use std::fs;
use std::path::PathBuf;

pub fn compile_car_contracts(car_name: &str) {
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR not set");
    let workspace_dir = PathBuf::from(manifest_dir).join("..").join("..");

    // 1. Compile Protobuf Definitions
    let proto_path = workspace_dir.join(format!("contracts/opencar/cars/{}/v1/{}.proto", car_name, car_name));
    if proto_path.exists() {
        println!("cargo:rerun-if-changed={}", proto_path.display());
        println!("cargo:rerun-if-changed={}", workspace_dir.join("contracts/opencar/core/v1/vehicle_state.proto").display());
        
        prost_build::Config::new()
            .compile_protos(
                &[proto_path],
                &[workspace_dir.join("contracts/opencar")]
            )
            .expect("Failed to compile protobuf files");
    }

    // 2. Compile DBC (if it exists)
    let dbc_path = workspace_dir.join(format!("contracts/opencar/cars/{}/dbc/{}.dbc", car_name, car_name));
    if dbc_path.exists() {
        println!("cargo:rerun-if-changed={}", dbc_path.display());
        let dbc_content = fs::read(&dbc_path).expect("Failed to read DBC file");
        
        let out_dir = env::var("OUT_DIR").expect("OUT_DIR not set");
        let dest_path = PathBuf::from(out_dir).join("messages.rs");
        let mut buffer = Vec::new();
        dbc_codegen::codegen(
            &format!("{}.dbc", car_name),
            &dbc_content,
            &mut buffer,
            false,
        ).expect("Failed to generate DBC rust code");
        
        let generated_str = String::from_utf8(buffer).expect("Invalid UTF-8 from dbc-codegen");
        let fixed_str = generated_str
            .replace("#![", "// #![")
            .replace("//!", "///");
        
        fs::write(dest_path, fixed_str).expect("Failed to write messages.rs");
    }
}
