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

    // 2. Compile all DBCs (if the dbc directory exists)
    let dbc_dir = workspace_dir.join(format!("contracts/opencar/cars/{}/dbc", car_name));
    if dbc_dir.exists() && dbc_dir.is_dir() {
        println!("cargo:rerun-if-changed={}", dbc_dir.display());
        
        let out_dir = env::var("OUT_DIR").expect("OUT_DIR not set");
        let out_dir_path = PathBuf::from(&out_dir);
        let mut index_content = String::new();
        
        for entry in fs::read_dir(&dbc_dir).expect("Failed to read dbc directory") {
            let entry = entry.expect("Failed to read dir entry");
            let path = entry.path();
            if path.is_file() && path.extension().map_or(false, |ext| ext == "dbc") {
                println!("cargo:rerun-if-changed={}", path.display());
                
                let file_stem = path.file_stem().unwrap().to_str().unwrap().replace("-", "_");
                let dest_path = out_dir_path.join(format!("{}.rs", file_stem));
                
                let dbc_content = fs::read(&path).expect("Failed to read DBC file");
                let mut buffer = Vec::new();
                dbc_codegen::codegen(
                    path.file_name().unwrap().to_str().unwrap(),
                    &dbc_content,
                    &mut buffer,
                    false,
                ).expect("Failed to generate DBC rust code");
                
                let generated_str = String::from_utf8(buffer).expect("Invalid UTF-8 from dbc-codegen");
                let fixed_str = generated_str
                    .replace("#![", "// #![")
                    .replace("//!", "///");
                
                fs::write(&dest_path, fixed_str).expect("Failed to write DBC module");
                
                index_content.push_str(&format!(
                    "pub mod {} {{\n    include!(concat!(env!(\"OUT_DIR\"), \"/{}.rs\"));\n}}\n",
                    file_stem, file_stem
                ));
            }
        }
        
        if !index_content.is_empty() {
            let index_path = out_dir_path.join("messages.rs");
            fs::write(index_path, index_content).expect("Failed to write messages.rs index");
        }
    }
}
