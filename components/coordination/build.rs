fn main() -> Result<(), Box<dyn std::error::Error>> {
    tonic_build::compile_protos("../picar-wheels/proto/picar.proto")?;
    tonic_build::compile_protos("../localization-coordination/interface.proto")?;
    Ok(())
}
