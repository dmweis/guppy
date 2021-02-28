fn main() {
    tonic_build::compile_protos("proto/guppy/guppy_service.proto").unwrap();
}
