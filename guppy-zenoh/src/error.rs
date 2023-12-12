use thiserror::Error;

#[derive(Error, Debug)]
pub enum WrapperError {
    #[error("Zenoh error {0:?}")]
    ZenohError(#[from] zenoh::Error),
}
