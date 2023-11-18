use anyhow::Result;
use clap::Parser;
use guppy_grpc::grpc_controller::{
    connect_to_arm, GuppyConfigHandler, GuppyConfigureServer, GuppyControllerHandler,
    GuppyControllerServer,
};
use std::sync::Arc;
use tonic::transport::Server;

#[derive(Parser)]
#[command(author, version)]
struct Args {
    /// Serial port to use
    #[arg()]
    port: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    let address = "0.0.0.0:5002".parse().unwrap();
    let driver = connect_to_arm(&args.port).await?;
    let config_handler = GuppyConfigHandler::new(Arc::clone(&driver));
    let controller_handler = GuppyControllerHandler::new(driver);

    println!("Starting service at {}", address);

    Server::builder()
        .add_service(GuppyConfigureServer::new(config_handler))
        .add_service(GuppyControllerServer::new(controller_handler))
        .serve(address)
        .await?;

    Ok(())
}
