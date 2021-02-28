use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use std::{fs, include_bytes, str};

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ArmConfig {
    pub base_id: u8,
    pub shoulder_id: u8,
    pub elbow_id: u8,
    pub wrist_id: u8,
    pub gripper_id: u8,
    pub shoulder: Vector3<f32>,
    pub elbow: Vector3<f32>,
    pub wrist: Vector3<f32>,
    pub end_effector: Vector3<f32>,
}

impl ArmConfig {
    /// Arm configuration with default values
    pub fn default() -> ArmConfig {
        ArmConfig {
            base_id: 1,
            shoulder_id: 2,
            elbow_id: 3,
            wrist_id: 4,
            gripper_id: 5,
            shoulder: Vector3::new(0.0, 0.0, 0.0),
            elbow: Vector3::new(0.0, 0.0, 0.0),
            wrist: Vector3::new(0.0, 0.0, 0.0),
            end_effector: Vector3::new(0.0, 0.0, 0.0),
        }
    }

    pub fn get_ids(&self) -> [u8; 5] {
        [
            self.base_id,
            self.shoulder_id,
            self.elbow_id,
            self.wrist_id,
            self.gripper_id,
        ]
    }

    /// Guppy comes with an included config file.
    ///
    /// This file is packaged with the binary
    /// This method retrieves this included version
    pub fn included() -> ArmConfig {
        let json = str::from_utf8(include_bytes!("../config/guppy.json")).unwrap();
        ArmConfig::parse_json(json).unwrap()
    }

    pub fn parse_json(text: &str) -> Result<ArmConfig, Box<dyn std::error::Error>> {
        let config: ArmConfig = serde_json::from_str(&text)?;
        Ok(config)
    }

    pub fn parse_yaml(text: &str) -> Result<ArmConfig, Box<dyn std::error::Error>> {
        let config: ArmConfig = serde_yaml::from_str(&text)?;
        Ok(config)
    }

    pub fn serialize_to_json(&self) -> Result<String, Box<dyn std::error::Error>> {
        let json = serde_json::to_string_pretty(self)?;
        Ok(json)
    }

    pub fn serialize_to_yaml(&self) -> Result<String, Box<dyn std::error::Error>> {
        let yaml = serde_yaml::to_string(self)?;
        Ok(yaml)
    }

    pub fn save_json(&self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        fs::write(path, &self.serialize_to_json()?)?;
        Ok(())
    }

    pub fn save_yaml(&self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        fs::write(path, &self.serialize_to_yaml()?)?;
        Ok(())
    }

    pub fn load_json(path: &str) -> Result<ArmConfig, Box<dyn std::error::Error>> {
        let text = fs::read_to_string(path)?;
        let config = ArmConfig::parse_json(&text)?;
        Ok(config)
    }

    pub fn load_yaml(path: &str) -> Result<ArmConfig, Box<dyn std::error::Error>> {
        let text = fs::read_to_string(path)?;
        let config = ArmConfig::parse_yaml(&text)?;
        Ok(config)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DEFAULT_JSON: &str = "{\"base_id\":1,\"shoulder_id\":2,\"elbow_id\":3,\"wrist_id\":4,\"gripper_id\":5,\"shoulder\":[0,0,0.0],\"elbow\":[0.0,0,0.0],\"wrist\":[0.0,0,0],\"end_effector\":[0,0,0.0]}";

    #[test]
    fn parse_from_json() {
        let json = DEFAULT_JSON;
        let config = ArmConfig::parse_json(json).unwrap();
        assert_eq!(config, ArmConfig::default());
    }

    #[test]
    fn parse_from_yaml() {
        let yaml = DEFAULT_JSON;
        let config = ArmConfig::parse_yaml(yaml).unwrap();
        assert_eq!(config, ArmConfig::default());
    }

    #[test]
    fn serialize_to_json() {
        let config = ArmConfig::default();
        let json = config.serialize_to_json().unwrap();
        let parsed_config = ArmConfig::parse_json(&json).unwrap();
        assert_eq!(config, parsed_config);
    }

    #[test]
    fn serialize_to_yaml() {
        let config = ArmConfig::default();
        let yaml = config.serialize_to_yaml().unwrap();
        let parsed_config = ArmConfig::parse_yaml(&yaml).unwrap();
        assert_eq!(config, parsed_config);
    }

    #[test]
    fn check_included() {
        let _ = ArmConfig::included();
    }
}
