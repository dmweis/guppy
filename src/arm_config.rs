use serde::{Deserialize, Serialize};
use std::fs;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ArmConfig {
    base_id: u8,
    shoulder_id: u8,
    elbow_id: u8,
    wrist_id: u8,
}

impl ArmConfig {
    /// Default values are
    ///
    /// - base_id: 1,
    /// - shoulder_id: 3,
    /// - elbow_id: 2,
    /// - wrist_id: 4,
    pub fn default() -> ArmConfig {
        ArmConfig {
            base_id: 1,
            shoulder_id: 3,
            elbow_id: 2,
            wrist_id: 4,
        }
    }

    pub fn parse_json(text: &str) -> Result<ArmConfig, Box<dyn std::error::Error>> {
        let config: ArmConfig = serde_json::from_str(&text)?;
        Ok(config)
    }

    pub fn load_json(path: &str) -> Result<ArmConfig, Box<dyn std::error::Error>> {
        let text = fs::read_to_string(path)?;
        let config = ArmConfig::parse_json(&text)?;
        Ok(config)
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simple_tts_request() {
        let json = "{
            \"base_id\": 1,
            \"shoulder_id\": 3,
            \"elbow_id\": 2,
            \"wrist_id\": 4
        }";
        let config = ArmConfig::parse_json(json).unwrap();
        assert_eq!(config, ArmConfig::default());
    }
}
