use reqwest;

pub(crate) async fn say(text: String) -> Result<(), Box<dyn std::error::Error>> {
    #[cfg(not(test))]
    let url = "http://pi4.local:3000/say";
    #[cfg(test)]
    let url = &format!("{}{}", &mockito::server_url(), "/say");

    let client = reqwest::Client::new();
    let _ = client.post(url).body(text).send().await?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mockito::{mock, Matcher};

    #[tokio::test]
    async fn speech_service_posts() {
        let mock_tts_api = mock("POST", "/say")
            .match_body(Matcher::Exact("test message".to_owned()))
            .with_status(200)
            .create();

        say("test message".to_owned()).await.unwrap();

        mock_tts_api.assert();
    }
}
