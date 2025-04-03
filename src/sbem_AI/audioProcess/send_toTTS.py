import requests



#Simple class to send a request to a TTS server and save the audio content to a file


class SenderTTS:
    """Class to manage local request to TTS server"""
    def __init__(self, typeTTS):
        
        self.typeTTS = typeTTS

        if self.typeTTS == "coqui":
            self.urlTTS = "http://localhost:5002/api/tts"
        elif self.typeTTS == "piper":
            self.urlTTS = "http://localhost:5000" #TODO: check the correct port
        else:
            print("Error: TTS type not supported")
            self.urlTTS = None
    

    def sendRequest(self, textToSpeak):
        """"Sends a request to the TTS server and returns the audio content"""
        # define parameters for the request based on the TTS type
        if(self.typeTTS == "coqui"):
            params = {
                "text": textToSpeak,  # Text to be synthesized
                "speaker_id": "Daisy Studious",  # Specify a speaker ID from the html interface drop-down
                "style_wav": "",   # Optional: specify a style WAV if needed
                "language_id": "it"  # Optiona(?)l: specify a language ID if needed
            }
            
        elif(self.typeTTS == "piper"):
            params = {'text': textToSpeak}

        #try to send the request
        try:
            response = requests.get(self.urlTTS, params=params)
            
            # Check if the request was successful
            if response.status_code == 200:
                return response.content
            else:
                print(f"Failed to get audio. Status code: {response.status_code}")
                print("Response:", response.text)
                return None
        
        except requests.exceptions.RequestException as e:
            print("An error occurred:", e)
            return None
        