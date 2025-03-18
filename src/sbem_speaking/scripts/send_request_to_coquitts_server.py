import requests


textToSpeak = "Questo script Python invia una richiesta GET a un server Piper locale per generare un file audio. Il testo da convertire in parlato è definito nella variabile 'textToSpeak'. L'URL del server Piper è specificato nella variabile 'urlPiper'. La risposta del server, che contiene il file audio, viene salvata in 'outputFilename'. La richiesta viene inviata utilizzando la libreria 'requests' con il testo come parametro. La risposta viene letta in blocchi di 128 byte e scritta nel file di output. Questo script è utile per generare audio parlato da testo in modo programmatico."
#textToSpeak = "Ciao io sono sbem il tuo robot personale, posso fare foto, dire barzellette, raccontare storie e molto altro. Come posso aiutarti oggi?"
urlCoqui = "http://localhost:5002/api/tts"


params = {
    "text": textToSpeak,  # Text to be synthesized
    "speaker_id": "Daisy Studious",  # Specify a speaker ID from the html interface drop-down
    "style_wav": "",   # Optional: specify a style WAV if needed
    "language_id": "it"  # Optiona(?)l: specify a language ID if needed
}


try:
    response = requests.get(urlCoqui, params=params)

    # Check if the request was successful
    if response.status_code == 200:
        print("Audio content received")
        # Save the audio content to a file
        with open("output.wav", "wb") as f:
            f.write(response.content)
        print("Audio file saved as output.wav")
    else:
        print(f"Failed to get audio. Status code: {response.status_code}")
        print("Response:", response.text)
except requests.exceptions.RequestException as e:
    print("An error occurred:", e)