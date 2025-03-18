import requests


textToSpeak = "Questo script Python invia una richiesta GET a un server Piper locale per generare un file audio. Il testo da convertire in parlato è definito nella variabile 'textToSpeak'. L'URL del server Piper è specificato nella variabile 'urlPiper'. La risposta del server, che contiene il file audio, viene salvata in 'outputFilename'. La richiesta viene inviata utilizzando la libreria 'requests' con il testo come parametro. La risposta viene letta in blocchi di 128 byte e scritta nel file di output. Questo script è utile per generare audio parlato da testo in modo programmatico."
#textToSpeak = "Ciao io sono sbem il tuo robot personale, posso fare foto, dire barzellette, raccontare storie e molto altro. Come posso aiutarti oggi?"
urlPiper = "http://[::1]:5002"
outputFilename = "output.wav"

payload = {'text': textToSpeak}

r = requests.get(urlPiper,params=payload)

with open(outputFilename, 'wb') as fd:
    for chunk in r.iter_content(chunk_size=128):
        fd.write(chunk)