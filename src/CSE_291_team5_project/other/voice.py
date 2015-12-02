import speech_recognition as sr

# obtain audio from the microphone
r = sr.Recognizer()

while(1):    
    with sr.Microphone() as source:
        print("Say something!")
        audio = r.listen(source)
    try:            
        text = r.recognize_google(audio)
        print text
    except:
        pass
