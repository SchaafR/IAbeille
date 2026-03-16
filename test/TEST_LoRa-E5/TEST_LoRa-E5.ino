const char* DevEUI = "70B3D57ED00761C6";
const char* appEui = "0000000000000000";
const char* appKey = "73D3232B6CA32A68C5695B8DCC3E4672"; 

void setup() {
  Serial.begin(9600);
  while (!Serial);
  
  Serial1.begin(9600);
  Serial.println("Connexion");

  delay(2000);

  if (connectToTTN()) {
    Serial.println("CONNEXION REUSSIE");
  } else {
    Serial.println("ATTENTE OU ECHEC");
  }
}

void loop() {
  Serial.println("Attente");
  delay(10000); 

  String payload = "192826050258";
  Serial.print("Tentative d'envoi des données : ");
  Serial.println(payload);

  Serial1.print("AT+MSGHEX=");
  Serial1.println(payload);

  unsigned long start = millis();
  while (millis() - start < 5000) {
    if (Serial1.available()) {
      String resp = Serial1.readString();
      Serial.print("Réponse module : ");
      Serial.print(resp);

      if (resp.indexOf("Done") != -1) {
        Serial.println("Envoie effectué");
      }
    }
  }

  Serial.println("Repos");
  delay(60000); 
}

bool connectToTTN() {
  while(Serial1.available()) Serial1.read();
  
  Serial1.print("AT+ID=DevEUI,\""); 
  Serial1.print(DevEUI); 
  Serial1.println("\"");
  delay(500);

  Serial1.print("AT+ID=AppEUI,\""); 
  Serial1.print(appEui); 
  Serial1.println("\"");
  delay(500);

  Serial1.print("AT+KEY=APPKEY,\""); 
  Serial1.print(appKey); 
  Serial1.println("\"");
  delay(500);

  Serial1.println("AT+MODE=LWOTAA");
  delay(500);

  Serial1.println("AT+DR=EU868"); 
  delay(500);
  
  Serial1.println("AT+JOIN");
  
  unsigned long timeout = millis() + 30000;
  while (millis() < timeout) {
    if (Serial1.available()) {
      String line = Serial1.readString();
      Serial.print(line); 
      if (line.indexOf("joined") != -1) return true;
      if (line.indexOf("Join failed") != -1) return false;
    }
  }
  return false;
}