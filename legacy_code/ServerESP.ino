#include <ESPAsyncWebServer.h>
#include <WiFi.h>

const char* ssid = "BadBitches";
const char* password = "12345678";
int button;

#define bumpIn 4

AsyncWebServer server(80);

String readBump() {
  return String(digitalRead(bumpIn));
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(bumpIn, INPUT);
  
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.on("/bump", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readBump().c_str());
  });

  server.begin();
}


void loop() {
  // put your main code here, to run repeatedly:
  button = digitalRead(bumpIn);
  Serial.println(button);
//  Serial.println(IP);
  delay(1000);
}
