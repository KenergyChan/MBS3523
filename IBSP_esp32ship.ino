#include "WiFi.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include <ESPAsyncWebServer.h>
#include <StringArray.h>
#include <SPIFFS.h> 
#include <FS.h>
#include <AsyncTCP.h>
#include <driver/ledc.h>

// Replace with your network credentials
const char* ssid = "kenergyChan";
const char* password = "69352187";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

unsigned long previousMillis = 0;
unsigned long interval = 20000;

#define left_motor_PWM_pin 12
#define right_motor_PWM_pin 15
int SPEED = 40;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { text-align:center; }
    .vert { margin-bottom: 10%; }
    .hori{ margin-bottom: 0%; }
  </style>
</head>
<body>
  <div id="container">
    <h2>ESP32-CAM ship control</h2>
    <p>Chan Tsz Kin IBSP project</p>
    <div>
    <p>
      <button onclick="forward();">Forward</button>
      <button onclick="stop();">Stop</button>
      <button onclick="turn_left();">Turn left</button>
      <button onclick="turn_right();">Turn right</button>     
    </p>
  </div>
    <div>
    <button onclick="lowSpeed();">Low</button>
    <button onclick="midSpeed();">Middle</button>
    <button onclick="highSpeed();">High</button>
  </div>
    <div>
      <button onclick="location.reload();">REFRESH PAGE</button>    
  </div>
  <label id="wifi-strength">Wifi strength: </label>
  <div><img src="saved-photo" id="photo" width="70%"></div>
</body>
<script>
  var deg = 0;
  function forward() {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', "/forward", true);
    xhr.send(); 
  }
  function stop() {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', "/stop", true);
    xhr.send(); 
  }
  function turn_left() {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', "/turn_left", true);
    xhr.send(); 
  }
  function turn_right() {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', "/turn_right", true);
    xhr.send(); 
  }
  function lowSpeed() {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', "/low", true);
    xhr.send(); 
  }
  function midSpeed() {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', "/middle", true);
    xhr.send(); 
  }
  function highSpeed() {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', "/high", true);
    xhr.send(); 
  }
  function updateWifiStrength() {
      var xhr = new XMLHttpRequest();
      xhr.open('GET', "/wifi_strength", true);
      xhr.onreadystatechange = function() {
        if (this.readyState === XMLHttpRequest.DONE && this.status === 200) {
          document.getElementById("wifi-strength").innerHTML = "Wifi strength: " + this.responseText;
        }
      };
      xhr.send();
    }
    setInterval(updateWifiStrength, 1000);
  function isOdd(n) { return Math.abs(n % 2) == 1; }
</script>
</html>)rawliteral";

void handleWifiStrength(AsyncWebServerRequest *request) {
  int wifiStrength = WiFi.RSSI();
  String response = String(wifiStrength);
  request->send(200, "text/plain", response);
}

void setup() {
  ledcSetup(2, 5000, 8);
  ledcSetup(4, 5000, 8);

  // attach LEDC channel to pin
  ledcAttachPin(left_motor_PWM_pin, 2);
  ledcAttachPin(right_motor_PWM_pin, 4);

  // Serial port for debugging purposes
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  // Print ESP32 Local IP Address
  Serial.print("IP Address: http://");
  Serial.println(WiFi.localIP());

  // Turn-off the 'brownout detector'
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/html", index_html);
  });

  server.on("/forward", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(200, "text/plain", "Moving forward");
      movement('w');
  });  

  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(200, "text/plain", "Stopped");
      movement('s');
  });

  server.on("/turn_left", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(200, "text/plain", "Turning left");
      movement('a');
  });

  server.on("/turn_right", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(200, "text/plain", "Turning right");
      movement('d');
  });

  server.on("/high", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(200, "text/plain", "Turn left");
      SPEED = 90;     
  });  

  server.on("/middle", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(200, "text/plain", "Turn right");
      SPEED = 60;
  });

  server.on("/low", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(200, "text/plain", "Low speed");
      SPEED = 30;
  });  

  server.on("/wifi_strength", HTTP_GET, handleWifiStrength);

  // Start server
  server.begin();

  ledcWrite(2, 0);
  ledcWrite(4, 0);
}

void movement(char mov){
  if (mov == 'w'){
    ledcWrite(2, SPEED);
    ledcWrite(4, SPEED);
    delay(1000);
  }
  if (mov == 's'){
    ledcWrite(2, 0);
    ledcWrite(4, 0);
    delay(1000);
  }
  if (mov == 'a'){
    ledcWrite(2, 0);
    ledcWrite(4, SPEED);
    delay(1000);
  }
  if (mov == 'd'){
    ledcWrite(2, SPEED);
    ledcWrite(4, 0);
    delay(1000);
  }
}


void loop() {
  unsigned long currentMillis = millis();
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= interval)){
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;
  }

  delay(1);
}

