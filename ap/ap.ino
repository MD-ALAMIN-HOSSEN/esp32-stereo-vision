#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include "esp_camera.h"

// CAMERA MODEL
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// === CONFIG ===
const char* ssid = "ESP32-CAM-AP";
const char* password = "12345678";
IPAddress clientIP(192,168,4,2);

#define SRC_WIDTH  160
#define SRC_HEIGHT 120
#define IMG_WIDTH   80
#define IMG_HEIGHT  60
#define HALF_WIDTH  40
int ColumnSums[56];

//
#define WINDOW_SIZE 5
#define HALF_WINDOW (WINDOW_SIZE / 2)

#define DS_WIDTH 80
#define DS_HEIGHT 60
#define DS_HALF_WIDTH 40

// === BUFFERS ===
uint8_t downsampledFrame[IMG_WIDTH * IMG_HEIGHT];
uint8_t apLeftHalf[IMG_HEIGHT * HALF_WIDTH];
uint8_t apRightHalf[IMG_HEIGHT * HALF_WIDTH];
uint8_t clientRightHalf[IMG_HEIGHT * HALF_WIDTH];
//uint8_t depthMap[IMG_WIDTH * IMG_HEIGHT];


//
uint8_t worldXmap[DS_HEIGHT * DS_HALF_WIDTH];
uint8_t worldYmap[DS_HEIGHT * DS_HALF_WIDTH];
float worldZmap[DS_HEIGHT * DS_HALF_WIDTH];

bool startDepthProcess;



WebServer server(80);
WiFiClient client;

void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size   = FRAMESIZE_QQVGA; // 160x120
  config.jpeg_quality = 12;
  config.fb_count     = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    while (1);
  }
}

/////////////////////////////////
void printClientRightHalf() {
  Serial.println("==== clientRightHalf (40x60) ====");

  for (int y = 0; y < IMG_HEIGHT; y++) {
    for (int x = 0; x < HALF_WIDTH; x++) {
      int index = y * HALF_WIDTH + x;
      Serial.print(clientRightHalf[index]);
      Serial.print("\t"); // Tab-separated for readability
    }
    Serial.println(); // Newline after each row
  }

  Serial.println("=================================");
}


void fetchClientRightHalf() {
  HTTPClient http;
  http.begin(client, "http://192.168.4.2/send_client_right");  // client IP must be correct
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    String body = http.getString();

    int idx = 0, start = 0;
    while (idx < IMG_HEIGHT * HALF_WIDTH && start < body.length()) {
      int end = body.indexOf(',', start);
      if (end == -1) end = body.length();
      clientRightHalf[idx++] = body.substring(start, end).toInt();
      start = end + 1;
    }

    Serial.println("✅ Right half received from client.");
  } else {
    Serial.printf("❌ Failed to fetch client right half. HTTP code: %d\n", httpCode);
  }

  http.end();
}


void downsample2x2(uint8_t* src, uint8_t* dest) {
  for (int y = 0; y < IMG_HEIGHT; y++) {
    for (int x = 0; x < IMG_WIDTH; x++) {
      int sx = x * 2;
      int sy = y * 2;
      uint16_t sum = 0;
      sum += src[ sy      * SRC_WIDTH + sx     ];
      sum += src[ sy      * SRC_WIDTH + (sx+1) ];
      sum += src[ (sy+1)  * SRC_WIDTH + sx     ];
      sum += src[ (sy+1)  * SRC_WIDTH + (sx+1) ];
      dest[ y * IMG_WIDTH + x ] = sum / 4;
    }
  }
}

void captureAndSplit() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Capture failed");
    return;
  }
  downsample2x2(fb->buf, downsampledFrame);
  esp_camera_fb_return(fb);

  for (int y = 0; y < IMG_HEIGHT; y++) {
    memcpy(&apLeftHalf[y * HALF_WIDTH], &downsampledFrame[y * IMG_WIDTH], HALF_WIDTH);
    memcpy(&apRightHalf[y * HALF_WIDTH], &downsampledFrame[y * IMG_WIDTH + HALF_WIDTH], HALF_WIDTH);
  }
}
/////////////////////////geting post value 
void handlePostClientRight() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    int idx = 0, start = 0;

    while (idx < IMG_HEIGHT * HALF_WIDTH && start < body.length()) {
      int end = body.indexOf(',', start);
      if (end == -1) end = body.length();
      apRightHalf[idx] = body.substring(start, end).toInt();
      idx++;
      start = end + 1;
    }

    server.send(200, "text/plain", "clientRightHalf received and stored in apRightHalf");
  } else {
    server.send(400, "text/plain", "Missing payload");
  }
}

//////////////
void calculateColumnSums(int x){
    for (int y = 2; y < IMG_HEIGHT-2; y++) {
        int index = y -2;
        ColumnSums[index] =  apRightHalf[(y-2)*HALF_WIDTH + (x-2)] +
                             apRightHalf[(y-2)*HALF_WIDTH + (x-1)] +
                             apRightHalf[(y-2)*HALF_WIDTH + (x)] +
                             apRightHalf[(y-2)*HALF_WIDTH + (x+1)] +
                             apRightHalf[(y-2)*HALF_WIDTH + (x+2)] +
                             apRightHalf[(y-1)*HALF_WIDTH + (x-2)] +
                             apRightHalf[(y-1)*HALF_WIDTH + (x-1)] +
                             apRightHalf[(y-1)*HALF_WIDTH + (x)] +
                             apRightHalf[(y-1)*HALF_WIDTH + (x+1)] +
                             apRightHalf[(y-1)*HALF_WIDTH + (x+2)] +
                             apRightHalf[y*HALF_WIDTH + (x-2)] +
                             apRightHalf[y*HALF_WIDTH + (x-1)] +
                             apRightHalf[y*HALF_WIDTH + (x)] +
                             apRightHalf[y*HALF_WIDTH + (x+1)] +
                             apRightHalf[y*HALF_WIDTH + (x+2)] +
                             apRightHalf[(y+1)*HALF_WIDTH + (x-2)] +
                             apRightHalf[(y+1)*HALF_WIDTH + (x-1)] +
                             apRightHalf[(y+1)*HALF_WIDTH + (x)] +
                             apRightHalf[(y+1)*HALF_WIDTH + (x+1)] +
                             apRightHalf[(y+1)*HALF_WIDTH + (x+2)] +
                             apRightHalf[(y+2)*HALF_WIDTH + (x-2)] +
                             apRightHalf[(y+2)*HALF_WIDTH + (x-1)] +
                             apRightHalf[(y+2)*HALF_WIDTH + (x)] +
                             apRightHalf[(y+2)*HALF_WIDTH + (x+2)]+
                             apRightHalf[(y+2)*HALF_WIDTH + (x+1)]; 
        
    }
}

int findBestVerticalDisparityRight(int x, int yy, int maxDisparity) {
    int bestDisparity = 0;
    int minSAD = INT32_MAX;

    for (int d = 0; d < maxDisparity; ++d) {
        int y = yy + d;
        int sad = 0;
        //if (y - 2 < 0 || y + 2 >= IMG_HEIGHT) continue;


        int sum= clientRightHalf[(y-2)*HALF_WIDTH + (x-2)] +
            clientRightHalf[(y-2)*HALF_WIDTH + (x-1)] +
            clientRightHalf[(y-2)*HALF_WIDTH + (x)] +
            clientRightHalf[(y-2)*HALF_WIDTH + (x+1)] +
            clientRightHalf[(y-2)*HALF_WIDTH + (x+2)] +
            clientRightHalf[(y-1)*HALF_WIDTH + (x-2)] +
            clientRightHalf[(y-1)*HALF_WIDTH + (x-1)] +
            clientRightHalf[(y-1)*HALF_WIDTH + (x)] +
            clientRightHalf[(y-1)*HALF_WIDTH + (x+1)] +
            clientRightHalf[(y-1)*HALF_WIDTH + (x+2)] +
            clientRightHalf[y*HALF_WIDTH + (x-2)] +
            clientRightHalf[y*HALF_WIDTH + (x-1)] +
            clientRightHalf[y*HALF_WIDTH + (x)] +
            clientRightHalf[y*HALF_WIDTH + (x+1)] +
            clientRightHalf[y*HALF_WIDTH + (x+2)] +
            clientRightHalf[(y+1)*HALF_WIDTH + (x-2)] +
            clientRightHalf[(y+1)*HALF_WIDTH + (x-1)] +
            clientRightHalf[(y+1)*HALF_WIDTH + (x)] +
            clientRightHalf[(y+1)*HALF_WIDTH + (x+1)] +
            clientRightHalf[(y+1)*HALF_WIDTH + (x+2)] +
            clientRightHalf[(y+2)*HALF_WIDTH + (x-2)] +
            clientRightHalf[(y+2)*HALF_WIDTH + (x-1)] +
            clientRightHalf[(y+2)*HALF_WIDTH + (x)] +
            clientRightHalf[(y+2)*HALF_WIDTH + (x+1)] +
            clientRightHalf[(y+2)*HALF_WIDTH + (x+2)];

        sad= abs(ColumnSums[yy-2]-sum);

        if (sad < minSAD) {
            minSAD = sad;
            bestDisparity = d;
        }
    }

    return bestDisparity;
}


float computeDepth(int disparity, float focalLength, float baseline) {
    if (disparity == 0) return 9999.0f;  // max depth cap // avoid division by zero, assume far away

    return (focalLength * baseline) / disparity;
}

// all calculations
void computeRightDepth() {
  float focalLength = 25.0f;  // Adjusted due to 2x downscale
  float baseline = 10.0f;     // cm
  float cx = HALF_WIDTH / 2.0f;
  float cy = IMG_HEIGHT / 2.0f;
  int index = 0;

  for (int x = 2; x < HALF_WIDTH-2; x++) {
    //skip the loop is x is not valid
    //if(x<2 || x >= HALF_WIDTH-2) continue;
    calculateColumnSums(x);

    for (int y = 2; y < IMG_HEIGHT-2; y++) {
    //skip the loop is y is not valid
      //if (y < 2 || y >= IMG_HEIGHT - 2) continue;
      int disp = findBestVerticalDisparityRight(x, y, IMG_HEIGHT - y - 2);  // maxDisparity = 57
      float depth = computeDepth(disp, focalLength, baseline);

      if (depth > 255.0f || disp == 0) depth = 255.0f;

      float worldX = (x - cx) * depth / focalLength;
      float worldY = (y - cy) * depth / focalLength;

      int scaledX = (int)(worldX + 127.5f);
      int scaledY = (int)(worldY + 127.5f);

      scaledX = constrain(scaledX, 0, 255);
      scaledY = constrain(scaledY, 0, 255);

      
      worldXmap[index] = (uint8_t)scaledX;
      worldYmap[index] = (uint8_t)scaledY;
      worldZmap[index] = depth;
        index++;
    }
  }
}


void handleGetDepth() {
  Serial.println("Start depth flow...");

  captureAndSplit();

  // Trigger client capture
  HTTPClient http;
  http.begin(client, "http://192.168.4.2/capture");
  http.GET();
  http.end();

  delay(100);////////////////////////

  // Send LEFT half to client
  HTTPClient http2;
  http2.begin(client, "http://192.168.4.2/receive_left_half");
  String payload = "";
  for (int i = 0; i < IMG_HEIGHT * HALF_WIDTH; i++) {
    payload += String(apLeftHalf[i]);
    if (i < IMG_HEIGHT * HALF_WIDTH - 1) payload += ",";
  }
  http2.POST(payload);
  http2.end();//////////////////////////////////////////

  //get the right half
    // ✅ NEW: Fetch clientRightHalf from client
  fetchClientRightHalf();
  printClientRightHalf();
  //calculate x, y, z values and store

  computeRightDepth();

  //last depth value sending at the end
  // String json = "[";
  // for (int i = 0; i < IMG_WIDTH * IMG_HEIGHT; i++) {
  //   json += String(worldZmap[i]);
  //   if (i < IMG_WIDTH * IMG_HEIGHT - 1) json += ",";
  // }

  String json = "{";
  
  json += "\"X\":[";
  for (int i = 0; i < DS_HEIGHT * DS_HALF_WIDTH; i++) {
    json += worldXmap[i];
    if (i < DS_HEIGHT * DS_HALF_WIDTH - 1) json += ",";
  }
  json += "],\"Y\":[";
  for (int i = 0; i < DS_HEIGHT * DS_HALF_WIDTH; i++) {
    json += worldYmap[i];
    if (i < DS_HEIGHT * DS_HALF_WIDTH - 1) json += ",";
  }
  json += "],\"Z\":[";
  for (int i = 0; i < DS_HEIGHT * DS_HALF_WIDTH; i++) {
    json += String(worldZmap[i], 2);
    if (i < DS_HEIGHT * DS_HALF_WIDTH - 1) json += ",";
  }
  json += "]}";

  server.send(200, "application/json", json);

  // json += "]";
  // server.send(200, "application/json", json);
  //server.send(200, "text/plain", "AP done");
}

void handleReceiveLeftDepth() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    int idx = 0, start = 0;
    while (idx < IMG_HEIGHT * HALF_WIDTH) {
      int end = body.indexOf(',', start);
      if (end == -1) end = body.length();
      depthMap[idx] = body.substring(start, end).toInt();
      idx++;
      start = end + 1;
    }
    server.send(200, "text/plain", "Left depth OK");
  } else {
    server.send(400, "text/plain", "No data");
  }
}
/*
void handleDepthMap() {
  String json = "[";
  for (int i = 0; i < IMG_WIDTH * IMG_HEIGHT; i++) {
    json += String(depthMap[i]);
    if (i < IMG_WIDTH * IMG_HEIGHT - 1) json += ",";
  }
  json += "]";
  server.send(200, "application/json", json);
}
*/
void doDepthProcess() {
  Serial.println("Start depth flow...");

  captureAndSplit();

  // Trigger client capture
  HTTPClient http;
  http.begin(client, "http://192.168.4.2/capture");
  int code = http.GET();
  http.end();

  if (code != HTTP_CODE_OK) {
    Serial.printf("Failed to trigger client capture: %d\n", code);
    return;
  }

  delay(100);

  // Send LEFT half to client
  HTTPClient http2;
  http2.begin(client, "http://192.168.4.2/receive_left_half");
  String payload;
  payload.reserve(IMG_HEIGHT * HALF_WIDTH * 4); // Rough reserve
  for (int i = 0; i < IMG_HEIGHT * HALF_WIDTH; i++) {
    payload += String(apLeftHalf[i]);
    if (i < IMG_HEIGHT * HALF_WIDTH - 1) payload += ",";
  }
  int code2 = http2.POST(payload);
  http2.end();

  if (code2 != HTTP_CODE_OK) {
    Serial.printf("Failed to send left half: %d\n", code2);
    return;
  }

  // Get right half from client
  fetchClientRightHalf();
  printClientRightHalf();

  // Calculate depth
  computeRightDepth();

  // (Optionally) send depth map back or do other tasks
}
/*
void WiFiEvent(WiFiEvent_t event) {
  switch(event) {
    case WIFI_EVENT_AP_STACONNECTED:
      Serial.println("Client connected to AP");
      break;
    case WIFI_EVENT_AP_STADISCONNECTED:
      Serial.println("Client disconnected from AP");
      break;
    default:
      break;
  }
}
*/
void handleDepthData() {
  String json = "{";
  
  json += "\"X\":[";
  for (int i = 0; i < DS_HEIGHT * DS_HALF_WIDTH; i++) {
    json += worldXmap[i];
    if (i < DS_HEIGHT * DS_HALF_WIDTH - 1) json += ",";
  }
  json += "],\"Y\":[";
  for (int i = 0; i < DS_HEIGHT * DS_HALF_WIDTH; i++) {
    json += worldYmap[i];
    if (i < DS_HEIGHT * DS_HALF_WIDTH - 1) json += ",";
  }
  json += "],\"Z\":[";
  for (int i = 0; i < DS_HEIGHT * DS_HALF_WIDTH; i++) {
    json += String(worldZmap[i], 2);
    if (i < DS_HEIGHT * DS_HALF_WIDTH - 1) json += ",";
  }
  json += "]}";

  server.send(200, "application/json", json);
}



void setup() {
  Serial.begin(115200);

 // WiFi.onEvent(WiFiEvent);  // Register event handler
  setupCamera();
  WiFi.softAP(ssid, password);
  Serial.println(WiFi.softAPIP());

  server.on("/get_depth", HTTP_GET, handleGetDepth);
  server.on("/post_left_depth", HTTP_POST, handleReceiveLeftDepth);
  server.on("/post_client_right", HTTP_POST, handlePostClientRight);/////////////

   //server.on("/depth", HTTP_GET, handleDepthData);

  startDepthProcess= true;
  //server.on("/depth_map", HTTP_GET, handleDepthMap);
  server.begin();
  Serial.println("AP ready");
  //doDepthProcess();
}

void loop() {
  server.handleClient();
  //handleGetDepth();
  //   if (startDepthProcess) {
  //   startDepthProcess = false;
  //  doDepthProcess();
  // }

  delay(2000);
 // doDepthProcess();
}
