#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include "esp_camera.h"

// CAMERA MODEL
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// NETWORK
const char* ssid = "ESP32-CAM-AP";
const char* password = "12345678";
IPAddress apIP(192,168,4,1);

#define SRC_WIDTH  160
#define SRC_HEIGHT 120
#define IMG_WIDTH   80
#define IMG_HEIGHT  60
#define HALF_WIDTH  40
/////////////
#define WINDOW_SIZE 5
#define HALF_WINDOW (WINDOW_SIZE / 2)
//
#define DS_WIDTH 80
#define DS_HEIGHT 60
#define DS_HALF_WIDTH 40

uint8_t worldXmap[DS_HEIGHT * DS_HALF_WIDTH];
uint8_t worldYmap[DS_HEIGHT * DS_HALF_WIDTH];
uint8_t worldZmap[DS_HEIGHT * DS_HALF_WIDTH];


uint8_t downsampledFrame[IMG_WIDTH * IMG_HEIGHT];
uint8_t apLeftHalf[IMG_HEIGHT * HALF_WIDTH];///////////
uint8_t clientLeftHalf[IMG_HEIGHT * HALF_WIDTH];
uint8_t clientRightHalf[IMG_HEIGHT * HALF_WIDTH];
uint8_t leftDepth[IMG_HEIGHT * HALF_WIDTH];

WebServer server(80);

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
  config.frame_size   = FRAMESIZE_QQVGA;
  config.jpeg_quality = 12;
  config.fb_count     = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    while (1);
  }
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
    memcpy(&clientLeftHalf[y * HALF_WIDTH], &downsampledFrame[y * IMG_WIDTH], HALF_WIDTH);
    memcpy(&clientRightHalf[y * HALF_WIDTH], &downsampledFrame[y * IMG_WIDTH + HALF_WIDTH], HALF_WIDTH);
  }
}
//////////////////////////////////////////////////

//////code stuck here                    /////////////////////////////////////////////
int findBestVerticalDisparity(int x, int y, int maxDisparity) {
    int bestDisparity = 0;
    int minSAD = INT32_MAX;

    for (int d = 0; d < maxDisparity; ++d) {
        int matchY = y + d;

        // Check window boundaries for top and bottom patches
        if (matchY + HALF_WINDOW >= IMG_HEIGHT ||  // bottom patch lower bound check
            y - HALF_WINDOW < 0 ||                  // top patch upper bound check
            matchY - HALF_WINDOW < 0)                // bottom patch upper bound check
        {
            continue; // Skip disparities that would go outside image boundaries
        }

        int sad = 0;

        for (int dy = -HALF_WINDOW; dy <= HALF_WINDOW; ++dy) {
            int topY = y + dy;
            int bottomY = matchY + dy;

            for (int dx = -HALF_WINDOW; dx <= HALF_WINDOW; ++dx) {
                int px = x + dx;

                if (px < 0 || px >= HALF_WIDTH || bottomY < 0 || bottomY >= IMG_HEIGHT)
                    continue;

                int topIndex = topY * HALF_WIDTH + px;
                int bottomIndex = bottomY * HALF_WIDTH + px;

                uint8_t topVal = apLeftHalf[topIndex];
                uint8_t bottomVal = clientLeftHalf[bottomIndex];

                sad += abs(topVal - bottomVal);
            }
        }

        if (sad < minSAD) {
            minSAD = sad;
            bestDisparity = d;
        }
    }

    return bestDisparity;
}

float computeDepth(int disparity, float focalLength, float baseline) {
    if (disparity == 0) return 255.0f;  // avoid division by zero, assume far away

    return (focalLength * baseline) / disparity;
}




//all calculations
void computeLeftDepth() {
  float focalLength = 25.0f;  // Adjusted due to 2x downscale
  float baseline = 10.0f;     // cm
  float cx = DS_HALF_WIDTH / 2.0f;
  float cy = DS_HEIGHT / 2.0f;

  for (int y = 0; y < DS_HEIGHT; y++) {
    for (int x = 0; x < DS_HALF_WIDTH; x++) {
      int disp = findBestVerticalDisparity(x, y, 60);  // You may lower this to 30 due to scale
      float depth = computeDepth(disp, focalLength, baseline);

      if (depth > 255.0f || disp == 0) depth = 255.0f;

      float worldX = (x - cx) * depth / focalLength;
      float worldY = (y - cy) * depth / focalLength;

      int scaledX = (int)(worldX + 127.5f);
      int scaledY = (int)(worldY + 127.5f);

      scaledX = constrain(scaledX, 0, 255);
      scaledY = constrain(scaledY, 0, 255);

      int index = y * DS_HALF_WIDTH + x;
      worldXmap[index] = (uint8_t)scaledX;
      worldYmap[index] = (uint8_t)scaledY;
      worldZmap[index] = (uint8_t)depth;
    }
  }
}
//////////////////////////////////////////////////////////////////
void sendClientRightHalfToAP() {
  HTTPClient http;
  String url = "http://" + apIP.toString() + "/post_client_right";
  http.begin(url);

  // Build comma-separated payload from clientRightHalf
  String payload = "";
  for (int i = 0; i < IMG_HEIGHT * HALF_WIDTH; i++) {
    payload += String(clientRightHalf[i]);
    if (i < IMG_HEIGHT * HALF_WIDTH - 1) payload += ",";
  }

  int httpCode = http.POST(payload);

  if (httpCode > 0) {
    Serial.printf("POST sent. Response code: %d\n", httpCode);
    String response = http.getString();
    Serial.println(response);
  } else {
    Serial.printf("POST failed. Error: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
}



void handleCapture() {
  captureAndSplit();
  server.send(200, "text/plain", "Client capture OK");
  sendClientRightHalfToAP();//send right half
}

//receved ap left half
void handleReceiveLeftHalf() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    int idx = 0, start = 0;
    while (idx < IMG_HEIGHT * HALF_WIDTH) {
      int end = body.indexOf(',', start);
      if (end == -1) end = body.length();
      apLeftHalf[idx] = body.substring(start, end).toInt();////////////////////
      idx++;
      start = end + 1;
    }

    ////////////////
    

    //then go to calculation


    computeLeftDepth();////////////////////////////

    HTTPClient http;
    String url = "http://" + apIP.toString() + "/post_left_depth";
    http.begin(url);
    String payload = "";
    for (int i = 0; i < IMG_HEIGHT * HALF_WIDTH; i++) {
      payload += String(leftDepth[i]);
      if (i < IMG_HEIGHT * HALF_WIDTH - 1) payload += ",";
    }
    http.POST(payload);
    http.end();

    server.send(200, "text/plain", "Left depth sent");
  } else {
    server.send(400, "text/plain", "No data");
  }
}

void setup() {
  Serial.begin(115200);
  setupCamera();
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Client connected");

  server.on("/capture", HTTP_GET, handleCapture);
  server.on("/receive_left_half", HTTP_POST, handleReceiveLeftHalf);
  server.begin();
  Serial.println("Client ready");
}

void loop() {
  server.handleClient();
}
