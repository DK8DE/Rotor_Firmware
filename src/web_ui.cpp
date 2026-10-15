#include "web_ui.h"

#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_ota_ops.h>
#include <string.h>

#include "web_app_api.h"
#include "web_content.h"

static WebServer* g_server = nullptr;
static bool g_webActive = false;
static bool g_apHadClient = false;
static bool g_routesReady = false;
static char g_ssidCache[40] = "";
static char g_ipCache[20] = "192.168.4.1";
static uint32_t g_apEmptySinceMs = 0;
static constexpr uint32_t AP_IDLE_SHUTDOWN_MS = 20000;

static const char* WEB_USER = "admin";
static const char* WEB_PASS = "Rotorconfig";

static constexpr uint32_t APP_DESC_MAGIC = 0xABCD5432UL;
static bool g_updReject = false;
static bool g_updFailed = false;

static bool checkAuth() {
  if (g_server == nullptr) return false;
  if (g_server->authenticate(WEB_USER, WEB_PASS)) return true;
  g_server->requestAuthentication();
  return false;
}

static void sendJson(int code, const String& body) {
  if (g_server == nullptr) return;
  g_server->sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  g_server->sendHeader("Pragma", "no-cache");
  g_server->sendHeader("Connection", "close");
  g_server->send(code, "application/json", body);
}

static void handleRoot() {
  if (!checkAuth()) return;
  g_server->sendHeader("Connection", "close");
  g_server->sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  g_server->sendHeader("Pragma", "no-cache");
  g_server->sendHeader("Expires", "0");
  g_server->send_P(200, "text/html", WEB_PAGE);
}

static void handleLive() {
  if (!checkAuth()) return;
  char buf[512];
  webAppFillLiveJson(buf, sizeof(buf), g_ssidCache, g_ipCache);
  sendJson(200, buf);
}

static void handleStatus() {
  handleLive();
}

static void handleUpdateInfo() {
  if (!checkAuth()) return;
  const esp_partition_t* running = esp_ota_get_running_partition();
  char buf[192];
  snprintf(buf, sizeof(buf),
           "{\"fw\":\"%s\",\"partition\":\"%s\",\"size\":%u}", webAppFwVersion(),
           running ? running->label : "?", running ? (unsigned)running->size : 0U);
  sendJson(200, buf);
}

static void handleUpdateUpload() {
  if (g_server == nullptr) return;
  HTTPUpload& upload = g_server->upload();
  if (upload.status == UPLOAD_FILE_START) {
    g_updReject = false;
    g_updFailed = false;
    size_t maxSketch = ESP.getFreeSketchSpace();
    if (!Update.begin(maxSketch)) {
      Update.printError(Serial);
      g_updFailed = true;
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (g_updReject || g_updFailed) return;
    if (upload.totalSize == upload.currentSize) {
      uint32_t magic = 0;
      if (upload.currentSize >= 36) {
        memcpy(&magic, upload.buf + 32, sizeof(magic));
      }
      if (upload.currentSize < 36 || upload.buf[0] != 0xE9 || magic != APP_DESC_MAGIC) {
        g_updReject = true;
        Update.abort();
        return;
      }
    }
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
      g_updFailed = true;
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (g_updReject || g_updFailed) return;
    if (!Update.end(true)) {
      Update.printError(Serial);
      g_updFailed = true;
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    g_updFailed = true;
  }
}

static void handleUpdateDone() {
  if (!checkAuth()) return;
  if (g_updReject) {
    sendJson(400, "{\"ok\":false,\"err\":\"not_app\"}");
    return;
  }
  if (g_updFailed || Update.hasError()) {
    sendJson(500, "{\"ok\":false}");
    return;
  }
  sendJson(200, "{\"ok\":true}");
}

static void handleReboot() {
  if (!checkAuth()) return;
  sendJson(200, "{\"ok\":true}");
  delay(200);
  webAppRequestReboot();
}

static void handleFactory() {
  if (!checkAuth()) return;
  webAppFactoryReset();
  sendJson(200, "{\"ok\":true}");
  delay(200);
  webAppRequestReboot();
}

static void handleCaptiveOk() {
  if (g_server == nullptr) return;
  g_server->sendHeader("Connection", "close");
  g_server->sendHeader("Cache-Control", "no-store");
  g_server->send(204, "text/plain", "");
}

static void handleCaptiveHtml() {
  if (g_server == nullptr) return;
  g_server->sendHeader("Connection", "close");
  g_server->sendHeader("Cache-Control", "no-store");
  g_server->send(200, "text/html",
                 "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>");
}

static void handleNotFound() {
  if (g_server == nullptr) return;
  g_server->sendHeader("Connection", "close");
  g_server->send(404, "text/plain", "Not Found");
}

void webUiBegin(const char* ssid, const char* password) {
  if (g_webActive) return;
  if (g_server == nullptr) {
    g_server = new WebServer(80);
  }

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1),
                    IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, password, 1, 0, 2);

  strncpy(g_ssidCache, ssid, sizeof(g_ssidCache) - 1);
  g_ssidCache[sizeof(g_ssidCache) - 1] = '\0';
  snprintf(g_ipCache, sizeof(g_ipCache), "%s", WiFi.softAPIP().toString().c_str());

  if (!g_routesReady) {
    g_server->on("/", HTTP_GET, handleRoot);
    g_server->on("/api/status", HTTP_GET, handleStatus);
    g_server->on("/api/live", HTTP_GET, handleLive);
    g_server->on("/api/update/info", HTTP_GET, handleUpdateInfo);
    g_server->on("/api/update", HTTP_POST, handleUpdateDone,
                 []() { handleUpdateUpload(); });
    g_server->on("/api/reboot", HTTP_POST, handleReboot);
    g_server->on("/api/factory", HTTP_POST, handleFactory);

    g_server->on("/generate_204", HTTP_GET, handleCaptiveOk);
    g_server->on("/gen_204", HTTP_GET, handleCaptiveOk);
    g_server->on("/hotspot-detect.html", HTTP_GET, handleCaptiveHtml);
    g_server->on("/library/test/success.html", HTTP_GET, handleCaptiveHtml);
    g_server->on("/connecttest.txt", HTTP_GET, handleCaptiveOk);
    g_server->on("/ncsi.txt", HTTP_GET, handleCaptiveOk);
    g_server->on("/fwlink", HTTP_GET, handleCaptiveOk);
    g_server->on("/canonical.html", HTTP_GET, handleCaptiveHtml);
    g_server->onNotFound(handleNotFound);
    g_routesReady = true;
  }

  g_server->begin();
  g_apHadClient = false;
  g_apEmptySinceMs = 0;
  g_webActive = true;
}

void webUiEnd() {
  if (!g_webActive) return;
  if (g_server != nullptr) {
    g_server->stop();
  }
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  g_apHadClient = false;
  g_apEmptySinceMs = 0;
  g_webActive = false;
}

void webUiLoop() {
  if (!g_webActive) return;
  if (g_server != nullptr) {
    for (int i = 0; i < 4; i++) {
      g_server->handleClient();
    }
  }

  const int stations = WiFi.softAPgetStationNum();
  if (stations > 0) {
    g_apHadClient = true;
    g_apEmptySinceMs = 0;
  } else if (g_apHadClient) {
    if (g_apEmptySinceMs == 0) {
      g_apEmptySinceMs = millis();
    } else if ((uint32_t)(millis() - g_apEmptySinceMs) >= AP_IDLE_SHUTDOWN_MS) {
      webUiEnd();
      webAppOnSoftApAutoOff();
    }
  }
}

bool webUiIsActive() {
  return g_webActive;
}
