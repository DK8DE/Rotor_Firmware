#include "web_ui.h"

#include <Preferences.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_ota_ops.h>
#include <string.h>

#include "web_app_api.h"
#include "web_content.h"

static WebServer* g_server = nullptr;
static bool g_webActive = false;
static bool g_routesReady = false;
static char g_ssidCache[40] = "";
static char g_ipCache[20] = "192.168.4.1";

static constexpr const char* NVS_NS = "webui";
static constexpr const char* DEF_USER = "admin";
static constexpr const char* DEF_PASS = "Rotorconfig";
static constexpr const char* DEF_PSK = "Rotorconfig";

static char g_webUser[32] = "admin";
static char g_webPass[64] = "Rotorconfig";
static char g_apPass[64] = "Rotorconfig";

// esp_app_desc_t.magic_word @ Offset 32 im App-Image (nicht in factory.bin).
static constexpr uint32_t APP_DESC_MAGIC = 0xABCD5432UL;
static bool g_updReject = false;
static bool g_updFailed = false;
static bool g_updBegun = false;
static bool g_updMagicOk = false;
static uint8_t g_updHdr[36];
static size_t g_updHdrLen = 0;
static const char* g_updErr = nullptr;

static void credsApplyDefaults() {
  strncpy(g_webUser, DEF_USER, sizeof(g_webUser) - 1);
  g_webUser[sizeof(g_webUser) - 1] = '\0';
  strncpy(g_webPass, DEF_PASS, sizeof(g_webPass) - 1);
  g_webPass[sizeof(g_webPass) - 1] = '\0';
  strncpy(g_apPass, DEF_PSK, sizeof(g_apPass) - 1);
  g_apPass[sizeof(g_apPass) - 1] = '\0';
}

static void credsLoad() {
  credsApplyDefaults();
  Preferences p;
  if (!p.begin(NVS_NS, true)) return;
  String u = p.getString("user", DEF_USER);
  String pw = p.getString("pass", DEF_PASS);
  String pk = p.getString("psk", DEF_PSK);
  p.end();
  if (u.length() > 0 && u.length() < sizeof(g_webUser)) {
    strncpy(g_webUser, u.c_str(), sizeof(g_webUser) - 1);
    g_webUser[sizeof(g_webUser) - 1] = '\0';
  }
  if (pw.length() > 0 && pw.length() < sizeof(g_webPass)) {
    strncpy(g_webPass, pw.c_str(), sizeof(g_webPass) - 1);
    g_webPass[sizeof(g_webPass) - 1] = '\0';
  }
  if (pk.length() >= 8 && pk.length() < sizeof(g_apPass)) {
    strncpy(g_apPass, pk.c_str(), sizeof(g_apPass) - 1);
    g_apPass[sizeof(g_apPass) - 1] = '\0';
  }
}

static bool credsSaveUserPass() {
  Preferences p;
  if (!p.begin(NVS_NS, false)) return false;
  bool ok = p.putString("user", g_webUser) > 0;
  ok = p.putString("pass", g_webPass) > 0 && ok;
  p.end();
  return ok;
}

static bool credsSavePsk() {
  Preferences p;
  if (!p.begin(NVS_NS, false)) return false;
  bool ok = p.putString("psk", g_apPass) > 0;
  p.end();
  return ok;
}

void webUiClearCredentials() {
  Preferences p;
  if (p.begin(NVS_NS, false)) {
    p.clear();
    p.end();
  }
  credsApplyDefaults();
}

const char* webUiSoftApPass() {
  return g_apPass;
}

static bool jsonGetStr(const char* json, const char* key, char* out, size_t outLen) {
  if (!json || !key || !out || outLen < 2) return false;
  out[0] = 0;
  char pat[40];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char* p = strstr(json, pat);
  if (!p) return false;
  p = strchr(p + strlen(pat), ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t') p++;
  if (*p != '"') return false;
  p++;
  size_t i = 0;
  while (*p && *p != '"' && i + 1 < outLen) {
    if (*p == '\\' && p[1]) p++;
    out[i++] = *p++;
  }
  out[i] = 0;
  return true;
}

static bool credOkUser(const char* u) {
  if (!u || !u[0]) return false;
  size_t n = strlen(u);
  if (n < 1 || n > 31) return false;
  for (size_t i = 0; i < n; i++) {
    unsigned char c = (unsigned char)u[i];
    if (c < 0x21 || c > 0x7E) return false;  // kein Leerzeichen / nicht-ASCII
  }
  return true;
}

static bool credOkPass(const char* pw, size_t minLen, size_t maxLen) {
  if (!pw) return false;
  size_t n = strlen(pw);
  if (n < minLen || n > maxLen) return false;
  for (size_t i = 0; i < n; i++) {
    unsigned char c = (unsigned char)pw[i];
    if (c < 0x20 || c > 0x7E) return false;
  }
  return true;
}

static bool checkAuth() {
  if (g_server == nullptr) return false;
  if (g_server->authenticate(g_webUser, g_webPass)) return true;
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

static void updResetState() {
  g_updReject = false;
  g_updFailed = false;
  g_updBegun = false;
  g_updMagicOk = false;
  g_updHdrLen = 0;
  g_updErr = nullptr;
  memset(g_updHdr, 0, sizeof(g_updHdr));
}

static bool updEnsureBegun() {
  if (g_updBegun) return true;
  const esp_partition_t* next = esp_ota_get_next_update_partition(nullptr);
  size_t maxSketch = next ? next->size : ESP.getFreeSketchSpace();
  if (maxSketch == 0) {
    g_updFailed = true;
    g_updErr = "no_ota_slot";
    return false;
  }
  if (!Update.begin(maxSketch, U_FLASH)) {
    Update.printError(Serial);
    g_updFailed = true;
    g_updErr = "begin";
    return false;
  }
  g_updBegun = true;
  return true;
}

static bool updCheckMagic(const uint8_t* hdr) {
  uint32_t magic = 0;
  memcpy(&magic, hdr + 32, sizeof(magic));
  return hdr[0] == 0xE9 && magic == APP_DESC_MAGIC;
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
  // Ohne Auth: Captive-/Handy-Browser sendet Basic-Auth bei fetch oft nicht mit.
  // SoftAP ist ohnehin passwortgeschuetzt (WLAN + Login fuer HTML/OTA/Reset).
  char buf[512];
  webAppFillLiveJson(buf, sizeof(buf), g_ssidCache, g_ipCache);
  sendJson(200, buf);
}

static void handleStatus() {
  if (!checkAuth()) return;
  char buf[512];
  webAppFillLiveJson(buf, sizeof(buf), g_ssidCache, g_ipCache);
  sendJson(200, buf);
}

static void handleSetupState() {
  if (!checkAuth()) return;
  static char buf[3072];
  webAppFillSetupStateJson(buf, sizeof(buf));
  sendJson(200, buf);
}

static void handleSetupApply() {
  if (!checkAuth()) return;
  if (g_server == nullptr) return;
  const String body = g_server->arg("plain");
  char out[256];
  webAppSetupApplyJson(body.c_str(), out, sizeof(out));
  sendJson(200, out);
}

static void handleSetupJog() {
  if (!checkAuth()) return;
  if (g_server == nullptr) return;
  const String body = g_server->arg("plain");
  uint8_t dir = 0;
  const int idx = body.indexOf("\"dir\"");
  if (idx >= 0) {
    const int colon = body.indexOf(':', idx);
    if (colon >= 0) {
      dir = (uint8_t)body.substring(colon + 1).toInt();
    }
  }
  // Tab Bewegen: limits:0 → Amin/Amax bleiben aktiv (kein Setup-Modus)
  bool pastLimits = true;
  if (body.indexOf("\"limits\":0") >= 0 || body.indexOf("\"limits\":false") >= 0) {
    pastLimits = false;
  }
  char out[128];
  webAppSetupJogEx(dir, pastLimits, out, sizeof(out));
  sendJson(200, out);
}

static void handleUserGet() {
  if (!checkAuth()) return;
  char buf[96];
  snprintf(buf, sizeof(buf), "{\"user\":\"%s\"}", g_webUser);
  sendJson(200, buf);
}

static void handleUserSet() {
  if (!checkAuth()) return;
  if (g_server == nullptr) return;
  const String body = g_server->arg("plain");
  char user[32] = {0};
  char pass[64] = {0};
  char pass2[64] = {0};
  if (!jsonGetStr(body.c_str(), "user", user, sizeof(user))) {
    sendJson(400, "{\"ok\":false,\"err\":\"user\"}");
    return;
  }
  if (!jsonGetStr(body.c_str(), "pass", pass, sizeof(pass))) {
    sendJson(400, "{\"ok\":false,\"err\":\"pass\"}");
    return;
  }
  (void)jsonGetStr(body.c_str(), "pass2", pass2, sizeof(pass2));
  if (!credOkUser(user)) {
    sendJson(400, "{\"ok\":false,\"err\":\"user\"}");
    return;
  }
  if (!credOkPass(pass, 4, 63)) {
    sendJson(400, "{\"ok\":false,\"err\":\"pass\"}");
    return;
  }
  if (strcmp(pass, pass2) != 0) {
    sendJson(400, "{\"ok\":false,\"err\":\"mismatch\"}");
    return;
  }
  strncpy(g_webUser, user, sizeof(g_webUser) - 1);
  g_webUser[sizeof(g_webUser) - 1] = '\0';
  strncpy(g_webPass, pass, sizeof(g_webPass) - 1);
  g_webPass[sizeof(g_webPass) - 1] = '\0';
  if (!credsSaveUserPass()) {
    sendJson(500, "{\"ok\":false,\"err\":\"nvs\"}");
    return;
  }
  sendJson(200, "{\"ok\":true}");
}

static void handleWifiGet() {
  if (!checkAuth()) return;
  char buf[96];
  snprintf(buf, sizeof(buf), "{\"ssid\":\"%s\"}", g_ssidCache);
  sendJson(200, buf);
}

static void handleWifiSet() {
  if (!checkAuth()) return;
  if (g_server == nullptr) return;
  const String body = g_server->arg("plain");
  char pass[64] = {0};
  char pass2[64] = {0};
  if (!jsonGetStr(body.c_str(), "pass", pass, sizeof(pass))) {
    sendJson(400, "{\"ok\":false,\"err\":\"pass\"}");
    return;
  }
  (void)jsonGetStr(body.c_str(), "pass2", pass2, sizeof(pass2));
  // ESP SoftAP WPA2: 8..63 Zeichen
  if (!credOkPass(pass, 8, 63)) {
    sendJson(400, "{\"ok\":false,\"err\":\"pass\"}");
    return;
  }
  if (strcmp(pass, pass2) != 0) {
    sendJson(400, "{\"ok\":false,\"err\":\"mismatch\"}");
    return;
  }
  strncpy(g_apPass, pass, sizeof(g_apPass) - 1);
  g_apPass[sizeof(g_apPass) - 1] = '\0';
  if (!credsSavePsk()) {
    sendJson(500, "{\"ok\":false,\"err\":\"nvs\"}");
    return;
  }
  // Antwort zuerst, dann Neustart — SoftAP danach aus bis wieder aktiviert (neuer PSK gilt).
  sendJson(200, "{\"ok\":true,\"reboot\":1}");
  delay(400);
  webAppRequestReboot();
}

static void handleUpdateInfo() {
  if (!checkAuth()) return;
  const esp_partition_t* running = esp_ota_get_running_partition();
  const esp_partition_t* next = esp_ota_get_next_update_partition(nullptr);
  char buf[256];
  snprintf(buf, sizeof(buf),
           "{\"fw\":\"%s\",\"partition\":\"%s\",\"size\":%u,\"ota\":\"%s\",\"otaSize\":%u}",
           webAppFwVersion(), running ? running->label : "?",
           running ? (unsigned)running->size : 0U, next ? next->label : "?",
           next ? (unsigned)next->size : 0U);
  sendJson(200, buf);
}

static void handleUpdateUpload() {
  if (g_server == nullptr) return;
  HTTPUpload& upload = g_server->upload();

  if (upload.status == UPLOAD_FILE_START) {
    updResetState();
    Serial.printf("[OTA] start name=%s\n", upload.filename.c_str());
    return;
  }

  if (upload.status == UPLOAD_FILE_WRITE) {
    if (g_updReject || g_updFailed) return;

    const uint8_t* data = upload.buf;
    size_t len = upload.currentSize;
    if (len == 0) return;

    // Magic erst pruefen, wenn die ersten 36 Bytes vollstaendig sind
    // (erster HTTP-Chunk kann < 36 sein — sonst fälschlich not_app).
    if (!g_updMagicOk) {
      size_t need = sizeof(g_updHdr) - g_updHdrLen;
      size_t take = (len < need) ? len : need;
      memcpy(g_updHdr + g_updHdrLen, data, take);
      g_updHdrLen += take;

      if (g_updHdrLen < sizeof(g_updHdr)) {
        return;  // warten auf weitere Bytes, noch nichts flashen
      }

      if (!updCheckMagic(g_updHdr)) {
        Serial.println("[OTA] reject: not app image (magic)");
        g_updReject = true;
        g_updErr = "not_app";
        if (g_updBegun) Update.abort();
        return;
      }
      g_updMagicOk = true;

      if (!updEnsureBegun()) return;

      if (Update.write(g_updHdr, sizeof(g_updHdr)) != sizeof(g_updHdr)) {
        Update.printError(Serial);
        g_updFailed = true;
        g_updErr = "write";
        return;
      }

      data += take;
      len -= take;
      if (len == 0) return;
    }

    if (!updEnsureBegun()) return;

    if (Update.write(const_cast<uint8_t*>(data), len) != len) {
      Update.printError(Serial);
      g_updFailed = true;
      g_updErr = "write";
    }
    return;
  }

  if (upload.status == UPLOAD_FILE_END) {
    if (g_updReject || g_updFailed) return;
    if (!g_updMagicOk || !g_updBegun) {
      g_updFailed = true;
      g_updErr = "short";
      if (g_updBegun) Update.abort();
      return;
    }
    if (!Update.end(true)) {
      Update.printError(Serial);
      g_updFailed = true;
      g_updErr = "end";
      return;
    }
    Serial.printf("[OTA] ok size=%u\n", (unsigned)upload.totalSize);
    return;
  }

  if (upload.status == UPLOAD_FILE_ABORTED) {
    if (g_updBegun) Update.abort();
    g_updFailed = true;
    g_updErr = "aborted";
  }
}

static void handleUpdateDone() {
  // Kein checkAuth(): XHR/FormData sendet Basic-Auth oft nicht mit.
  // SoftAP-PSK schuetzt den Zugang; Magic-Check verhindert factory.bin.
  if (g_updReject) {
    sendJson(400, "{\"ok\":false,\"err\":\"not_app\"}");
    return;
  }
  if (g_updFailed || Update.hasError()) {
    char buf[96];
    snprintf(buf, sizeof(buf), "{\"ok\":false,\"err\":\"%s\"}",
             g_updErr ? g_updErr : "update");
    sendJson(500, buf);
    return;
  }
  if (!g_updMagicOk || !g_updBegun) {
    sendJson(500, "{\"ok\":false,\"err\":\"short\"}");
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

void webUiBegin(const char* ssid) {
  if (g_webActive) return;
  if (g_server == nullptr) {
    g_server = new WebServer(80);
  }

  credsLoad();

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1),
                    IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, g_apPass, 1, 0, 2);

  strncpy(g_ssidCache, ssid, sizeof(g_ssidCache) - 1);
  g_ssidCache[sizeof(g_ssidCache) - 1] = '\0';
  snprintf(g_ipCache, sizeof(g_ipCache), "%s", WiFi.softAPIP().toString().c_str());

  if (!g_routesReady) {
    g_server->on("/", HTTP_GET, handleRoot);
    g_server->on("/api/status", HTTP_GET, handleStatus);
    g_server->on("/api/live", HTTP_GET, handleLive);
    g_server->on("/api/user", HTTP_GET, handleUserGet);
    g_server->on("/api/user", HTTP_POST, handleUserSet);
    g_server->on("/api/wifi", HTTP_GET, handleWifiGet);
    g_server->on("/api/wifi", HTTP_POST, handleWifiSet);
    g_server->on("/api/update/info", HTTP_GET, handleUpdateInfo);
    g_server->on("/api/update", HTTP_POST, handleUpdateDone,
                 []() { handleUpdateUpload(); });
    g_server->on("/api/reboot", HTTP_POST, handleReboot);
    g_server->on("/api/factory", HTTP_POST, handleFactory);
    g_server->on("/api/setup/state", HTTP_GET, handleSetupState);
    g_server->on("/api/setup/apply", HTTP_POST, handleSetupApply);
    g_server->on("/api/setup/jog", HTTP_POST, handleSetupJog);

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
  g_webActive = true;
}

void webUiEnd() {
  if (!g_webActive) return;
  if (g_server != nullptr) {
    g_server->stop();
  }
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  g_webActive = false;
}

void webUiLoop() {
  if (!g_webActive) return;
  // SoftAP bleibt aktiv bis SETWLAN:0 oder Neustart (kein Idle-Shutdown).
  if (g_server != nullptr) {
    // Mehr Durchlaeufe waehrend OTA-Upload (grosse firmware.bin).
    const int n = (g_updBegun || g_updHdrLen > 0) ? 16 : 4;
    for (int i = 0; i < n; i++) {
      g_server->handleClient();
      yield();
    }
  }
}

bool webUiIsActive() {
  return g_webActive;
}
