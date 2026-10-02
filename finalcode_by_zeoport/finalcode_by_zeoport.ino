/*
  ================================================================
  Smart Environmental Monitor - ESP32-C3 SuperMini
  Arduino-ESP32 Core: 2.0.16
  ================================================================

  Features
  - AHT20: Temperature / Humidity
  - BMP280: Pressure / Altitude
  - BH1750: Lux
  - Rain sensor: Analog + Digital
  - SSD1306 128x64 OLED
  - WiFiManager non-blocking captive portal
  - Blynk IoT over SSL
  - Blynk reconnect with non-blocking retry scheduling
  - Blynk.Air OTA
  - NTP clock
  - Button-driven OLED state machine
  - Task watchdog
  - 12-hour automatic reboot
  - ESP32-C3 TX power hardware workaround:
      WiFi.setTxPower(WIFI_POWER_8_5dBm);

  Blynk virtual pins (matching the supplied configuration):
    V0 = AHT20 Temperature       (double, °C)
    V1 = AHT20 Humidity          (double, %)
    V2 = Update interval mode    (integer, 0/1)
    V3 = BMP280 Altitude         (double, m)
    V4 = BMP280 Pressure         (integer, hPa)
    V5 = Rain Analog             (integer, ADC 0..4095)
    V6 = Rain Digital            (integer, 0=dry, 1=rain)
    V7 = BH1750 Lux               (double, lx)
    V8 = ESP32 internal temp      (double, °C)
    V9 = OLED display control      (integer, 1=ON, 0=OFF)
    V10 = Blynk button for D10 (integer, 0/1)

  IMPORTANT:
  1) Replace BLYNK_TEMPLATE_ID / BLYNK_TEMPLATE_NAME / BLYNK_AUTH_TOKEN.
  2) Blynk.Air OTA requires the device/template to be configured in Blynk.
  3) Install library versions compatible with ESP32 Core 2.0.16.
*/

#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "ID"
#define BLYNK_TEMPLATE_NAME "NAME"
#define BLYNK_AUTH_TOKEN "TOKEN"
#define BLYNK_FIRMWARE_VERSION "VER"

// Blynk SSL connection stability settings.
// BLYNK_TIMEOUT_MS is the protocol network/login timeout.
#define BLYNK_TIMEOUT_MS 10000UL
#define BLYNK_HEARTBEAT 30

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

#include <BlynkSimpleEsp32_SSL.h>

#include <HTTPClient.h>
#include <Update.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <BH1750.h>

#include "esp_task_wdt.h"

// ================================================================
// Hardware
// ================================================================

static constexpr uint8_t PIN_I2C_SDA       = 0;
static constexpr uint8_t PIN_I2C_SCL       = 1;

static constexpr uint8_t PIN_RAIN_ANALOG   = 3;  // A0
static constexpr uint8_t PIN_RAIN_DIGITAL  = 4;  // D0

static constexpr uint8_t PIN_BUTTON        = 10;  // active LOW

static constexpr uint8_t OLED_WIDTH        = 128;
static constexpr uint8_t OLED_HEIGHT       = 64;
static constexpr int8_t  OLED_RESET        = -1;
static constexpr uint8_t OLED_ADDR         = 0x3C;

static constexpr uint8_t BH1750_ADDR       = 0x23;
static constexpr uint8_t BMP280_ADDR_1     = 0x76;
static constexpr uint8_t BMP280_ADDR_2     = 0x77;

// ================================================================
// Timing
// ================================================================

// How long we wait during one connection attempt before starting another.
static constexpr uint32_t WIFI_INITIAL_TIMEOUT_MS  = 60000UL;       // 1 minute
static constexpr uint32_t WIFI_RECONNECT_MS        = 15000UL;
static constexpr uint32_t BLYNK_RECONNECT_MS       = 5000UL;

static constexpr uint32_t SENSOR_INTERVAL_MS      = 1000UL;
static constexpr uint32_t DISPLAY_INTERVAL_MS     = 250UL;
static constexpr uint32_t NTP_INTERVAL_MS         = 30000UL;
static constexpr uint32_t MAIN_PAGE_ROTATE_MS     = 3000UL;

static constexpr uint32_t UI_TIMEOUT_MS            = 10000UL;
static constexpr uint32_t BUTTON_DEBOUNCE_MS       = 35UL;

static constexpr uint32_t BLYNK_FAST_INTERVAL_MS   = 5000UL;       // V2 = 1
static constexpr uint32_t BLYNK_SLOW_INTERVAL_MS   = 600000UL;    // V2 = 0

static constexpr uint32_t OTA_HTTP_TIMEOUT_MS      = 15000UL;

// 12 hours continuous uptime
static constexpr uint32_t AUTO_REBOOT_MS           = 12UL * 60UL * 60UL * 1000UL;

// 10 second watchdog
static constexpr uint32_t WDT_TIMEOUT_SECONDS      = 10UL;

// ================================================================
// WiFi provisioning
// ================================================================

static constexpr char AP_SSID[] = "SmartMonitor-Setup";
// Open AP: easier provisioning from a phone.
// If you prefer WPA2 AP, add a password >= 8 characters to startConfigPortal().
static constexpr char AP_PASSWORD[] = "";

WiFiManager wifiManager;
Preferences wifiPreferences;

// We keep a second, application-owned copy of the WiFi credentials.
// This makes our connection logic independent from WiFiManager's internal
// storage and, importantly, allows us to verify whether credentials exist
// before deciding to start the captive portal.
static constexpr char WIFI_PREF_NAMESPACE[] = "wifi_cfg";
static constexpr char WIFI_PREF_SSID_KEY[]  = "ssid";
static constexpr char WIFI_PREF_PASS_KEY[]  = "pass";

String savedWiFiSSID;
String savedWiFiPassword;

// ================================================================
// Sensors / Display
// ================================================================

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
Adafruit_AHTX0 aht20;
Adafruit_BMP280 bmp280;
BH1750 bh1750;

bool oledOk       = false;
bool ahtOk        = false;
bool bmpOk        = false;
bool bh1750Ok     = false;

// Blynk V9: OLED display control (1=ON, 0=OFF)
bool oledEnabled = true;

// ================================================================
// NTP
// ================================================================

// Vietnam UTC+7
WiFiUDP ntpUDP;
NTPClient ntpClient(ntpUDP, "pool.ntp.org", 7L * 3600L, 60000UL);

// ================================================================
// Sensor data
// ================================================================

struct SensorData {
  float temperatureC   = NAN;
  float humidityPct    = NAN;
  float pressureHpa    = NAN;
  float altitudeM      = NAN;
  float lux             = NAN;
  uint16_t rainAnalog   = 0;
  bool rainActive       = false;
  float chipTempC       = NAN;
};

SensorData sensors;

// ================================================================
// UI state
// ================================================================

enum DisplayState : uint8_t {
  DISPLAY_MAIN = 0,
  DISPLAY_NETWORK = 1,
  DISPLAY_DETAILS = 2
};

DisplayState displayState = DISPLAY_MAIN;

uint32_t lastButtonChangeMs = 0;
uint32_t lastUserInteractionMs = 0;
uint32_t lastDisplayMs = 0;
uint32_t lastMainPageChangeMs = 0;

bool buttonRawState = HIGH;
bool buttonStableState = HIGH;

uint8_t mainSubPage = 0;


// ================================================================
// WiFi / Blynk state
// ================================================================

bool initialWiFiConnectInProgress = false;
bool configPortalActive = false;
uint32_t wifiConnectStartMs = 0;
uint32_t lastWiFiReconnectMs = 0;

bool blynkConfigured = false;
uint32_t lastBlynkReconnectMs = 0;

bool fastBlynkMode = false;
uint32_t nextBlynkSendMs = 0;

bool wifiCredentialsAreSaved = false;

// ================================================================
// OTA state
// ================================================================

volatile bool otaRequested = false;
String otaURL;
bool otaInProgress = false;

// ================================================================
// WDT state
// ================================================================

bool loopTaskWatchdogSubscribed = false;

// ================================================================
// Uptime / reboot
// ================================================================

uint32_t bootMillis = 0;

// ================================================================
// Helpers
// ================================================================

static void feedWatchdog() {
  if (loopTaskWatchdogSubscribed) {
    esp_task_wdt_reset();
  }
}

static void oledHeader(const char* title) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(title);
}

static void showBootMessage(const char* line1, const char* line2 = nullptr) {
  if (!oledOk || !oledEnabled) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("Smart Environmental");

  display.setCursor(0, 16);
  display.println(line1);

  if (line2 != nullptr) {
    display.setCursor(0, 30);
    display.println(line2);
  }

  display.display();
}

static void showAPPortalScreen() {
  if (!oledOk || !oledEnabled) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.println("WiFi SETUP MODE");

  display.setCursor(0, 14);
  display.print("SSID: ");
  display.println(AP_SSID);

  display.setCursor(0, 28);
  display.print("IP: ");
  display.println(WiFi.softAPIP());

  display.setCursor(0, 42);
  display.println("Open browser / captive");

  display.setCursor(0, 54);
  display.println("No QR code required");

  display.display();
}

static bool timeIsAvailable() {
  return WiFi.status() == WL_CONNECTED && ntpClient.isTimeSet();
}

static void drawClock() {
  if (!timeIsAvailable()) {
    display.print("--:--:--");
    return;
  }
  display.print(ntpClient.getFormattedTime());
}

static void drawMainPage0() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // NTP clock on the top line only.
  display.setCursor(0, 0);
  drawClock();

  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  // Temperature + humidity
  display.setCursor(0, 13);
  display.print("T: ");
  if (isnan(sensors.temperatureC)) display.print("--.-");
  else display.print(sensors.temperatureC, 1);
  display.print(" C");

  display.setCursor(67, 13);
  display.print("RH: ");
  if (isnan(sensors.humidityPct)) display.print("--");
  else display.print(sensors.humidityPct, 0);
  display.print("%");

  // Light
  display.setCursor(0, 25);
  display.print("Lux: ");
  if (isnan(sensors.lux)) display.print("----");
  else display.print(sensors.lux, 1);
  display.print(" lx");

  // Pressure
  display.setCursor(0, 37);
  display.print("P: ");
  if (isnan(sensors.pressureHpa)) display.print("----");
  else display.print(sensors.pressureHpa, 0);
  display.print(" hPa");

  // Rain
  display.setCursor(0, 49);
  display.print("Rain: ");
  display.print(sensors.rainActive ? "YES" : "NO");

  display.display();
}

static void drawMainPage1() {
  // Kept for source compatibility; MAIN now uses a single page.
  drawMainPage0();
}

static void drawNetworkPage() {
  drawDetailsPage();
}

static void drawDetailsPage() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  oledHeader("WIFI DETAILS");

  // SSID
  String ssid = wifiManager.getWiFiSSID(true);
  display.setCursor(0, 12);
  display.print("SSID: ");
  if (ssid.length() == 0) display.println("--");
  else display.println(ssid);

  // Saved WiFi password
  String password = wifiManager.getWiFiPass(true);
  display.setCursor(0, 25);
  display.print("PASS: ");
  if (password.length() == 0) display.println("--");
  else display.println(password);

  // ESP32 IP address
  display.setCursor(0, 38);
  display.print("IP: ");
  if (WiFi.status() == WL_CONNECTED) {
    display.println(WiFi.localIP());
  } else if (configPortalActive) {
    display.println(WiFi.softAPIP());
  } else {
    display.println("--");
  }

  // Blynk connection state
  display.setCursor(0, 51);
  display.print("Blynk: ");
  display.println(Blynk.connected() ? "CONNECTED" : "OFFLINE");

  display.display();
}

static void drawSystemStatusPage() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  oledHeader("SYSTEM STATUS");

  const uint32_t uptimeSeconds = (millis() - bootMillis) / 1000UL;
  uint32_t totalSeconds = uptimeSeconds;

  uint32_t days = totalSeconds / 86400UL;
  totalSeconds %= 86400UL;

  uint32_t hours = totalSeconds / 3600UL;
  totalSeconds %= 3600UL;

  uint32_t minutes = totalSeconds / 60UL;
  uint32_t seconds = totalSeconds % 60UL;

  // Display is limited to 99d 99h 99m 60s.
  if (days > 99UL) days = 99UL;
  if (hours > 99UL) hours = 99UL;
  if (minutes > 99UL) minutes = 99UL;
  if (seconds > 60UL) seconds = 60UL;

  display.setCursor(0, 18);
  display.print("Since reset:");

  display.setCursor(0, 31);
  if (days < 10) display.print('0');
  display.print(days);
  display.print("d ");

  if (hours < 10) display.print('0');
  display.print(hours);
  display.print("h ");

  if (minutes < 10) display.print('0');
  display.print(minutes);
  display.print("m ");

  if (seconds < 10) display.print('0');
  display.print(seconds);
  display.print("s");

  display.setCursor(0, 46);
  display.print("CPU Temp: ");
  if (isnan(sensors.chipTempC)) display.print("--.-");
  else display.print(sensors.chipTempC, 1);
  display.print(" C");

  display.display();
}

static void drawOTAProgress(int percent, const char* stateText) {
  if (!oledOk || !oledEnabled) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.println("Updating Firmware...");

  display.setCursor(0, 15);
  display.println(stateText);

  // Border
  display.drawRect(4, 30, 120, 12, SSD1306_WHITE);

  int barWidth = (percent * 116) / 100;
  if (barWidth > 116) barWidth = 116;
  if (barWidth > 0) {
    display.fillRect(6, 32, barWidth, 8, SSD1306_WHITE);
  }

  display.setCursor(49, 49);
  display.print(percent);
  display.print("%");

  display.display();
}

static void drawCurrentUI() {
  if (!oledOk || !oledEnabled) return;

  switch (displayState) {
    case DISPLAY_MAIN:
      // One fixed main screen; no automatic sub-page rotation.
      drawMainPage0();
      break;

    case DISPLAY_NETWORK:
      // Screen 2: WiFi + Blynk information.
      drawNetworkPage();
      break;

    case DISPLAY_DETAILS:
      // Screen 3: uptime since reset + ESP32 CPU temperature.
      drawSystemStatusPage();
      break;
  }
}

// ================================================================
// OLED control from Blynk V9
// ================================================================

static void setOledEnabled(bool enabled) {
  oledEnabled = enabled;

  if (!oledOk) return;

  if (oledEnabled) {
    display.ssd1306_command(SSD1306_DISPLAYON);
    lastDisplayMs = 0;
    drawCurrentUI();
  } else {
    display.clearDisplay();
    display.display();
    display.ssd1306_command(SSD1306_DISPLAYOFF);
  }
}

// ================================================================
// Sensor handling
// ================================================================

static float readInternalChipTemperature() {
  // ESP32-C3 supports temperatureRead() in the selected Arduino core.
  // Returns Celsius.
  return temperatureRead();
}

static void readSensors() {
  sensors.rainAnalog = analogRead(PIN_RAIN_ANALOG);

  // Common rain-module convention: D0 LOW = rain detected.
  sensors.rainActive = (digitalRead(PIN_RAIN_DIGITAL) == LOW);

  if (ahtOk) {
    sensors_event_t humidityEvent;
    sensors_event_t temperatureEvent;
    aht20.getEvent(&humidityEvent, &temperatureEvent);

    sensors.temperatureC = temperatureEvent.temperature;
    sensors.humidityPct = humidityEvent.relative_humidity;
  }

  if (bmpOk) {
    sensors.pressureHpa = bmp280.readPressure() / 100.0F;
    sensors.altitudeM = bmp280.readAltitude(1013.25F);
  }

  if (bh1750Ok) {
    const float value = bh1750.readLightLevel();
    if (value >= 0.0F) {
      sensors.lux = value;
    }
  }

  sensors.chipTempC = readInternalChipTemperature();
}

// ================================================================
// Button / UI
// ================================================================

static void advanceDisplayState() {
  displayState =
      static_cast<DisplayState>((static_cast<uint8_t>(displayState) + 1U) % 3U);


  lastUserInteractionMs = millis();

  if (displayState == DISPLAY_MAIN) {
    mainSubPage = 0;
    lastMainPageChangeMs = millis();
  }
}

static void handleButton() {
  const uint32_t now = millis();
  const bool rawPressed = (digitalRead(PIN_BUTTON) == LOW);

  if (rawPressed != buttonRawState) {
    buttonRawState = rawPressed;
    lastButtonChangeMs = now;
  }

  if ((uint32_t)(now - lastButtonChangeMs) >= BUTTON_DEBOUNCE_MS) {
    if (buttonStableState != buttonRawState) {
      buttonStableState = buttonRawState;

      // Trigger only on press (HIGH -> LOW because INPUT_PULLUP).
      if (buttonStableState == LOW) {
        advanceDisplayState();
      }
    }
  }

  if (displayState != DISPLAY_MAIN &&
      (uint32_t)(now - lastUserInteractionMs) >= UI_TIMEOUT_MS) {
    displayState = DISPLAY_MAIN;
    mainSubPage = 0;
    lastMainPageChangeMs = now;
  }
}

static void serviceMainPageRotation() {
  // MAIN is now a single fixed OLED page.
  // Kept as a no-op to preserve the original program structure.
  if (displayState != DISPLAY_MAIN) return;
  mainSubPage = 0;
}

// ================================================================
// WiFiManager callbacks
// ================================================================

// Forward declarations used by WiFiManager callbacks / startup flow.
static bool saveWiFiCredentialsToNVS(const String& ssid, const String& password);
static void startProvisioningPortal();

static void onWiFiManagerAPStart(WiFiManager* manager) {
  (void)manager;

  configPortalActive = true;
  showAPPortalScreen();

  Serial.println();
  Serial.println(F("=== WiFi provisioning AP ==="));
  Serial.print(F("SSID: "));
  Serial.println(AP_SSID);
  Serial.print(F("IP: "));
  Serial.println(WiFi.softAPIP());
}

static void onWiFiManagerSave() {
  const String ssid = wifiManager.getWiFiSSID(true);
  const String password = wifiManager.getWiFiPass(true);

  if (saveWiFiCredentialsToNVS(ssid, password)) {
    Serial.println(F("WiFi credentials saved and copied to application NVS."));
  } else {
    Serial.println(F("WiFi credentials saved by WiFiManager, but application NVS save failed."));
  }
}

// ================================================================
// WiFi provisioning / reconnect
// ================================================================

static bool saveWiFiCredentialsToNVS(const String& ssid, const String& password) {
  if (ssid.length() == 0) {
    Serial.println(F("Cannot save WiFi: SSID is empty."));
    return false;
  }

  if (!wifiPreferences.begin(WIFI_PREF_NAMESPACE, false)) {
    Serial.println(F("Cannot open WiFi Preferences namespace."));
    return false;
  }

  const size_t ssidWritten = wifiPreferences.putString(WIFI_PREF_SSID_KEY, ssid);
  wifiPreferences.putString(WIFI_PREF_PASS_KEY, password);
  wifiPreferences.end();

  // Password may legitimately be empty for an open WiFi network.
  const bool ok = (ssidWritten > 0);

  if (ok) {
    savedWiFiSSID = ssid;
    savedWiFiPassword = password;
    wifiCredentialsAreSaved = true;

    Serial.print(F("WiFi credentials saved to application NVS. SSID="));
    Serial.println(savedWiFiSSID);
  } else {
    Serial.println(F("WiFi credentials could not be written to application NVS."));
  }

  return ok;
}

static bool loadWiFiCredentialsFromNVS() {
  savedWiFiSSID = "";
  savedWiFiPassword = "";

  if (!wifiPreferences.begin(WIFI_PREF_NAMESPACE, true)) {
    Serial.println(F("Application WiFi NVS namespace not available."));
  } else {
    savedWiFiSSID = wifiPreferences.getString(WIFI_PREF_SSID_KEY, "");
    savedWiFiPassword = wifiPreferences.getString(WIFI_PREF_PASS_KEY, "");
    wifiPreferences.end();
  }

  if (savedWiFiSSID.length() > 0) {
    wifiCredentialsAreSaved = true;
    Serial.print(F("Found application-saved WiFi credentials. SSID="));
    Serial.println(savedWiFiSSID);
    return true;
  }

  // Migration path:
  // older firmware may have WiFiManager credentials but not our own copy.
  const String wmSSID = wifiManager.getWiFiSSID(true);
  const String wmPassword = wifiManager.getWiFiPass(true);

  if (wmSSID.length() > 0) {
    Serial.print(F("Found WiFiManager credentials. Migrating them to application NVS. SSID="));
    Serial.println(wmSSID);

    return saveWiFiCredentialsToNVS(wmSSID, wmPassword);
  }

  wifiCredentialsAreSaved = false;
  Serial.println(F("No saved WiFi credentials found anywhere."));
  return false;
}

static void startInitialWiFiAttempt() {
  // IMPORTANT:
  // We only enter WiFiManager setup mode when no saved credentials exist.
  // A failed connection to an existing WiFi network is handled by retrying;
  // it does NOT erase credentials and does NOT open the captive portal.
  if (!loadWiFiCredentialsFromNVS()) {
    startProvisioningPortal();
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);

  // Do not let WiFi.begin() overwrite our application-owned credentials.
  // They are explicitly supplied from NVS.
  WiFi.persistent(false);

  Serial.println(F("Starting saved WiFi connection..."));
  Serial.print(F("SSID: "));
  Serial.println(savedWiFiSSID);
  WiFi.begin(savedWiFiSSID.c_str(), savedWiFiPassword.c_str());

  // ESP32-C3 SuperMini hardware workaround.
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  initialWiFiConnectInProgress = true;
  wifiConnectStartMs = millis();
}

static void startProvisioningPortal() {
  Serial.println(F("No saved WiFi credentials found."));
  Serial.println(F("Starting non-blocking WiFiManager captive portal..."));

  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setConfigPortalTimeout(0);       // User closes it or we keep processing.
  wifiManager.setAPClientCheck(false);
  wifiManager.setWebPortalClientCheck(true);
  wifiManager.setShowPassword(false);          // Do not expose saved WiFi password.
  wifiManager.setDebugOutput(false);

  wifiManager.setAPCallback(onWiFiManagerAPStart);
  wifiManager.setSaveConfigCallback(onWiFiManagerSave);

  // This returns immediately because blocking=false.
  wifiManager.startConfigPortal(AP_SSID, AP_PASSWORD);

  configPortalActive = wifiManager.getConfigPortalActive();

  if (configPortalActive) {
    showAPPortalScreen();
  }
}

static void onWiFiConnected() {
  initialWiFiConnectInProgress = false;
  const bool hadSavedCredentials = wifiCredentialsAreSaved;
  const String connectedSSID = WiFi.SSID();
  wifiCredentialsAreSaved = (connectedSSID.length() > 0);

  Serial.println();
  Serial.println(F("WiFi connected."));
  Serial.print(F("SSID: "));
  Serial.println(WiFi.SSID());
  Serial.print(F("IP: "));
  Serial.println(WiFi.localIP());
  Serial.print(F("RSSI: "));
  Serial.print(WiFi.RSSI());
  Serial.println(F(" dBm"));

  // Keep the application-owned copy synchronized with the actual
  // connected network. This is useful after a WiFiManager provisioning.
  if (connectedSSID.length() > 0 &&
      (savedWiFiSSID != connectedSSID || !hadSavedCredentials)) {
    const String currentPassword = wifiManager.getWiFiPass(true);
    if (currentPassword.length() > 0) {
      saveWiFiCredentialsToNVS(connectedSSID, currentPassword);
    }
  }

  // Set the TX power again after a mode/configuration transition.
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  if (!ntpClient.isTimeSet()) {
    ntpClient.begin();
  }

  if (!blynkConfigured) {
    // Explicitly use Blynk Cloud over SSL/TLS port 443.
    Blynk.config(BLYNK_AUTH_TOKEN, "blynk.cloud", 443);
    blynkConfigured = true;
  }

  lastBlynkReconnectMs = 0;
  nextBlynkSendMs = millis() + 1000UL;
}

static void serviceWiFiProvisioning() {
  const uint32_t now = millis();

  if (configPortalActive) {
    const bool connectedByPortal = wifiManager.process();

    // A successful save normally results in WL_CONNECTED.
    if (WiFi.status() == WL_CONNECTED || connectedByPortal) {
      wifiManager.stopConfigPortal();
      configPortalActive = false;
      onWiFiConnected();
      return;
    }

    // If the portal was closed/aborted, return to normal offline operation.
    if (!wifiManager.getConfigPortalActive()) {
      configPortalActive = false;
      initialWiFiConnectInProgress = false;

      WiFi.mode(WIFI_STA);
      WiFi.setTxPower(WIFI_POWER_8_5dBm);
      Serial.println(F("WiFiManager portal closed. Continuing offline."));
    }

    return;
  }

  if (initialWiFiConnectInProgress) {
    if (WiFi.status() == WL_CONNECTED) {
      onWiFiConnected();
      return;
    }

    if ((uint32_t)(now - wifiConnectStartMs) >= WIFI_INITIAL_TIMEOUT_MS) {
      // IMPORTANT:
      // Do NOT start the portal here. We already have saved credentials.
      // Continue retrying them forever until the network comes back.
      initialWiFiConnectInProgress = false;
      lastWiFiReconnectMs = now;

      Serial.println(F("Saved WiFi did not connect within 60s."));
      Serial.println(F("Keeping saved credentials and retrying; setup portal will NOT be opened."));
    }

    return;
  }

  // Normal runtime: if WiFi is lost, keep sensors/OLED fully operational.
  // Reconnect using our explicit saved credentials rather than depending
  // on whatever the WiFi library happens to have stored internally.
  if (WiFi.status() != WL_CONNECTED &&
      savedWiFiSSID.length() > 0) {
    if ((uint32_t)(now - lastWiFiReconnectMs) >= WIFI_RECONNECT_MS) {
      lastWiFiReconnectMs = now;

      Serial.println(F("WiFi disconnected. Attempting saved-credential reconnect..."));

      WiFi.mode(WIFI_STA);
      WiFi.setAutoReconnect(true);
      WiFi.persistent(false);
      WiFi.begin(savedWiFiSSID.c_str(), savedWiFiPassword.c_str());
      WiFi.setTxPower(WIFI_POWER_8_5dBm);

      initialWiFiConnectInProgress = true;
      wifiConnectStartMs = now;
    }
  }
}

// ================================================================
// NTP
// ================================================================

static void serviceNTP() {
  static uint32_t lastNtpCallMs = 0;
  const uint32_t now = millis();

  if (WiFi.status() != WL_CONNECTED) return;

  if ((uint32_t)(now - lastNtpCallMs) < NTP_INTERVAL_MS) return;
  lastNtpCallMs = now;

  ntpClient.update();
}

// ================================================================
// Blynk
// ================================================================

BLYNK_CONNECTED() {
  Serial.println(F("Blynk connected."));
  Blynk.syncVirtual(V2);
  Blynk.syncVirtual(V9);


  // Send current values quickly after a connection.
  nextBlynkSendMs = millis() + 500UL;
}

BLYNK_DISCONNECTED() {
  Serial.println(F("Blynk disconnected."));
}

BLYNK_WRITE(V2) {
  fastBlynkMode = (param.asInt() != 0);

  Serial.print(F("Blynk interval mode V2 = "));
  Serial.println(fastBlynkMode ? F("FAST (5s)") : F("SLOW (10min)"));

  // Apply mode change immediately.
  nextBlynkSendMs = millis() + 100UL;
}

BLYNK_WRITE(V9) {
  const bool enableOLED = (param.asInt() != 0);

  Serial.print(F("Blynk V9 OLED = "));
  Serial.println(enableOLED ? F("ON (1)") : F("OFF (0)"));

  setOledEnabled(enableOLED);
}

// Blynk V10 is a 0/1 button.
// Pressing it once performs exactly one action,
// identical to pressing the physical D10 once.
BLYNK_WRITE(V10) {
  if (param.asInt() == 1) {
    Serial.println(F("Blynk V10 pressed -> virtual D10 press"));
    advanceDisplayState();
  }
}

static void sendSensorDataToBlynk() {
  if (!Blynk.connected()) return;

  // Match the supplied Blynk datastream mapping.
  if (!isnan(sensors.temperatureC)) Blynk.virtualWrite(V0, sensors.temperatureC);
  if (!isnan(sensors.humidityPct))  Blynk.virtualWrite(V1, sensors.humidityPct);

  Blynk.virtualWrite(V2, fastBlynkMode ? 1 : 0);

  if (!isnan(sensors.altitudeM))    Blynk.virtualWrite(V3, sensors.altitudeM);
  if (!isnan(sensors.pressureHpa))  Blynk.virtualWrite(V4, (int)lroundf(sensors.pressureHpa));

  Blynk.virtualWrite(V5, sensors.rainAnalog);
  Blynk.virtualWrite(V6, sensors.rainActive ? 1 : 0);

  if (!isnan(sensors.lux))          Blynk.virtualWrite(V7, sensors.lux);
  if (!isnan(sensors.chipTempC))    Blynk.virtualWrite(V8, sensors.chipTempC);

  Serial.println(F("Blynk sensor update sent."));
}

static void serviceBlynk() {
  const uint32_t now = millis();

  // If WiFi disappears, explicitly drop the old Blynk socket state.
  if (WiFi.status() != WL_CONNECTED || configPortalActive) {
    if (Blynk.connected()) {
      Blynk.disconnect();
      Serial.println(F("Blynk disconnected because WiFi is unavailable."));
    }
    return;
  }

  if (!blynkConfigured) {
    return;
  }

  if (!Blynk.connected()) {
    if ((uint32_t)(now - lastBlynkReconnectMs) >= BLYNK_RECONNECT_MS) {
      lastBlynkReconnectMs = now;

      Serial.println(F("Trying Blynk reconnect..."));

      // Give the SSL connection enough time to complete.
      const bool connected = Blynk.connect(5000);

      Serial.println(connected ? F("Blynk reconnect OK.")
                               : F("Blynk reconnect failed."));
    }
    return;
  }

  // Must be called very frequently while online. This also handles
  // Blynk heartbeat/ping traffic and incoming callbacks.
  Blynk.run();

  // If the transport dropped during Blynk.run(), do not wait for a
  // full reconnect interval before allowing the next reconnect attempt.
  if (!Blynk.connected()) {
    lastBlynkReconnectMs = now;
    Serial.println(F("Blynk transport lost; reconnect scheduled."));
    return;
  }

  // Blynk data timer is intentionally independent from OLED refresh.
  if ((int32_t)(now - nextBlynkSendMs) >= 0) {
    sendSensorDataToBlynk();

    const uint32_t interval =
      fastBlynkMode ? BLYNK_FAST_INTERVAL_MS : BLYNK_SLOW_INTERVAL_MS;

    nextBlynkSendMs = now + interval;
  }
}

// ================================================================
// Blynk.Air OTA
// ================================================================

/*
  Blynk.Air sends a firmware download URL through InternalPinOTA.
  We only set a flag in the callback. The actual HTTP/update work is
  executed from loop(), which keeps the callback short and predictable.
*/
BLYNK_WRITE(InternalPinOTA) {
  otaURL = param.asString();

  if (otaURL.length() == 0) {
    Serial.println(F("OTA callback received empty URL."));
    return;
  }

  otaRequested = true;

  Serial.println();
  Serial.println(F("=== Blynk.Air OTA requested ==="));
  Serial.print(F("URL: "));
  Serial.println(otaURL);
}

static bool suspendLoopTaskWatchdogForOTA(bool& restoreAfterOTA) {
  restoreAfterOTA = false;

  if (!loopTaskWatchdogSubscribed) {
    return true;
  }

  const esp_err_t err = esp_task_wdt_delete(NULL);

  if (err == ESP_OK) {
    restoreAfterOTA = true;
    loopTaskWatchdogSubscribed = false;
    return true;
  }

  Serial.print(F("Could not suspend loop watchdog, err="));
  Serial.println((int)err);

  return false;
}

static void restoreLoopTaskWatchdog(bool restoreAfterOTA) {
  if (!restoreAfterOTA) return;

  const esp_err_t err = esp_task_wdt_add(NULL);

  if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
    loopTaskWatchdogSubscribed = true;
  } else {
    Serial.print(F("Warning: watchdog re-add failed, err="));
    Serial.println((int)err);
  }
}

static bool performBlynkAirOTA(const String& url) {
  otaInProgress = true;

  drawOTAProgress(0, "Preparing...");

  // Stop any config portal and Blynk traffic before the firmware transfer.
  if (configPortalActive) {
    wifiManager.stopConfigPortal();
    configPortalActive = false;
  }

  Blynk.disconnect();

  bool restoreWatchdog = false;
  if (!suspendLoopTaskWatchdogForOTA(restoreWatchdog)) {
    // If the watchdog cannot be detached, we can still continue, but
    // the transfer loop below feeds it regularly.
    Serial.println(F("Continuing OTA with active watchdog."));
  }

  HTTPClient http;
  http.setConnectTimeout(OTA_HTTP_TIMEOUT_MS);
  http.setTimeout(OTA_HTTP_TIMEOUT_MS);
  http.setReuse(false);

  bool success = false;

  do {
    Serial.println(F("Starting firmware download..."));

    // This follows the Blynk.Edgent OTA implementation style.
    if (!http.begin(url)) {
      Serial.println(F("HTTP begin() failed."));
      break;
    }

    const char* headerKeys[] = {"x-MD5"};
    http.collectHeaders(headerKeys, 1);

    drawOTAProgress(1, "Downloading...");

    const int httpCode = http.GET();

    if (httpCode != HTTP_CODE_OK) {
      Serial.print(F("OTA HTTP error: "));
      Serial.println(httpCode);
      break;
    }

    const int contentLength = http.getSize();
    if (contentLength <= 0) {
      Serial.println(F("OTA Content-Length invalid."));
      break;
    }

    Serial.print(F("Firmware size: "));
    Serial.print(contentLength);
    Serial.println(F(" bytes"));

    if (!Update.begin((size_t)contentLength, U_FLASH)) {
      Serial.print(F("Update.begin failed, error="));
      Serial.println(Update.getError());
      break;
    }

    if (http.hasHeader("x-MD5")) {
      String md5 = http.header("x-MD5");
      md5.toLowerCase();

      if (md5.length() == 32) {
        Update.setMD5(md5.c_str());
        Serial.print(F("MD5: "));
        Serial.println(md5);
      }
    }

    Client& stream = http.getStream();

    uint8_t buffer[2048];
    size_t writtenTotal = 0;
    uint32_t lastProgressDrawMs = 0;
    int lastPercent = -1;

    while (writtenTotal < (size_t)contentLength) {
      // Safety: if watchdog is still active, feed it frequently.
      feedWatchdog();

      const size_t availableBytes = stream.available();

      if (availableBytes > 0) {
        const size_t toRead =
          (availableBytes > sizeof(buffer)) ? sizeof(buffer) : availableBytes;

        const size_t readBytes = stream.readBytes(buffer, toRead);

        if (readBytes == 0) {
          Serial.println(F("OTA stream read returned 0."));
          break;
        }

        const size_t written = Update.write(buffer, readBytes);

        if (written != readBytes) {
          Serial.print(F("Flash write error. Written="));
          Serial.print(written);
          Serial.print(F(" expected="));
          Serial.println(readBytes);
          break;
        }

        writtenTotal += written;

        const int percent =
          (int)((writtenTotal * 100ULL) / (size_t)contentLength);

        const uint32_t now = millis();

        if (percent != lastPercent &&
            (uint32_t)(now - lastProgressDrawMs) >= 100UL) {
          lastPercent = percent;
          lastProgressDrawMs = now;
          drawOTAProgress(percent, "Flashing...");
        }
      } else {
        // Non-blocking yield while waiting for the network.
        feedWatchdog();
        yield();
      }
    }

    if (writtenTotal != (size_t)contentLength) {
      Serial.println(F("OTA transfer incomplete."));
      break;
    }

    if (!Update.end()) {
      Serial.print(F("Update.end failed, error="));
      Serial.println(Update.getError());
      break;
    }

    if (!Update.isFinished()) {
      Serial.println(F("OTA image is not marked finished."));
      break;
    }

    success = true;

  } while (false);

  http.end();

  restoreLoopTaskWatchdog(restoreWatchdog);

  if (success) {
    drawOTAProgress(100, "Update complete");
    Serial.println(F("=== OTA SUCCESS ==="));
    Serial.println(F("Rebooting..."));
    delay(300);
    ESP.restart();
  }

  otaInProgress = false;

  if (oledOk) {
    showBootMessage("OTA FAILED", "Continuing normal operation");
  }

  Serial.println(F("=== OTA FAILED ==="));
  return false;
}

static void serviceOTA() {
  if (!otaRequested || otaInProgress) return;

  otaRequested = false;

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("OTA ignored: WiFi is offline."));
    return;
  }

  const String urlCopy = otaURL;
  performBlynkAirOTA(urlCopy);
}

// ================================================================
// Watchdog
// ================================================================

static void setupWatchdog() {
  /*
    ESP32 Arduino Core 2.0.16 uses the ESP-IDF 4.x Task WDT API:
      esp_task_wdt_init(timeout_seconds, panic)
      esp_task_wdt_add(NULL)

    The code tolerates the case where Arduino/core has already
    initialized the watchdog.
  */

  esp_err_t initErr = esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);

  if (initErr == ESP_OK) {
    Serial.println(F("Task WDT initialized: 10s / panic reset."));
  } else if (initErr == ESP_ERR_INVALID_STATE) {
    Serial.println(F("Task WDT was already initialized by the core."));
  } else {
    Serial.print(F("Task WDT init error="));
    Serial.println((int)initErr);
  }

  const esp_err_t addErr = esp_task_wdt_add(NULL);

  if (addErr == ESP_OK) {
    loopTaskWatchdogSubscribed = true;
    Serial.println(F("loopTask added to Task WDT."));
  } else if (addErr == ESP_ERR_INVALID_STATE) {
    // Already registered.
    loopTaskWatchdogSubscribed = true;
    Serial.println(F("loopTask already subscribed to Task WDT."));
  } else {
    Serial.print(F("Task WDT add error="));
    Serial.println((int)addErr);
    loopTaskWatchdogSubscribed = false;
  }
}

// ================================================================
// Setup
// ================================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println(F("=============================================="));
  Serial.println(F("Smart Environmental Monitor"));
  Serial.println(F("ESP32-C3 SuperMini / Core 2.0.16"));
  Serial.println(F("=============================================="));

  bootMillis = millis();

  // ----------------------------
  // GPIO
  // ----------------------------
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_RAIN_DIGITAL, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_RAIN_ANALOG, ADC_11db);

  // ----------------------------
  // I2C
  // ----------------------------
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000UL);

  // ----------------------------
  // OLED
  // ----------------------------
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

  if (oledOk) {
    oledEnabled = true;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(F("Smart Environmental"));
    display.println(F("Monitor starting..."));
    display.display();
  }

  // ----------------------------
  // Sensors
  // ----------------------------
  showBootMessage("Initializing sensors...");

  ahtOk = aht20.begin(&Wire);
  Serial.print(F("AHT20: "));
  Serial.println(ahtOk ? F("OK") : F("FAIL"));

  bh1750Ok = bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE,
                          BH1750_ADDR,
                          &Wire);

  Serial.print(F("BH1750: "));
  Serial.println(bh1750Ok ? F("OK") : F("FAIL"));

  bmpOk = bmp280.begin(BMP280_ADDR_1);
  if (!bmpOk) {
    bmpOk = bmp280.begin(BMP280_ADDR_2);
  }

  Serial.print(F("BMP280: "));
  Serial.println(bmpOk ? F("OK") : F("FAIL"));

  if (bmpOk) {
    bmp280.setSampling(
      Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X2,
      Adafruit_BMP280::SAMPLING_X16,
      Adafruit_BMP280::FILTER_X16,
      Adafruit_BMP280::STANDBY_MS_500
    );
  }

  // First sensor sample before WiFi provisioning.
  readSensors();
  drawCurrentUI();

  // ----------------------------
  // Watchdog
  // ----------------------------
  setupWatchdog();
  feedWatchdog();

  // ----------------------------
  // WiFi
  // ----------------------------
  showBootMessage("Connecting WiFi...", "Timeout: 60 seconds");

  // CRITICAL HARDWARE FIX:
  // Set reduced TX power immediately after WiFi initialization.
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  // Initialize WiFiManager callbacks/settings.
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setDebugOutput(false);
  wifiManager.setShowPassword(false);
  wifiManager.setAPClientCheck(false);
  wifiManager.setWebPortalClientCheck(true);

  wifiManager.setAPCallback(onWiFiManagerAPStart);
  wifiManager.setSaveConfigCallback(onWiFiManagerSave);

  // Prefer our application-owned NVS credentials. If none exist,
  // migrate the old WiFiManager credentials before deciding whether
  // setup mode is necessary.
  startInitialWiFiAttempt();

  // ----------------------------
  // Timers / UI
  // ----------------------------
  lastUserInteractionMs = millis();
  lastMainPageChangeMs = millis();
  lastDisplayMs = 0;
}

// ================================================================
// Main loop
// ================================================================

void loop() {
  const uint32_t now = millis();

  // Feed WDT early every loop.
  feedWatchdog();

  // ----------------------------
  // Hardware / sensor servicing
  // ----------------------------

  static uint32_t lastSensorMs = 0;

  if ((uint32_t)(now - lastSensorMs) >= SENSOR_INTERVAL_MS) {
    lastSensorMs = now;
    readSensors();
  }

  handleButton();
  serviceMainPageRotation();

  // ----------------------------
  // Network provisioning
  // ----------------------------

  serviceWiFiProvisioning();

  // ----------------------------
  // NTP + Blynk + OTA
  // ----------------------------

  if (!configPortalActive) {
    // Keep Blynk serviced before NTP so the SSL connection is not
    // unnecessarily delayed by network time synchronization.
    serviceBlynk();
    serviceNTP();
    serviceOTA();
  }

  // ----------------------------
  // OLED refresh (independent from Blynk update timing)
  // ----------------------------

  if ((uint32_t)(now - lastDisplayMs) >= DISPLAY_INTERVAL_MS) {
    lastDisplayMs = now;

    if (oledEnabled) {
      if (configPortalActive) {
        showAPPortalScreen();
      } else if (!otaInProgress) {
        drawCurrentUI();
      }
    }
  }

  // ----------------------------
  // 12-hour anti-fragmentation reboot
  // ----------------------------
  if (!otaInProgress &&
      (uint32_t)(now - bootMillis) >= AUTO_REBOOT_MS) {
    Serial.println(F("12-hour uptime reached. Restarting..."));
    delay(100);
    ESP.restart();
  }

  // Let FreeRTOS/network tasks run.
  yield();
}
