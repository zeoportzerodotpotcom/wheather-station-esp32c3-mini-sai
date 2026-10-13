#define BLYNK_PRINT Serial

#define BLYNK_FIRMWARE_VERSION "1.1.1"

#define BLYNK_TIMEOUT_MS 6000UL

#define BLYNK_HEARTBEAT 20

// Define Blynk template settings BEFORE including any Blynk headers.

// Replace these placeholders with the real values from your Blynk device.

#define BLYNK_TEMPLATE_ID ""
#define BLYNK_TEMPLATE_NAME ""
#define BLYNK_AUTH_TOKEN ""

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

#include "esp_sleep.h"

#include "esp_system.h"

#include <time.h>

#include <math.h>

// ================================================================

// Hardware and timing

// ================================================================

static constexpr uint8_t PIN_I2C_SDA = 1;

static constexpr uint8_t PIN_I2C_SCL = 0;

static constexpr uint8_t PIN_RAIN_ANALOG = 3;

static constexpr uint8_t PIN_RAIN_DIGITAL = 4;

static constexpr uint8_t PIN_BUTTON = 10;

static constexpr uint8_t OLED_WIDTH = 128;

static constexpr uint8_t OLED_HEIGHT = 64;

static constexpr int8_t OLED_RESET = -1;

static constexpr uint8_t OLED_ADDR = 0x3C;

static constexpr uint8_t BH1750_ADDR = 0x23;

static constexpr uint8_t BMP280_ADDR_1 = 0x76;

static constexpr uint8_t BMP280_ADDR_2 = 0x77;

static constexpr uint32_t WIFI_NO_CONNECTION_AP_MS = 30000UL;

static constexpr uint32_t WIFI_RECONNECT_MS = 15000UL;

static constexpr uint32_t BLYNK_RECONNECT_MS = 5000UL;
static constexpr uint32_t BLYNK_MAINTENANCE_INTERVAL_MS =
    30UL * 60UL * 1000UL;

static constexpr uint32_t BLYNK_CONNECT_TIMEOUT_MS = 6000UL;

static constexpr uint32_t BLYNK_OFFLINE_RESTART_MS = 180000UL;

static constexpr uint32_t NETWORK_STARTUP_GRACE_MS = 30000UL;

static constexpr uint32_t SENSOR_INTERVAL_MS = 1000UL;

static constexpr uint32_t DISPLAY_INTERVAL_MS = 250UL;

static constexpr uint32_t NTP_INTERVAL_MS = 60000UL;

static constexpr uint32_t UI_TIMEOUT_MS = 10000UL;

static constexpr uint32_t BUTTON_DEBOUNCE_MS = 35UL;

static constexpr uint32_t BLYNK_FAST_INTERVAL_MS = 5000UL;

static constexpr uint32_t BLYNK_SLOW_INTERVAL_MS = 600000UL;

static constexpr uint32_t OTA_HTTP_TIMEOUT_MS = 15000UL;

static constexpr uint32_t AUTO_REBOOT_MS = 12UL * 60UL * 60UL * 1000UL;

static constexpr uint32_t WDT_TIMEOUT_SECONDS = 35UL;

static constexpr uint32_t MANUAL_DEEP_SLEEP_SECONDS = 300UL;

static constexpr uint64_t MANUAL_DEEP_SLEEP_US =

    (uint64_t)MANUAL_DEEP_SLEEP_SECONDS * 1000000ULL;

static constexpr char AP_SSID[] = "SmartMonitor-Setup";

static constexpr char AP_PASSWORD[] = ""; // Open setup AP

static constexpr char WIFI_PREF_NAMESPACE[] = "wifi_cfg";

static constexpr char WIFI_PREF_SSID_KEY[] = "ssid";

static constexpr char WIFI_PREF_PASS_KEY[] = "pass";

static constexpr char RESET_PREF_NAMESPACE[] = "reset_ctrl";

static constexpr char RESET_PREF_KEY[] = "pending_sleep";

static constexpr char BLYNK_CONNECTION_EVENT_CODE[] = "blynk_connection_log";

WiFiManager wifiManager;

Preferences wifiPreferences;

Preferences resetPreferences;

String savedWiFiSSID;

String savedWiFiPassword;

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

Adafruit_AHTX0 aht20;

Adafruit_BMP280 bmp280;

BH1750 bh1750;

WiFiUDP ntpUDP;

NTPClient ntpClient(ntpUDP, "pool.ntp.org", 7L * 3600L, 60000UL);

struct SensorData {

  float temperatureC = NAN;

  float humidityPct = NAN;

  float pressureHpa = NAN;

  float altitudeM = NAN;

  float lux = NAN;

  uint16_t rainAnalog = 0;

  bool rainActive = false;

  float chipTempC = NAN;

};

SensorData sensors;

bool oledOk = false;

bool ahtOk = false;

bool bmpOk = false;

bool bh1750Ok = false;

bool oledEnabled = true;

bool configPortalActive = false;

bool blynkConfigured = false;

bool fastBlynkMode = false;

bool wifiCredentialsAreSaved = false;

bool wifiEverConnected = false;

bool wifiOfflineTimerActive = false;

bool blynkEverConnected = false;

bool blynkOfflineTimerActive = false;

bool blynkDisconnectPending = false;

bool blynkConnectionLogPending = false;

bool otaRequested = false;

bool otaInProgress = false;

bool loopTaskWatchdogSubscribed = false;

uint32_t wifiOfflineSinceMs = 0;

uint32_t lastWiFiReconnectMs = 0;

uint32_t lastBlynkReconnectMs = 0;

uint32_t blynkOfflineSinceMs = 0;

uint32_t blynkDisconnectMillis = 0;

uint32_t blynkDisconnectUptimeMs = 0;

uint32_t blynkDisconnectEpoch = 0;

uint32_t nextBlynkSendMs = 0;

uint32_t bootMillis = 0;

uint32_t lastDisplayMs = 0;

uint32_t lastButtonChangeMs = 0;

uint32_t lastUserInteractionMs = 0;

uint32_t lastMainPageChangeMs = 0;

String blynkConnectionLogMessage;

String otaURL;

enum DisplayState : uint8_t {

  DISPLAY_DETAILS = 0,
  DISPLAY_DASHBOARD = 1,
  DISPLAY_MAIN = 2,
  DISPLAY_NETWORK = 3

};

DisplayState displayState = DISPLAY_DASHBOARD;

bool buttonRawState = HIGH;

bool buttonStableState = HIGH;

uint8_t mainSubPage = 0;

// ================================================================

// General helpers

// ================================================================

static void feedWatchdog() {

  if (loopTaskWatchdogSubscribed) esp_task_wdt_reset();

}

static String formatUptime(uint32_t uptimeMs) {

  uint32_t s = uptimeMs / 1000UL;

  const uint32_t days = s / 86400UL;

  s %= 86400UL;

  const uint32_t hours = s / 3600UL;

  s %= 3600UL;

  const uint32_t minutes = s / 60UL;

  const uint32_t seconds = s % 60UL;

  char buffer[40];

  if (days > 0) {

    snprintf(buffer, sizeof(buffer), "%lud %02luh %02lum %02lus",

      (unsigned long)days, (unsigned long)hours,

      (unsigned long)minutes, (unsigned long)seconds);

  } else if (hours > 0) {

    snprintf(buffer, sizeof(buffer), "%luh %02lum %02lus",

      (unsigned long)hours, (unsigned long)minutes,

      (unsigned long)seconds);

  } else if (minutes > 0) {

    snprintf(buffer, sizeof(buffer), "%lum %02lus",

      (unsigned long)minutes, (unsigned long)seconds);

  } else {

    snprintf(buffer, sizeof(buffer), "%lus", (unsigned long)seconds);

  }

  return String(buffer);

}

static uint32_t getCurrentLocalEpoch() {

  return ntpClient.isTimeSet() ? (uint32_t)ntpClient.getEpochTime() : 0UL;

}

static String formatLocalDateTime(uint32_t epoch) {

  if (epoch == 0) return "--/--/---- --:--:--";

  time_t rawTime = (time_t)epoch;

  struct tm timeInfo;

  gmtime_r(&rawTime, &timeInfo);

  char buffer[24];

  snprintf(buffer, sizeof(buffer), "%02d/%02d/%04d %02d:%02d:%02d",

    timeInfo.tm_mday, timeInfo.tm_mon + 1, timeInfo.tm_year + 1900,

    timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);

  return String(buffer);

}

static void startWiFiOfflineTimer() {

  if (wifiOfflineTimerActive) return;

  wifiOfflineSinceMs = millis();

  wifiOfflineTimerActive = true;

  Serial.println(F("WiFi offline timer started."));

}

static void clearWiFiOfflineTimer() {

  wifiOfflineTimerActive = false;

  wifiOfflineSinceMs = 0;

}

static void startBlynkOfflineTimer() {

  if (blynkOfflineTimerActive) return;

  blynkOfflineSinceMs = millis();

  blynkOfflineTimerActive = true;

  Serial.println(F("Blynk offline timer started."));

}

static void clearBlynkOfflineTimer() {

  blynkOfflineTimerActive = false;

  blynkOfflineSinceMs = 0;

}

static void recordBlynkDisconnect() {

  if (!blynkEverConnected || blynkDisconnectPending) return;

  blynkDisconnectPending = true;

  blynkDisconnectMillis = millis();

  blynkDisconnectUptimeMs = blynkDisconnectMillis - bootMillis;

  blynkDisconnectEpoch = getCurrentLocalEpoch();

  Serial.println(F("Blynk disconnect recorded."));

  Serial.print(F("Disconnect time: "));

  Serial.println(formatLocalDateTime(blynkDisconnectEpoch));

  Serial.print(F("Disconnect uptime: "));

  Serial.println(formatUptime(blynkDisconnectUptimeMs));

}

static void prepareBlynkConnectionLog() {

  const uint32_t now = millis();

  const uint32_t uptime = now - bootMillis;

  const uint32_t epoch = getCurrentLocalEpoch();

  if (blynkDisconnectPending) {

    blynkConnectionLogMessage =

      "DISCONNECTED: " + formatLocalDateTime(blynkDisconnectEpoch) +

      " | uptime " + formatUptime(blynkDisconnectUptimeMs) +

      "\nRECONNECTED: " + formatLocalDateTime(epoch) +

      " | uptime " + formatUptime(uptime) +

      " | offline " + formatUptime(now - blynkDisconnectMillis);

    blynkDisconnectPending = false;

  } else {

    blynkConnectionLogMessage =

      "CONNECTED: " + formatLocalDateTime(epoch) +

      " | uptime " + formatUptime(uptime);

  }

  blynkConnectionLogPending = true;

}

static void sendPendingBlynkConnectionLog() {

  if (!blynkConnectionLogPending || !Blynk.connected()) return;

  Blynk.logEvent(BLYNK_CONNECTION_EVENT_CODE, blynkConnectionLogMessage);

  Serial.println(F("Blynk connection log sent."));

  Serial.println(blynkConnectionLogMessage);

  blynkConnectionLogMessage = "";

  blynkConnectionLogPending = false;

}

static void networkRecoveryRestart(const char* reason) {

  Serial.println();

  Serial.println(F("=== NETWORK RECOVERY RESTART ==="));

  Serial.print(F("Reason: "));

  Serial.println(reason);

  if (Blynk.connected()) Blynk.disconnect();

  WiFi.disconnect(false, false);

  delay(100);

  ESP.restart();

}

static void enterManualDeepSleepIfRequested() {

  Preferences prefs;

  if (!prefs.begin(RESET_PREF_NAMESPACE, false)) return;

  const bool pending = prefs.getBool(RESET_PREF_KEY, false);

  if (pending) prefs.putBool(RESET_PREF_KEY, false);

  prefs.end();

  if (!pending) return;

  esp_sleep_enable_timer_wakeup(MANUAL_DEEP_SLEEP_US);

  esp_deep_sleep_start();

}

// ================================================================

// OLED rendering

// ================================================================

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

  display.println(F("Smart Environmental"));

  display.setCursor(0, 16);

  display.println(line1);

  if (line2) {

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

  display.println(F("WiFi SETUP MODE"));

  display.setCursor(0, 14);

  display.print(F("SSID: "));

  display.println(AP_SSID);

  display.setCursor(0, 28);

  display.print(F("IP: "));

  display.println(WiFi.softAPIP());

  display.setCursor(0, 42);

  display.println(F("Open browser / captive"));

  display.setCursor(0, 54);

  display.println(F("No AP password"));

  display.display();

}

static bool timeIsAvailable() {

  return WiFi.status() == WL_CONNECTED && ntpClient.isTimeSet();

}

static void drawClock() {

  if (!timeIsAvailable()) display.print(F("--:--:--"));

  else display.print(ntpClient.getFormattedTime());

}

static void drawThermometerIcon(int16_t x, int16_t y) {

  display.drawRoundRect(x + 3, y, 5, 9, 2, SSD1306_WHITE);

  display.drawLine(x + 5, y + 3, x + 5, y + 9, SSD1306_WHITE);

  display.fillCircle(x + 5, y + 10, 3, SSD1306_WHITE);

}

static void drawDropletIcon(int16_t x, int16_t y) {

  display.drawLine(x + 5, y, x + 1, y + 6, SSD1306_WHITE);

  display.drawLine(x + 5, y, x + 9, y + 6, SSD1306_WHITE);

  display.drawLine(x + 1, y + 6, x + 2, y + 8, SSD1306_WHITE);

  display.drawLine(x + 2, y + 8, x + 5, y + 9, SSD1306_WHITE);

  display.drawLine(x + 5, y + 9, x + 8, y + 8, SSD1306_WHITE);

  display.drawLine(x + 8, y + 8, x + 9, y + 6, SSD1306_WHITE);

}

static void drawSunIcon(int16_t x, int16_t y) {

  display.drawCircle(x + 5, y + 5, 3, SSD1306_WHITE);

  display.drawLine(x + 5, y, x + 5, y + 1, SSD1306_WHITE);

  display.drawLine(x + 5, y + 9, x + 5, y + 10, SSD1306_WHITE);

  display.drawLine(x, y + 5, x + 1, y + 5, SSD1306_WHITE);

  display.drawLine(x + 9, y + 5, x + 10, y + 5, SSD1306_WHITE);

  display.drawLine(x + 1, y + 1, x + 2, y + 2, SSD1306_WHITE);

  display.drawLine(x + 8, y + 8, x + 9, y + 9, SSD1306_WHITE);

  display.drawLine(x + 1, y + 9, x + 2, y + 8, SSD1306_WHITE);

  display.drawLine(x + 8, y + 2, x + 9, y + 1, SSD1306_WHITE);

}

static void drawPressureIcon(int16_t x, int16_t y) {

  display.drawCircle(x + 5, y + 5, 5, SSD1306_WHITE);

  display.drawCircle(x + 5, y + 5, 1, SSD1306_WHITE);

  display.drawLine(x + 5, y + 5, x + 8, y + 2, SSD1306_WHITE);

}

static void drawSmallCloudIcon(int16_t x, int16_t y, bool rain) {

  display.drawCircle(x + 3, y + 5, 2, SSD1306_WHITE);

  display.drawCircle(x + 6, y + 3, 3, SSD1306_WHITE);

  display.drawCircle(x + 9, y + 5, 2, SSD1306_WHITE);

  display.drawLine(x + 2, y + 6, x + 10, y + 6, SSD1306_WHITE);

  if (rain) {

    display.drawLine(x + 4, y + 7, x + 3, y + 9, SSD1306_WHITE);

    display.drawLine(x + 8, y + 7, x + 7, y + 9, SSD1306_WHITE);

  }

}

static void drawWeatherHeader() {

  const int16_t iconX = 70;

  const int16_t iconY = 0;

  int16_t textX = 83;

  if (sensors.rainActive) {

    drawSmallCloudIcon(iconX, iconY, true);

    display.setCursor(textX, 0);

    display.print(F("RAIN"));

  } else if (isnan(sensors.lux) || !bh1750Ok) {

    display.drawCircle(iconX + 5, iconY + 4, 4, SSD1306_WHITE);

    display.drawLine(iconX + 3, iconY + 2, iconX + 7, iconY + 6, SSD1306_WHITE);

    display.setCursor(textX, 0);

    display.print(F("NO DATA"));

  } else if (sensors.lux >= 10000.0F) {

    drawSunIcon(iconX, iconY - 1);

    display.setCursor(textX, 0);

    display.print(F("SUN"));

  } else if (sensors.lux >= 1000.0F) {

    drawSmallCloudIcon(iconX, iconY, false);

    display.setCursor(textX, 0);

    display.print(F("CLOUD"));

  } else {

    drawSmallCloudIcon(iconX, iconY, false);

    display.setCursor(textX, 0);

    display.print(F("DARK"));

  }

}

static void drawDashboardPage() {

  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);

  // Header: clock at left, indicative weather state at right.

  display.setCursor(0, 0);

  drawClock();

  drawWeatherHeader();

  // Four compact information tiles fit the 128x64 framebuffer.

  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.drawLine(63, 11, 63, 63, SSD1306_WHITE);

  display.drawLine(0, 37, 127, 37, SSD1306_WHITE);

  // Tile 1: temperature.

  drawThermometerIcon(2, 15);

  display.setCursor(15, 13);

  display.print(F("TEMP"));

  display.setCursor(15, 24);

  if (isnan(sensors.temperatureC)) display.print(F("---.-C"));

  else {

    display.print(sensors.temperatureC, 1);

    display.print(F("C"));

  }

  // Tile 2: relative humidity.

  drawDropletIcon(66, 15);

  display.setCursor(80, 13);

  display.print(F("HUM"));

  display.setCursor(80, 24);

  if (isnan(sensors.humidityPct)) display.print(F("---%"));

  else {

    display.print(sensors.humidityPct, 0);

    display.print(F("%"));

  }

  // Tile 3: illuminance. Keep the value short enough for 64-pixel cells.

  drawSunIcon(2, 42);

  display.setCursor(15, 39);

  display.print(F("LUX"));

  display.setCursor(15, 50);

  if (isnan(sensors.lux) || !bh1750Ok) {

    display.print(F("----"));

  } else if (sensors.lux >= 100000.0F) {

    display.print(F("100klx+"));

  } else if (sensors.lux >= 10000.0F) {

    display.print(sensors.lux / 1000.0F, 1);

    display.print(F("klx"));

  } else {

    display.print((uint32_t)(sensors.lux + 0.5F));

    display.print(F("lx"));

  }

  // Tile 4: atmospheric pressure.

  drawPressureIcon(66, 42);

  display.setCursor(80, 39);

  display.print(F("PRESS"));

  display.setCursor(80, 50);

  if (isnan(sensors.pressureHpa)) {

    display.print(F("----"));

  } else {

    display.print((int)lroundf(sensors.pressureHpa));

    display.print(F("hPa"));

  }

  display.display();

}

static void drawMainPage0() {

  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);

  display.setCursor(0, 0);

  drawClock();

  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  display.setCursor(0, 13);

  display.print(F("T: "));

  if (isnan(sensors.temperatureC)) display.print(F("--.-"));

  else display.print(sensors.temperatureC, 1);

  display.print(F(" C"));

  display.setCursor(67, 13);

  display.print(F("RH: "));

  if (isnan(sensors.humidityPct)) display.print(F("--"));

  else display.print(sensors.humidityPct, 0);

  display.print(F("%"));

  display.setCursor(0, 25);

  display.print(F("Lux: "));

  if (isnan(sensors.lux)) display.print(F("----"));

  else display.print(sensors.lux, 1);

  display.print(F(" lx"));

  display.setCursor(0, 37);

  display.print(F("P: "));

  if (isnan(sensors.pressureHpa)) display.print(F("----"));

  else display.print(sensors.pressureHpa, 0);

  display.print(F(" hPa"));

  display.setCursor(0, 49);

  display.print(F("Rain: "));

  display.print(sensors.rainActive ? F("YES") : F("NO"));

  display.display();

}

static void drawDetailsPage() {

  static uint32_t lastDetailsDrawMs = 0;

  const uint32_t now = millis();

  if ((uint32_t)(now - lastDetailsDrawMs) < 1000UL) return;

  lastDetailsDrawMs = now;

  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);

  oledHeader("WIFI DETAILS");

  String ssid = savedWiFiSSID;

  display.setCursor(0, 12);

  display.print(F("SSID: "));

  if (ssid.length() == 0) display.println(F("--"));

  else display.println(ssid);

  display.setCursor(0, 25);

  display.print(F("PASS: "));

  display.println(savedWiFiPassword.length() ? F("(saved)") : F("(open)"));

  display.setCursor(0, 38);

  display.print(F("IP: "));

  if (WiFi.status() == WL_CONNECTED) display.println(WiFi.localIP());

  else if (configPortalActive) display.println(WiFi.softAPIP());

  else display.println(F("--"));

  display.setCursor(0, 51);

  display.print(F("Blynk: "));

  display.println(Blynk.connected() ? F("CONNECTED") : F("OFFLINE"));

  display.display();

}

static void drawSystemStatusPage() {

  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);

  oledHeader("SYSTEM STATUS");

  uint32_t totalSeconds = (millis() - bootMillis) / 1000UL;

  uint32_t days = totalSeconds / 86400UL;

  totalSeconds %= 86400UL;

  uint32_t hours = totalSeconds / 3600UL;

  totalSeconds %= 3600UL;

  uint32_t minutes = totalSeconds / 60UL;

  uint32_t seconds = totalSeconds % 60UL;

  display.setCursor(0, 18);

  display.print(F("Since reset:"));

  display.setCursor(0, 31);

  if (days < 10) display.print('0');

  display.print(days);

  display.print(F("d "));

  if (hours < 10) display.print('0');

  display.print(hours);

  display.print(F("h "));

  if (minutes < 10) display.print('0');

  display.print(minutes);

  display.print(F("m "));

  if (seconds < 10) display.print('0');

  display.print(seconds);

  display.print('s');

  display.setCursor(0, 46);

  display.print(F("CPU Temp: "));

  if (isnan(sensors.chipTempC)) display.print(F("--.-"));

  else display.print(sensors.chipTempC, 1);

  display.print(F(" C"));

  display.display();

}

static void drawOTAProgress(int percent, const char* stateText) {

  if (!oledOk || !oledEnabled) return;

  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);

  display.setCursor(0, 0);

  display.println(F("Updating Firmware..."));

  display.setCursor(0, 15);

  display.println(stateText);

  display.drawRect(4, 30, 120, 12, SSD1306_WHITE);

  int barWidth = (percent * 116) / 100;

  if (barWidth > 116) barWidth = 116;

  if (barWidth > 0) display.fillRect(6, 32, barWidth, 8, SSD1306_WHITE);

  display.setCursor(49, 49);

  display.print(percent);

  display.print('%');

  display.display();

}

static void drawCurrentUI() {

  if (!oledOk || !oledEnabled) return;

  switch (displayState) {

    case DISPLAY_DASHBOARD: drawDashboardPage(); break;

    case DISPLAY_MAIN: drawMainPage0(); break;

    case DISPLAY_NETWORK: drawDetailsPage(); break;

    case DISPLAY_DETAILS: drawSystemStatusPage(); break;

  }

}

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

// Sensors and local button

// ================================================================

static float readInternalChipTemperature() {

  return temperatureRead();

}

static void readSensors() {

  sensors.rainAnalog = analogRead(PIN_RAIN_ANALOG);

  sensors.rainActive = digitalRead(PIN_RAIN_DIGITAL) == LOW;

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

    if (value >= 0.0F) sensors.lux = value;

  }

  sensors.chipTempC = readInternalChipTemperature();

}

static void advanceDisplayState() {

  displayState = static_cast<DisplayState>(

    (static_cast<uint8_t>(displayState) + 1U) % 4U);

  lastUserInteractionMs = millis();

  if (displayState == DISPLAY_DASHBOARD) {

    mainSubPage = 0;

    lastMainPageChangeMs = millis();

  }

}

static void handleButton() {

  const uint32_t now = millis();

  const bool rawPressed = digitalRead(PIN_BUTTON) == LOW;

  if (rawPressed != buttonRawState) {

    buttonRawState = rawPressed;

    lastButtonChangeMs = now;

  }

  if ((uint32_t)(now - lastButtonChangeMs) >= BUTTON_DEBOUNCE_MS) {

    if (buttonStableState != buttonRawState) {

      buttonStableState = buttonRawState;

      if (buttonStableState == LOW) advanceDisplayState();

    }

  }

  if (displayState != DISPLAY_DASHBOARD &&

      (uint32_t)(now - lastUserInteractionMs) >= UI_TIMEOUT_MS) {

    displayState = DISPLAY_DASHBOARD;

    mainSubPage = 0;

    lastMainPageChangeMs = now;

  }

}

static void serviceMainPageRotation() {

  if (displayState == DISPLAY_DASHBOARD) mainSubPage = 0;

}

// ================================================================

// Wi-Fi credentials and WiFiManager portal

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

  const size_t passWritten = wifiPreferences.putString(WIFI_PREF_PASS_KEY, password);

  wifiPreferences.end();

  if (ssidWritten == 0 || (password.length() > 0 && passWritten == 0)) {

    Serial.println(F("WiFi credentials could not be written to NVS."));

    return false;

  }

  savedWiFiSSID = ssid;

  savedWiFiPassword = password;

  wifiCredentialsAreSaved = true;

  Serial.print(F("WiFi credentials saved. SSID="));

  Serial.println(savedWiFiSSID);

  return true;

}

static bool loadWiFiCredentialsFromNVS() {

  savedWiFiSSID = "";

  savedWiFiPassword = "";

  if (wifiPreferences.begin(WIFI_PREF_NAMESPACE, true)) {

    savedWiFiSSID = wifiPreferences.getString(WIFI_PREF_SSID_KEY, "");

    savedWiFiPassword = wifiPreferences.getString(WIFI_PREF_PASS_KEY, "");

    wifiPreferences.end();

  }

  if (savedWiFiSSID.length() > 0) {

    wifiCredentialsAreSaved = true;

    Serial.print(F("Found application WiFi credentials. SSID="));

    Serial.println(savedWiFiSSID);

    return true;

  }

  const String wmSSID = wifiManager.getWiFiSSID(true);

  const String wmPassword = wifiManager.getWiFiPass(true);

  if (wmSSID.length() > 0) {

    Serial.println(F("Migrating WiFiManager credentials to application NVS."));

    return saveWiFiCredentialsToNVS(wmSSID, wmPassword);

  }

  wifiCredentialsAreSaved = false;

  Serial.println(F("No saved WiFi credentials found."));

  return false;

}

static void onWiFiManagerAPStart(WiFiManager* manager) {

  (void)manager;

  configPortalActive = true;

  showAPPortalScreen();

  Serial.println();

  Serial.println(F("=== WiFi provisioning AP active ==="));

  Serial.print(F("SSID: "));

  Serial.println(AP_SSID);

  Serial.print(F("IP: "));

  Serial.println(WiFi.softAPIP());

}

static void onWiFiManagerSave() {

  const String ssid = wifiManager.getWiFiSSID(true);

  const String password = wifiManager.getWiFiPass(true);

  if (saveWiFiCredentialsToNVS(ssid, password)) {

    Serial.println(F("New WiFi credentials saved."));

  } else {

    Serial.println(F("Saving new WiFi credentials failed."));

  }

}

static void configureWiFiManager() {

  wifiManager.setConfigPortalBlocking(false);

  wifiManager.setConfigPortalTimeout(0);

  wifiManager.setAPClientCheck(false);

  wifiManager.setWebPortalClientCheck(true);

  wifiManager.setShowPassword(false);

  wifiManager.setDebugOutput(false);

  wifiManager.setAPCallback(onWiFiManagerAPStart);

  wifiManager.setSaveConfigCallback(onWiFiManagerSave);

}

static void startProvisioningPortal() {

  if (configPortalActive) return;

  Serial.println(F("Starting SmartMonitor-Setup AP and WiFi configuration portal."));

  configureWiFiManager();

  // AP+STA lets the setup AP remain active while the station retries Wi-Fi.

  WiFi.mode(WIFI_AP_STA);

  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  wifiManager.startConfigPortal(AP_SSID, AP_PASSWORD);

  configPortalActive = wifiManager.getConfigPortalActive();

  if (configPortalActive) {

    showAPPortalScreen();

  } else {

    Serial.println(F("WiFi setup portal did not start; retry will be scheduled."));

  }

}

static void onWiFiConnected() {

  const bool hadSavedCredentials = wifiCredentialsAreSaved;

  const String connectedSSID = WiFi.SSID();

  wifiCredentialsAreSaved = connectedSSID.length() > 0;

  Serial.println();

  Serial.println(F("WiFi connected."));

  Serial.print(F("SSID: "));

  Serial.println(connectedSSID);

  Serial.print(F("IP: "));

  Serial.println(WiFi.localIP());

  Serial.print(F("RSSI: "));

  Serial.print(WiFi.RSSI());

  Serial.println(F(" dBm"));

  if (connectedSSID.length() > 0 &&

      (savedWiFiSSID != connectedSSID || !hadSavedCredentials)) {

    const String currentPassword = wifiManager.getWiFiPass(true);

    if (currentPassword.length() > 0) {

      saveWiFiCredentialsToNVS(connectedSSID, currentPassword);

    }

  }

  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  wifiEverConnected = true;

  clearWiFiOfflineTimer();

  if (!ntpClient.isTimeSet()) ntpClient.begin();

  ntpClient.update();

  if (!blynkConfigured) {

    Blynk.config(BLYNK_AUTH_TOKEN, "blynk.cloud", 443);

    blynkConfigured = true;

  }

  lastBlynkReconnectMs = 0;

  nextBlynkSendMs = millis() + 1000UL;

}

static void startSavedWiFiAttempt() {

  if (savedWiFiSSID.length() == 0) return;

  if (configPortalActive) WiFi.mode(WIFI_AP_STA);

  else WiFi.mode(WIFI_STA);

  WiFi.setAutoReconnect(true);

  WiFi.persistent(false);

  Serial.print(F("Trying saved WiFi SSID: "));

  Serial.println(savedWiFiSSID);

  WiFi.begin(savedWiFiSSID.c_str(), savedWiFiPassword.c_str());

  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  lastWiFiReconnectMs = millis();

}

static void startInitialWiFiAttempt() {

  if (!loadWiFiCredentialsFromNVS()) {

    startProvisioningPortal();

    return;

  }

  startWiFiOfflineTimer();

  WiFi.mode(WIFI_STA);

  WiFi.setAutoReconnect(true);

  WiFi.persistent(false);

  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  Serial.println(F("Starting initial saved WiFi connection..."));

  WiFi.begin(savedWiFiSSID.c_str(), savedWiFiPassword.c_str());

  lastWiFiReconnectMs = millis();

}

static void serviceWiFiProvisioning() {

  const uint32_t now = millis();

  // Always process captive portal requests while the setup AP is active.

  if (configPortalActive) {

    wifiManager.process();

    // A connection may have succeeded using saved or newly submitted credentials.

    if (WiFi.status() == WL_CONNECTED) {

      wifiManager.stopConfigPortal();

      configPortalActive = false;

      onWiFiConnected();

      return;

    }

    // Keep trying saved credentials while leaving AP active for the phone.

    if (savedWiFiSSID.length() > 0 &&

        (uint32_t)(now - lastWiFiReconnectMs) >= WIFI_RECONNECT_MS) {

      Serial.println(F("AP is active; retrying saved WiFi credentials."));

      startSavedWiFiAttempt();

    }

    // If the portal unexpectedly stops, fall back to STA and the timer

    // will start the portal again if Wi-Fi is still unavailable.

    if (!wifiManager.getConfigPortalActive()) {

      configPortalActive = false;

      WiFi.mode(WIFI_STA);

      WiFi.setAutoReconnect(true);

      Serial.println(F("Setup portal stopped."));

    }

    return;

  }

  if (WiFi.status() == WL_CONNECTED) {

    if (!wifiEverConnected) onWiFiConnected();

    else clearWiFiOfflineTimer();

    return;

  }

  if (!wifiOfflineTimerActive) startWiFiOfflineTimer();

  // Requirement: after 30 seconds without a Wi-Fi connection, bring up AP.

  if ((uint32_t)(now - wifiOfflineSinceMs) >= WIFI_NO_CONNECTION_AP_MS) {

    startProvisioningPortal();

    if (configPortalActive) {

      // The AP remains up; the saved credentials will also be retried.

      lastWiFiReconnectMs = now;

      return;

    }

    // If starting the AP failed, retry its startup after another 30 seconds.

    wifiOfflineSinceMs = now;

    wifiOfflineTimerActive = true;

  }

  // Retry saved credentials every 15 seconds, including on initial boot.

  if (savedWiFiSSID.length() > 0 &&

      (uint32_t)(now - lastWiFiReconnectMs) >= WIFI_RECONNECT_MS) {

    startSavedWiFiAttempt();

  }

}

// ================================================================

// NTP and Blynk data transfer

// ================================================================

static void serviceNTP() {

  static uint32_t lastNtpCallMs = 0;

  const uint32_t now = millis();

  if (WiFi.status() != WL_CONNECTED) return;

  if ((uint32_t)(now - lastNtpCallMs) < NTP_INTERVAL_MS) return;

  lastNtpCallMs = now;

  ntpClient.update();

}

BLYNK_CONNECTED() {

  Serial.println(F("Blynk connected."));

  blynkEverConnected = true;

  clearBlynkOfflineTimer();

  prepareBlynkConnectionLog();

  Blynk.syncVirtual(V2);

  Blynk.syncVirtual(V9);

  nextBlynkSendMs = millis() + 500UL;

}

BLYNK_DISCONNECTED() {

  Serial.println(F("Blynk disconnected."));

  recordBlynkDisconnect();

  if (WiFi.status() == WL_CONNECTED) startBlynkOfflineTimer();

}

BLYNK_WRITE(V2) {

  fastBlynkMode = param.asInt() != 0;

  Serial.print(F("Blynk interval mode V2 = "));

  Serial.println(fastBlynkMode ? F("FAST (5s)") : F("SLOW (10min)"));

  nextBlynkSendMs = millis() + 100UL;

}

BLYNK_WRITE(V9) {

  const bool enableOLED = param.asInt() != 0;

  Serial.print(F("Blynk V9 OLED = "));

  Serial.println(enableOLED ? F("ON (1)") : F("OFF (0)"));

  setOledEnabled(enableOLED);

}

BLYNK_WRITE(V10) {

  if (param.asInt() == 1) {

    Serial.println(F("Blynk V10 pressed -> virtual D10 press"));

    advanceDisplayState();

  }

}

BLYNK_WRITE(V100) {

  if (param.asInt() != 1) return;

  Serial.println(F("Blynk V100 active -> reset + 300s deep sleep."));

  if (!resetPreferences.begin(RESET_PREF_NAMESPACE, false)) {

    Serial.println(F("Could not open reset control NVS."));

    Blynk.virtualWrite(V100, 0);

    return;

  }

  const size_t written = resetPreferences.putBool(RESET_PREF_KEY, true);

  resetPreferences.end();

  if (written == 0) {

    Serial.println(F("Could not save deep-sleep request. Reset aborted."));

    Blynk.virtualWrite(V100, 0);

    return;

  }

  Blynk.virtualWrite(V100, 0);

  delay(100);

  ESP.restart();

}

static void sendSensorDataToBlynk() {

  if (!Blynk.connected()) return;

  if (!isnan(sensors.temperatureC)) Blynk.virtualWrite(V0, sensors.temperatureC);

  if (!isnan(sensors.humidityPct)) Blynk.virtualWrite(V1, sensors.humidityPct);

  Blynk.virtualWrite(V2, fastBlynkMode ? 1 : 0);

  if (!isnan(sensors.altitudeM)) Blynk.virtualWrite(V3, sensors.altitudeM);

  if (!isnan(sensors.pressureHpa)) {

    Blynk.virtualWrite(V4, (int)lroundf(sensors.pressureHpa));

  }

  Blynk.virtualWrite(V5, sensors.rainAnalog);

  Blynk.virtualWrite(V6, sensors.rainActive ? 1 : 0);

  if (!isnan(sensors.lux)) Blynk.virtualWrite(V7, sensors.lux);

  if (!isnan(sensors.chipTempC)) Blynk.virtualWrite(V8, sensors.chipTempC);

  Serial.println(F("Blynk sensor update sent."));

}

static void serviceBlynk() {

  const uint32_t now = millis();

  if (WiFi.status() != WL_CONNECTED || configPortalActive) {

    if (Blynk.connected()) {

      Blynk.disconnect();

      Serial.println(F("Blynk disconnected because WiFi is unavailable."));

    }

    return;

  }

  if (!blynkConfigured) return;

  if (!Blynk.connected()) {

    startBlynkOfflineTimer();

    if ((uint32_t)(now - blynkOfflineSinceMs) >= BLYNK_OFFLINE_RESTART_MS &&

        (uint32_t)(now - bootMillis) >= NETWORK_STARTUP_GRACE_MS) {

      networkRecoveryRestart("Blynk offline for 3 minutes");

      return;

    }

    if ((uint32_t)(now - lastBlynkReconnectMs) >= BLYNK_RECONNECT_MS) {

      lastBlynkReconnectMs = now;

      Serial.println(F("Trying Blynk reconnect..."));

      const bool connected = Blynk.connect(BLYNK_CONNECT_TIMEOUT_MS);

      Serial.println(connected ? F("Blynk reconnect OK.") : F("Blynk reconnect failed."));

    }

    return;

  }

  clearBlynkOfflineTimer();

  Blynk.run();

  if (!Blynk.connected()) {

    startBlynkOfflineTimer();

    lastBlynkReconnectMs = now;

    Serial.println(F("Blynk transport lost; reconnect scheduled."));

    return;

  }

  sendPendingBlynkConnectionLog();

  if ((int32_t)(now - nextBlynkSendMs) >= 0) {

    sendSensorDataToBlynk();

    const uint32_t interval = fastBlynkMode ?

      BLYNK_FAST_INTERVAL_MS : BLYNK_SLOW_INTERVAL_MS;

    nextBlynkSendMs = now + interval;

  }

}

static void maintainBlynkConnection() {
  static uint32_t lastMaintenanceMs = 0;
  const uint32_t now = millis();

  if ((uint32_t)(now - lastMaintenanceMs) <
      BLYNK_MAINTENANCE_INTERVAL_MS) {
    return;
  }

  lastMaintenanceMs = now;

  if (WiFi.status() != WL_CONNECTED || configPortalActive) {
    Serial.println(F("Blynk maintenance skipped: WiFi unavailable."));
    return;
  }

  if (!blynkConfigured) return;

  if (Blynk.connected()) {
    Serial.println(F("Blynk maintenance: connection is healthy."));
    return;
  }

  Serial.println(F("Blynk maintenance: reconnect requested."));
  lastBlynkReconnectMs = 0;
  serviceBlynk();
}

// ================================================================

// Blynk.Air OTA

// ================================================================

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

  if (!loopTaskWatchdogSubscribed) return true;

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

  if (configPortalActive) {

    wifiManager.stopConfigPortal();

    configPortalActive = false;

  }

  Blynk.disconnect();

  bool restoreWatchdog = false;

  if (!suspendLoopTaskWatchdogForOTA(restoreWatchdog)) {

    Serial.println(F("Continuing OTA with active watchdog."));

  }

  HTTPClient http;

  http.setConnectTimeout(OTA_HTTP_TIMEOUT_MS);

  http.setTimeout(OTA_HTTP_TIMEOUT_MS);

  http.setReuse(false);

  bool success = false;

  do {

    Serial.println(F("Starting firmware download..."));

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

      feedWatchdog();

      const size_t availableBytes = stream.available();

      if (availableBytes > 0) {

        const size_t toRead = availableBytes > sizeof(buffer) ?

          sizeof(buffer) : availableBytes;

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

        const int percent = (int)((writtenTotal * 100ULL) / (size_t)contentLength);

        const uint32_t progressNow = millis();

        if (percent != lastPercent &&

            (uint32_t)(progressNow - lastProgressDrawMs) >= 100UL) {

          lastPercent = percent;

          lastProgressDrawMs = progressNow;

          drawOTAProgress(percent, "Flashing...");

        }

      } else {

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

  if (oledOk) showBootMessage("OTA FAILED", "Continuing normal operation");

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

// Task watchdog

// ================================================================

static void setupWatchdog() {

  const esp_err_t initErr = esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);

  if (initErr == ESP_OK) {

    Serial.println(F("Task WDT initialized."));

  } else if (initErr == ESP_ERR_INVALID_STATE) {

    Serial.println(F("Task WDT was already initialized by the core."));

  } else {

    Serial.print(F("Task WDT init error="));

    Serial.println((int)initErr);

  }

  const esp_err_t addErr = esp_task_wdt_add(NULL);

  if (addErr == ESP_OK || addErr == ESP_ERR_INVALID_STATE) {

    loopTaskWatchdogSubscribed = true;

    Serial.println(F("loopTask subscribed to Task WDT."));

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

  enterManualDeepSleepIfRequested();

  Serial.begin(115200);

  delay(200);

  Serial.println();

  Serial.println(F("=============================================="));

  Serial.println(F("Smart Environmental Monitor"));

  Serial.println(F("ESP32-C3 SuperMini / Core 2.0.16"));

  Serial.println(F("Firmware version: " BLYNK_FIRMWARE_VERSION));

  Serial.println(F("WiFi fallback AP timeout: 30 seconds"));

  Serial.println(F("=============================================="));

  Serial.print(F("ESP reset reason: "));

  Serial.println((int)esp_reset_reason());

  bootMillis = millis();

  pinMode(PIN_BUTTON, INPUT_PULLUP);

  pinMode(PIN_RAIN_DIGITAL, INPUT_PULLUP);

  analogReadResolution(12);

  analogSetPinAttenuation(PIN_RAIN_ANALOG, ADC_11db);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  Wire.setClock(400000UL);

  Wire.setTimeOut(50);

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

  showBootMessage("Initializing sensors...");

  ahtOk = aht20.begin(&Wire);

  Serial.print(F("AHT20: "));

  Serial.println(ahtOk ? F("OK") : F("FAIL"));

  bh1750Ok = bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, BH1750_ADDR, &Wire);

  Serial.print(F("BH1750: "));

  Serial.println(bh1750Ok ? F("OK") : F("FAIL"));

  bmpOk = bmp280.begin(BMP280_ADDR_1);

  if (!bmpOk) bmpOk = bmp280.begin(BMP280_ADDR_2);

  Serial.print(F("BMP280: "));

  Serial.println(bmpOk ? F("OK") : F("FAIL"));

  if (bmpOk) {

    bmp280.setSampling(

      Adafruit_BMP280::MODE_NORMAL,

      Adafruit_BMP280::SAMPLING_X2,

      Adafruit_BMP280::SAMPLING_X16,

      Adafruit_BMP280::FILTER_X16,

      Adafruit_BMP280::STANDBY_MS_500);

  }

  readSensors();

  drawCurrentUI();

  setupWatchdog();

  feedWatchdog();

  showBootMessage("Connecting WiFi...", "AP fallback after 30s");

  WiFi.mode(WIFI_STA);

  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  configureWiFiManager();

  startInitialWiFiAttempt();

  lastUserInteractionMs = millis();

  lastMainPageChangeMs = millis();

  lastDisplayMs = 0;

}

// ================================================================

// Main loop

// ================================================================

void loop() {

  const uint32_t now = millis();

  feedWatchdog();

  static uint32_t lastSensorMs = 0;

  if ((uint32_t)(now - lastSensorMs) >= SENSOR_INTERVAL_MS) {

    lastSensorMs = now;

    readSensors();

  }

  handleButton();

  serviceMainPageRotation();

  serviceWiFiProvisioning();

  if (!configPortalActive) {

    serviceBlynk();
    maintainBlynkConnection();

    feedWatchdog();

    serviceNTP();

    feedWatchdog();

    serviceOTA();

  }

  if ((uint32_t)(now - lastDisplayMs) >= DISPLAY_INTERVAL_MS) {

    lastDisplayMs = now;

    if (oledEnabled) {

      if (configPortalActive) showAPPortalScreen();

      else if (!otaInProgress) drawCurrentUI();

    }

  }

  if (!otaInProgress &&

      (uint32_t)(now - bootMillis) >= AUTO_REBOOT_MS) {

    Serial.println(F("12-hour uptime reached. Controlled restart..."));

    delay(100);

    ESP.restart();

  }

  yield();

}
