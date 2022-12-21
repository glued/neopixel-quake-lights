#define ARDUINOJSON_USE_LONG_LONG 1
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <Adafruit_NeoPixel.h>

#define NUMPIXELS 60
#define LED_PIN A3
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

#define SERVER "earthquake.usgs.gov"
#define PATH "/earthquakes/feed/v1.0/summary/1.0_hour.geojson"
#define EARTH_RADIUS 6371
#define MAX_HUE 65536
#define MAX_MAG 8
#define MIN_MAG 1
#define MAX_COL 255
#define TO_RAD M_PI / 180

// CHANGE THIS TO YOUR LOCATION
const double home_lat = 47.645326;    // Gasworks park
const double home_lng = -122.334930;  // Seattle, WA

// CHANGE THIS TO YOUR WIFI CREDENTIALS
const char ssid[] = "YOUR_WIFI_SSID";
const char pass[] = "YOUR_WIFI_PASS";
int status = WL_IDLE_STATUS;

bool connecting = true;
char last_feature_id[32];
unsigned long startTime = 0;
double distance = 5000;
uint8_t mag = 0;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, pass);

#if defined(NEOPIXEL_POWER)
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
#endif
  pixels.begin();
  pixels.setBrightness(20);

  while (!Serial) {
    delay(10);
  }

  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("Connected to WiFi");
  printWifiStatus();
}

void loop() {
  while (connecting == true) {
    WiFiClientSecure client;
    client.setInsecure();
    Serial.println("\nStarting connection to server...");
    if (client.connect(SERVER, 443)) {
      Serial.println("connected to server");
      client.println("GET " PATH " HTTP/1.0");
      client.println("Host: " SERVER);
      client.println("Connection: close");
      client.println();
    }

    // Check HTTP status
    char status[32] = { 0 };
    client.readBytesUntil('\r', status, sizeof(status));
    if (strcmp(status, "HTTP/1.1 200 OK") != 0) {
      Serial.print(F("Unexpected response: "));
      Serial.println(status);
      return;
    }

    // Skip HTTP headers
    char endOfHeaders[] = "\r\n\r\n";
    if (!client.find(endOfHeaders)) {
      Serial.println(F("Invalid response"));
      return;
    }

    // Filter is used to limit memory usage
    // see https://arduinojson.org/v6/assistant/
    StaticJsonDocument<128> filter;

    JsonObject filter_features_0 = filter["features"].createNestedObject();
    filter_features_0["id"] = true;

    JsonObject filter_features_0_properties = filter_features_0.createNestedObject("properties");
    filter_features_0_properties["place"] = true;
    filter_features_0_properties["mag"] = true;
    filter_features_0["geometry"]["coordinates"] = true;
    DynamicJsonDocument doc(3072);
    DeserializationError error = deserializeJson(doc, client, DeserializationOption::Filter(filter));

    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }

    // Extract values
    JsonArray features = doc["features"];
    JsonObject features_0 = features[0];
    JsonObject features_0_properties = features_0["properties"];
    JsonObject features_0_geometry = features_0["geometry"];
    JsonArray features_coordinates = features_0_geometry["coordinates"];
    double lat = features_coordinates[1];
    double lng = features_coordinates[0];
    long feature_mag = features_0_properties["mag"];
    mag = clamp(round(feature_mag), MIN_MAG, MAX_MAG);
    const char *feature_id = features_0["id"];

    if (strcmp(last_feature_id, feature_id) == 0) {
      Serial.print("OLD ID: ");
      Serial.println(feature_id);
    } else if (feature_id) {
      Serial.print("NEW ID: ");
      Serial.println(feature_id);
      strcpy(last_feature_id, feature_id);
    }

    distance = dist(home_lat, home_lng, lat, lng);
    startTime = millis();
    const char *place = features_0_properties["place"];

    Serial.print("Place: ");
    Serial.println(place);
    Serial.print("Mag: ");
    Serial.println(feature_mag);
    Serial.println(mag);
    Serial.print("Dist: ");
    Serial.println(distance);
    Serial.print("geo: ");
    Serial.println(lat);
    Serial.println(lng);
    // Disconnect
    client.stop();
    connecting = false;
  }

  unsigned long currentMillis = millis();
  double timeDelta = currentMillis - startTime;

  if (connecting == false) {
    uint32_t hue = round(MAX_HUE - ((mag + 2) * MAX_HUE) / MAX_MAG);
    double freq = freqForDist(distance);
    colorWipe(hue, freq, timeDelta);
    pixels.show();
  }

  if (timeDelta >= 60000 * 4) {
    connecting = true;
  }
}

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void colorWipe(uint32_t hue, double freq, double delta) {
  for (uint8_t i = 0; i < pixels.numPixels(); i++) {
    double sv = sin((delta + i * 100) * freq);
    uint8_t v = round(abs(sv * MAX_COL));
    uint8_t a = clamp(v, 100, MAX_COL);
    uint32_t c = pixels.gamma32(pixels.ColorHSV(hue, MAX_COL, a));
    pixels.setPixelColor(i, c);
  }
}

// Returns a frequency value used to create a sine pattern
double freqForDist(double dist) {
  // EARTH_RADIUS / dist * 0.0002
  // More bespoke differences at smaller values
  if (dist <= 25) { return 0.05; }
  if (dist <= 50) { return 0.01; }
  if (dist <= 100) { return 0.005; }
  if (dist <= 500) { return 0.001; }
  if (dist <= 1000) { return 0.0004; }
  return 0.0001;
}

double dist(double latitude1, double longitude1, double latitude2, double longitude2) {
  double latitude1Rad = latitude1 * TO_RAD;
  double latitude2Rad = latitude2 * TO_RAD;
  double longitude1Rad = longitude1 * TO_RAD;
  double longitude2Rad = longitude2 * TO_RAD;
  double deltaLat = latitude2Rad - latitude1Rad;
  double deltaLon = longitude2Rad - longitude1Rad;
  double dlat2 = pow(sin(deltaLat / 2), 2);
  double dlon2 = pow(sin(deltaLon / 2), 2);
  double a = dlat2 + cos(latitude1Rad) * cos(latitude2Rad) * dlon2;
  return EARTH_RADIUS * atan2(sqrt(a), sqrt(1 - a));
}

int clamp(int value, int minValue, int maxValue) {
  return max(minValue, min(value, maxValue));
}
