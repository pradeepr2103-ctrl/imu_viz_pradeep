#include <WiFi.h>
#include <WiFiUdp.h>
#include <MPU6050_light.h>

// ── User configuration (change for each board) ────────────────
constexpr int SENSOR_ID = 0;          // 0 to 9 (see mapping table)
const char* WIFI_SSID   = "TP-Link_DF6C_Cave";
const char* WIFI_PASS   = "Caveiot@123";
const char* PC_IP       = "192.168.1.182";   // IP of the Ubuntu PC
constexpr int PC_PORT   = 5005;
// ────────────────────────────────────────────────────────────────

WiFiUDP udp;
MPU6050 mpu(Wire);

void setup() {
  Serial.begin(115200);
  delay(1000);

  // WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected. IP: " + WiFi.localIP().toString());

  // MPU6050
  Wire.begin();
  byte status = mpu.begin();
  if (status != 0) {
    Serial.printf("MPU6050 init failed, error: %d\n", status);
    while (1) delay(1000);
  }
  Serial.println("MPU6050 ready");

  // Calibrate offsets – keep the sensor STILL during the next seconds
  mpu.calcOffsets();
}

void loop() {
  mpu.update();

  if (mpu.getQuatUpdated()) {
    float qw = mpu.getQuatW();
    float qx = mpu.getQuatX();
    float qy = mpu.getQuatY();
    float qz = mpu.getQuatZ();

    // Pack: 1 byte sensor ID + 16 bytes (qw,qx,qy,qz)
    uint8_t buf[17];
    buf[0] = (uint8_t)SENSOR_ID;
    memcpy(&buf[1], &qw, 4);
    memcpy(&buf[5], &qx, 4);
    memcpy(&buf[9], &qy, 4);
    memcpy(&buf[13], &qz, 4);

    udp.beginPacket(PC_IP, PC_PORT);
    udp.write(buf, sizeof(buf));
    udp.endPacket();

    Serial.printf("ID:%d qw:%.3f qx:%.3f qy:%.3f qz:%.3f\n",
                  SENSOR_ID, qw, qx, qy, qz);
  }
}