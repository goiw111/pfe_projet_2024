#include <WiFi.h>
#include <WiFiUdp.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "Wire.h"

const char* ssid = "LB_ADSL_TUDS";
const char* password = "chahmot4ever";
const unsigned int port = 1234; // Port number
const unsigned long receiveTimeout = 800; // Timeout in milliseconds
IPAddress remoteip;

WiFiUDP udp;
MPU6050   mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

bool boot = 0;

typedef enum _state { 
  INIT    = 0x00,
  STARTED = 0x01,
  BOOT    = 0x02,
} State;

State state = INIT;

typedef enum _RXMsg {
  IS_CONNECTED  = 0,
  BOOT_ON       = 1,
  BOOT_OFF      = 2,
  COMMAND       = 3,
} RXMsg;

typedef enum _TXMsg {
  CONNECTED = 0x01,
  BOOTED    = 0x02,
} TXMsg;

union {
  struct {
    float     y,p,r;
  };
  uint8_t   bytes[12];
} data;

typedef struct _cdata {
  int8_t x,y,z;
} CData;

CData cmd = CData {   0,   0,   0 };

const size_t responseBufferSize = 20; // Size of the response buffer
uint8_t responseBuffer[responseBufferSize];

bool connected = false; // Flag to track Wi-Fi connection status

void connectToWiFi(const char *ssid, const char *pwd) {
  Serial.println("Connecting to WiFi network: " + String(ssid));
  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);  // Will call WiFiEvent() from another thread.
  //Initiate connection
  WiFi.begin(ssid, pwd);
  Serial.println("Waiting for WIFI connection...");
}

// WARNING: WiFiEvent is called from a separate FreeRTOS task (thread)!
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      //When connected set
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());
      //initializes the UDP state
      //This initializes the transfer buffer
      udp.begin(port);
      connected = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      connected = false;
      break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);
  connectToWiFi(ssid, password);

  Wire.begin();
  Wire.setClock(400000);
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(133);
  mpu.setYGyroOffset(67);
  mpu.setZGyroOffset(-83);
  mpu.setXAccelOffset(-2468);
  mpu.setYAccelOffset(-78);
  mpu.setZAccelOffset(2880);

    // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  while(!connected) {
    delay(200);
  }
  Serial.println("wait to connect");
  int size = 0;
  while(!size) {
    delay(500);
    size = udp.parsePacket();
    if(size) {
      uint8_t init;
      remoteip = udp.remoteIP();
      udp.beginPacket();
      udp.write(state);
      udp.endPacket();
      Serial.print("remote IP:");
      Serial.println(remoteip.toString());
    }
  }
  Serial.println("connected");
  state = STARTED;
}

void loop() {
  if (!dmpReady) {
    Serial.println("dmp non ready");
    delay(9000);
    return;
  }
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    float ypr[3];
    Quaternion  q;
    VectorFloat gravity;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    data.y = ypr[0];
    data.p = ypr[1];
    data.r = ypr[2];
  }
  if (connected) {
    // Receive data with timeout
    unsigned long startTime = millis();
    int packetSize;
    char RX_DATA[4];
    do {
      udp.parsePacket();
      int len = udp.read(RX_DATA, 4);
      udp.beginPacket();

      switch(RX_DATA[0]) {
        case BOOT_ON:
          state = BOOT;
        break;
        case BOOT_OFF:
          state = STARTED;
        break;
      }
      udp.write(state);
      if (state == BOOT) {
        cmd.x = (int8_t)RX_DATA[1];
        cmd.y = (int8_t)RX_DATA[2];
        cmd.z = (int8_t)RX_DATA[3];
        //Serial.printf("x: %.2f ,y: %.2f,z: %.2f \n",data.r,data.p,data.y);
        udp.write((uint8_t *)&data, 12);
      }
      udp.endPacket();
      delay(20);
      if( len > 0) {
        break;
      }
      // Resend the response buffer to the remote IP and port
    } while (millis() - startTime < receiveTimeout);

    if (millis() - startTime >= receiveTimeout) {
      Serial.println("Receive timeout");
      cmd.x = 0; cmd.y = 0; cmd.z = 0;
    }
  }
  if (cmd.x != 0 | cmd.y != 0 | cmd.z != 0) {
    Serial.printf("x: %d,y: %d,z: %d \n",cmd.x,cmd.y,cmd.z);
  }
}