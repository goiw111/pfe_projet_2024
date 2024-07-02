#include <WiFi.h>
#include <WiFiUdp.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include <Adafruit_PWMServoDriver.h>
#include "Wire.h"

const char* ssid = "LB_ADSL_TUDS";
const char* password = "chahmot4ever";
const unsigned int port = 1234; // Port number
const unsigned long receiveTimeout = 400; // Timeout in milliseconds
IPAddress remoteip;

#define SERVO_FREQ 44 // Analog servos run at ~50 Hz updates

WiFiUDP                 udp;
MPU6050                 mpu;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

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

const float g = 9.80665;        //[m/s^2]
CData cmd = CData {   0,   0,   -127 };

const size_t responseBufferSize = 20; // Size of the response buffer
uint8_t responseBuffer[responseBufferSize];

bool connected = false; // Flag to track Wi-Fi connection status

const float m = 0.673;          //[kg]
const float l = 0.22;              //[m]
const float r = 0.065;
const float cos_a = cos(PI/4 - 0.01);
const float sin_a = sin(PI/4 - 0.01);

struct rx_data {
  uint8_t   key;
  uint8_t   cmd_x;
  uint8_t   cmd_y;
  uint8_t   cmd_z;
  float     m_mass;
  float     c_amorti;
  float     p_propre;
  uint16_t  thrust;
};

union{
  struct  rx_data data;
  uint8_t         bytes[18];
} RX_DATA;

float * control(float z_dd,float cos_th,float * err) {
  const float     ms= RX_DATA.data.m_mass;
  const uint16_t  th= RX_DATA.data.thrust;
  const float cm= (m*g)/(th*4);
  const float i = (2/5)*(m - 4*ms)*r*r + 2*l*l*ms;   //[kg*m^2]
  const float iz= (2/5)*(m - 4*ms)*r*r + 4*l*l*ms;   //[kg*m^2]
  const float a = m/(4*cm);                          //[]
  const float b = i/(2*l*cm);                         //[]
  const float d = cm*0.8;
  const float c = iz/(4*l*d);                         //[]

  static float  r[4];
  r[0] = a*(g + z_dd)/cos_th + b*(err[0]*sin_a - err[1]*cos_a) + c*err[2];
  r[1] = a*(g + z_dd)/cos_th + b*(err[0]*cos_a + err[1]*sin_a) - c*err[2];
  r[2] = a*(g + z_dd)/cos_th - b*(err[0]*sin_a - err[1]*cos_a) + c*err[2];
  r[3] = a*(g + z_dd)/cos_th - b*(err[0]*cos_a + err[1]*sin_a) - c*err[2];
  return r;
}

unsigned long pt;
float         i_x,i_y   = 0;
float         perr[3]   = { 0 };

float * pid(float * err) {
  static float  r[3];
  const float   amo = RX_DATA.data.c_amorti;
  const float   ppr = RX_DATA.data.p_propre;

  unsigned long time = millis();
  float et = (time - pt) / 1000.0;
  i_x += err[0] * et;
  i_y += err[1] * et;
  Serial.println( ppr * ppr * i_x);

  float z_d = 0.24875;
  r[0]  = 10*(2*ppr*amo*err[0] + (err[0] - perr[0]) / et + ppr*ppr*i_x);
  Serial.println(r[0]);
  r[1]  = 10*(2*ppr*amo*err[1] + (err[1] - perr[1]) / et + ppr*ppr*i_y);
  r[2]  = 0 /*err[2] + z_d*(err[2] - perr[2]) / et*/;
  perr[0] = err[0];
  perr[1] = err[1];
  perr[2] = err[2];

  pt = time;
  return r;
}

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

  Serial.println(F("Initializing I2C devices..."));
  Wire.begin();
  Wire.setClock(400000);

  Serial.println(F("Initializing PCA9668 devices..."));
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  pwm.setPWM(12,  0, 0 );
  pwm.setPWM(10,  0, 0 );

  Serial.println(F("Initializing MPU6050 devices..."));
  mpu.initialize();

  Serial.println(F("Testing MPU6050 connections..."));
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
    Serial.println("init ok");
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
  /*for(int i = 150; i < 160; i++) {
    delay(1000);
    pwm.setPWM(14,  0,29 + i ); //+19
    pwm.setPWM(12,  0, i );
    pwm.setPWM(10,  0, i );
    pwm.setPWM(8,   0, i ); //+29
    Serial.print("power: ");
    Serial.println(i);
  }
  delay(2000);*/

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
  float xyz[3] = { -1 * data.r , data.p, -1 * data.y };
  Serial.printf("x: %.2f, y: %.2f ,z: %.2f \n",data.r, data.p,data.y);
  float * cxyz = { 0 };
  cxyz = pid(xyz);
  float * w = control( cmd.z * (g/127) ,1,cxyz);

  //Serial.printf("w1: %.2f, w2: %.2f, w3: %.2f, w4: %.2f \n",w[0],w[1],w[2],w[3]);

  pwm.setPWM(12,  0, 0 +  w[3] );
  pwm.setPWM(10,  0, 0 +  w[1] );
  pwm.setPWM(8,   0, 0 +  w[0] );
  pwm.setPWM(14,  0, 0 +  w[2] );

  if (connected) {
    // Receive data with timeout
    unsigned long startTime = millis();
    int packetSize;
    do {
      udp.parsePacket();
      int len = udp.read(RX_DATA.bytes, 18);
      /*Serial.printf("[%d",RX_DATA.bytes[0]);
      for(int loop = 1; loop < 18; loop++)
      Serial.printf(",%d ", RX_DATA.bytes[loop]);
      Serial.println("]");*/

      udp.beginPacket();

      switch(RX_DATA.data.key) {
        case BOOT_ON:
          state = BOOT;
        break;
        case BOOT_OFF:
          state = STARTED;
        break;
      }
      udp.write(state);
      if (state == BOOT) {
        cmd.x = RX_DATA.data.cmd_x;
        cmd.y = RX_DATA.data.cmd_y;
        cmd.z = RX_DATA.data.cmd_z;
        udp.write((uint8_t *)&data, 12);
      }
      udp.endPacket();
      delay(25);
      //if( len > 0) {
        break;
      //}
      // Resend the response buffer to the remote IP and port
    } while (millis() - startTime < receiveTimeout);

    if (millis() - startTime >= receiveTimeout) {
      Serial.println("Receive timeout");
      cmd.x = 0; cmd.y = 0; cmd.z = -127;
    }
  }
}