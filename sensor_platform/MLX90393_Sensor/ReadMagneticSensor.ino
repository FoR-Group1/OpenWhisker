#include <Adafruit_MLX90393.h>

#include <Arduino.h>
#include <Wire.h>

Adafruit_MLX90393 sensor = Adafruit_MLX90393();

#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_ADDR 0x18
#define BAUD 115200

TwoWire wire = Wire.begin(SDA_PIN, SCL_PIN);

long prev_time_freq = micros();
long time_freq = 0;
int freq = 0;
int last_freq;
int count = 0;

void setup() {

  Serial.begin(BAUD);

  while (!Serial) {
    delay(10);
  }

  delay(500);

  Serial.println("Code starting");

  // configs
  sensor.setResolution(MLX90393_X, MLX90393_RES_16);
  sensor.setResolution(MLX90393_Y, MLX90393_RES_16);
  sensor.setResolution(MLX90393_Z, MLX90393_RES_16);

  // Wire.setPins(SDA_PIN, SCL_PIN);
  if (! sensor.begin_I2C(0x18, &Wire)) {
    while (1) { delay(10); Serial.println("noot noot - "); }
  }
}

void loop() {
  time_freq = micros();



  // put your main code here, to run repeatedly:
  // while(Wire.available()) {
  //       char c = Wire.read();    // Receive a byte as character
  //       Serial.print(c);         // Print the character
  //   }
  // Serial.println("Ping");

  // sensor.getGain();
  // switch (sensor.getGain()) {
  //   case MLX90393_GAIN_1X: break;
  //   case MLX90393_GAIN_1_33X:  break;
  //   case MLX90393_GAIN_1_67X: break;
  //   case MLX90393_GAIN_2X: break;
  //   case MLX90393_GAIN_2_5X: break;
  //   case MLX90393_GAIN_3X: break;
  //   case MLX90393_GAIN_4X: break;
  //   case MLX90393_GAIN_5X: break;
  // }
  // sensor.setGain(MLX90393_GAIN_5X);
  // sensor.setFilter(MLX90393_FILTER_0);
  // sensor.setOversampling(MLX90393_OSR_0);

  float x, y, z;
  sensors_event_t event;
  // if (sensor.readData(&x, &y, &z)) {
  //     Serial.print("\nX: "); Serial.print(x, 4);
  //     Serial.print("\tY: "); Serial.print(y, 4);
  //     Serial.print("\tZ: "); Serial.print(z, 4);
  // } 
  // else {
  //     Serial.println("Unable to read XYZ data from the sensor.");
  // }

  if (sensor.getEvent(&event)) {
    if ((count%5 == 0)) { // reducing the actual Serial print rate because this affects the performace quite a bit
      // Serial.println("X: "); Serial.print(event.magnetic.x);
      // Serial.print(" \tY: "); Serial.print(event.magnetic.y);
      // Serial.print(" \tZ: "); Serial.print(event.magnetic.z);
      Serial.print(String(event.magnetic.x) + "\t\t" + String(event.magnetic.y) + "\t\t" + String(event.magnetic.z)+ "\t");
      Serial.print("\tFreq: "); Serial.print(last_freq);Serial.println("Hz");
    }
  } else {
      Serial.println("Unable to read event data.");
      // while (1) { delay(50); Serial.println("kill me now");}
  }

  // Frequency Measurements
  count++;
  if ((time_freq - prev_time_freq) > 1000000) {
    prev_time_freq = time_freq;
    last_freq = count;
    count = 0;
    // Serial.print("\tFreq: "); Serial.print(last_freq);Serial.println("Hz");
  }

}

