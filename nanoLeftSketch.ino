#include <SPI.h>
#include <WiFiNINA.h>
#include <MadgwickAHRS.h>
#include <Arduino_LSM6DS3.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
//WiFi Setup Stuff
char ssid[] = "roboHandLeft";
char pass[] = "swiggityswoo";
int status = WL_IDLE_STATUS;

IPAddress serverIP(192,168,4,1);
unsigned int localUdpPort = 2390;

WiFiUDP udp;
//Assigning pins to different flex sensors
const int elbowPin = A0;
const int thumbPin = A1;
const int indexPin = A2;
const int middlePin = A3;
const int ringPin = A6;
const int pinkyPin = A7;
//Finger data to 8-bit integer to be sent over wifi packets
struct ArmTrackingData{
  uint8_t thumb;
  uint8_t index;
  uint8_t middle;
  uint8_t ring;
  uint8_t pinky;
  uint8_t wrist;
  uint8_t elbow;
  uint8_t shoulderF;
  uint8_t shoulderS;
};
ArmTrackingData armData;

//IMU (accelerometer and gyroscope) setup
float gyroXOffset = 0;
float gyroYOffset = 0;
float gyroZOffset = 0;
float accelXOffset = 0;
float accelYOffset = 0;
float accelZOffset = 0;
float wristRoll = 0;
Madgwick filter;
unsigned long lastMillis = 0;
unsigned long lastTxMillis = 0;
float frontPitch = 90.0;
float sideRoll = 90.0;
//Default and side addressi (plural of address)
Adafruit_MPU6050 mpuFront;
Adafruit_MPU6050 mpuSide;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("chip started");
  delay(500);
  //Checks to make sure sensors are connected
  while(!mpuFront.begin(0x68)){
    Serial.println("Didn't connect to FRONT shoulder IMU!");
    delay(1000);
  }
  while(!mpuSide.begin(0x69)){
    Serial.println("Didn't connect to SIDE shoulder IMU!");
    delay(1000);
  }
  mpuFront.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpuSide.setFilterBandwidth(MPU6050_BAND_21_HZ);
  //Setup code pauses until connected to the R4 card
  while(status != WL_CONNECTED){
    Serial.print("Attempting to connect to network: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(2000);
  }
  //Starts sending info
  Serial.println("Connected to Robot Hand Network.");
  udp.begin(localUdpPort);
  
  IMU.begin();
  //Filters IMU, has to match frequency (roughly 104 Hz)
  filter.begin(104);
  //This is to account for the discrepencies in the onboard IMU readings
  //Reads the offset of the chip while standing still, this'll be used later to make sure the onboard IMU readings don't start increasing or decreasing without preferred input
  Serial.println("Keep still");
  delay(1000);

  float totalGX = 0;
  float totalGY = 0;
  float totalGZ = 0;
  float totalAX = 0;
  float totalAY = 0;
  float totalAZ = 0;
  int validSamples = 0;

  while (validSamples < 500) {
    if (IMU.gyroscopeAvailable()) {
      float gx;
      float gy;
      float gz;
      float ax;
      float ay;
      float az;
      IMU.readGyroscope(gx, gy, gz);
      IMU.readAcceleration(ax, ay, az);
      totalGX += gx;
      totalGY += gy;
      totalGZ += gz;
      totalAX += ax;
      totalAY += ay;
      totalAZ += az;
      validSamples++;
      delay(3); // Small delay to let the sensor register a fresh packet
    }
  }
  gyroXOffset = totalGX / 500;
  gyroYOffset = totalGY / 500;
  gyroZOffset = totalGZ / 500;
  accelXOffset = totalAX / 500;
  accelYOffset = totalAY / 500;
  accelZOffset = totalAZ / 500;

  Serial.println("Calibration complete");
  Serial.print("Offsets calculated - X: "); 
  Serial.print(gyroXOffset);
  Serial.print(" Y: "); 
  Serial.print(gyroYOffset);
  Serial.print(" Z: ");
  Serial.println(gyroZOffset);

  //Measuring time as to 
  lastMillis = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  // Reads and assigns raw flex sensor values being returned
  int rawThumb = analogRead(thumbPin);
  int rawIndex = analogRead(indexPin);
  int rawMiddle = analogRead(middlePin);
  int rawRing = analogRead(ringPin);
  int rawPinky = analogRead(pinkyPin);
  int rawElbow = analogRead(elbowPin);
  //For reading flex values (each one is slightly different so they need to be constrained with different tested parameters)
/*  Serial.println("VALUES");
  Serial.println(rawThumb);
  Serial.println(rawIndex);
  Serial.println(rawMiddle);
  Serial.println(rawRing);
  Serial.println(rawPinky);
  Serial.println(rawElbow);
*/  unsigned long currentMicros = micros();
  float ax, ay, az, gx, gy, gz;
  //This is mapping raw values to angles 0-180, and constraining them to integers
  */unsigned long currentMicros = micros();
  float ax, ay, az, gx, gy, gz;
  armData.thumb = map(rawThumb,175,1025,0,180);
  armData.thumb = constrain(armData.thumb, 0, 180);
  armData.index = map(rawIndex,40,1030,0,180);
  armData.index = constrain(armData.index, 0, 180);
  //The middle finger flex sensor is a lil wonky and generally returns values within a seally small range, which means that
  //mapping the raw value will result in constantly changing output angles, which looks janky and burns out servo motors due to voltage spikes
  //The middle finger only stays straight or curls all the way, which is fine since mid-range values aren't super important
  if(rawMiddle < 658){
    rawMiddle = 640;
  } else{
    rawMiddle=70;
  }
  armData.middle = map(rawMiddle,640,670,0,180);
  armData.middle = constrain(armData.middle, 0, 180);
  armData.ring = map(rawRing,30,1030,0,180);
  armData.ring = constrain(armData.ring, 0, 180);
  armData.pinky = map(rawPinky,90,1030,0,180);
  armData.pinky = constrain(armData.pinky, 0, 180);
  armData.elbow = map(rawElbow,350,1020,0,90);
  armData.elbow = constrain(armData.elbow, 0, 90);
  armData.elbow = 90 - armData.elbow;
//This is to make sure the chip isn't running the filter (IMU) code faster than it can handle, which can cause it to crash
  float deltaT = (currentMicros-lastMillis)/1000000;
  lastMillis = currentMicros;
  if (deltaT <= 0){
    deltaT=0.001;
  }
  if (deltaT > 0){
    filter.begin(1.0/deltaT);
  }
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()){
    //This is where the onboard gyroscope and accelerometer gets read and where its discrepensies are counted for
    
    IMU.readAcceleration(ax,ay,az);
    IMU.readGyroscope(gx,gy,gz);
    gx -= gyroXOffset;
    gy -= gyroYOffset;
    gz -= gyroZOffset;
    ax -= accelXOffset;
    ay -= accelYOffset;
    az -= accelZOffset;
    float accelRollAngle = (atan2(ay,az) * 180) / PI;
    //The following was GENERATED BY GEMINI (I DID NOT CODE THIS PART MYSELF)
    wristRoll = 0.98 * (wristRoll + (gx*deltaT)) + (0.02*accelRollAngle);
    //Back to my code
    filter.updateIMU(gx,gy,gz,ax,ay,az);
  }
  

  sensors_event_t f_accel, f_gyro, f_temp;
  sensors_event_t s_accel, s_gyro, s_temp;
  mpuFront.getEvent(&f_accel, &f_gyro, &f_temp);
  mpuSide.getEvent(&s_accel, &s_gyro, &s_temp);
    //The following was GENERATED BY GEMINI - comments state what it is they actually do
    //A drift into the y axis when raising arm along z-axis by combining the two and converting from radians to degrees
    float frontAccelAngle = atan2(f_accel.acceleration.y, f_accel.acceleration.z) * 180.0 / PI;
    //Accounts for the slight error of the gyroscope by allowing the accelerometer angle to partially dictate movement (stops spiking)
    frontPitch = 0.96 * (frontPitch + (f_gyro.gyro.x * 180.0 / PI) * deltaT) + 0.04 * frontAccelAngle;
    //Same thing just for the side IMU
    float sideAccelAngle = atan2(s_accel.acceleration.x, s_accel.acceleration.z) * 180.0 / PI;
    sideRoll = 0.96 * (sideRoll + (s_gyro.gyro.y * 180.0 / PI) * deltaT) + 0.04 * sideAccelAngle;
//    End of non me-made code
  //}
  unsigned long currentMillis = millis();
  //The IMU constraining stuff as well as packets being sent only happens once per 50 ms (also helps with spikes due to sensitivity)
  if (currentMillis - lastTxMillis >=50){
    lastTxMillis = currentMillis;
    float rawRoll = filter.getRoll();
    if(wristRoll<0){
      wristRoll+=360;
    }
    if(wristRoll>360){
      wristRoll-=360;
    }
    if (rawRoll<349){
      rawRoll=320;
    }
    if (rawRoll=80){
      rawRoll=80;
    }
    
    if(wristRoll<0){
      wristRoll+=360;
    }
    if(wristRoll>360){
      wristRoll-=360;
    }

    float floatMappedRoll = 180-wristRoll;
    if ((0 > floatMappedRoll) && (floatMappedRoll >= -90)){
      floatMappedRoll=0;
    } if((-90 > floatMappedRoll) && (floatMappedRoll>= -180)){
      floatMappedRoll = 180;
    }
    float floatMappedShoulderF = map(frontPitch,-90,90,0,180);
    float floatMappedShoulderS = map(sideRoll,-90,70,0,180);
    armData.wrist = constrain((int)floatMappedRoll, 0, 180);
    armData.shoulderF = constrain((int)floatMappedShoulderF, 0, 180);
    armData.shoulderS = constrain((int)floatMappedShoulderS, 0, 180);
    //armData.thumb = 90;
    armData.shoulderF = 180 - armData.shoulderF;
    armData.wrist = 180 - armData.wrist;
    //The shoulder mount only allows for movement up to 120 degrees E of S (I don't have a need for it to go further anyways) so it's capped at that value
    if (armData.shoulderS > 120){
      armData.shoulderS = 120;
    }
    //This is all for checking the values live to make sure movement matches measured values (and so that measured values are correct)
/*    Serial.print("Wrist Motion (Roll): ");
    Serial.println(floatMappedRoll);
    Serial.print("Constrained Wrist Motion: ");
    Serial.println(armData.wrist);
    Serial.println("");
    Serial.println(frontPitch);
    Serial.println(sideRoll);
    Serial.println("Front servo 1 angle: ");
    Serial.println(armData.shoulderF);
    Serial.println("Side servo 1 angle: ");
    Serial.println(armData.shoulderS);
*/    
/*    checkSensors(rawElbow, armData.elbow, "Elbow");
    checkSensors(rawThumb, armData.thumb, "Thumb");
    checkSensors(rawIndex, armData.index, "Index");
    checkSensors(rawMiddle, armData.middle, "Middle");
    checkSensors(rawRing, armData.ring, "Ring");
    checkSensors(rawPinky, armData.pinky, "Pinky");
*/  
//Actually constructs packet and sends it
  udp.beginPacket(serverIP, localUdpPort);
  udp.write((uint8_t*)&armData, sizeof(armData));
  udp.endPacket();
  }
}
//This is a function to check flex sensor in and output
/*
void checkSensors(const int &rawAngle, uint8_t &armData, const String &jointName){
  Serial.print("Raw "), Serial.println(jointName);
  Serial.print(rawAngle);
  Serial.println("Output");
  Serial.println(armData);
}*/