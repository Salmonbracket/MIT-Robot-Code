#include <WiFiS3.h>
#include <Servo.h>
//Assigning Udp Port along with ssid and password (different from the other hand)
const char* ssid = "roboHandLeft";
const char* pass = "swiggityswoo";
unsigned int localUdpPort = 2390;

WiFiUDP udp;

//Restructuring the sent data packages so it can be read (MUST BE SAME ORDER (learned this one the hard way))
struct ArmTrackingData {
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

ArmTrackingData incomingHand;

//Assigning servos
Servo thumb;
Servo index;
Servo middle;
Servo ring;
Servo pinky;
Servo wrist;
Servo elbow;
Servo shoulderFMain;
Servo shoulderFHelp;
Servo shoulderSMain;
Servo shoulderSHelp;



uint8_t lastThumbAngle = 0;
uint8_t lastIndexAngle = 0;
uint8_t lastMiddleAngle = 0;
uint8_t lastRingAngle = 0;
uint8_t lastPinkyAngle = 0;
uint8_t lastWristAngle = 0;
uint8_t lastElbowAngle = 90;
uint8_t lastFrontAngle = 0;
uint8_t lastSideAngle = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);

  Serial.println("Configuring Access Point... ");

  WiFi.beginAP(ssid,pass);

  IPAddress myIP = WiFi.localIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  
  udp.begin(localUdpPort);
  Serial.println("UDP listener started. Ready for glove data.");
  //After connected to glove, attatches servos to their respective pins (skips 8 because I broke a pin and it's stuck in there)
  wrist.attach(2);
  thumb.attach(3);
  index.attach(4);
  middle.attach(5);
  ring.attach(6);
  pinky.attach(7);
  elbow.attach(8);
  shoulderFMain.attach(9);
  shoulderFHelp.attach(10);
  shoulderSMain.attach(12);
  shoulderSHelp.attach(13);

}

void loop() {
  // put your main code here, to run repeatedly:
  int packetSize = udp.parsePacket();
  if(packetSize>0){
    //Actually writes sent angles to servo motors
    //Zhu Li, do the thing!
    udp.read((char*)&incomingHand, sizeof(incomingHand));
    doTheThing(incomingHand.thumb, lastThumbAngle, thumb, "Thumb: ");
    doTheThing(incomingHand.index, lastThumbAngle, index, "Index: ");
    doTheThing(incomingHand.middle, lastMiddleAngle, middle, "Middle: ");
    doTheThing(incomingHand.ring, lastRingAngle, ring, "Ring: ");
    doTheThing(incomingHand.pinky, lastPinkyAngle, pinky, "Pinky: ");
    doTheThing(incomingHand.wrist, lastWristAngle, wrist, "Wrist: ");
    doTheThing(incomingHand.elbow, lastElbowAngle, elbow, "Elbow: ");
    zhuLi(incomingHand.shoulderF, lastFrontAngle, shoulderFMain, shoulderFHelp, "Front_Joint");
    zhuLi(incomingHand.shoulderS, lastSideAngle, shoulderSMain, shoulderSHelp, "Side_Joint");
  }
  //This delay is needed to reduce voltage spikes, it only runs once every 50 ms
  delay(50);
}

void doTheThing(uint8_t incomingHand, uint8_t &lastAngle, Servo &servo, const String &fingerName) {
  //This is the servo movement function for all non-servo motors
  if (abs(incomingHand-lastAngle) >=10){
      servo.write(incomingHand);
      lastAngle = incomingHand;
    }
//    Serial.print(fingerName); Serial.print(incomingHand);
}

void zhuLi(uint8_t newAngle, uint8_t &lastAngle, Servo &mainServo, Servo &helpServo, const String &jointName){
  //This is the servo movement function for shoulder motors, since they come in pairs with one mirroring the movement of the other*
  if(abs(newAngle - lastAngle)>=2){
    mainServo.write(newAngle);
    uint8_t invertedAngle = 180 - newAngle;
    helpServo.write(invertedAngle);
    lastAngle = newAngle;
/*    Serial.print(jointName);
    Serial.print(" Joint Moved To: ");
    Serial.println(newAngle);
*/  }
}