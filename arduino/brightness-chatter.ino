/*
  Arduino Nano 33 BLE Getting Started
  BLE peripheral with a simple Hello World greeting service that can be viewed
  on a mobile phone
  Adapted from Arduino BatteryMonitor example
*/
#include <Arduino_APDS9960.h>
#include <ArduinoBLE.h>
#include <ros.h>
#include <std_msgs/Int32.h>

// ROS
ros::NodeHandle nh;
std_msgs::Int32 int_msg;
ros::Publisher chatter("brightness", &int_msg);

// BLE
static const char* greeting = "Hello World!";
BLEService greetingService("180C");  // User defined service
BLEStringCharacteristic greetingCharacteristic("2A56",  // standard 16-bit characteristic UUID
    BLERead, 13); // remote clients will only be able to read this

void setup() {
  Serial.begin(115200);    // initialize serial communication
  nh.getHardware()->setBaud(115200);
  while (!Serial);

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin

  // Check BLE
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  // Check APDS Sensor
  if (!APDS.begin()) {
    Serial.println("Error initializing APDS9960 sensor.");
  }

  BLE.setLocalName("Nano33BLE");  // Set name for connection
  BLE.setAdvertisedService(greetingService); // Advertise service
  greetingService.addCharacteristic(greetingCharacteristic); // Add characteristic to service
  BLE.addService(greetingService); // Add service
  greetingCharacteristic.setValue(greeting); // Set greeting string

  // Init ROS node
  nh.initNode();
  nh.advertise(chatter);
  
  BLE.advertise();  // Start advertising
  Serial.print("Peripheral device MAC: ");
  Serial.println(BLE.address());
  Serial.println("Waiting for connections...");
}

void loop() {
  BLEDevice central = BLE.central();  // Wait for a BLE central to connect

  // check if a color reading is available
  while (! APDS.colorAvailable()) {
    delay(5);
  }
  int r, g, b, a; // a -> brightness
  // read the color
  APDS.readColor(r, g, b, a);

  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central MAC: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    int_msg.data = a;
    chatter.publish( &int_msg );
    nh.spinOnce();

    while (central.connected()){} // keep looping while connected
    
    // when the central disconnects, turn off the LED:
    Serial.print("Disconnected from central MAC: ");
    Serial.println(central.address());
  }
}
