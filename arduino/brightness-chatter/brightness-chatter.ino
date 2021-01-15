#include <Arduino_APDS9960.h>
#include <ArduinoBLE.h>
#include <ros.h>
#include <std_msgs/Int32.h>

// ROS
ros::NodeHandle nh;
std_msgs::Int32 int_msg;
ros::Publisher chatter("brightness", &int_msg);

char hello[13] = "hello world!";

void setup() {
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  while (!Serial);

  if (!APDS.begin()) {
    Serial.println("Error initializing APDS9960 sensor.");
  }

  nh.initNode();
  nh.advertise(chatter);
}

void loop() {
  // check if a color reading is available
  while (! APDS.colorAvailable()) {
    delay(5);
  }
  int r, g, b, a;

  // read the color
  APDS.readColor(r, g, b, a);

  int_msg.data = a;
  chatter.publish( &int_msg );
  nh.spinOnce();

  // wait a bit before reading again
  delay(100);
}