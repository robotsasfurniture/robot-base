//   This program is based on template code for programming small esp32 powered wifi controlled robots.
//   https://github.com/rcmgames/RCMv2
//   for information about rcmgames, see this page: https://github.com/RCMgames

//   This program is for a robot base with 4 mecanum wheels, for the robots as furniture project.
//   https://github.com/robotsasfurniture/robot-base

// #define RCM_HARDWARE_VERSION 10 // uncomment if you have an RCMByte board
// to use ROS mode switch the platformio project environment to one of the environments that says ROS in the name

#include "rcm.h" //defines pins

const int dacUnitsPerVolt = 280; // increasing this number decreases the calculated voltage
const byte batteryMonitorPin = 35;
JVoltageCompMeasure<10> voltageComp = JVoltageCompMeasure<10>(batteryMonitorPin, dacUnitsPerVolt);
// set up motors and anything else you need here
// https://github.com/joshua-8/JMotor/wiki/How-to-set-up-a-drivetrain
const int encoderTicksPerRev = 530;
JTwoDTransform robotToWheelScalar = { 3.5, 4, .676 }; // adjust until it converts robot speed in your chosen units to wheel units (increasing numbers makes robot faster)

#define blEncPins 36, 39
#define flEncPins 19, 27
#define frEncPins 22, 34
#define brEncPins 23, 21

// byte _pinPosCh, byte _pinPos, byte _pinNeg, byte _pinNegCh
#define blMotPins 8, 25, 26, 9
#define flMotPins 10, 33, 32, 11
#define frMotPins 12, 16, 17, 13
#define brMotPins 14, 5, 18, 15

JMotorDriverEsp32HBridge blMotorDriver = JMotorDriverEsp32HBridge(blMotPins, false, 2000, 12, false);
JMotorDriverEsp32HBridge flMotorDriver = JMotorDriverEsp32HBridge(flMotPins, false, 2000, 12, false);
JMotorDriverEsp32HBridge frMotorDriver = JMotorDriverEsp32HBridge(frMotPins, false, 2000, 12, false);
JMotorDriverEsp32HBridge brMotorDriver = JMotorDriverEsp32HBridge(brMotPins, false, 2000, 12, false);

JEncoderQuadratureAttachInterrupt blEncoder = JEncoderQuadratureAttachInterrupt(blEncPins, 1.0 / encoderTicksPerRev, true, 200000);
JEncoderQuadratureAttachInterrupt flEncoder = JEncoderQuadratureAttachInterrupt(flEncPins, 1.0 / encoderTicksPerRev, true, 200000);
JEncoderQuadratureAttachInterrupt frEncoder = JEncoderQuadratureAttachInterrupt(frEncPins, 1.0 / encoderTicksPerRev, true, 200000);
JEncoderQuadratureAttachInterrupt brEncoder = JEncoderQuadratureAttachInterrupt(brEncPins, 1.0 / encoderTicksPerRev, true, 200000);

JMotorCompBasic motorCompensator = JMotorCompBasic(voltageComp, 3, 0.5); // volts per rps, min rps

JControlLoopBasic flCtrlLoop = JControlLoopBasic(30);
JControlLoopBasic frCtrlLoop = JControlLoopBasic(30);
JControlLoopBasic blCtrlLoop = JControlLoopBasic(30);
JControlLoopBasic brCtrlLoop = JControlLoopBasic(30);

JMotorControllerClosed flMotor = JMotorControllerClosed(flMotorDriver, motorCompensator, flEncoder, flCtrlLoop, INFINITY, INFINITY, 0.15);
JMotorControllerClosed frMotor = JMotorControllerClosed(frMotorDriver, motorCompensator, frEncoder, frCtrlLoop, INFINITY, INFINITY, 0.15);
JMotorControllerClosed blMotor = JMotorControllerClosed(blMotorDriver, motorCompensator, blEncoder, blCtrlLoop, INFINITY, INFINITY, 0.15);
JMotorControllerClosed brMotor = JMotorControllerClosed(brMotorDriver, motorCompensator, brEncoder, brCtrlLoop, INFINITY, INFINITY, 0.15);

JDrivetrainMecanum drivetrain = JDrivetrainMecanum(frMotor, flMotor, blMotor, brMotor, robotToWheelScalar); // drivetrain converts from robot speed to motor speeds
JDrivetrainControllerBasic drivetrainController = JDrivetrainControllerBasic(drivetrain, { INFINITY, INFINITY, INFINITY }, { 2, 2, 8 }, { 0.05, 0.05, 0.5 }, false);

JTwoDTransform velCmd = JTwoDTransform();

void Enabled()
{
    // code to run while enabled, put your main code here
}

void Enable()
{
    // turn on outputs
    drivetrainController.resetDist();
    drivetrainController.moveDist({ 0, 0, 0 });
    drivetrainController.enable();
    velCmd = { 0, 0, 0 };
}

void Disable()
{
    // turn off outputs
    drivetrainController.disable();
    velCmd = { 0, 0, 0 };
}

jENCODER_MAKE_ISRS_MACRO(blEncoder);
jENCODER_MAKE_ISRS_MACRO(flEncoder);
jENCODER_MAKE_ISRS_MACRO(frEncoder);
jENCODER_MAKE_ISRS_MACRO(brEncoder);
void PowerOn()
{
    // runs once on robot startup, set pin modes and use begin() if applicable here
    blMotorDriver.pwmDriverPos.disableState = HIGH;
    blMotorDriver.pwmDriverNeg.disableState = HIGH;
    flMotorDriver.pwmDriverPos.disableState = HIGH;
    flMotorDriver.pwmDriverNeg.disableState = HIGH;
    frMotorDriver.pwmDriverPos.disableState = HIGH;
    frMotorDriver.pwmDriverNeg.disableState = HIGH;
    brMotorDriver.pwmDriverPos.disableState = HIGH;
    brMotorDriver.pwmDriverNeg.disableState = HIGH;

    // runs once on robot startup, set pin modes and use begin() if applicable here
    blEncoder.setUpInterrupts(blEncoder_jENCODER_ISR_A, blEncoder_jENCODER_ISR_B);
    flEncoder.setUpInterrupts(flEncoder_jENCODER_ISR_A, flEncoder_jENCODER_ISR_B);
    frEncoder.setUpInterrupts(frEncoder_jENCODER_ISR_A, frEncoder_jENCODER_ISR_B);
    brEncoder.setUpInterrupts(brEncoder_jENCODER_ISR_A, brEncoder_jENCODER_ISR_B);
}

void Always()
{
    // always runs if void loop is running, JMotor run() functions should be put here
    // (but only the "top level", for example if you call drivetrainController.run() you shouldn't also call leftMotorController.run())
    drivetrainController.run();
    delay(1);
}

#ifndef RCM_ROS
void WifiDataToParse()
{
    enabled = EWD::recvBl();
    // add data to read here: (EWD::recvBl, EWD::recvBy, EWD::recvIn, EWD::recvFl)(boolean, byte, int, float)
}
void WifiDataToSend()
{
    EWD::sendFl(voltageComp.getSupplyVoltage());
    // add data to send here: (EWD::sendBl(), EWD::sendBy(), EWD::sendIn(), EWD::sendFl())(boolean, byte, int, float)
}

void configWifi()
{
    EWD::mode = EWD::Mode::connectToNetwork;
    EWD::routerName = "router";
    EWD::routerPassword = "password";
    EWD::routerPort = 25210;

    // EWD::mode = EWD::Mode::createAP;
    // EWD::APName = "rcm0";
    // EWD::APPassword = "rcmPassword";
    // EWD::APPort = 25210;
}
#else ////////////// ignore everything below this line unless you're using ROS mode/////////////////////////////////////////////
void ROSWifiSettings()
{
    // SSID, password, IP, port (on a computer run: sudo docker run -it --rm --net=host microros/micro-ros-agent:iron udp4 --port 8888 )

    // Serial.print("\nESP Board MAC Address:  "); Serial.println(WiFi.macAddress()); //go.brown.edu/wirelessdevices
    // NOTE: HERE IS WHERE YOU TELL THE ROBOT WHAT TO CONNECT TO
    set_microros_wifi_transports("router", "password", "192.168.43.110", 8888);
    nodeName = "robot_base";
    // numSubscribers = 10; // change max number of subscribers
}

#include <example_interfaces/msg/bool.h>
#include <std_msgs/msg/byte.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
// and lots of other message types are available (see file available_ros2_types)
#include <geometry_msgs/msg/twist.h>

// declare publishers
declarePub(battery, std_msgs__msg__Float32);
declarePub(odom_vel, geometry_msgs__msg__Twist);

// declare subscribers and write callback functions
declareSubAndCallback(cmd_vel, geometry_msgs__msg__Twist);
velCmd.x = cmd_velMsg->linear.x;
velCmd.y = cmd_velMsg->linear.y;
velCmd.theta = cmd_velMsg->angular.z;
drivetrainController.moveVel(velCmd, false);
} // end of callback

void ROSbegin()
{
    // create publishers
    createPublisher(battery, std_msgs__msg__Float32, "/rcm/battery");
    batteryMsg.data = 0;

    // add subscribers
    addSub(cmd_vel, geometry_msgs__msg__Twist, "/rcm/cmd_vel");

    createPublisher(odom_vel, geometry_msgs__msg__Twist, "/rcm/odom_vel");
}

void ROSrun()
{
    rosSpin(1);
    // you can add more publishers here
    batteryMsg.data = voltageComp.getSupplyVoltage();
    publish(battery);

    odom_velMsg.linear.x = drivetrainController.getVel().x;
    odom_velMsg.linear.y = drivetrainController.getVel().y;
    odom_velMsg.angular.z = drivetrainController.getVel().theta;
    publish(odom_vel);
}
#endif

#include "rcmutil.h"
