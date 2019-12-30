#include <ESP8266WiFi.h>
#include <Wire.h>
// #include "ardprintf.h"

#include <Servo.h>
#include <A1335Utils.h>
#include <SPI.h>

#include <ros.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/JointState.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/MotorStatus.h>
#include <roboy_cognition_msgs/RecognizedFaces.h>
#include <roboy_middleware_msgs/ControlMode.h>

#define LOGGING 1
#define SPIN_PERIOD 200

#define BODY_PART_ID 6

#define MYOBRICK0_ID 0
#define MYOBRICK1_ID 1
#define WRIST_MOTOR_ID 2
#define HAND_MOTOR_ID 3

#define ELBOW_JOINT_NAME "elbow_right"
#define WRIST_JOINT_NAME "wrist_right"

#define SERVO_PIN D0
#define DC_PWM_PIN D3
#define DC_DIR_PIN D4

#define POSITION 0
#define VELOCITY 1
#define DISPLACEMENT 2
#define DIRECT_PWM 3
#define maxPWM 500

static const int spiClk = 2000000; // 2 MHz

const int ss_n[2] = {9,10};

float Kp[2] = {0.1,0.1}, Kd[2] = {0.01,0.01}, err[2] = {0,0}, err_prev[2] = {0,0}, myo_setpoint[2] = {0,0}, result[2] = {0,0};
int control_mode[2] = {POSITION, POSITION};

int32_t position[2] = {0,0};
int16_t velocity[2] = {0,0}, current[2] = {0,0}, pwmRef[2] = {0,0};

Servo myservo;
int servo_setpoint = 0;
A1335State state;
int i;

const char* ssid = "roboy";
const char* password = "wiihackroboy";

IPAddress server(192, 168, 0, 114); // ip of your ROS server
IPAddress ip_address;
int status = WL_IDLE_STATUS;

WiFiClient client;

class WiFiHardware {

  public:
  WiFiHardware() {};

  void init() {
    // do your initialization here. this probably includes TCP server/client setup
    client.connect(server, 11411);
  }

  // read a byte from the serial port. -1 = failure
  int read() {
    // implement this method so that it reads a byte from the TCP connection and returns it
    //  you may return -1 is there is an error; for example if the TCP connection is not open
    return client.read();         //will return -1 when it will works
  }

  // write data to the connection to ROS
  void write(uint8_t* data, int length) {
    // implement this so that it takes the arguments and writes or prints them to the TCP connection
    for(int i=0; i<length; i++)
      client.write(data[i]);
  }

  // returns milliseconds since start of program
  unsigned long time() {
     return millis(); // easy; did this one for you
  }
};

void motorCommandCallback(const roboy_middleware_msgs::MotorCommand& msg) {

}

void controlModeCallback(const roboy_middleware_msgs::ControlMode::Request& req, roboy_middleware_msgs::ControlMode::Response& res) {


}

void myobrickLoop() {
  for(int motor=0;motor<2;motor++){
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
    uint16_t data[2];
    for(int i=0;i<12;i++){
      digitalWrite(ss_n[motor],LOW);
      delayMicroseconds(1);
      if(i==0)
        SPI.transfer16(0x8000); // header
      else if(i==1)
        SPI.transfer16((pwmRef[motor]& 0x7fff));
      else{
        switch(i){
          case 4:
            data[0] = SPI.transfer16(0);
            break;
          case 5:
            data[1] = SPI.transfer16(0);
            position[motor] =  ((data[0]>>8)<<24|(data[0]&0xff)<<16|(data[1]>>8)<<8|(data[1]&0xff));
            break;
          case 6:
            velocity[motor] = SPI.transfer16(0);
            break;
          case 7:
            current[motor] = SPI.transfer16(0);
            break;
          default:
            SPI.transfer16(0);
           break;
         }
      }
      digitalWrite(ss_n[motor],HIGH);
    }
    SPI.endTransaction();
  }
  // controller
  for(int motor=0;motor<2;motor++){
    switch(control_mode[motor]){
      case POSITION:
        err[motor] = myo_setpoint[motor]-position[motor];
        break;
      case VELOCITY:
        err[motor] = myo_setpoint[motor]-velocity[motor];
        break;
      case DISPLACEMENT: // not implemented
        err[motor] = 0;
        break;
      case DIRECT_PWM:
        result[motor] = myo_setpoint[motor];
        break;
    }
    if(control_mode[motor]!=DIRECT_PWM){
      result[motor] = Kp[motor]*err[motor] + Kd[motor]*(err_prev[motor]-err[motor]);
      err_prev[motor] = err[motor];
    }
    if(result[motor] > maxPWM){
      result[motor] = maxPWM;
    }
    if(result[motor] < -maxPWM){
      result[motor] = -maxPWM;
    }
    pwmRef[motor] = result[motor];
  }
  for(int motor=0;motor<2;motor++){
    // Serial.printf("pwmRef: %d positions: %d velocities: %d currents: %d\n",
    //    pwmRef[motor], position[motor], velocity[motor], current[motor]
    // );
  }
}

ros::Subscriber<roboy_middleware_msgs::MotorCommand> sub("/roboy/middleware/MotorCommand", &motorCommandCallback);
ros::ServiceServer<roboy_middleware_msgs::ControlMode::Request, roboy_middleware_msgs::ControlMode::Response> controlModeServer("/roboy/middleware/ControlMode", &controlModeCallback);

sensor_msgs::JointState rosmsg;
std_msgs::Int16 wrist_msg;
roboy_middleware_msgs::MotorStatus motor_status_msg;
ros::Publisher joint_states_pub("/external_joint_states", &rosmsg);
// ros::Publisher wrist_pub("/roboy/middleware/wrist", &wrist_msg);
ros::Publisher motor_status_pub("/roboy/middleware/MotorStatus", &motor_status_msg);

ros::NodeHandle_<WiFiHardware> nh;
long last_spin = 0;

void setupWiFi()
{
  WiFi.begin(ssid, password);

  if (LOGGING) {
    Serial.print("\nConnecting to "); Serial.println(ssid);
  }
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    if (LOGGING) {
      Serial.print("Could not connect to"); Serial.println(ssid);
    }
    while(1) delay(500);
  }
  if (LOGGING) {
    Serial.print("Ready! Use ");
    Serial.print(WiFi.localIP());
    Serial.println(" to access client");
  }
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  if (LOGGING) {
    Serial.begin(115200);
  }
  setupWiFi();
  delay(2000);

  // servo
  myservo.attach(SERVO_PIN);

  // hand DC
  pinMode(DC_PWM_PIN, OUTPUT);
  pinMode(DC_DIR_PIN, OUTPUT);

  // myobrick
  SPI.begin();

  for (int motor=0; motor<2; motor++){
    pinMode (ss_n[motor], OUTPUT);
    digitalWrite(ss_n[motor],HIGH);
  }

  int8 length = 2;

  rosmsg.name_length = length;
  rosmsg.position_length = length;
  rosmsg.velocity_length = length;
  rosmsg.effort_length = length;
  rosmsg.position = (float *)malloc(sizeof(float)*length);
  rosmsg.velocity = (float *)malloc(sizeof(float)*length);
  rosmsg.effort = (float *)malloc(sizeof(float)*length);
  rosmsg.name = (char**) calloc(length, sizeof(char*));
  for ( i = 0; i < length; i++ )
  {
      rosmsg.name[i] = (char*) calloc(20, sizeof(char));
  }
  rosmsg.name[0] = ELBOW_JOINT_NAME;
  rosmsg.name[1] = WRIST_JOINT_NAME;
  for (int i=0; i<length; i++) {
    rosmsg.velocity[i] = 0.0;
    rosmsg.effort[i] = 0.0;
  }

  length = 2;
  motor_status_msg.id = BODY_PART_ID;
  motor_status_msg.position_length = length;
  motor_status_msg.velocity_length = length;
  motor_status_msg.current_length = length;
  motor_status_msg.pwm_ref_length = length;
  motor_status_msg.position = (int32 *)malloc(sizeof(int32)*length);
  motor_status_msg.velocity = (int32 *)malloc(sizeof(int32)*length);
  motor_status_msg.current = (int16_t *)malloc(sizeof(int16_t)*length);
  motor_status_msg.pwm_ref = (int32 *)malloc(sizeof(int32)*length);


  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(joint_states_pub);
  // nh.advertise(wrist_pub);
  nh.advertise(motor_status_pub);
  nh.advertiseService(controlModeServer);
  nh.spinOnce();

  delay(5000);
  Serial.println("ros initialized");
}


void loop() {
  myobrickLoop();

  long now = millis();
  if (now - last_spin > SPIN_PERIOD) {
    readDeviceState(0xF, &state);

    rosmsg.position[0] = state.angle;
    rosmsg.position[1] = servo_setpoint;
    joint_states_pub.publish(&rosmsg);

    for (int i=0; i<2; i++) {
      motor_status_msg.position[i] = position[i];
      motor_status_msg.velocity[i] = velocity[i];
      motor_status_msg.pwm_ref[i] = pwmRef[i];
      motor_status_msg.current[i] = current[i];
    }
    motor_status_pub.publish(&motor_status_msg);

    nh.spinOnce();
    last_spin = now;
  }

}
