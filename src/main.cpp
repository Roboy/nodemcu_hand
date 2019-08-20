#include <ESP8266WiFi.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <Servo.h>
#include <A1335Utils.h>
#include <DualMAX14870MotorShield.h>


#include <sensor_msgs/JointState.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_cognition_msgs/RecognizedFaces.h>

#define HAND_MOTOR_ID 71
#define WRIST_MOTOR_ID 70
#define JOINT_NAME "elbow_right"
#define SERVO_PIN D7
#define DC_PWM_PIN D6
#define DC_DIR_PIN D5
#define DC_EN_PIN D8
#define DC_FAULT_PIN 2
#define LED_PIN 16
//////////////////////
// WiFi Definitions //
//////////////////////
const char* ssid = "roboy";
const char* password = "wiihackroboy";

IPAddress server(192, 168, 0, 134); // ip of your ROS server
IPAddress ip_address;
int status = WL_IDLE_STATUS;

WiFiClient client;

DualMAX14870MotorShield motors(DC_DIR_PIN, DC_PWM_PIN, 50, 50, DC_EN_PIN, DC_FAULT_PIN);

void stopIfFault()
{
  if (motors.getFault())
  {
    Serial.println("DC motors fault");
    while (1)
    {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
}

// void setupMotors() {
//   motors = DualMax14870MotorShield(DC_DIR_PIN, DC_PWM_PIN, 16, 16, 16, DC_FAULT_PIN);
// }

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

Servo myservo;
A1335State state;
int i;

void chatterCallback(const roboy_middleware_msgs::MotorCommand& msg) {
    for (int i=0; i<(sizeof(msg.motors)/sizeof(*msg.motors)); i++) {
      if (msg.motors[i] == WRIST_MOTOR_ID) {
        Serial.print("Got servo setpoint: ");
        Serial.println(msg.set_points[i]);
        myservo.write(msg.set_points[i]);
      }
      else if (msg.motors[i] == HAND_MOTOR_ID) {
        // motors.enableDrivers();
        // motors.setM1Speed(100);
        // stopIfFault();
        // delay(1000);
        // motors.setM2Speed(-100);
        // stopIfFault();

        Serial.print("Got DC setpoint: ");
        Serial.println(msg.set_points[i]);

        //
        digitalWrite(DC_EN_PIN, LOW);
        digitalWrite(DC_PWM_PIN, HIGH);
        if (msg.set_points[i] > 0) {
          digitalWrite(DC_DIR_PIN, HIGH);
        }
        else {
            digitalWrite(DC_DIR_PIN, LOW);
        }


        delay(100);
        digitalWrite(DC_EN_PIN, HIGH);

        // pinMode(DC_PWM_PIN, OUTPUT);
        // digitalWrite(DC_PWM_PIN, HIGH);
        // pinMode(DC_DIR_PIN, OUTPUT);
        // digitalWrite(DC_DIR_PIN, HIGH);
        // pinMode(DC_EN_PIN, OUTPUT);
        // digitalWrite(DC_EN_PIN, LOW);
        // pinMode(DC_FAULT_PIN, INPUT_PULLUP);

        // motors.setM1Speed(msg.set_points[i]);
        // if (msg.set_points[i] >= 0) {
        //   for (int speed = 0; speed <= msg.set_points[i]; speed++)
        //   {
        //     motors.setM1Speed(speed);
        //     // stopIfFault();
        //     delay(20);
        //   }
        // }
        // else {
        //   for (int speed = 0; speed >=msg.set_points[i]; speed--)
        //   {
        //     motors.setM1Speed(speed);
        //     // stopIfFault();
        //     delay(2);
        //   }
        // }

        // stopIfFault();
        // delay(1000);
        // motors.setM1Speed(0);
        // stopIfFault();
        // // motors.disableDrivers();
        // motors.disableDrivers();
      }
    }
}

ros::Subscriber<roboy_middleware_msgs::MotorCommand> sub("/roboy/middleware/MotorCommand", &chatterCallback);

sensor_msgs::JointState rosmsg;
ros::Publisher elbow_pub("/joint_states", &rosmsg);

ros::NodeHandle_<WiFiHardware> nh;

void setupWiFi()
{
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    Serial.print("Could not connect to"); Serial.println(ssid);
    while(1) delay(500);
  }
  Serial.print("Ready! Use ");
  Serial.print(WiFi.localIP());
  Serial.println(" to access client");
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  setupWiFi();
  delay(2000);
  myservo.attach(SERVO_PIN);

  pinMode(DC_PWM_PIN, OUTPUT);
  // digitalWrite(DC_PWM_PIN, HIGH);
  pinMode(DC_DIR_PIN, OUTPUT);
  // digitalWrite(DC_DIR_PIN, HIGH);
  pinMode(DC_EN_PIN, OUTPUT);
  // digitalWrite(DC_EN_PIN, LOW);
  // pinMode(DC_FAULT_PIN, INPUT_PULLUP);
  // delay(10000);

  // setupMotors();


  int8 length = 1;

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
  rosmsg.name[0] = JOINT_NAME;
  rosmsg.velocity[0] = 0.0;
  rosmsg.effort[0] = 0.0;

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(elbow_pub);
}

void loop() {


  readDeviceState(0xF, &state);
  Serial.print(F("    Angle:  "));
  Serial.println(state.angle);
  rosmsg.position[0] = state.angle;
  elbow_pub.publish(&rosmsg);
  nh.spinOnce();

  delay(500);
}
