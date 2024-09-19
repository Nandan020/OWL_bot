#include <ESP8266WiFi.h>                                                          // Including libraries for wifi
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <Servo.h>                                                                 // Including library for Servo
#include <NewPing.h>

/* Defining motor control pins */
#define left_motor_forward    D1   // D1->IN1
#define left_motor_backward   D2   // D2->IN2
#define right_motor_backward  D3   // D3->IN3
#define right_motor_forward   D4   // D4->IN4

#define left_ir_sensor    A0                                                               // Infrared sensor pins
#define right_ir_sensor   D0

#define trigger_pin  D8                                                                  // Ultrasonic sensor pins
#define echo_pin     D7

#define MAX_DISTANCE 250

/* At the time of code uploading remove connections of Rx and Tx pin */

String command;                                                             //  String to store app command state

Servo ultrasonic_servo_motor;
Servo arm_servo_motor;

int arm_angle = 0;
float distance = 100.0;

const char* ssid = "XYZ_NAME";                                             // Wifi Name, set as per your convenience
const char* password = "13578642";                                                                // Set password // must be of 8 digit
ESP8266WebServer server(80);                                                // Starting the Web-Server at port:80

void setup() 
{
  pinMode(left_motor_backward, OUTPUT);
  pinMode(left_motor_forward, OUTPUT);
  pinMode(right_motor_backward, OUTPUT);
  pinMode(right_motor_forward, OUTPUT);

  pinMode(left_ir_sensor, INPUT);
  pinMode(right_ir_sensor, INPUT);

  ultrasonic_servo_motor.attach(D6);                  // Using TX pin, remove this connection while uploading code
  ultrasonic_servo_motor.write(90);  
  NewPing sonar(trigger_pin, echo_pin, MAX_DISTANCE);

  arm_servo_motor.attach(D5);                                                       // Attaching the Servo to pin
  delay(1000);
  arm_servo_motor.write(arm_angle);

  Serial.begin(9600);                                                        // Setting the up the Serial Monitor

  WiFi.mode(WIFI_AP);                                                                      // Setting up the Wifi
  WiFi.softAP(ssid, password);

  /* Starting the Web-Server */

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.on ( "/", HTTP_handleRoot );
  server.onNotFound ( HTTP_handleRoot );
  server.begin();

  Stop();  
}

/*********************************************** MOTOR FUNCTIONS ***********************************************/
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

/*************************************************** FORWARD ***************************************************/

void Forward(int speed = 255)
{
  analogWrite(left_motor_forward,speed);
  analogWrite(right_motor_forward,speed);
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_backward,LOW);
  Serial.println("Forward//////");
}

/*************************************************** BACKWARD **************************************************/
void Backward(int speed = 255)
{
  analogWrite(left_motor_backward,speed);
  analogWrite(right_motor_backward,speed);
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(right_motor_forward,LOW);
  Serial.println("Backward//////");
}

/************************************************** TURN LEFT **************************************************/
void Left(int speed = 255)
{
  analogWrite(right_motor_forward,speed);
  analogWrite(left_motor_backward,speed);
  digitalWrite(right_motor_backward,LOW);
  digitalWrite(left_motor_forward,LOW);
  Serial.println("Left//////");
}

/************************************************** TURN RIGHT *************************************************/
void Right(int speed = 255)
{
  analogWrite(left_motor_forward,speed);
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_forward,LOW);
  analogWrite(right_motor_backward,speed);
  Serial.println("Right//////");
}
/**************************************************** STOP *****************************************************/
void Stop()
{
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_forward,LOW);
  digitalWrite(right_motor_backward,LOW);
}

/********************************************* LINE FOLLOWER MODE **********************************************/

void line_follower_mode()
{
  while(command!="ST" and command!="m" and command!="o"){
    // Serial.println("Line follower mode ");
    if (analogRead(left_ir_sensor) < 512  and digitalRead(right_ir_sensor) == LOW)
      Serial.println("Forward"), Forward(100);

    if (analogRead(left_ir_sensor) >= 512  and digitalRead(right_ir_sensor) == LOW)
      Serial.println("Left"), Left(100);

    if (analogRead(left_ir_sensor) < 512  and digitalRead(right_ir_sensor) == HIGH)
      Serial.println("Right"), Right(100);

    if (analogRead(left_ir_sensor) >= 512  and digitalRead(right_ir_sensor) == HIGH)
      Serial.println("Stop"), Stop();
    
    server.handleClient();
    command = server.arg("State");
  }
}

/******************************************* OBSTACLE AVOIDANCE MODE *******************************************/

void obstacle_avoidance_mode()
{
  while(command!="ST" and command!="m" and command!="l")
  {
    distance = measure_distance();
    while((distance<=20 or distance<0) and command!="ST" and command!="m" and command!="l")            // If forward distance is less than 20 cm or reading is incorrect 
    {                                            // then stop for 50 miliseconds and check right distance.
      back_Stop();                              
      delay(50);
      ultrasonic_servo_motor.write(0);                                                 // Ultrasonic sensor faces right side
      delay(300);
      distance = measure_distance();
   
      ultrasonic_servo_motor.write(90);                            // Ultrasonic sensor is back to position ,i.e faces front
      delay(300);
     
      if(distance>20)                                   // If right distance is greater than 20 cm than turn right
      {
        Right();
        delay(400);
        distance = measure_distance();
      }
      else                                            // If right distance is not greater than 20 cm it turns left
      {
        Left(255);
        delay(400);
        distance = measure_distance();
      }
    }
    distance = measure_distance();                                 // If forward distance is greater than 20 cm means bot goes forward

    while(distance>20 and command!="ST" and command!="m" and command!="l")  
    {
      ultrasonic_servo_motor.write(90);                               // U.S sensor is at original position ,i.e faces front
      delay(300);
      Forward(255);
      distance = measure_distance();
    }
  }
}

float measure_distance()                       // This function measures the distance of obstacle from the Ultrasonic sensor
{
  digitalWrite(trigger_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_pin, LOW);

  float duration = pulseIn(echo_pin, HIGH);         // Calculates after how much time we are getting the response
  float distance = (duration * 0.034) / 2;                           // Speed of Sound=340m/s i.e. 0.034cm/microseconds

  Serial.print("Distance=");
  Serial.print(distance);
  Serial.print(" ");

  server.handleClient();
  command = server.arg("State");
  return distance;
}

void back_Stop()
{
  Backward(255);
  delay(250);
  Stop();  
}

/*************************************************** ARM MODE ***************************************************/

void arm_up()
{
  if(arm_angle<=0)
    arm_angle=0;
  arm_angle -= 5;
  arm_servo_motor.write(arm_angle);
}

void arm_down()
{
   if(arm_angle>=180)
    arm_angle=180;
  arm_angle += 5;
  arm_servo_motor.write(arm_angle);
}

bool wifi = 0, line = 0, obs = 0;

void loop() 
{
  server.handleClient();
  command = server.arg("State");

  if (command == "m")
  {
    Stop();
    wifi  = 1;
    line = 0;
    obs = 0;
  }

  else if (command == "l")
  {
    Stop();
    wifi = 0;
    line = 1;
    obs = 0;
  }

  else if (command == "o")
  {
    Stop();
    wifi = 0;
    line = 0;
    obs = 1;
  }

  else if (command == "ST")
  {
    Stop();
  }

/************************************************** WIFI MODE ***************************************************/
  if (wifi)
  {
    if (command == "F") 
      Forward();
    else if (command == "B") 
      Backward();
    else if (command == "L") 
      Left();
    else if (command == "R") 
      Right();
    else if (command == "V")  
      Stop();
    else if (command == "P") 
      arm_up();
    else if (command == "S") 
      arm_down();
  }
  if(line)
    line_follower_mode();
  if(obs)
    obstacle_avoidance_mode();

}

void HTTP_handleRoot(void)
{
  if ( server.hasArg("State") )
    Serial.println(server.arg("State"));
  
  server.send ( 200, "text/html", "" );

  delay(1);
}
