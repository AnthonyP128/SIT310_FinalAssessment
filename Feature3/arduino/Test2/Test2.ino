// defines pins numbers
const int echoPinLeft = 14;
const int trigPinLeft = 15;

const int echoPinFront = 10;
const int trigPinFront = 9;

const int echoPinRight = 3;
const int trigPinRight = 2;

// defines variables
long duration;
int distance;

int motorright = 9;                                                              
int motorrightdir  = 7;
int motorleft = 10;
int motorleftdir  = 8;

//For Bluetooth.
#include <SoftwareSerial.h>
SoftwareSerial mySerial(A2, A3); // RX, TX

//For serial receive.
const byte numChars = 11;
char receivedChars[numChars]; // an array to store the received data
String received; //The data as a string
boolean newData = false;

void setup() {
  pinMode(motorright, OUTPUT);                                                      
  pinMode(motorleft, OUTPUT);     
  pinMode(motorrightdir, OUTPUT);  
  pinMode(motorleftdir, OUTPUT);  

  pinMode(trigPinLeft, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinLeft, INPUT); // Sets the echoPin as an Input
  pinMode(trigPinFront, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinFront, INPUT); // Sets the echoPin as an Input
  pinMode(trigPinRight, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinRight, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); //normal printing
  mySerial.begin(57600); // Starts the bluetooth serial communication
}

void loop() 
{
  int left = ultrasonic(echoPinLeft,trigPinLeft);
  int front = ultrasonic(echoPinFront,trigPinFront);
  int right = ultrasonic(echoPinRight,trigPinRight);
  String message = "";
  message = message + "[" + left + "cm," + front + "cm," + right + "cm]\n";
  Serial.print(message);
  mySerial.print(message);
  delay(100); //wait for 1/10th of a second.
  
  recvWithEndMarker();
  processCommand();              
}

int ultrasonic(int echoPin, int trigPin) {
// Clears the trigPin
digitalWrite(trigPin, LOW);
delayMicroseconds(2);

// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);

// Reads the echoPin, returns the sound wave travel time in microseconds
duration = pulseIn(echoPin, HIGH);
// Calculating the distance
distance= duration*0.034/2;
return distance;
}

void processCommand() {
 if (newData == true)
 {
   String instruction = received.substring(0,5);
   String data = received.substring(6,10);
   if(instruction == "CONTF") goForward(data.toInt()); //Need to fix this one
   if(instruction == "MOVEF") forward(data.toInt()); 
   if(instruction == "MOVEB") backward(data.toInt());
   if(instruction == "TURNR") right(data.toInt());
   if(instruction == "TURNL") left(data.toInt());
   if(instruction == "FAST1") fast(data.toInt());
   if(instruction == "FAST2") fastest(data.toInt());
   if(instruction == "STOP1") stop();
 }
}

void recvWithEndMarker() 
{
 static byte ndx = 0;
 char endMarker = '\n';
 char rc;
 
 while (Serial.available() > 0 && newData == false) 
 {
  rc = Serial.read();

  if (rc != endMarker) 
  {
    receivedChars[ndx] = rc;
    ndx++;
    if (ndx >= numChars) 
    {
      ndx = numChars - 1;
    }
 }
 else 
  {
  receivedChars[ndx] = '\0'; // terminate the string
  received = String(receivedChars);
  ndx = 0;
  newData = true;
  }
 }
}

//direction is controlled by the digital pin 7 and 8.
// HIGH is backward, LOW is forward
// Pins 9 and 10 control speed.
// Length of time controls the distance

void forward(int time)
{
  digitalWrite(motorrightdir, LOW); 
  analogWrite(motorright,75); 
  digitalWrite(motorleftdir, LOW); 
  analogWrite(motorleft, 75); 
  delay(time);
  stop(); 
}
void backward(int time)
{
  digitalWrite(motorrightdir, HIGH); 
  analogWrite(motorright,75);
  digitalWrite(motorleftdir, HIGH); 
  analogWrite(motorleft, 75); 
  delay(time);
  stop();
}

void right(int time)
{
  digitalWrite(motorrightdir, HIGH);
  analogWrite(motorright, 50);
  digitalWrite(motorleftdir, LOW);
  analogWrite(motorleft, 100);
  delay(time);
  stop();
}
void left(int time)
{
  digitalWrite(motorrightdir, LOW);
  analogWrite(motorright, 100);
  digitalWrite(motorleftdir, HIGH);
  analogWrite(motorleft, 50);
  delay(time);
  stop();
}
void fast(int time)
{
  digitalWrite(motorrightdir, LOW); 
  analogWrite(motorright,100); 
  digitalWrite(motorleftdir, LOW); 
  analogWrite(motorleft, 100); 
  delay(time);
  stop(); 
}
void fastest(int time)
{
  digitalWrite(motorrightdir, LOW); 
  analogWrite(motorright,125); 
  digitalWrite(motorleftdir, LOW); 
  analogWrite(motorleft, 125); 
  delay(time);
  stop(); 
}
void goForward(int speed)
{
  digitalWrite(motorrightdir, LOW); 
  analogWrite(motorright,speed); 
  digitalWrite(motorleftdir, LOW); 
  analogWrite(motorleft, speed);
}
void stop()
{
  analogWrite(motorright, 0); 
  analogWrite(motorleft, 0); 
}

