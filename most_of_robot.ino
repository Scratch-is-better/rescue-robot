
//motor stuff-------------------------
const int in1 = 3;
const int in2 = 4;
const int in3 = 7;
const int in4 = 2;
const int en1 = 6;
const int en2 = 5;
int speed = 80;

// ultrasonic stuff-------------------


const int pingPin = 10;

// defines variables
unsigned long duration;
unsigned int distance;


unsigned long previousMillis2;
unsigned long distanceMillis;

bool Flag1 = false;
bool Flag2 = false;
//line array stuff-------------------

#include <QTRSensors.h>
#include <stdint.h>

QTRSensors qtr;

const uint8_t SensorCount = 4 ;

//colour sensor stuff----------------
#define S0 A0
#define S1 A1
#define S2 A2
#define S3 A3
#define sensorOut A4
#define OE A5

int green[] = {55, 61, 80};
int blue[] = {155, 145, 150};
//int purple[] = {,,};
//int yellow[] = {,,};


bool greenFlag[] = {false, false, false};
bool purpleFlag[] = {false, false, false};


int difference = 0;
int frequency = 0;


void setup()
{
  Serial.begin(9600);
  Serial.println("Hiasdijasidjads");
  // All motor control pins are outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);


  //lines
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    9, 11, 12, 8
  }, SensorCount);

  auto motorTime = 700u;

  rightForward();
  leftBack();

  auto changeDirectionTime = millis() + motorTime;
  while (millis() < changeDirectionTime) qtr.calibrate();

  leftForward();
  rightBack();

  changeDirectionTime = millis() + motorTime * 2;
  while (millis() < changeDirectionTime) qtr.calibrate();

  rightForward();
  leftBack();

  changeDirectionTime = millis() + motorTime;
  while (millis() < changeDirectionTime) qtr.calibrate();
  endLeft();
  endRight();


}




long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}

long getDistance()
{
  // establish variables for duration of the ping, and the distance result
  // in inches and centimeters:


  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH pulse
  // whose duration is the time (in microseconds) from the sending of the ping
  // to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  return pulseIn(pingPin, HIGH);

}


void ultrasonicScan() {

  auto scanTime = 3000u;

  auto startTime = millis();
  long lowest = 9999999;
  auto changeDirection = startTime + scanTime;
  long lowestTime;

  rightForward();
  leftBack();
  // spin anti-clockwise scanning for closest object
  Serial.println("start");

  while (millis() < changeDirection) {
    auto distance = getDistance();

    if (distance < lowest) {
      lowestTime = millis() - startTime;
      lowest = distance;
      Serial.println(lowest);
    }
  }
  Serial.println("second part");
  //return microsecondsToCentimeters(duration);

  Serial.println("third");
  // continue spin until we end up pointing at closest object

  changeDirection = millis() + scanTime - lowestTime;
  while (true) {
    auto distance = getDistance();
    Serial.println(distance);
    if (distance < lowest + 96) {
      break;
    }
  }

  Serial.println("four");
#if 0
  changeDirection = millis() + 100000;

  bool motorToggle = false;
  lowest = getDistance();

  leftForward();
  rightForward();


  while (millis() < changeDirection) {

    auto current = getDistance();


    /* if (current > lowest) {
       // if (motorToggle) {
       leftForward();
       endRight();
      }
      if (lowest > current) {
       rightForward();
       endLeft();
      }

      else {

      }
      motorToggle ^= 1;

      lowest = current;
    */
  }


#endif
  Serial.println("end");

  endRight();
  endLeft();
}


void rightForward()   //run right motor

{
  // turn on motor A
  analogWrite(en1, speed);

  digitalWrite(in1, HIGH);

  digitalWrite(in2, LOW);


}

void rightBack()   //run right motor

{
  // turn on motor A
  analogWrite(en1, speed);

  digitalWrite(in1, LOW);

  digitalWrite(in2, HIGH);


}

void endRight()
{
  digitalWrite(in1, LOW);

  digitalWrite(in2, LOW);
}


void leftBack()   //run left motor

{
  // turn on motor A
  analogWrite(en2, speed + 5);

  digitalWrite(in3, HIGH);

  digitalWrite(in4, LOW);


}

void leftForward()
{
  analogWrite(en2, speed + 5);

  digitalWrite(in3, LOW);

  digitalWrite(in4, HIGH);


}

void endLeft()
{
  digitalWrite(in3, LOW);

  digitalWrite(in4, LOW);
}


void motorControl(int32_t left, int32_t right)   //run both motor

{
  // turn on motor A
  int32_t speed = abs(left) + 75;
  if (speed > 255) speed = 255;

  analogWrite(en2, speed);

  if (left < 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }

  else if (left > 0) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

  speed = abs(right) + 70;
  if (speed > 255) speed = 255;

  analogWrite(en1, speed);

  if (right < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else if (right > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else {
    digitalWrite(in2, LOW);
    digitalWrite(in1, LOW);
  }
}



struct FourPointLSQQuadratic
{
  // a, b & c scaled by 1280
  int32_t a, b, c, sum;

  FourPointLSQQuadratic(uint16_t* x /* expect 4 value sensor array*/)
  {
    // ensure input data processed as 32 bit
    int32_t x0 = x[0], x1 = x[1], x2 = x[2], x3 = x[3];

    // calculate correlation scores
    int32_t y0 = 9 * (x0 + x3) + x1 + x2;
    sum = x0 + x1 + x2 + x3;

    // determine quadratic coefficients
    a = 80 * y0 - 400 * sum;
    b = -384 * x0 - 128 * x1 + 128 * x2 + 384 * x3;
    c = -100 * y0 + 820 * sum;
  }
};


void loop1()
{
  /*
    getDistance();

    qtr.read(sensorValues);

    // print the sensor values as numbers from 0 to 1023, where 0 means maximum

    // reflectance and 1023 means minimum reflectance

    for (uint8_t i = 0; i < SensorCount; i++)

    {

      Serial.print(sensorValues[i]);

      Serial.print('\t');

    }

    auto position = qtr.readLineBlack(sensorValues);
    Serial.println(position);
    auto error = 1500 - position;

    //en1 is right, en2 is left

    if (error < 0)
    {
    //   en1 = position //scale stuff/
    }

    if (error > 0)
    {
      //en2 = position //scale

    }


    //  range 0-3000
    //0 is sensor 0 and 3000 is sensor 3 with the line being directly under

    if (sensorValues[0] > 0)//readCalibrated(calibrationOff)))
    {
      leftForward();

    }

    if (sensorValues[1] > readCalibrated(calibrationOff))
    {
      rightForward();

    }

    else if(){
      leftForward();
      rightForward();
  */
}




void loop()
{



  //ultrasonicScan();
  //delay(99999999);




  uint16_t data[4];

  qtr.readCalibrated(data);



  for (int x : data)
  {
    Serial.print(x);
    Serial.print('\t');
  }



  if (data[0] + 50 < data[1] && data[3] + 50 < data[2]) {

    FourPointLSQQuadratic lsq(data);
    Serial.print(-lsq.b * 100 / (2 * lsq.a));
    auto center = -lsq.b * 100 / (2 * lsq.a);

    if (center < 0)
      motorControl(speed - center, speed);
    else

      motorControl(speed, speed - center);
  }
  else {
    if (data[0] > data[3])motorControl(0, 100);
    else motorControl(100, 0);

  }

}
