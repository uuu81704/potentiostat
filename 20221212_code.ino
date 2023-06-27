#include <Wire.h>
#define pwmPin 9
#define current_sensor A0
#define voltage_sensor A1
#define PWM_sensor A2

unsigned long timer = 0; // for time delay.
unsigned long loopTime = 200000;   // 1e6 [microseconds] = 1000 ms
double set_current = 0.0;
double Read_current = 0.0;
double Read_voltage = 0.0;
boolean startFlag = 0;
int PWM_value = 0; //0~255

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(current_sensor, INPUT);
  analogWrite(pwmPin, 0); // zero current
  Serial.begin(115200);
  timer = micros();
  Serial.setTimeout(1);
}

void loop() {
  while(Serial.available()){
    set_current = Serial.readString().toFloat();
    delay(1);
    Serial.println(set_current);
    startFlag = 1;
	PWM_value = set_current*100 + 150;
  }
  
  // =============================================== //
  if (startFlag){
    timeSync(loopTime); 
	  analogWrite(pwmPin, PWM_value);
	
	// analogRead
	 int repeat_times = 100;
    for (int i = 0; i < repeat_times; i++){
      Read_current += analogRead(current_sensor)* (5 / 1023.00)/ 1; //resistor_value
      Read_voltage += analogRead(voltage_sensor)* (5 / 1023.00);
	 }
    Read_current = Read_current/repeat_times;
    Read_voltage = Read_voltage/repeat_times;
	
    sendToPC2(&Read_current, &Read_voltage);
	
	if (Read_current < set_current) {
     PWM_value++;
    }
    else if(Read_current == set_current)
    {
    //Do Nothing
    loopTime = 2000000;
    }
    else {
     PWM_value--;
		}
  }
}


void timeSync(unsigned long loopTime)
{
  unsigned long currTime = micros();
  long timeToDelay = loopTime - (currTime - timer);
  if (timeToDelay > 5000)
  {
    delay(timeToDelay / 1000);
    delayMicroseconds(timeToDelay % 1000);
  }
  else if (timeToDelay > 0)
  {
    delayMicroseconds(timeToDelay);
  }
  else
  {
  // timeToDelay is negative so we start immediately
  }
  timer = currTime + timeToDelay;
}


void sendToPC2(double* data1, double* data2)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte buf[8] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3]};
  Serial.write(buf, 8);
}
