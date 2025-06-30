#define PhaseA 2
#define Motor_1 A0
#define Motor_2 A1


String inputString = "";
bool stringComplete = false;


const float codor_full_rotation = 7.0;
const float gear_ratio = 99.3;
const float Interrupts = 2;
const float total_intervals = 1/(codor_full_rotation*gear_ratio*Interrupts);


volatile float rotations = 0;
volatile int times=0;
volatile float RPM =0;
volatile unsigned long time= 0;
int val = 3.0/5*255;
int forward = 0;
int reverse = 0;

volatile int pulseCount=0;

void phaseInterrupt()
{
  times= times+1;
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
      break;
    } else {
      inputString += inChar;
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);
  pinMode(PhaseA, INPUT);
  pinMode(Motor_1,OUTPUT);
  pinMode(Motor_2,OUTPUT);

  analogWrite(Motor_2,0);
  attachInterrupt(digitalPinToInterrupt(PhaseA),  phaseInterrupt, CHANGE);
}


void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  int spaceIndex = cmd.indexOf(' ');
  String command;
  String params;

  if (spaceIndex == -1) {
    command = cmd;
    params = "";
  } else {
    command = cmd.substring(0, spaceIndex);
    params = cmd.substring(spaceIndex + 1);
  }

  float Val = NAN;

  if(command == "G1")
  {
  // Look for X and Y in the rest of the string
  int xIndex = params.indexOf('F');
  if (xIndex != -1) {
    
    int spaceAfterX = params.indexOf(' ', xIndex);
    String xStr = params.substring(xIndex + 1, (spaceAfterX == -1) ? params.length() : spaceAfterX);
    Val = xStr.toFloat();
    forward = Val;
    
  }}
  else {
    Serial.println("Unknown macro!");
    return;
  }

  
 
}


void loop() {

  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  rotations = times*total_intervals;
  

  //Serial.println(forward);
  Serial.println(rotations);
  if(rotations>forward )
  {
    analogWrite(Motor_1,0);
    analogWrite(Motor_2,0); 
  forward=0;
  //times = 0;
  }
  else if(rotations<forward)
    {
      analogWrite(Motor_1,val);
      analogWrite(Motor_2,0);
      
  }
  

}
