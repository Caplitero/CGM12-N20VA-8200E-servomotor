#define PhaseA 2

// Motors must be placed on digital PWM compatible pins 
#define Motor_1 5 
#define Motor_2 6


String inputString = "";
bool stringComplete = false;

double Kp = 2;  // Reduce by 20% (smoother response)
double Ki = 2;  // Slightly increase to maintain rejection
double Kd = 0.0001; // More damping
double integral =0;
double previous=0;

double dt=0;
double last_time = 0;

const float codor_full_rotation = 7.0;
const float gear_ratio = 99.3;
const float Interrupts = 2;
const float total_intervals = 1/(codor_full_rotation*gear_ratio*Interrupts);


volatile float rotations = 0;
volatile int times=0;
volatile int TIMES_DT=0;
int val = map(2,0,5,0,255);
float speed=0;

double a_const= 0.972727;
double b_const= 0.013637;

double x_prev=0;
double y_prev=0;

double position  = 0;




void phaseInterrupt()
{
  times= times+1;
  TIMES_DT +=1;
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
  Serial.begin(38400);
  pinMode(PhaseA, INPUT);
  pinMode(Motor_1,OUTPUT);
  pinMode(Motor_2,OUTPUT);
  last_time = millis();
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
    position += Val;
    
  }}
  else if(command =="G0")
  {
    times= 0;
    speed =0;
    rotations=0;
    position = 0;
    TIMES_DT = 0;
  }
  else if(command ="S")
  {
    int xIndex = params.indexOf('V');
    if (xIndex != -1)
    {int spaceAfterX = params.indexOf(' ', xIndex);
    String xStr = params.substring(xIndex + 1, (spaceAfterX == -1) ? params.length() : spaceAfterX);
    speed = xStr.toFloat();}

    int PID_INDEX_KP  = params.indexOf('P');
    int PID_INDEX_KI  = params.indexOf('I');
    int PID_INDEX_KD  = params.indexOf('D');
    if (PID_INDEX_KP != -1)
    {int spaceAfterX = params.indexOf(' ', PID_INDEX_KP);
    String xStr = params.substring(PID_INDEX_KP + 1, (spaceAfterX == -1) ? params.length() : spaceAfterX);
    Kp = xStr.toDouble();}
    if (PID_INDEX_KI != -1)
    {int spaceAfterX = params.indexOf(' ', PID_INDEX_KI);
    String xStr = params.substring(PID_INDEX_KI + 1, (spaceAfterX == -1) ? params.length() : spaceAfterX);
    Ki = xStr.toDouble();}
    if (PID_INDEX_KD != -1)
    {int spaceAfterX = params.indexOf(' ', PID_INDEX_KD);
    String xStr = params.substring(PID_INDEX_KD + 1, (spaceAfterX == -1) ? params.length() : spaceAfterX);
    Kd = xStr.toDouble();}
  }

  else {
    Serial.println("Unknown macro!");
    return;
  }

  
 
}

double clam(double _X,double min,double max)
{
  if(_X>max)return max;
  if(_X<min)return min;
  return _X;
}

double PID_Correction(double err)
{
  double proportional = err*Kp;
  integral += err *dt*Ki;
  integral = clam(integral,-255,255);
  double derivative =Kd*(err-previous)/dt;
  double output = proportional+integral+derivative;
  return output;
}

void loop() {

  double current_time = millis();
  dt = ((current_time-last_time)+0.0001)/1000.00;
  last_time = current_time;

  

  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  rotations = times*total_intervals;
  
  if(speed ==0)
    {integral=0;}
  
  double RPM = (TIMES_DT*total_intervals)/dt*60;
  double RPM_FILTERED = b_const*RPM+x_prev*b_const+y_prev*a_const;
  y_prev = RPM_FILTERED;
  x_prev = RPM;
  TIMES_DT= 0;
  

  double error = speed - RPM_FILTERED ;
  double output = PID_Correction(error);
  output = clam(output,0,255);
  


    analogWrite(Motor_1,output);
    analogWrite(Motor_2,0);
    Serial.print(speed,4);
    Serial.print(" ");
    Serial.print(RPM_FILTERED,4);
    Serial.print(" ");
    Serial.println(output,4);

  
    
  

}
