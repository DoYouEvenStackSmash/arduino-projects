
// set pin numbers for switch, joystick axes, and LED:
 // input pin for the mouse pushButton
const int xAxis = A0;       // joystick X axis
const int yAxis = A1;       // joystick Y axis
const int ledPin = 5;       // Mouse control LED
double threshold = 1023.0 / 200.0;  // resting threshold
double center = 1023.0 / 2.0;     // resting position value

bool mouseIsActive = false;  // whether or not to control the mouse
int lastSwitchState = LOW;   // previous switch state
double maxn = 2.24;//sqrt(pow(2,2) + pow(0.001,2));

// Motor A connections
int enA = 9;
int in1 = 8;
int in2 = 7;
// Motor B connections
int enB = 3;
int in3 = 5;
int in4 = 4;

void setup() {
  pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  double LR[2] = {0.0,0.0};

  control_read(LR);
  double FB[2] = {0.0, 0.0};
  direction_get(LR,FB);
  directionControl(LR, FB);

}

void direction_get(double* LR, double* FB) {
  double adj = 255;
  double ctr = adj/2;
  double L = (LR[0]/maxn);
  double R = (LR[1]/maxn);
  FB[0] = L < 0 ? -1 : 1;
  if ( L == 0) {
    FB[0] = 0;
  }
  FB[1] = R < 0 ? -1 : 1;
  if ( R == 0) {
    FB[1] = 0;
  }

  LR[0] = pow(sgn(L),2) * abs(L * ctr * 2);
  LR[1] = pow(sgn(R),2) * abs(R * ctr * 2);

}

void directionControl(double* LR, double* FB) {
  
	// Turn on motor A & B
  double A = FB[0];
  double B = FB[1];
  int control_sig[2] = {0,0};
  int LF = HIGH;
  int LB = LOW;
  int RF = HIGH;
  int RB = LOW;
  if (A < 0) {
    LF = LOW;
    LB = HIGH;
  }
  if (B < 0) {
    RB = HIGH;
    RF = LOW;
  }
  if (B == 0) {
    RB = LOW;
    RF = LOW;
  }
  if (A == 0) {
    LF = LOW;
    LB = LOW;
  }
  control_sig[1] = B != 0 ? 1 : 0;

  if (A != 0) {
	  analogWrite(enA, LR[0]);
  }else
    analogWrite(enB, 0);

  digitalWrite(in1, LF);
  digitalWrite(in2, LB);
  if (B != 0) {
	  analogWrite(enB, LR[1]);
    // digitalWrite(in3, RF);
	  // digitalWrite(in4, RB);
  }else
      analogWrite(enB, 0);

  digitalWrite(in3, RF);
  digitalWrite(in4, RB);
	delay(100);
  // Turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void control_read(double* output_buf) {

  // read and scale the two axes:
  double x = readAxis(A0)/center;
  double y = readAxis(A1)/center;
  double x1 = -1;
  double x2 = 1;
  if (x == 0.0) {
    x1 = 0.0;
    x2 = 0.0;
  }
  if (y == 0.0 && x != 0.0) {
    y = 0.001;
  }
  // double maxn = sqrt(pow(1,2)*2);
  
  double L = sgn(y) * sqrt(pow(x - x1,2) + pow(y,2));
  double R = sgn(y) * sqrt(pow(x - x2,2) + pow(y,2));

  output_buf[0] = L;
  output_buf[1] = R;
}
/*
  reads an axis (0 or 1 for x or y) and scales the analog input range to a range
  from 0 to <range>
*/

int sgn(double a) {
  return a == 0 ? 0 : abs(a) / a;
}
void printer(double* LR) {
  double L = LR[0];
  double R = LR[1];
  Serial.print(String(L));
  Serial.print(" "); 
  Serial.print(String(R));
  Serial.println();
}

double readAxis(int thisAxis) {
  // read the analog input:
  double reading = analogRead(thisAxis);

  // map the reading from the analog input range to the output range:
  // reading = map(reading, 0, 1023, 0, range);

  // if the output reading is outside from the rest position threshold, use it:
  double distance = reading - center;

  if (abs(distance) < threshold) {
    distance = 0.0;
  }

  // return the distance for this axis:
  return distance;
}