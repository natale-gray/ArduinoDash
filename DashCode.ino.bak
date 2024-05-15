
//Define variables for each input signal 
//Define outputs: Dash LED and buzzer 
#define SASignal A0 
#define FLWS A1
#define RLWS A2
#define IS 3
#define LEDControl 6
// #define CANRX 13 
#define driveSignal 12
// #define CANTX 10
#define buzzer 9
#define LEDout 8

void setup() {
  // put your setup code here, to run once:
  //Set up LED 
  // define pinmodes
  pinMode(SASignal, INPUT);
  pinMode(FLWS, INPUT);
  pinMode(RLWS, INPUT);
  pinMode(IS, INPUT);
  pinMode(LEDControl, INPUT);
  // pinMode(CANRX, INPUT);
  pinMode(driveSignal, INPUT);
  // pinMode(CANTX, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(LEDout, OUTPUT);
}

//define initial states 
float wheelSpeedL;
float wheelSpeedR;
bool inertia;
bool controlLED;
bool drive;


void loop() {
  // put your main code here, to run repeatedly:
  //check to see if a signal is recieved from one of the inputs
  wheelSpeedL = float(analogRead(FLWS));
  wheelSpeedR = float(analogRead(RLWS));

  digitalWrite(LEDout,HIGH);

  if(digitalRead(IS) == HIGH) {
    inertia = true;
  }
  else{
    inertia = false;
  }

  if(digitalRead(LEDControl) == HIGH) {
    controlLED = true;
  }
  else{
    controlLED = false;
  }

  if(digitalRead(driveSignal) == HIGH) {
    drive = true;
  }
  else{
    drive = false;
  }

  //speaker code 
  if(digitalRead(buzzer == HIGH)) {
    tone(buzzer,1000);
    delay(1000);
    noTone(buzzer);
    delay(1000); 
  }
  else{
    noTone(buzzer);
  }
}

//methods to convert input signal into CAN frames 
