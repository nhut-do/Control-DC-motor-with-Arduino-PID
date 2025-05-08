#define ENCODER_A 2
#define ENCODER_B 3
#define SDL_CONTROL_R 8
#define SDL_CONTROL_L 7
#define PWM_R 6
#define PWM_L 5

volatile long demxung;
double preTime, preErro, erro;
double currPos, prePos, iTerm, setPoint;
float kp = 3.3, ki = 0.000096, kd = 0.0022;
double outMin = -150, outMax = 160, outPut;

void encoder()
{
  if (digitalRead(ENCODER_B) == HIGH) {
    demxung++;
  }  
  else {
    demxung--;
  }
}
void pwmPulse(double pwm) {
  if (pwm > 0 ) {
    analogWrite(PWM_L, pwm);
    analogWrite(PWM_R, 0);
  }
  else {
    analogWrite(PWM_L, 0);
    analogWrite(PWM_R, abs(pwm));
  }
}
void PID_Calculate() {
  if (millis() > preTime + 0.1) {              // sampletime 0.1ms
    erro = setPoint - currPos;
    //Serial.println(erro);
    // ki
    iTerm += (ki * erro * 0.0001);
    if (iTerm > outMax) iTerm= outMax;
    else if (iTerm < outMin) iTerm= outMin;
    //kd
    double dTerm = (currPos - prePos) * 0.0001;
    //Serial.println(dTerm);

    outPut = erro*kp + iTerm + kd*dTerm;
    if(outPut > outMax) outPut = outMax;
    else if(outPut < outMin) outPut = outMin;

    prePos = currPos;
    preTime = millis();
  }
}

void setup() {
  TCCR0B = TCCR0B & 0b11111000 | 1; // set frequency ~31kHz
  attachInterrupt(digitalPinToInterrupt(2), encoder, RISING);
  Serial.begin(115200);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(A0, INPUT_PULLUP);
  pinMode(PWM_R, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  digitalWrite(SDL_CONTROL_R, HIGH);
  digitalWrite(SDL_CONTROL_L, HIGH);
}

void loop() {
  currPos = (demxung * 360) / 2000;
  setPoint = 360;
  PID_Calculate();
  pwmPulse(outPut);
  //Serial.print(setPoint);
  //Serial.print(" ");
//  Serial.print("$");
//  Serial.print(i);
//  Serial.print(" ");
//  Serial.print(x);
//  Serial.print(";");
//  Serial.print("$");
//  Serial.print(currPos);
//  Serial.print(" ");
//  Serial.println(outPut);
//  Serial.println(";");
// Serial.print("$");
 Serial.print(erro);
 Serial.println();
// Serial.print(";");
}
