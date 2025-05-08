/*working variables*/
#define ENCODER_A 2
#define ENCODER_B 3
#define SD_CONTROL_R 8
#define SD_CONTROL_L 7
#define PWM_R 6
#define PWM_L 5

volatile long demxung;
double preTime, erroSum, preErro, erro;
double currSpeed, preSpeed, iTerm, setPoint;
double kp = 0.003, ki = 0.000009, kd = 0.8;
double outMin = -130, outMax = 160, outPut;

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
  if ((millis() > preTime + 5)) {               // sampletime 5ms
    currSpeed = demxung * 27;                    // (v/p) 
    erro = setPoint - currSpeed;
    
    // ki
    erroSum += (erro*ki)*0.005;
    // kd
    double dTerm = (currSpeed - preSpeed)*0.005;
    
    double tempOutut = erro*kp + erroSum + kd*dTerm;
    if(tempOutut > outMax) tempOutut = outMax;
    else if(tempOutut < outMin) tempOutut = outMin;

    outPut += tempOutut;
    if(outPut > outMax) outPut = outMax;
    else if(outPut < outMin) outPut = outMin;
    preSpeed = currSpeed;
    preTime = millis();
    demxung = 0;
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
  digitalWrite(SD_CONTROL_R, HIGH);
  digitalWrite(SD_CONTROL_L, HIGH);
}

void loop() {
  //setPoint = analogRead(A0)*3;
  setPoint = 600;
  //currSpeed = analogRead(A0);
  PID_Calculate();
  pwmPulse(outPut);
  

  //Serial.print("$");
  Serial.print(outPut);
  Serial.print(" ");
  Serial.println(currSpeed);
  //Serial.println(";");
}