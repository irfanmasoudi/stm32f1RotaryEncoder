#define PPR   3000
#define ENA   PB8
#define ENB   PA0
#define PWMA  PA2
#define PWMB  PA3
#define DIRA  PB5
#define DIRB  PB9
#define A     0
#define B     1

void setup()
{
  Serial.begin(9600);
  pinMode(PA4, INPUT_ANALOG);
  pinMode(PA5, INPUT_ANALOG);
  
  pinMode(PWMA, PWM);
  pinMode(PWMB, PWM);

  pinMode(PA6, INPUT_PULLUP);  //channel A timer 3
  pinMode(PA7, INPUT_PULLUP);  //channel B
  pinMode(PB6, INPUT_PULLUP);  //channel A timer 4
  pinMode(PB7, INPUT_PULLUP);  //channel B

  Timer1.setMode(TIMER_CH1, TIMER_OUTPUTCOMPARE);
  Timer1.setPeriod(10000);                           // in microseconds
  Timer1.setCompare(TIMER_CH1, 1);                  // overflow might be small
  Timer1.attachInterrupt(TIMER_CH1, motorRPM);

  //configure timer as encoder
  Timer3.setMode(0, TIMER_ENCODER);                   //set mode, the channel is not used when in this mode.
  Timer3.pause();                                     //stop...
  Timer3.setPrescaleFactor(1);                        //normal for encoder to have the lowest or no prescaler.
  Timer3.setOverflow(65535);                          //use this to match the number of pulse per revolution of the encoder. Most industrial use 1024 single channel steps.
  Timer3.setCount(0);                                 //reset the counter.
  Timer3.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3);    //or TIMER_SMCR_SMS_ENCODER1 or TIMER_SMCR_SMS_ENCODER2. This uses both channels to count and ascertain direction.
  //timer.attachInterrupt(0, func);                   //channel doesn't mean much here either.
  Timer3.resume();                                    //start the encoder...

  Timer4.setMode(0, TIMER_ENCODER);                   //set mode, the channel is not used when in this mode.
  Timer4.pause();                                     //stop...
  Timer4.setPrescaleFactor(1);                        //normal for encoder to have the lowest or no prescaler.
  Timer4.setOverflow(65535);                          //use this to match the number of pulse per revolution of the encoder. Most industrial use 1024 single channel steps.
  Timer4.setCount(0);                                 //reset the counter.
  Timer4.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3);    //or TIMER_SMCR_SMS_ENCODER1 or TIMER_SMCR_SMS_ENCODER2. This uses both channels to count and ascertain direction.
  //timer.attachInterrupt(0, func);                   //channel doesn't mean much here either.
  Timer4.resume();                                    //start the encoder...

  pinMode(ENA, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(DIRB, OUTPUT);
}

float FreqA, RPMA, FreqB, RPMB, PID[2];
float samplingRotA, lastRotaryA, RotaryA;
float samplingRotB, lastRotaryB, RotaryB;
float right, left;
float Error[2], Derror[2], Ierror[2], LastError[2], SetRPM[2];
int tik;

void loop()
{
  int sensorValue = analogRead(PA5);
  int MotorSpeed = map(sensorValue, 0, 4095, -4000, 4000);

  SetRPM[A] = MotorSpeed;

  Serial.print("Potensio : ");
  Serial.print(MotorSpeed);

  Serial.print("            ||          ");
  Serial.print(" ");
  Serial.print("  ErrorA : ");
  Serial.print(Error[A]);
  Serial.print("  PID : ");
  Serial.print(PID[A]);
  Serial.print("  RPMA : ");
  Serial.print(RPMA);

  Serial.print("            ||          ");

  Serial.print("    ErrorB : ");
  Serial.print(Error[B]);
  Serial.print("    PID : ");
  Serial.print(PID[B]);
  Serial.print("    RPMB : ");
  Serial.println(RPMB);
}

void motorRPM()
{
  tik++;
  if (tik == 2)
  {
    RotaryA = Timer3.getCount();
    RotaryB = Timer4.getCount();

    if (RotaryA > 32767)
      RotaryA = (65535 - RotaryA);

    if (RotaryB > 32767)
      RotaryB = (65535 - RotaryB);

    samplingRotA  = RotaryA - lastRotaryA;
    FreqA         = samplingRotA * 50;
    RPMA          = FreqA * 60 / 2000;

    samplingRotB  = RotaryB - lastRotaryB;
    FreqB         = samplingRotB * 50;
    RPMB          = FreqB * 60 / 2000;

    if (RPMA < 0) RPMA *= -1;
    if (RPMB < 0) RPMB *= -1;

    if (SetRPM[A] >= 0)         Error[A] = SetRPM[A] - RPMA;
    else                        Error[A] = (-SetRPM[A]) - RPMA;

    if (SetRPM[B] >= 0)         Error[B] = SetRPM[B] - RPMB;
    else                        Error[B] = (-SetRPM[B]) - RPMB;

    Derror[A] = Error[A] - LastError[A];
    PID[A] += (5 * Error[A]) + (1 * Derror[A]);

    Derror[B] = Error[B] - LastError[B];
    PID[B] += (5 * Error[B]) + (1 * Derror[B]);

    lastRotaryA = RotaryA;
    LastError[A] = Error[A];

    lastRotaryB = RotaryB;
    LastError[B] = Error[B];

    if (PID[A] >= 65535)        PID[A] = 65535;
    if (PID[B] >= 65535)        PID[B] = 65535;

    motorA(PID[A]);
    motorB(PID[B]);

    tik = 0;
  }
}

void motorA(int kecepatan)
{
  int pwmPositif;
  int pwmNegatif;

  digitalWrite(ENA, HIGH);

  if (SetRPM[A] > 0)
  {
    pwmPositif = kecepatan;
    digitalWrite(DIRA, LOW);
    pwmWrite(PWMA, pwmPositif);
  }
  else if (SetRPM[A] < 0)
  {
    pwmNegatif = 65535 - kecepatan;
    digitalWrite(DIRA, HIGH);
    pwmWrite(PWMA, pwmNegatif);
  }
  else
  {
    digitalWrite(DIRA, LOW);
    pwmWrite(PWMA, 0);
  }
}

void motorB(int kecepatan)
{
  int pwmPositif;
  int pwmNegatif;

  digitalWrite(ENB, HIGH);

  if (SetRPM[B] > 0)
  {
    pwmPositif = kecepatan;
    digitalWrite(DIRB, LOW);
    pwmWrite(PWMB, pwmPositif);
  }
  else if (SetRPM[B] < 0)
  {
    pwmNegatif = 65535 - kecepatan;
    digitalWrite(DIRB, HIGH);
    pwmWrite(PWMB, pwmNegatif);
  }
  else
  {
    digitalWrite(DIRB, LOW);
    pwmWrite(PWMB, 0);
  }
}
