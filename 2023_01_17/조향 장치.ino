#define AOpin  A0     // Analog output - yellow
#define SIpin  11     // Start Integration - orange
#define CLKpin 12    // Clock - red
// Vcc - brown
// GND - black

#define NPIXELS 128  // No. of pixels in array

byte Pixel[NPIXELS]; // Field for measured values <0-255>

int LineSensor_Data[NPIXELS];           // line sensor data(original)
int LineSensor_Data_Adaption[NPIXELS];  // line sensor data(modified)
int MAX_LineSensor_Data[NPIXELS];       // Max value of sensor
int MIN_LineSensor_Data[NPIXELS];       // Min value of sensor
int flag_line_adapation;          // flag to check line sensor adpation

#define FASTADC 1
// defines for setting and clearing register bits
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

void setup() {
  // put your setup code here, to run once:
  int i;
  for (i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data[i] = 0;
    LineSensor_Data_Adaption[i] = 0;
    MAX_LineSensor_Data[i] = 1023; //0;
    MIN_LineSensor_Data[i] = 0; //1023;
  }

  pinMode(SIpin, OUTPUT);
  pinMode(CLKpin, OUTPUT);
  pinMode (AOpin, INPUT);

  digitalWrite(SIpin, LOW);   // IDLE state
  digitalWrite(CLKpin, LOW);  // IDLE state

#if FASTADC
  // set prescale to 16
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
#endif

  flag_line_adapation = 0;

  Serial.begin(115200);

}
void line_adaptation(void)
{
  int i;
  for (i = 0; i < NPIXELS; i++)
  {
    if (LineSensor_Data[i] >= MAX_LineSensor_Data[i])  MAX_LineSensor_Data[i] = LineSensor_Data[i];
    if (LineSensor_Data[i] <= MIN_LineSensor_Data[i])  MIN_LineSensor_Data[i] = LineSensor_Data[i];
  }

  /*for (i = 0; i < NPIXELS; i++)
    {
    Serial.print("[");
    Serial.print(i);
    Serial.print("]");
    Serial.print("   : ");
    Serial.print(MAX_LineSensor_Data[i]);
    Serial.print(" | ");
    Serial.print(MIN_LineSensor_Data[i]);
    Serial.println(" ");
    }*/
}
void read_line_sensor(void)
{
  int i;

  delayMicroseconds (1);  /* Integration time in microseconds */
  delay(10);              /* Integration time in miliseconds  */


  digitalWrite (CLKpin, LOW);
  digitalWrite (SIpin, HIGH);
  digitalWrite (CLKpin, HIGH);
  digitalWrite (SIpin, LOW);

  delayMicroseconds (1);

  for (i = 0; i < NPIXELS; i++) {
    Pixel[i] = analogRead (AOpin) / 4 ; // 8-bit is enough
    digitalWrite (CLKpin, LOW);
    delayMicroseconds (1);
    digitalWrite (CLKpin, HIGH);
  }

  for (i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data_Adaption[i] = map(Pixel[i], MIN_LineSensor_Data[i], MAX_LineSensor_Data[i], 0, 256);
  }

}

//이진화 할 함수
#define threshold_value 180
#define camera_pixel_offset 5;
void threshold(void)
{
  int i;
  
  for (i = 0; i < NPIXELS; i++)
  {
    if ((byte)Pixel[i] >= threshold_value) LineSensor_Data_Adaption[i] = 255;
    else LineSensor_Data_Adaption[i] = 0;
  }
  
}
//무게중심 함수
void steering_by_camera(void)
{
  int i;
  long sum = 0;
  long x_sum = 0;
  int steer_data = 0;
   
  for (i = 0; i < NPIXELS; i++)
  {
    sum += LineSensor_Data_Adaption[i];
    x_sum += LineSensor_Data_Adaption[i] * i;
  }
  steer_data = x_sum/sum;
  steering_control(steer_data); //내가 원하는 고정치에서 핸들을 많이 꺽겠다면 1.3 곱하기 값을 올려줘
  Serial.println(steer_data) - NPIXELS/2 + camera_pixel_offset;

}

//////////////////////////// motrol Control///////////////////////////////////////
#define MOTOR_DIR 4
#define MOTOR_PWM 5

int Motor_Speed = 0;
#define NORMAL_SPEED 100
#define SLOW_SPEED 70

void motor_control(int direction, int speed)
{
  digitalWrite(MOTOR_DIR, 1-direction); //digitalWrite(NOTOR_DIR, 0);
  analogWrite(MOTOR_PWM, speed);
}

//////////////////// steering Servo Control///////////////////////////////////////
#include <Servo.h>
#define RC_SERVO_PIN 8         // 나는 8번이야
#define NEURAL_ANGLE 130 //82 직선으로 간다   
#define LEFT_STEER_ANGLE -40  //서보의 방향에 따라 -또는 +가 될 수 있다.
#define RIGHT_STEER_ANGLE 40

Servo Steeringservo;
int Steering_Angle = NEURAL_ANGLE;  //초기값은 NEURAL_ANGLE으로

void steering_control(int steer_angle)

{
  if(steer_angle <= LEFT_STEER_ANGLE )  steer_angle  = LEFT_STEER_ANGLE;
  if(steer_angle >= RIGHT_STEER_ANGLE)  steer_angle =  RIGHT_STEER_ANGLE;
  Steeringservo.write(NEURAL_ANGLE + steer_angle);  
}



////////////////////////////////////////////////////////////////////////////////

void loop() {
  // put your main code here, to run repeatedly:

  int i;

  read_line_sensor();
  motor_control(0, 80);

  /*
  if (digitalRead(CLKpin) == HIGH)
  {
  //  line_adaptation();
    flag_line_adapation = 1;
  }
  */

  threshold();  //255가 라인이 추출되는 곳이다. 
  steering_by_camera();
  for (i = 0; i < NPIXELS; i++)
  {
    //if (digitalRead(CLKpin) == LOW)    Serial.print(LineSensor_Data_Adaption[i]); // Serial.print(LineSensor_Data[i] );
    //else                               Serial.print ((byte)Pixel[i] + 1);
    //Serial.print ((byte)Pixel[i] + 1);
    //Serial.print(LineSensor_Data_Adaption[i]);
    //Serial.print(" ");
  }

  //Serial.println("  ");
  delay(100);
}
