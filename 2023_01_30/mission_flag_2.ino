
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Servo.h>
#define RC_SERVO_PIN 8 // 나는 8번이야
#define NEURAL_ANGLE 82 //82 직선으로 간다   
#define LEFT_STEER_ANGLE -40  //서보의 방향에 따라 -또는 +가 될 수 있다.
#define RIGHT_STEER_ANGLE 40
int mission_flag = 0;
Servo Steeringservo;
int steer_angle = NEURAL_ANGLE;  //초기값은 NEURAL_ANGLE으로

void steering_control(int steer_angle)

{
  if(steer_angle <= LEFT_STEER_ANGLE+ NEURAL_ANGLE)  steer_angle  = LEFT_STEER_ANGLE+ NEURAL_ANGLE;
  if(steer_angle >= RIGHT_STEER_ANGLE+ NEURAL_ANGLE)  steer_angle =  RIGHT_STEER_ANGLE+ NEURAL_ANGLE;
  Steeringservo.write(steer_angle);  
}

////////////Sensor///////////////////////////////////////////////////////////////////////////
#include <NewPing.h>
#define SONAR_NUM 3
#define MAX_DISTANCE 250 //cm
#define DEBUG 1 //1하면 출력을 할 것이다.

#define MAX_DISTANCE 250
float R_Sonar_Distance = 0.0;
float R_Sonar_Distance_old = 0.0;
float R_Sonar_Error = 0.0;
float L_Sonar_Distance = 0.0;
float L_Sonar_Distance_old = 0.0;
float L_Sonar_Error = 0.0;
float F_Sonar_Distance = 0.0;
float F_Sonar_Distance_old = 0.0;
float F_Sonar_Error = 0.0;

float UlteasonicSeneorData[SONAR_NUM];

NewPing sonar[SONAR_NUM] ={
  NewPing(48, 50, MAX_DISTANCE), //앞
  NewPing(42, 44, MAX_DISTANCE),// 오른쪽
  NewPing(9, 10, MAX_DISTANCE) // 왼쪽
  };


void read_sonar_sensor(void)
{

   F_Sonar_Distance = sonar[0].ping_cm() * 10;
   R_Sonar_Distance = sonar[1].ping_cm() * 10;
   L_Sonar_Distance = sonar[2].ping_cm() * 10;

   if(F_Sonar_Distance == 0) F_Sonar_Distance = MAX_DISTANCE *10.0;
   if(R_Sonar_Distance == 0) R_Sonar_Distance = MAX_DISTANCE *10.0;
    if(L_Sonar_Distance == 0) L_Sonar_Distance = MAX_DISTANCE *10.0;
  
}

void serial_com(void)
{
  if(DEBUG != 1) 
  {
      return;
  }
  
  Serial.print("F_Sonar : ");
  Serial.print(F_Sonar_Distance);   //1인 경우에는 다음과 같이 sonar 값을 출략한다. 
  Serial.print(" ");
    
  Serial.print("R_Sonar : ");
  Serial.print(R_Sonar_Distance);
  Serial.print(" ");

  Serial.print("L_Sonar : ");
  Serial.println(L_Sonar_Distance);   //1인 경우에는 다음과 같이 sonar 값을 출략한다. 
  Serial.print(" ");
  
}

void update_sonar_old(void){ //초음파 센서의 옛날값 저장
    R_Sonar_Distance_old = R_Sonar_Distance;
    L_Sonar_Distance_old = L_Sonar_Distance;
    F_Sonar_Distance_old = F_Sonar_Distance;
  }

void update_sonar_error(void){ //초음파 센서의 옛날값과 현재값의 차이 저장
    R_Sonar_Error = R_Sonar_Distance - R_Sonar_Distance_old;
    L_Sonar_Error = L_Sonar_Distance - L_Sonar_Distance_old;
    F_Sonar_Error = F_Sonar_Distance - F_Sonar_Distance_old;
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define AOpin  A0     // Analog output - yellow
#define SIpin  11     // Start Integration - orange
#define CLKpin 12    // Clock - red

#define NPIXELS 128  // No. of pixels in array

byte Pixel[NPIXELS]; // Field for measured values <0-255>

int LineSensor_Data[NPIXELS];           // line sensor data(original)
int LineSensor_Data_Adaption[NPIXELS];  // line sensor data(modified)
int MAX_LineSensor_Data[NPIXELS];       // Max value of sensor
int MIN_LineSensor_Data[NPIXELS];       // Min value of sensor
int flag_line_adapation;          // flag to check line sensor adpation

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

void line_adaptation(void)
{
  int i;
  for (i = 0; i < NPIXELS; i++)
  {
    if (LineSensor_Data[i] >= MAX_LineSensor_Data[i])  MAX_LineSensor_Data[i] = LineSensor_Data[i];
    if (LineSensor_Data[i] <= MIN_LineSensor_Data[i])  MIN_LineSensor_Data[i] = LineSensor_Data[i];
  }
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


//  misson_flag=0;
/////////////////////////////이진화 할 함수/////////////////////////////////////////////////////////////////////////////////
#define threshold_value 60
#define camera_pixel_offset 5
void threshold(void)
{
  int i;
  
  for (i = 0; i < NPIXELS; i++)
  {
    if ((byte)Pixel[i] >= threshold_value) LineSensor_Data_Adaption[i] = 255;
    else LineSensor_Data_Adaption[i] = 0;
    //Serial.print(LineSensor_Data_Adaption[i]);
    //Serial.print(" ");
  }
  //Serial.println("  ");
}

/////////////////////////////무게중심 함수/////////////////////////////////////////////////////////////////////////////////
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
  steering_control((steer_data - NPIXELS/2 + camera_pixel_offset)*1.5 + NEURAL_ANGLE ); //내가 원하는 고정치에서 핸들을 많이 꺽겠다면 1.3 곱하기 값을 올려줘
  //Serial.println(steer_data - NPIXELS/2 + camera_pixel_offset);

}

//////////////////////////// motrol Control////////////////////////////////////////////////////////////////////////
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


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  Steeringservo.attach(RC_SERVO_PIN);  // 8번 핀을 서보로 사용하겠다.
  Steeringservo.write(NEURAL_ANGLE);  //초기는 90도 중립을, 0~90도로 움직인다.
  

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




///turn_right_90//////////////////////////////////////////////////////////////////////////////////////
int a=0;



void turn_right_90(void){
  
  int min_Distance = 0;
  
  if(F_Sonar_Distance <= 650)
  { 
    steering_control(172);
    min_Distance = F_Sonar_Distance;
    motor_control(0,150);
    delay(200);  //0.5초
    
    while(1){
      
      read_sonar_sensor();
      motor_control(0, 150);
      
      if(L_Sonar_Distance <= min_Distance-50)  //+5  30  40// -60
      {  motor_control(0,150);
         delay(80+a);
         mission_flag++;
         break;
      }
    }
  }
}
////////////////////////////////////////////////////////////////////////////////////////////
void read_line(){
  line_adaptation();
  read_line_sensor();
  threshold(); //255가 라인이 추출되는 곳이다.
  steering_by_camera();//Serial.print되는 곳
}
////////////////////////////////////////////////////////////////////////////////
int count;


void loop() {

  read_sonar_sensor();
  update_sonar_error();

  if(mission_flag == 0)
  {
    read_sonar_sensor();
    read_line();
    motor_control(0, 150);

    serial_com();

    if(L_Sonar_Distance + R_Sonar_Distance <=1200)
      {
        count++;
      }
      else
         count == 0;

      if(count >= 4){
        
        //motor_control(0, 80);
        mission_flag = 1;
      }

  }

  if(mission_flag == 1)
  {
    read_sonar_sensor();
    update_sonar_error();
    motor_control(0, 130);

   if(R_Sonar_Distance >= 1500)
   {
       motor_control(0, 130);
       turn_right_90();
   }
  }

  if(mission_flag == 2)
  {
    if(L_Sonar_Distance > R_Sonar_Distance)
      {
        steering_control(68); //61
        motor_control(0,120); //좌측으로 붙기
      }
      else if(L_Sonar_Distance < R_Sonar_Distance && L_Sonar_Distance!=0 &&  R_Sonar_Distance !=0)
      {
        steering_control(93);
        motor_control(0,120); //우측으로 붙기
      }
      else if(L_Sonar_Distance == R_Sonar_Distance)
      {
        //line_following();
        motor_control(0,150);

      }
  }
  
}
