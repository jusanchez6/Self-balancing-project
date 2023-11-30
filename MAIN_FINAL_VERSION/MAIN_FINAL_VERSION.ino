#include <PID_v1.h>
#include "MPU6050.h"
#include "Wire.h"

MPU6050 mpu;

int ax, ay, az;
int gx, gy, gz;
float time;

long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

double Kp = 6.5; //6.25
double Ki = 0.1; //0.0005
double Kd = 2;  //3


double setpoint = 16;
double input, output;

int ENA = 9; //(Linea rosada)
int IN1 = 12;
int IN2 = 11;
int ENB = 10;
int IN3 = 8;
int IN4 = 7;

float error = 0;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT); // Debes declarar pid aquí

void (*resetFunc)(void)=0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  mpu.setDMPEnabled(true);

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(50);
  pid.SetOutputLimits(-255, 255);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {

  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);
  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  float accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
  
  //Calcular angulo de rotación con giroscopio y acelerometro
  // y complemento con el filtro complemento, se le suma +15 al angulo
  // como correción para ubicar el angulo 0 donde queremos que se establezca. 

  ang_x = (0.98*(ang_x_prev+(gx/131)*dt) + 0.02*accel_ang_x);
  ang_x_prev=ang_x;

  input = map(ang_x, -16 , 16, 0, 40);

  

  error = setpoint - input; 

  pid.Compute();

  if (abs(error) <= 1.5){ 
      pid.SetTunings( 3,5,0);}
  else if((abs(error) > 1.5)&(abs(error) < 3.5)){
    pid.SetTunings( 4,0.0000004,0.02);}   
  else if((abs(error) > 3.5)&(abs(error) < 5.5)){
    pid.SetTunings( 5,0.000008,0.03);}  
  else if((abs(error) > 5.5)&(abs(error) < 8.5)){
    pid.SetTunings( 6, Ki,1);}  
  else{
    pid.SetTunings(Kp,Ki,Kd);
    }

  double motorSpeed = map(output,-255,255,-100,100);
  
  int motorSpeedLeft = (100 - motorSpeed)*1;
  int motorSpeedRight = (100 + motorSpeed)*1.3;

 

  analogWrite(ENA, motorSpeedLeft);//(Linea rosada)
  analogWrite(ENB, motorSpeedRight); 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);


  Serial.print("ANGULO: ");
  Serial.print(input);
  Serial.print("  error: ");
  Serial.print(error);
  Serial.print("  PDI OUT: ");
  Serial.print(output);
  Serial.print("  motorSpeed: ");
  Serial.println(motorSpeed);

}
