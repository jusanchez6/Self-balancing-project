#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 mpu;

int ax, ay, az;
int gx, gy, gz;
float time;

long tiempo_prev = 0;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

double Kp = 6;
double Ki = 0.005;
double Kd = 0;

int motorSpeedLeft  = 0;
int motorSpeedRight = 0;

double setpoint = 18;
double input, output;

int ENA = 9; //(Linea rosada)
int IN1 = 8;
int IN2 = 7;
int ENB = 10;
int IN3 = 12;
int IN4 = 11;

float error = 0;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT); // Debes declarar pid aquí

void (*resetFunc)(void)=0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(10000);
  mpu.initialize();
  mpu.setDMPEnabled(true);

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  mpu.setExternalFrameSync(0);
  pid.SetMode(AUTOMATIC);
  //pid.SetSampleTime(50);
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

  dt = (millis()-tiempo_prev)/1000.0;
  tiempo_prev=millis();
  
  //Calcular los ángulos con acelerometro
  float accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
  
  //Calcular angulo de rotación con giroscopio y acelerometro
  // y complemento con el filtro complemento, se le suma +15 al angulo
  // como correción para ubicar el angulo 0 donde queremos que se establezca. 

  ang_x = (0.98*(ang_x_prev+(gx/131)*dt) + 0.02*accel_ang_x);
  ang_x_prev=ang_x;

  input = map(ang_x, -33 , 38, 0, 42);

  /*Serial.print("Angulo de rotacion: ");
  Serial.println(ang_y);*/

  //Serial.print("Angulo cambiado: ");
  //Serial.println(input);

  
  error = setpoint - input; 


  /*Serial.print("Velocidad lado rosadito: ");
  Serial.println(motorSpeedLeft);
  Serial.print("Velocidad otro lado: ");
  Serial.println(motorSpeedRight);*/


  if (abs(error) <= 6){
     //Serial.print("Entré a la condición ");
      pid.SetTunings(6,0.01,0); 
  }
  else{
     pid.SetTunings(Kp,Ki,Kd); 
  }

  
  pid.Compute();

  double motorSpeed = map(output,-180,180,-170,170);

  //Serial.print("Salida controlador: ");
  //Serial.println(motorSpeed);



    motorSpeedLeft = (100 + motorSpeed)*1;
    motorSpeedRight = (100 - motorSpeed)*1;

  //Serial.print("Velocidad Arduino; ");
  //Serial.println(motorSpeedLeft);
  //Serial.print("Velocidad puente H: ");
  //Serial.println(motorSpeedRight);


  

   /*Serial.print("Salida: ");
  Serial.println(output);*/

  //Serial.print("Error: ");
  //Serial.println(error);

  analogWrite(ENA,motorSpeedRight );//(Linea rosada)
  analogWrite(ENB,motorSpeedLeft );
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4,LOW);



  Serial.print("Angulo en Y:  ");
  //Serial.print(ang_x);
  Serial.print(input);
  Serial.print("  Error:  ");
  Serial.print(error);   
  Serial.print("  PID OUTPUT: ");
  Serial.print(output);
  Serial.print("  PID PWM: ");
  Serial.print(motorSpeed);
  Serial.print("  PWM_A: ");
  Serial.print(motorSpeedLeft);
  Serial.print("  PWM_B: ");
  Serial.println(motorSpeedRight);



    
}