#include <I2Cdev.h>
#include <MPU6050.h>
#include <PID_v1.h>
#include <helper_3dmath.h>
#include "Wire.h"


//Definición de pines para el controlador L298N
#define enA 9         //PIN PMW del arduino NANO
#define enB 10        //PIN PMW del arduino NANO
#define inA_1 12
#define inA_2 11
#define inB_1 8
#define inB_2 7   
#define PERIOD 80
#define PWM_M 110
//Objeto de los motores

//Variables de controlador PID
double Setpoint=0, Input, Output, PWM_A, PWM_B;
//funciones para calcular los angulos por dos metodos (necesario para aplicar el filtro que evita el drif)
void Angulo_aceleracion(int Ax, int Ay, int Az, int ang_Ax, int ang_Ay);



//objeto tipo MPU6050
MPU6050 sensor;
///Constantes del PID
int Ax, Ay, Az;
float ang_Ax, ang_Ay;
float time;
float kp = 2.5;
float ki = 0.8;
float kd = 600;
float ang_Ay_sp =0;
float pid_p, pid_i, pid_d, pid_total;
float angle_error;
float angle_previous_error;
float left_speed_fact = 1;
float right_speed_fact = 0.7;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  sensor.initialize();
  time = millis();
  if(sensor.testConnection()) 
    Serial.println("Sensor iniciado correctamente.");
  else 
    Serial.println("Error al iniciar el sensor.");

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(inA_1, OUTPUT);
  pinMode(inA_2, OUTPUT);
  pinMode(inB_1, OUTPUT);
  pinMode(inB_2, OUTPUT);



  

}

void loop() {
   // Leer las aceleraciones 
  
  if(millis()>=time+PERIOD)
  {
    sensor.getAcceleration(&Ax, &Ay, &Az);
    Angulo_aceleracion(Ax, Ay, Az, &ang_Ax, &ang_Ay);

    float ang_Ay_aux = ang_Ay;

    if(ang_Ay > 0){
      ang_Ay *= -1;
      }


    angle_error = ang_Ay_sp - ang_Ay;
    pid_p = kp * angle_error;
    float dist_difference = angle_error - angle_previous_error;
    pid_d = kd*((dist_difference)/PERIOD);

    if(-3 < angle_error && angle_error < 3) {
      pid_i = pid_i + (ki * angle_error);
    } else {
      pid_i = 0;
    }

    pid_total = abs(pid_p + pid_i + pid_d);

    /*if(pid_total < 10)
      pid_total = 10;
    else if(pid_total > 50)
      pid_total = 50;
*/
    
    float pid_total_pwm = map(pid_total, 10, 50, 0, 150);

    Serial. print("Error del angulo: ");
    Serial.print(angle_error);
    Serial.print("   PID: ");
    Serial.print(pid_total);
    Serial.print("   Angulo: ");
    Serial.print(ang_Ay_aux);
    

    angle_previous_error = angle_error;


    digitalWrite(inA_1, HIGH);
    digitalWrite(inA_2, LOW);
    digitalWrite(inB_1, HIGH);
    digitalWrite(inB_2, LOW);

    if(ang_Ay_aux < 0){
      PWM_A = abs(PWM_M - pid_total_pwm);
      PWM_B = PWM_M + pid_total_pwm;
    }
    else if(ang_Ay_aux > 0){
      PWM_A = PWM_M + pid_total_pwm;
      PWM_B = abs(PWM_M - pid_total_pwm);
    }

    

    

    analogWrite(enA, PWM_A*1.1);
    analogWrite(enB, PWM_B);



        
    Serial.print("     PWM_A: ");
    Serial.print(PWM_A*1.1);
    Serial.print("     PWM_B: ");
    Serial.println(PWM_B);



  }






 /* pidControl.Compute();

  Serial.print("PID OUTPUT: ");
  Serial.print(Output);
  Serial.print("   Angulo: ");
  Serial.println(ang_Ay);

  delay(100); */

 
}



void Angulo_aceleracion(int Ax, int Ay, int Az, int ang_Ax, int ang_Ay){
  double *ap_x = ang_Ax;
  double *ap_y = ang_Ay;
  
  *ap_x = atan(Ax/sqrt(pow(Ay, 2) + pow(Az, 2)))*(180.0/3.14);
  *ap_y = atan(Ay/sqrt(pow(Ax, 2) + pow(Az, 2)))*(180.0/3.14);

}



