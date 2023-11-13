
//Incluimos las librerias necesarias

#include <I2Cdev.h>
#include <MPU6050.h>
#include <PID_v1.h>
#include <Wire.h>

//Se define los pines para el control 

#define enA 9
#define enB 10
#define inA_1 12
#define inA_2 11
#define inB_1 8
#define inB_2 7


//Valor del pwm que saca de inercia a los motores. y donde
// se almacenará la información de pwm que irá a cada motor
#define PWM_I 100
double PWM_A, PWM_B;

//definición de las variables crudas donde se almacenará la lectua
//de la mpu 6050. También se inicia el sensor de tipo MPU6050

int ax, ay, az;
int gx, gy, gz;
long tiempo_prev;
float dt;
double ang_x;
float ang_x_prev, ang_y_prev;
MPU6050 sensor;


//Definición de las variables de las constantes del controlador
//PID. y de los parametros necesarios para el constructor del objeto tipo´PID
//de la libreria PID_V1.h.También se inicia el objeto tipo PID.

double kp = 2.5; 
double ki = 0.8;
double kd = 600;
double Setpoint = 0;
double PID_OUT;

PID Controlador(&ang_x,&PID_OUT,&Setpoint,kp,ki,kd,DIRECT);


//Se define la función de reset por software:
void (*resetFunc)(void) = 0;

void setup() {
  // Se inicializa la comunicación serial, la comunicación I2C
  //el sensor mpu6050 y el modo de los pines del puente H
  Serial.begin(115200);
  Wire.begin();
  sensor.initialize();
  pinMode(enA,OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(inA_1,OUTPUT);
  pinMode(inA_2,OUTPUT);
  pinMode(inB_1,OUTPUT);
  pinMode(inB_2,OUTPUT);


  //Se ajusta el Setpoint a 0, que es el error que queremos
  //alcanzar

  Setpoint = 0;

  //se enciendo el PID
  Controlador.SetMode(AUTOMATIC);

}

void loop() {
  // Leer las aceleraciones y velocidades angulares, se calcula
  // el tiempo actual y la diferencia de tiempo para el calculo del
  // angulo mediante la rotación del giroescopio.

  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);


  dt = (millis()-tiempo_prev)/1000.0;
  tiempo_prev=millis();
  
  //Calcular los ángulos con acelerometro
  float accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
  
  //Calcular angulo de rotación con giroscopio y acelerometro
  // y complemento con el filtro complemento, se le suma +8 al angulo
  // como correción para ubicar el angulo 0 donde queremos que se establezca. 
  ang_x = (0.98*(ang_x_prev+(gx/131)*dt) + 0.02*accel_ang_x);
  ang_x_prev=ang_x;
  ang_x = ang_x+7,8;
  if (isnan(ang_x)) {
      resetFunc();
    }


  double ang_x_aux = ang_x;
  if(ang_x > 0){
      ang_x *= -1;
      }

  //Si el angulo es menor a un rango establecido por un error, le enviaremos
  // angulo en 0 para que disminuya la salida del pid y calculamos el pid total.
  // mediante la función Compute de la libreria. 
  
  float angle_error = Setpoint-ang_x_aux;

  if(-3 < angle_error && angle_error < 3){
      ang_x = 0;
      kp=0;
      kd =0;
      }


   Controlador.Compute();


  //Mapeamos los valores de pid para obtener el valor para enviarle
  // al motor. De 0 a 250 para el pid y la salida será de 0 a 150
  // debido a que ya tenemos un valor inicial que es de donde sale
  // los motores salen de la inercia.
  
  float PID_PWM = map(PID_OUT, 0, 250, 0, 150);

  //Analizamos mediante unos condicionales en que motor se necesita mas potencia. 
  // y los restamos o sumamos al valor donde el motor sale de la inercia. Lo hacemos
  // con la varibale auxiliar debido a que en esta está almacenada la información del
  // angulo. 

  if(ang_x_aux < 0){
      PWM_A = abs(PWM_I - PID_PWM);
      PWM_B = (PWM_I + PID_PWM);
    }
    else if(ang_x_aux > 0){
      PWM_A = PWM_I + PID_PWM;
      PWM_B = abs(PWM_I - PID_PWM);
    }

  //finalmente mandamos el valor analogico al puente H.

  analogWrite(enA, PWM_A);
  analogWrite(enB, PWM_B*0.9);

  digitalWrite(inA_1, HIGH);
  digitalWrite(inA_2, LOW);
  digitalWrite(inB_1, HIGH);
  digitalWrite(inB_2, LOW);


  //Se muestra el angulo en x y los parametros calculados durante una ejecución 
  // del loop

  Serial.print("Angulo en X:  ");
  Serial.print(ang_x);
  Serial.print("  Angulo en X aux:  ");
  Serial.print(ang_x_aux);   
  Serial.print("  PID OUTPUT: ");
  Serial.print(PID_OUT);
  Serial.print("  PID PWM: ");
  Serial.print(PID_PWM);
  Serial.print("  PWM_A: ");
  Serial.print(PWM_A);
  Serial.print("  PWM_B: ");
  Serial.println(PWM_B*0.9);

  delay(20);

}
