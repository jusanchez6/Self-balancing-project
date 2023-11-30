
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
//y un factor qeue ayuda a regular los motores.

int PWM_I = 110;
float factor_motorA=0.9;
float factor_motorB =0.7;
double PWM_A, PWM_B;

//definición de las variables crudas donde se almacenará la lectua
//de la mpu 6050. También se inicia el sensor de tipo MPU6050

int ax, ay, az;
int gx, gy, gz;
long tiempo_prev;
float dt;
double ang_x, ang_x1;
float ang_x_prev, ang_y_prev;
MPU6050 sensor;


//Definición de las variables de las constantes del controlador
//PID. y de los parametros necesarios para el constructor del objeto tipo´PID
//de la libreria PID_V1.h.También se inicia el objeto tipo PID. Se declaran dos tipos
//de variables de las constantes PID una agresiva para cuando el error sea muy grande
//y un arreglo conservativo para cuando el error sea mucho más pequeño.


double aggKp=50, aggKi=0.008, aggKd=25;
double consKp=50, consKi=0.08, consKd=1;
double Setpoint = 22;
double PID_OUT;

PID Controlador(&ang_x1, &PID_OUT, &Setpoint, aggKp, aggKi, aggKd, DIRECT);

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

  sensor.setXGyroOffset(220);
  sensor.setYGyroOffset(76);
  sensor.setZGyroOffset(-85);
  sensor.setZAccelOffset(1688);
  
  Controlador.SetMode(AUTOMATIC);
  Controlador.SetOutputLimits(0, 255);  


}

void loop() {
  // Leer las aceleraciones y velocidades angulares, se calcula
  // el tiempo actual y la diferencia de tiempo para el calculo del
  // angulo mediante la rotación del giroescopio.

  digitalWrite(inA_1, HIGH);
  digitalWrite(inA_2, LOW);
  digitalWrite(inB_1, HIGH);
  digitalWrite(inB_2, LOW);

  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);


  dt = (millis()-tiempo_prev)/1000.0;
  tiempo_prev=millis();
  
  //Calcular los ángulos con acelerometro
  float accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
  
  //Calcular angulo de rotación con giroscopio y acelerometro
  // y complemento con el filtro complemento, se le suma +15 al angulo
  // como correción para ubicar el angulo 0 donde queremos que se establezca. 

  ang_x = (0.98*(ang_x_prev+(gx/131)*dt) + 0.02*accel_ang_x);
  ang_x_prev=ang_x;

  ang_x1 = map(ang_x,-16,16,0,40);
  //ang_x1 = ang_x;

  if (isnan(ang_x)) {
      resetFunc();
    }


  //Si el angulo es menor a un rango establecido por un error, le enviaremos
  // angulo en 0 para que disminuya la salida del pid y calculamos el pid total.
  // mediante la función Compute de la libreria. 
  
 float angle_error = Setpoint-ang_x1;

  if(-6 < angle_error && angle_error < 7){
      //Cambiamos los valores de las constantes pid para realizar un control
      //mas conservativo, o mas suave. 
      Controlador.SetTunings(consKp,consKi,consKd);

      }

  //Si no estamos en el rango establecido por -10 y 10 entonces queremos volver a los valores de
  // las constantes para un control mas agresivo.

  else{
    Controlador.SetTunings(aggKp,aggKi,aggKd);
  }
 
  Controlador.Compute();
  //Mapeamos los valores de pid para obtener el valor para enviarle
  // al motor. De 0 a 250 para el pid y la salida será de 0 a 150
  // debido a que ya tenemos un valor inicial que es de donde sale
  // los motores salen de la inercia.
  /*if(PID_OUT < 10){
    PID_OUT = 10;
  }*/

  float PID_PWM = map(PID_OUT, -255, 255, 0, 150);

  //Analizamos mediante unos condicionales en que motor se necesita mas potencia. 
  // y los restamos o sumamos al valor donde el motor sale de la inercia. Lo hacemos
  // con la varibale auxiliar debido a que en esta está almacenada la información del
  // angulo. 

  if(angle_error < -4){
      PWM_B = abs(PWM_I - PID_PWM);
      PWM_A = (PWM_I + PID_PWM);
      factor_motorA = 0.7;
      factor_motorB = 0.6;
      
    }
    else if(angle_error > 7){
      PWM_B = PWM_I + PID_PWM;
      PWM_A = abs(PWM_I - PID_PWM);
       factor_motorA = 0.7;
      factor_motorB = 0.6;

    }
    else if(-4 < angle_error && angle_error < 6){
      PWM_B = PID_PWM;
      PWM_A = PID_PWM;
      factor_motorA = 1.1;
      factor_motorB = 0.9;
      
    }

  analogWrite(enA, PWM_A*factor_motorA);
  analogWrite(enB, PWM_B*factor_motorB);


  //Se muestra el angulo en x y los parametros calculados durante una ejecución 
  // del loop

  Serial.print("Angulo en X:  ");
  Serial.print(ang_x1);
  Serial.print("  error:  ");
  Serial.print(angle_error);   
  Serial.print("  PID OUTPUT: ");
  Serial.print(PID_OUT);
  Serial.print("  PID PWM: ");
  Serial.print(PID_PWM);
  Serial.print("  PWM_A: ");
  Serial.print(PWM_A*factor_motorA);
  Serial.print("  PWM_B: ");
  Serial.println(PWM_B*factor_motorB);



}
