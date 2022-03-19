#include <TimerOne.h>
//  Pines
#define UMBRAL 500 //si valor > UMBRAL ---> BLANCO
#define VEL_NORMAL 150
#define TRUE 1
#define FALSE 0
#define TM 25000
#define INTMAX 500
#define VMAX 250
#define KP 10

volatile int flag = 0;

//MOTOR A, derecho

//1 izq, 6 derec
int CNY1 = A0;
int CNY2 = A1;
int CNY3 = A2;
int CNY4 = A3;
int CNY5 = A4;
int CNY6 = A5;

int vec_cny70[6]; //Tabla que almacena los valores de los sensores

const int pinENA = 6;  //Controla velocidad de motor A
const int pinENB = 11;  //Controla velocidad de motor B
//  Pines para el movimiento de los motores
//  Control motor A
const int pinIN1 = 7;
const int pinIN2 = 8;
//  Control motor B
const int pinIN3 = 9;
const int pinIN4 = 10;

int vel = VEL_NORMAL;

const int pinMotorA[3] = {pinENA, pinIN1, pinIN2};
const int pinMotorB[3] = {pinENB, pinIN3, pinIN4};

int valor1 = 0;
int valor2 = 0;
int valor3 = 0;
int valor4 = 0;
int valor5 = 0;
int valor6 = 0;

int error = 0;

int w = 0;

void setup() {

  Timer1.initialize(TM);
  Timer1.attachInterrupt(func_ISR);
  
  Serial.begin(9600);

  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);
  pinMode(pinIN3, OUTPUT);
  pinMode(pinIN4, OUTPUT);
  pinMode(pinENA, OUTPUT);
  pinMode(pinENB, OUTPUT);
}

void loop() {

  int Vl = 0;
  int Vr = 0;

  int ang = get_ang();
  
  leerSensores();

if(flag == 1){
  
    w = pid(ang);

  Serial.print("ANGULO: ");
  Serial.println(String(ang));
  Serial.println();


    if(ang > 0){ // ang > 0 -> w < 0
    
    Vl = vel - w;
    Vr = vel + w;
  
    if(Vl > 250){
      Vl = VMAX;
    }
    else if(Vr < 0){
      Vr = 0;
    }
    }
    if(ang < 0){ // ang < 0 -> w > 0

    Vl = vel - w;
    Vr = vel + w;
  
    if(Vr > 255){
      Vr = VMAX;
    }
    else if(Vl < 0){
      Vl = 0;
    }
    }

    Serial.print("w: ");
    Serial.println(String(w));
    Serial.print("Vl: ");
    Serial.println(String(Vl));
    Serial.print("Vr: ");
    Serial.println(String(Vr));

    set_vel_motor(Vl, pinMotorA);
    set_vel_motor(Vr, pinMotorB);
  }
}

void set_vel_motor(int velocidad, const int pinMotor[3]){

    digitalWrite(pinMotor[1], HIGH);
    digitalWrite(pinMotor[2], LOW);
    analogWrite(pinMotor[0], velocidad);
  
}

int pid(int ang){
  
  static int sum_errores = 0;
  static int error_anterior = 0;
  float Kp = KP;
  int Kd = 0; //300
  int Ki = 0;
  int d_error = 0;
  int omega = 0;

  error_anterior = error;
  error = -ang;
  
  d_error = (error - error_anterior);  // Calculamos la derivada despreciando el tiempo de muestreo

  sum_errores += error;

  if(sum_errores >= INTMAX){
    sum_errores = INTMAX;
  }
  if(sum_errores <= -INTMAX){
    sum_errores = -INTMAX;
  }
  
  
  omega = Kp*error + Kd*d_error + Ki*sum_errores;

  if(omega >= 1000){
    omega = 1000;
  }
  if(omega <= -1000){
    omega = -1000;
  }

  flag = 0; 
  return omega;
}
  
void leerSensores(){

  vec_cny70[0] = analogRead(CNY1);  //Sensor 1
  vec_cny70[1] = analogRead(CNY2);  //Sensor 2
  vec_cny70[2] = analogRead(CNY3);  //Sensor 3
  vec_cny70[3] = analogRead(CNY4);  //Sensor 4
  vec_cny70[4] = analogRead(CNY5);  //Sensor 5
  vec_cny70[5] = analogRead(CNY6);  //Sensor 6

  for(int i = 0; i < 6; i++){
    Serial.print("SENSOR " + String(i + 1) + ": ");
    Serial.println(String(vec_cny70[i]));

  }
  
}


void func_ISR(){

   flag = 1;
}

int get_ang(){

  static int ang_ant = 0;
  int cny_70[6];
  int ang = 0;
  int n = 0;
  int aux = 0;

  for(int i = 0; i < 6; i++){
    if(analogRead(i) < UMBRAL){
      cny_70[i] = 1;  //  NEGRO
    }
    else{
      cny_70[i] = 0;  //  BLANCO
    }
  }

 for(int i = 0; i < 6; i++){
   ang += cny_70[i]*10*(1+i);
   n += cny_70[i];
  }

 if(n == 0){
  if(ang_ant <= -25)  //La línea se ha ido por la izquierda
    ang = -35;
  if(ang_ant >= 25)  //La línea de ha ido por la derecha
    ang = 35;
  if(ang_ant < 25 && ang_ant > -25)  //La línea se ha acabado
    ang = ang_ant;
 }
 else if(n > 3){  //Estaríamos cruzando con otra línea
  for(int i = 0; i < 3 && aux == 0; i++){
    if(cny_70[i] != 0 || cny_70[5 - i] != 0){
      aux = 1;
      if(cny_70[i] != 0 && cny_70[5 - i] == 0)
        ang = 10*(i+1);
      else if(cny_70[i] == 0 && cny_70[5 - i] != 0)
        ang = 10*((5 - i)+1);
      else
      ang = 35;
    }
  }
 ang = ang - 35;
 }
 else //Medida correcta
  ang = (ang/n) - 35;
  
ang_ant = ang;

  return ang;  
}

void fullStop(){

  stopMotor(pinMotorA);
  stopMotor(pinMotorB);
}

void stopMotor(const int pinMotor[3]){

  digitalWrite(pinMotor[1], LOW);
  digitalWrite(pinMotor[2], LOW);

  analogWrite(pinMotor[0], 0);
}

void forward(){

  vel = VEL_NORMAL;

  moveForward(pinMotorA, vel);
  moveForward(pinMotorB, vel);
}

void moveLeft(){

  fullStop();
  vel = VEL_NORMAL; 
  stopMotor(pinMotorB);
  moveForward(pinMotorA, vel + 35);

}

void moveRight(){

  fullStop();
  vel = VEL_NORMAL;
  stopMotor(pinMotorA);
  moveForward(pinMotorB, vel);

}


void moveForward(const int pinMotor[3], int speed){

   digitalWrite(pinMotor[1], HIGH);
   digitalWrite(pinMotor[2], LOW);

   analogWrite(pinMotor[0], speed);
}

void moveBackward(const int pinMotor[3], int speed){
 
   digitalWrite(pinMotor[1], LOW);
   digitalWrite(pinMotor[2], HIGH);

   analogWrite(pinMotor[0], speed);
}
