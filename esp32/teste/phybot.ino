#include "Adafruit_VL53L0X.h"
// Inclusão das Bibliotecas
#include <Wire.h>

#define PINO_TRIG 2
#define PINO_ECHO 4

#define  M1_IN1 19
#define  M1_IN2 18

const int M2_IN1 = 5;
const int M2_IN2 = 17;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

unsigned long tempo_HC_SR04 = millis();
unsigned long tempo_VL53L0X = millis();
unsigned long tempo_MPU_6050 = millis();
unsigned long tempoLeitura  = millis();

const long intervalo_HC_SR04 = 1000;
const long intervalo_VL53L0X = 100;
const long intervalo_MPU6050 = 100;

typedef struct MPU_DATA {
  float AccX;
  float AccY;
  float AccZ;
  float Temp;
  float GyrX;
  float GyrY;
  float GyrZ;
  float anguloXZ;
  float anguloYZ;
} MPU_DATA;

// Endereco I2C do sensor MPU-6050
const int MPU = 0x68;
// Distância do sensor HCSR04
float distancia_HC_SR04 = 0;
// Distância do sensor VL53L0X
float distancia_VL53L0X = 0;
// Angulo do Robô
float anguloZ = 0;
unsigned long tempoAnterior = 0;
// Variavel para armazenar valores do Sensor MPU6050
MPU_DATA valores_MPU6050 = {0};

float HC_SR04(void);
float VL53L0X(void);
MPU_DATA MPU_6050(void);
float deltaZ(unsigned long tempoAtual, unsigned long tempoAnterior, float GyrZ, float ERRO);
void show(void);
void motors(int velocidadeMotor1, int velocidadeMotor2, bool sentidoHMotor1, bool sentidoHMotor2);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Setup dos canais PWM para controle dos motores
  ledcSetup(0, 5000, 8); //Canal, Frequência, Resolução
  ledcSetup(1, 5000, 8);
  ledcSetup(2, 5000, 8);
  ledcSetup(3, 5000, 8);

  ledcAttachPin(M1_IN1, 0); //Porta, Canal
  ledcAttachPin(M1_IN2, 1);
  ledcAttachPin(M2_IN1, 2);
  ledcAttachPin(M2_IN2, 3);

  // Inicializa o barramento I2C (usa SDA e SCL padrão do ESP32)
  Wire.begin();
  // Inicia comunicação com o endereço do MPU (0x68)
  Wire.beginTransmission(MPU);
  // Seleciona o registrador 0x6B (PWR_MGMT_1)
  Wire.write(0x6B);
  // Escreve 0 → tira o sensor do modo sleep (liga o MPU)
  Wire.write(0);
  // Finaliza a transmissão
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  // Seleciona o registrador 0x1B (GYRO_CONFIG)
  Wire.write(0x1B);
  // Define o fundo de escala do giroscópio (±2000°/s)
  Wire.write(0x18); // 0b00011000
  Wire.endTransmission();

  Wire.beginTransmission(MPU);
  // Seleciona o registrador 0x1C (ACCEL_CONFIG)
  Wire.write(0x1C);
  // Define o fundo de escala do acelerômetro (±16g)
  Wire.write(0b00011000);  // Trocar esse comando para fundo de escala desejado conforme acima
  Wire.endTransmission();

  // Verifica conexão Serial 
  while (!Serial) {
    delay(1);
  }

  //Teste de conexão com o sensor MPU6050
  Wire.beginTransmission(MPU);
  if (Wire.endTransmission() != 0) {
    Serial.println("MPU6050 NÃO encontrado!");
    while(1);
  } else {
    Serial.println("MPU6050 OK!");
  }

  //Teste de conexão com o sensor VL53L0X
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }

  //Configuração dos pinos do Sensor HC_SR04
  pinMode(PINO_TRIG, OUTPUT);
  pinMode(PINO_ECHO, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long tempoAtual = millis();  

  // Atualiza os valores da MPU
  if(tempoAtual - tempo_MPU_6050 >= intervalo_MPU6050) {
    valores_MPU6050 = MPU_6050();
    tempo_MPU_6050 = tempoAtual; 
    
  }
  // Atualiza o valor do HCSR04
  if(tempoAtual - tempo_HC_SR04 >= intervalo_HC_SR04) {
    distancia_HC_SR04 = HC_SR04();
    tempo_HC_SR04 = tempoAtual; 
  }
  // Atualiza o valor do VL53L0X
  if(tempoAtual - tempo_VL53L0X >= intervalo_VL53L0X) {
    distancia_VL53L0X = VL53L0X();
    tempo_VL53L0X = tempoAtual;
  }
  // Atualiza o angulo de rotação do robô pelo Giroscópio Z
  anguloZ += deltaZ(tempoAtual);
  // Controla as velocidades do motor 1 e 2 e seus respectivos sentidos (true para sentido horário)
  motors(255,255,true,true);
  // Mostra no Serial todos os dados dos sensores a cada 1 segundo 
  if(tempoAtual - tempoLeitura >= 1000) {
    show();
    tempoLeitura = tempoAtual;
  }
}

MPU_DATA MPU_6050(void) {
  unsigned long tempoAtual = millis();
  float dt = (tempoAtual - tempoAnterior) / 1000.0;
  tempoAnterior = tempoAtual;
  
  MPU_DATA data;

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  int bytes = Wire.requestFrom(MPU, 14, true);

  if (bytes == 14) {

    int16_t raw;

    raw = (Wire.read() << 8) | Wire.read();
    data.AccX = raw;

    raw = (Wire.read() << 8) | Wire.read();
    data.AccY = raw;

    raw = (Wire.read() << 8) | Wire.read();
    data.AccZ = raw;

    raw = (Wire.read() << 8) | Wire.read();
    data.Temp = raw;

    raw = (Wire.read() << 8) | Wire.read();
    data.GyrX = raw;

    raw = (Wire.read() << 8) | Wire.read();
    data.GyrY = raw;

    raw = (Wire.read() << 8) | Wire.read();
    data.GyrZ = raw;

  } else {
    Serial.print("Erro I2C, bytes recebidos: ");
    Serial.println(bytes);

    data.AccX = data.AccY = data.AccZ = 0;
    data.GyrX = data.GyrY = data.GyrZ = 0;
  }

  // Ângulo pelo acelerômetro
  float angulo_inclinacao_frente = atan2(data.AccX,data.AccZ) * 180/ PI;
  float angulo_inclinacao_lado = atan2(data.AccY,data.AccZ) * 180/ PI;
  data.anguloXZ = angulo_inclinacao_frente;
  data.anguloYZ = angulo_inclinacao_lado;
  return data;
}

float HC_SR04(void) {
  digitalWrite(PINO_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PINO_TRIG,HIGH);
  delayMicroseconds(10);
  digitalWrite(PINO_TRIG,LOW);

  long duracao = pulseIn(PINO_ECHO, HIGH);
  float distancia = (duracao * 0.343) / 2;

  return distancia;
}

float VL53L0X(void) {
  VL53L0X_RangingMeasurementData_t measure;
    
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    return measure.RangeMilliMeter;
  } else {
    return -1;
  }
}

float deltaZ(unsigned long tempoAtual) {
  float dt = (tempoAtual - tempoAnterior) / 1000.0; // em segundos
  tempoAnterior = tempoAtual;

  float gyroZ = valores_MPU6050.GyrZ/ 16.4; // em graus por segundo
  //Serial.println(gyroZ);

  const float ERRO = 1.15;
  
  float deltaZ = (gyroZ - ERRO) * dt;
  //Serial.println(deltaZ);
  return deltaZ;
}

void show(void) {
  Serial.print("A distância do Sensor HC_SR04 em mm:");
  Serial.println(distancia_HC_SR04);
  Serial.print("A distância do Sensor VL53L0X em mm:");
  Serial.println(distancia_VL53L0X);
  /*
  Serial.print("AccX = ");
  Serial.println(valores_MPU6050.AccX / 2048);
  Serial.print("AccY = ");
  Serial.println(valores_MPU6050.AccY / 2048);
  Serial.print("AccZ = ");
  Serial.println(valores_MPU6050.AccZ / 2048);
  Serial.print("GyrX = ");
  Serial.println(valores_MPU6050.GyrX / 16.4);
  Serial.print("GyrY = ");
  Serial.println(valores_MPU6050.GyrY / 16.4);
  Serial.print("GyrZ = ");
  Serial.println(valores_MPU6050.GyrZ / 16.4);
  */
  Serial.print("Angulo lateral = ");
  Serial.println(valores_MPU6050.anguloYZ);
  Serial.print("Angulo frontal = ");
  Serial.println(valores_MPU6050.anguloXZ);
  Serial.print("Angulo de giro = ");
  Serial.println(anguloZ);  
}

void motors(int velocidadeMotor1, int velocidadeMotor2, bool sentidoHMotor1, bool sentidoHMotor2)
{
  if(sentidoHMotor1 && sentidoHMotor2) {
    // Motor 1 Sentido Horário
    ledcWrite(0, velocidadeMotor1);
    ledcWrite(1, 0);

    // Motor 2 Sentido Horário
    ledcWrite(2, velocidadeMotor2);//Canal, Velocidade
    ledcWrite(3, 0);
  } else if(!sentidoHMotor1 && sentidoHMotor2)
  {
    // Motor 1 Sentido Anti-Horário
    ledcWrite(0, 0);
    ledcWrite(1, velocidadeMotor1);

    // Motor 2 Sentido Horário
    ledcWrite(2, velocidadeMotor2);//Canal, Velocidade
    ledcWrite(3, 0);
  } else if(sentidoHMotor1 && !sentidoHMotor2)
  {
    // Motor 1 Sentido Horário
    ledcWrite(0, velocidadeMotor1);
    ledcWrite(1, 0);

    // Motor 2 Sentido Anti-Horário
    ledcWrite(2, 0);//Canal, Velocidade
    ledcWrite(3, velocidadeMotor2);
  } else {
    // Motor 1 Sentido Anti-Horário
    ledcWrite(0, 0);
    ledcWrite(1, velocidadeMotor1);

    // Motor 2 Sentido Anti-Horário
    ledcWrite(2, 0);//Canal, Velocidade
    ledcWrite(3, velocidadeMotor2);
  }
}