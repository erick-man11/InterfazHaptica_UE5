#include <SPI.h>      //Library for using SPI Communication
#include <mcp2515.h>  //Library for using CAN Communication
#include <SD.h>

// Value Limits
#define P_MIN -3.141592f
#define P_MAX 3.141592f
#define V_MIN -65.0f
#define V_MAX 65.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -4.0f
#define T_MAX 4.0f
#define TEXT_MIN -10.0f
#define TEXT_MAX 10.0f

File myFile;

unsigned int dir4 = 0x04;

// Set values
//Valores que se pueden cambiar////////////////////////////
float p_in = 0.0f;
float v_in = 0.0f;
float kp_in = 0.0f;
float kd_in = 0.0f;
float t_in = 0.0f;
/////////////////////////////////////

float p_sk = 0.0f;
float k1 = 5.0f;
float k2 = 1.0f;

// measured values
float p_out = 0.0f;
float v_out = 0.0f;
float t_out = 0.0f;

struct can_frame canMsg;

int32_t pos;

union can_msg_rx {
  struct
  {
    uint8_t cmd;
    int32_t pos_0;
    uint8_t null_1;
    uint16_t null_2;  //En Arduino UNO, Nano y Mega, int es equivalente a uint16_t y long a uint36_t.
  } bf_1;

  struct
  {
    uint8_t b0;
    uint8_t b1;
    uint8_t b2;
    uint8_t b3;
    uint8_t b4;
    uint8_t b5;
    uint8_t b6;
    uint8_t b7;
  } bf_2;
};
union can_msg_tx {
  struct
  {
    uint8_t cmd;
    int8_t null_1;
    int16_t null_2;
    int16_t iq;
    uint16_t null_3;
  } bf_1;

  struct
  {
    uint8_t b0;
    uint8_t b1;
    uint8_t b2;
    uint8_t b3;
    uint8_t b4;
    uint8_t b5;
    uint8_t b6;
    uint8_t b7;
  } bf_2;
};

union can_msg_rx msg_rx;
union can_msg_tx msg_tx;

MCP2515 mcp2515(10);

int len = 10;

/*Interrupciones de seguridad y led de advertencia*/
const int ledEnableIrandi = 3;  //Permite observar si el sistema esta activo
const int pinEnableIrandi = 4;  //Permite salir de la rutina de control

const byte numChars = 32;
char receivedChars[numChars];  // an array to store the received data

boolean newData = false;

String posString = "";

//Variables para diferenciador 1
double ddqe = 0;
double dqe = 0;
double qe = 0;
double a = 10;

//Variables para diferenciador 2
double eps = 0.1;
double h1 = 1/eps;
double h2 = 1/(eps*eps);
double x1 = 0;
double dx1 = 0;
double x2 = 0;
double dx2 = 0;
double dx10 = 0;
double dx20 = 0;


unsigned long timeNow = 0;
unsigned long timeBefore = 0;
unsigned long dt = 0;

double ddqe0 = 0;
double dqe0 = 0;

void setup() {
  while (!Serial)
    ;
  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  /* Leds de arranque y paro de motor */
  pinMode(ledEnableIrandi, OUTPUT);  //Hacia led indicador
  pinMode(pinEnableIrandi, INPUT);   //Hacia el interruptor de emergencia

  Serial.print("Iniciando SD ...");
  if (!SD.begin(5)) {
    Serial.println("No se pudo inicializar");
    return;
  }
  Serial.println("inicializacion exitosa");

  if (!SD.exists("dataue.csv")) {
    myFile = SD.open("dataue.csv", FILE_WRITE);
    if (myFile) {
      Serial.println("Archivo nuevo, Escribiendo encabezado(fila 1)");
      myFile.println("Tiempo(ms),PosDigital,PosReal,VelReal,TorqueReal,PosEstimada,VelEstimada,AcelEstimada,x1,x2,dx2");
      myFile.close();
    } else {
      Serial.println("Error creando el archivo dataue.csv");
    }
  }
}

void enter_motor_mode(unsigned int dir)  //
{
  canMsg.can_id = dir;
  canMsg.can_dlc = 0x08;
  canMsg.data[0] = 0xFF;
  canMsg.data[1] = 0xFF;
  canMsg.data[2] = 0xFF;
  canMsg.data[3] = 0xFF;
  canMsg.data[4] = 0xFF;
  canMsg.data[5] = 0xFF;
  canMsg.data[6] = 0xFF;
  canMsg.data[7] = 0xFC;
  mcp2515.sendMessage(&canMsg);  //передаем CAN сообщение (enviar mensaje CAN)
  len = 10;
  while ((mcp2515.readMessage(&canMsg) != MCP2515::ERROR_OK)) {
    delay(1);
    len--;
    if ((len <= 0)) {
      break;
    }
  }
  msg_rx.bf_2.b0 = canMsg.data[0];
  msg_rx.bf_2.b1 = canMsg.data[1];
  msg_rx.bf_2.b2 = canMsg.data[2];
  msg_rx.bf_2.b3 = canMsg.data[3];
  msg_rx.bf_2.b4 = canMsg.data[4];
  msg_rx.bf_2.b5 = canMsg.data[5];
  msg_rx.bf_2.b6 = canMsg.data[6];
  msg_rx.bf_2.b7 = canMsg.data[7];
}

void exit_motor_mode(unsigned int dir) {
  canMsg.can_id = dir;
  canMsg.can_dlc = 0x08;
  //Buffer
  canMsg.data[0] = 0xFF;
  canMsg.data[1] = 0xFF;
  canMsg.data[2] = 0xFF;
  canMsg.data[3] = 0xFF;
  canMsg.data[4] = 0xFF;
  canMsg.data[5] = 0xFF;
  canMsg.data[6] = 0xFF;
  canMsg.data[7] = 0xFD;
  mcp2515.sendMessage(&canMsg);  //передаем CAN сообщение (enviar mensaje CAN)
  len = 10;
  while ((mcp2515.readMessage(&canMsg) != MCP2515::ERROR_OK)) {
    delay(1);
    len--;
    if ((len <= 0)) {
      break;
    }
  }
  msg_rx.bf_2.b0 = canMsg.data[0];
  msg_rx.bf_2.b1 = canMsg.data[1];
  msg_rx.bf_2.b2 = canMsg.data[2];
  msg_rx.bf_2.b3 = canMsg.data[3];
  msg_rx.bf_2.b4 = canMsg.data[4];
  msg_rx.bf_2.b5 = canMsg.data[5];
  msg_rx.bf_2.b6 = canMsg.data[6];
  msg_rx.bf_2.b7 = canMsg.data[7];
}

void set_zero(unsigned int dir) {
  canMsg.can_id = dir;
  canMsg.can_dlc = 0x08;
  //Buffer
  canMsg.data[0] = 0xFF;
  canMsg.data[1] = 0xFF;
  canMsg.data[2] = 0xFF;
  canMsg.data[3] = 0xFF;
  canMsg.data[4] = 0xFF;
  canMsg.data[5] = 0xFF;
  canMsg.data[6] = 0xFF;
  canMsg.data[7] = 0xFE;
  mcp2515.sendMessage(&canMsg);
  len = 10;
  while ((mcp2515.readMessage(&canMsg) != MCP2515::ERROR_OK)) {
    delay(1);
    len--;
    if ((len <= 0)) {
      break;
    }
  }
  msg_rx.bf_2.b0 = canMsg.data[0];
  msg_rx.bf_2.b1 = canMsg.data[1];
  msg_rx.bf_2.b2 = canMsg.data[2];
  msg_rx.bf_2.b3 = canMsg.data[3];
  msg_rx.bf_2.b4 = canMsg.data[4];
  msg_rx.bf_2.b5 = canMsg.data[5];
  msg_rx.bf_2.b6 = canMsg.data[6];
  msg_rx.bf_2.b7 = canMsg.data[7];
}

//Función de comunicación al bus SPI el cuál se comunica al driver del motor por bus CAN
void pack_cmd(unsigned int dir) {
  byte buf[8];
  /// CAN Command Packet Structure ///
  /// 16 bit position command, between -4*pi and 4*pi
  /// 12 bit velocity command, between -30 and + 30 rad/s
  /// 12 bit kp, between 0 and 500 N-m/rad
  /// 12 bit kd, between 0 and 100 N-m*s/rad
  /// 12 bit feed forward torque, between -18 and 18 N-m
  /// CAN Packet is 8 8-bit words
  /// Formatted as follows.  For each quantity, bit 0 is LSB
  /// 0: [position[15-8]]
  /// 1: [position[7-0]]
  /// 2: [velocity[11-4]]
  /// 3: [velocity[3-0], kp[11-8]]
  /// 4: [kp[7-0]]
  /// 5: [kd[11-4]]
  /// 6: [kd[3-0], torque[11-8]]
  /// 7: [torque[7-0]]

  /// limit data to be within bounds ///
  float p_des = constrain(p_in, P_MIN, P_MAX);  //fminf(fmaxf(P_MIN, p_in), P_MAX);
  float v_des = constrain(v_in, V_MIN, V_MAX);  //fminf(fmaxf(V_MIN, v_in), V_MAX);
  float kp = constrain(kp_in, KP_MIN, KP_MAX);  //fminf(fmaxf(KP_MIN, kp_in), KP_MAX);
  float kd = constrain(kd_in, KD_MIN, KD_MAX);  //fminf(fmaxf(KD_MIN, kd_in), KD_MAX);
  float t_ff = constrain(t_in, T_MIN, T_MAX);   //fminf(fmaxf(T_MIN, t_in), T_MAX);

  /// convert floats to unsigned ints ///
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  /// pack ints into the can buffer ///

  buf[0] = p_int >> 8;
  buf[1] = p_int & 0xFF;
  buf[2] = v_int >> 4;
  buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  buf[4] = kp_int & 0xFF;
  buf[5] = kd_int >> 4;
  buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  buf[7] = t_int & 0xFF;

  canMsg.can_id = dir;
  canMsg.can_dlc = 0x08;
  canMsg.data[0] = buf[0];
  canMsg.data[1] = buf[1];
  canMsg.data[2] = buf[2];
  canMsg.data[3] = buf[3];
  canMsg.data[4] = buf[4];
  canMsg.data[5] = buf[5];
  canMsg.data[6] = buf[6];
  canMsg.data[7] = buf[7];
  mcp2515.sendMessage(&canMsg);  //передаем CAN сообщение (enviar mensaje CAN)
  len = 10;
  while ((mcp2515.readMessage(&canMsg) != MCP2515::ERROR_OK))  //Leer mensaje del CAN
  {
    delay(1);
    len--;
    if ((len <= 0)) {
      break;
    }
  }
  msg_rx.bf_2.b0 = canMsg.data[0];
  msg_rx.bf_2.b1 = canMsg.data[1];
  msg_rx.bf_2.b2 = canMsg.data[2];
  msg_rx.bf_2.b3 = canMsg.data[3];
  msg_rx.bf_2.b4 = canMsg.data[4];
  msg_rx.bf_2.b5 = canMsg.data[5];
  msg_rx.bf_2.b6 = canMsg.data[6];
  msg_rx.bf_2.b7 = canMsg.data[7];
  /// unpack ints from can buffer ///
  unsigned int id = msg_rx.bf_2.b0;
  unsigned int p_int_rx = (msg_rx.bf_2.b1 << 8) | msg_rx.bf_2.b2;
  // unsigned int v_int_rx = (msg_rx.bf_2.b3 << 4) | (msg_rx.bf_2.b4 >> 4);
  unsigned int v_int_rx = (msg_rx.bf_2.b3 << 4) | (msg_rx.bf_2.b4 >> 4);
  unsigned int i_int_rx = ((msg_rx.bf_2.b4 & 0xF) << 8) | msg_rx.bf_2.b5;
  /// convert uints to floats ///
  p_out = uint_to_float(p_int_rx, P_MIN, P_MAX, 16);
  v_out = uint_to_float(v_int_rx, V_MIN, V_MAX, 12);
  t_out = uint_to_float(i_int_rx, -T_MAX, T_MAX, 12);
}


void cmd_cero(unsigned int dir) {
  p_in = 0.0f;
  v_in = 0.0f;
  kp_in = 0.0f;
  kd_in = 0.0f;
  t_in = 0.0f;
  byte buf[8];

  /// CAN Command Packet Structure ///
  /// 16 bit position command, between -4*pi and 4*pi
  /// 12 bit velocity command, between -30 and + 30 rad/s
  /// 12 bit kp, between 0 and 500 N-m/rad
  /// 12 bit kd, between 0 and 100 N-m*s/rad
  /// 12 bit feed forward torque, between -18 and 18 N-m
  /// CAN Packet is 8 8-bit words
  /// Formatted as follows.  For each quantity, bit 0 is LSB
  /// 0: [position[15-8]]
  /// 1: [position[7-0]]
  /// 2: [velocity[11-4]]
  /// 3: [velocity[3-0], kp[11-8]]
  /// 4: [kp[7-0]]
  /// 5: [kd[11-4]]
  /// 6: [kd[3-0], torque[11-8]]
  /// 7: [torque[7-0]]

  /// limit data to be within bounds ///
  float p_des = constrain(p_in, P_MIN, P_MAX);  //fminf(fmaxf(P_MIN, p_in), P_MAX);
  float v_des = constrain(v_in, V_MIN, V_MAX);  //fminf(fmaxf(V_MIN, v_in), V_MAX);
  float kp = constrain(kp_in, KP_MIN, KP_MAX);  //fminf(fmaxf(KP_MIN, kp_in), KP_MAX);
  float kd = constrain(kd_in, KD_MIN, KD_MAX);  //fminf(fmaxf(KD_MIN, kd_in), KD_MAX);
  float t_ff = constrain(t_in, T_MIN, T_MAX);   //fminf(fmaxf(T_MIN, t_in), T_MAX);
  /// convert floats to unsigned ints ///
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  /// pack ints into the can buffer ///
  buf[0] = p_int >> 8;
  buf[1] = p_int & 0xFF;
  buf[2] = v_int >> 4;
  buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  buf[4] = kp_int & 0xFF;
  buf[5] = kd_int >> 4;
  buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  buf[7] = t_int & 0xFF;

  canMsg.can_id = dir;
  canMsg.can_dlc = 0x08;
  canMsg.data[0] = buf[0];
  canMsg.data[1] = buf[1];
  canMsg.data[2] = buf[2];
  canMsg.data[3] = buf[3];
  canMsg.data[4] = buf[4];
  canMsg.data[5] = buf[5];
  canMsg.data[6] = buf[6];
  canMsg.data[7] = buf[7];
  mcp2515.sendMessage(&canMsg);  //передаем CAN сообщение
  len = 10;
  while ((mcp2515.readMessage(&canMsg) != MCP2515::ERROR_OK)) {
    delay(1);
    len--;
    if ((len <= 0)) {
      break;
    }
  }
  msg_rx.bf_2.b0 = canMsg.data[0];
  msg_rx.bf_2.b1 = canMsg.data[1];
  msg_rx.bf_2.b2 = canMsg.data[2];
  msg_rx.bf_2.b3 = canMsg.data[3];
  msg_rx.bf_2.b4 = canMsg.data[4];
  msg_rx.bf_2.b5 = canMsg.data[5];
  msg_rx.bf_2.b6 = canMsg.data[6];
  msg_rx.bf_2.b7 = canMsg.data[7];
  /// unpack ints from can buffer ///
  unsigned int id = msg_rx.bf_2.b0;
  unsigned int p_int_rx = (msg_rx.bf_2.b1 << 8) | msg_rx.bf_2.b2;
  unsigned int v_int_rx = (msg_rx.bf_2.b3 << 4) | (msg_rx.bf_2.b4 >> 4);
  unsigned int i_int_rx = ((msg_rx.bf_2.b4 & 0xF) << 8) | msg_rx.bf_2.b5;
  /// convert uints to floats ///
  p_out = uint_to_float(p_int_rx, P_MIN, P_MAX, 16);
  v_out = uint_to_float(v_int_rx, V_MIN, V_MAX, 12);
  t_out = uint_to_float(i_int_rx, -T_MAX, T_MAX, 12);
}


unsigned int float_to_uint(float x, float x_min, float x_max, int bits) {
  /// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  if (bits == 12) {
    pgg = (unsigned int)((x - offset) * 4095.0 / span);
  }
  if (bits == 16) {
    pgg = (unsigned int)((x - offset) * 65535.0 / span);
  }
  return pgg;
}

float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits == 12) {
    pgg = ((float)x_int) * span / 4095.0 + offset;
  }
  if (bits == 16) {
    pgg = ((float)x_int) * span / 65535.0 + offset;
  }
  return pgg;
}

void loop() {
  while (digitalRead(pinEnableIrandi) == HIGH) {
    //Serial.println("0.000");
  }  //Esperamos a que el interruptor se cierre

  //Activación
  delay(500);

  set_zero(dir4);
  delay(500);

  enter_motor_mode(dir4);
  enter_motor_mode(dir4);
  delay(500);

  // Serial.println("\nSe ha establecido la posicion actual como cero...\n");

  /* Reinicia valores de control */
  p_in = 0.0f;
  v_in = 0.0f;
  kp_in = 0.0f;
  kd_in = 0.0f;
  t_in = 0.0f;
  //Serial.print("\nSe envían ceros al motor...\n");
  pack_cmd(dir4);
  delay(500);
  digitalWrite(ledEnableIrandi, HIGH);

  myFile = SD.open("dataue.csv", FILE_WRITE);  //abrimos  el archivo

  ddqe = 0;
  dqe = 0;
  qe = 0;

  timeNow = micros();

  int cont = 0;

  //Activo mientras interruptor cerrado
  while (digitalRead(pinEnableIrandi) == LOW) {
    // Lectura del serial
    recvWithEndMarker();
    showNewData();
    timeBefore = timeNow;
    timeNow = micros();

    dt = (timeNow - timeBefore);

    //Estimador de velocidad 1

    ddqe0 = ddqe;
    ddqe = (a * a) * (p_out - qe) - 2 * a * dqe;

    dqe0 = dqe;
    dqe = dqe + dt * (ddqe0 + ddqe) / 2000000.0;

    qe = qe + dt * (dqe0 + dqe) / 2000000.0;

    //Estimador de velocidad 2

    dx10 = dx1;
    dx20 = dx2;

    dx1 = x2 + h1*(p_out - x1);
    dx2 = h2*(p_out - x1);

    x1 = x1 + dt * (dx10 + dx1) / 2000000.0;
    x2 = x2 + dt * (dx20 + dx2) / 2000000.0;


    // Lazo de control del motor
    t_in = k1 * (p_sk - p_out) - 1 * k2 * dqe;
    pack_cmd(dir4);

    // Mandamos el dato por el puerto serial
    posString = String(p_out, 3);
    
    if(p_out > 0){
      Serial.print("+");
    }
    Serial.println(posString);
    
    // Serial.println(cont);
    // Serial.print(",");
    // Serial.print(qe);
    // Serial.print(",");
    // Serial.print(dqe);
    // Serial.print(",");
    // Serial.println(ddqe);
  }

  myFile.close();  //cerramos el archivo

  //Desactivación
  //Serial.print("\nMotor Deshabilitado\n");
  exit_motor_mode(dir4);
  exit_motor_mode(dir4);
  kp_in = 0.0f;  //No se utiliza
  kd_in = 0.0f;  //No se utiliza
  p_in = 0.0f;
  v_in = 0.0f;
  t_in = 0.0f;
  // pack_cmd(dir4);
  digitalWrite(ledEnableIrandi, LOW);  //Apagamos el led de puesta en marcha
}

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    } else {
      receivedChars[ndx] = '\0';  // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

void showNewData() {
  if (newData == true) {
    //Serial.print("This just in ... ");
    //Serial.println(receivedChars);
    newData = false;
    String s = String(receivedChars);
    p_sk = s.toFloat();
    String p = String(qe, 3);
    String dp = String(dqe, 3);
    String ddp = String(ddqe, 3);

    if (myFile) {
      myFile.print(millis());
      myFile.print(",");
      myFile.print(s);  //PosDigital
      myFile.print(",");
      myFile.print(posString);  //PosReal
      myFile.print(",");
      myFile.print(v_out);  //VelReal
      myFile.print(",");
      myFile.print(t_out);  //TorqueReal
      myFile.print(",");
      myFile.print(p);  //PosEstimada
      myFile.print(",");
      myFile.print(dp);  //VelEstimada
      myFile.print(",");
      myFile.print(ddp);  //AcelEstimada
      myFile.print(",");
      myFile.print(x1);  //x1
      myFile.print(",");
      myFile.print(x2);  //x2
      myFile.print(",");
      myFile.println(dx2);  //dx2
    }
  }
}
