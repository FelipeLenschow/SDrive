#define MotorID 0x11

#include <SimpleFOC.h>
#include "STM32_CAN.h"
#include <NTC_Thermistor.h>

Thermistor* th0;
Thermistor* th1;
Thermistor* th2;
Thermistor* th3;

#define LED1 PC12
#define CurrentSenseGain 50


// Motor instance
BLDCMotor motor = BLDCMotor(11, 0.193, 380);
BLDCDriver6PWM driver = BLDCDriver6PWM(PB11, PB10, PB0, PA7, PB1, PA6);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.0015f, 50.0f, PA3, PA0, PA1);
STM32_CAN Can(CAN1, ALT);

HardwareSerial SerialX(PA10, PA9);

// instantiate the commander
Commander command = Commander(SerialX, '\n', true);

static CAN_message_t CAN_RX_msg;
float Angle;

void doMotor(char* cmd) {
  command.motor(&motor, cmd);
}

void setup() {
  SerialX.begin(115200);
  pinMode(LED1, OUTPUT);

  while (0) {
    digitalWrite(LED1, HIGH);
    delay(100);
    digitalWrite(LED1, LOW);
    delay(200);
    SerialX.println(".................");
  }

  digitalWrite(LED1, HIGH);

  th0 = new NTC_Thermistor(PC0, 10000, 10000, 25, 3950, 1023);
  th1 = new NTC_Thermistor(PC1, 10000, 10000, 25, 3950, 1023);
  th2 = new NTC_Thermistor(PC2, 10000, 10000, 25, 3950, 1023);
  th3 = new NTC_Thermistor(PC3, 10000, 10000, 25, 3950, 1023);


  Can.begin();
  Can.setBaudRate(1000000);  //1000KBPS
  Can.setMBFilter(MB0, MotorID);


  command.add('M', doMotor, "motor");
  motor.useMonitoring(SerialX);
  motor.monitor_downsample = 0;  // disable monitor at first - optional
  SimpleFOCDebug::enable(&SerialX);
  SerialX.println("STARTING...");


  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  //Wire.begin();
  sensor.init();
  motor.linkSensor(&sensor);


  driver.voltage_power_supply = 15;
  driver.init();
  motor.linkDriver(&driver);


  currentSense.linkDriver(&driver);
  currentSense.init();
  //currentSense.skip_align = false;
  currentSense.driverAlign(3);
  motor.linkCurrentSense(&currentSense);


  motor.voltage_sensor_align = 1;
  motor.velocity_index_search = 3;




  //******************************************************************************************************************************
  // control loop type and torque mode
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::angle;
  motor.motion_downsample = 0.0;



  // velocity loop PID
  motor.PID_velocity.P = 0.15;
  motor.PID_velocity.I = 10.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 1000.0;
  motor.PID_velocity.limit = 4;  //2.0;
  // Low pass filtering time constant
  motor.LPF_velocity.Tf = 0.005;
  // angle loop PID
  motor.P_angle.P = 40;  //20.0;
  motor.P_angle.I = 0.0;
  motor.P_angle.D = 0.0;
  motor.P_angle.output_ramp = 0.0;
  motor.P_angle.limit = 40;  //20.0;
  // Low pass filtering time constant
  motor.LPF_angle.Tf = 0.0;
  // current q loop PID
  motor.PID_current_q.P = 3.0;
  motor.PID_current_q.I = 300.0;
  motor.PID_current_q.D = 0.0;
  motor.PID_current_q.output_ramp = 0.0;
  motor.PID_current_q.limit = 12.0;
  // Low pass filtering time constant
  motor.LPF_current_q.Tf = 0.005;
  // current d loop PID
  motor.PID_current_d.P = 3.0;
  motor.PID_current_d.I = 300.0;
  motor.PID_current_d.D = 0.0;
  motor.PID_current_d.output_ramp = 0.0;
  motor.PID_current_d.limit = 12.0;
  // Low pass filtering time constant
  motor.LPF_current_d.Tf = 0.005;
  // Limits
  motor.velocity_limit = 500.0;
  motor.voltage_limit = 25.0;
  motor.current_limit = 20;
  // sensor zero offset - home position
  motor.sensor_offset = sensor.getSensorAngle();
  // general settings
  // motor phase resistance
  //motor.phase_resistance = 3.898;
  // pwm modulation settings
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.modulation_centered = 1.0;



  //******************************************************************************************************************************
  motor.init();
  motor.initFOC();

  motor.monitor_variables = 0b01111111;
  motor.monitor_downsample = 10;  // default 10

  delay(100);
  digitalWrite(LED1, HIGH);
}


unsigned long timePrint;

void loop() {

  if (Can.read(CAN_RX_msg)) {
    memcpy(&Angle, CAN_RX_msg.buf, 4);
    motor.move(Angle);
  }

  motor.loopFOC();
  motor.move();
  command.run();


  float Temp = max(max(th0->readCelsius(), th1->readCelsius()), max(th2->readCelsius(), th3->readCelsius()));
  if (Temp > 70) {
    SerialX.println("Ai ai, ta quente aqui meu...");
    while (1)
      motor.disable();
  } 
  if (millis() - timePrint > 100) {

    SerialX.println(Temp);
    timePrint = millis();
  }
  digitalWrite(LED1, !digitalRead(LED1));


  /*
  SerialX.print(currents.a * 1000);  // milli Amps
  SerialX.print("\t");
  SerialX.print(currents.b * 1000);  // milli Amps
  SerialX.print("\t");
  SerialX.print(currents.c * 1000);  // milli Amps
  SerialX.print("\t");
  SerialX.println(current_magnitude * 1000);  // milli Amp
  */
}
