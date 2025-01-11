#include <SimpleFOC.h>
// MagneticSensorSPI(MagneticSensorSPIConfig_s config, int cs)
//  config  - SPI config
//  cs      - SPI chip select pin 


MagneticSensorSPIConfig_s AS5048A_SPI_Config{
    .spi_mode = SPI_MODE1,
    .clock_speed = 10000000,
    .bit_resolution = 14,
    .angle_register = 0x3FFF,
    .data_start_bit = 13,
    .command_rw_bit = 14,
    .command_parity_bit = 15
};
// magnetic sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048A_SPI_Config, 1);
// alternative constructor (chipselsect, bit_resolution, angle_read_register, )
// MagneticSensorSPI sensor = MagneticSensorSPI(10, 14, 0x3FFF);
BLDCMotor motor = BLDCMotor(7, 11);
BLDCDriver6PWM driver = BLDCDriver6PWM(4, 2, 3, 5, 10, 11, 8);

// instantiate the commander
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {
  // monitoring port
  Serial.begin(115200);

  while(!Serial);
  
  SimpleFOCDebug::enable(&Serial);
  // initialise magnetic sensor hardware
  sensor.init(&SPI);
  motor.linkSensor(&sensor);

  
  // power supply voltage [V]
  driver.voltage_power_supply = 5;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 5;
  // daad_zone [0,1] - default 0.02f - 2%
  driver.dead_zone = 0.05f;

  // driver init
  if (!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }

  // link the motor and the driver
  motor.linkDriver(&driver);
  motor.useMonitoring(Serial);

  motor.motion_downsample = 5.0;

// velocity loop PID
motor.PID_velocity.P = 0.2;
motor.PID_velocity.I = 1.0;
motor.PID_velocity.D = 0.001;
motor.PID_velocity.output_ramp = 1000.0;
motor.PID_velocity.limit = 2.0;
// Low pass filtering time constant 
motor.LPF_velocity.Tf = 0.25;
// angle loop PID
motor.P_angle.P = 10.0;
motor.P_angle.I = 0.0;
motor.P_angle.D = 0.0;
motor.P_angle.output_ramp = 10000.0;
motor.P_angle.limit = 4.0;
// Low pass filtering time constant 
motor.LPF_angle.Tf = 0.05;
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
motor.velocity_limit = 60.0;
motor.voltage_limit = 5.0;
motor.current_limit = 2.0;
// sensor zero offset - home position 
motor.sensor_offset = 0.0;
  motor.useMonitoring(Serial);
  // init motor hardware
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }
 // align sensor and start FOC
  if(!motor.initFOC()){
    Serial.println("FOC init failed!");
    return;
  }



  // add target command M
  command.add('M', doMotor, "Motor");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target using serial terminal and command M:"));
  _delay(1000);
}

void loop() {
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();
motor.monitor();
  // user communication
  command.run();
}
