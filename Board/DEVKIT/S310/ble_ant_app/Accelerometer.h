
//Accelerometer
#define START_CONDITION      0x4C


#define XOUT_REG_ADDRESS     0x00
#define YOUT_REG_ADDRESS     0x01
#define ZOUT_REG_ADDRESS     0x02
#define TILT_REG_ADDRESS     0x03
#define SRST_REG_ADDRESS     0x04
#define SPCNT_REG_ADDRESS    0x05
#define INTSU_REG_ADDRESS    0x06
#define MODE_REG_ADDRESS     0x07
#define SR_REG_ADDRESS       0x08
#define PDET_REG_ADDRESS     0x09
#define PD_REG_ADDRESS       0x0A

//Humidity and temperaturte sensor
#define HUMIDITY_SENSOR_ADD  0x40												//Slave address

#define HUMIDITY_REG_ADDRESS_HOLD 			0xE5						//humidity data command in hold condition(Clock stretching)
#define HUMIDITY_REG_ADDRESS_NO_HOLD 		0xF5						//Humidity data command in no hold condition(repeted start)
#define TEMPERATURE_REG_ADDRESS_HOLD	 	0xE3						//temperature data command in hold condition(Clock stretching)
#define TEMPERATURE_REG_ADDRESS_NO_HOLD 	0xF3						//temperature data command in no hold condition(repeted start)


void Read_Coordinate_Data(void);
void Initialize_Accelerometer(void);
void Tilt_Reg_Data(void);

void Humidity_sensor_data(void);
void Temperature_sensor_data(void);


