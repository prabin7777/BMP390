#include<Wire.h>
#define BMP390_I2C_ADDR       0x77        // The BMP390 I2C address
#define BMP390_I2C_ALT_ADDR   0x76        // The BMP390 I2C alternate address
#define BMP390_ID             0x50        // The BMP390 device ID
#define BMP390_ID             0x60        // The BMP390 device ID
#define RESET_CODE            0xB6        // The BMP390 reset code
#define FIFO_FLUSH            0xB0        // The BMP390 flush FIFO code
#define Address               0x77        // I2C so we used 
#define BMPAdd                0x77


enum {
  BMP390_CHIP_ID         = 0x00,          // Chip ID register sub-address
  BMP390_ERR_REG         = 0x02,          // Error register sub-address
  BMP390_STATUS          = 0x03,          // Status register sub-address
  BMP390_DATA_0          = 0x04,          // Pressure eXtended Least Significant Byte (XLSB) register sub-address
  BMP390_DATA_1          = 0x05,          // Pressure Least Significant Byte (LSB) register sub-address
  BMP390_DATA_2          = 0x06,          // Pressure Most Significant Byte (MSB) register sub-address
  BMP390_DATA_3          = 0x07,          // Temperature eXtended Least Significant Byte (XLSB) register sub-address
  BMP390_DATA_4          = 0x08,          // Temperature Least Significant Byte (LSB) register sub-address
  BMP390_DATA_5          = 0x09,          // Temperature Most Significant Byte (MSB) register sub-address
  BMP390_SENSORTIME_0    = 0x0C,          // Sensor time register 0 sub-address
  BMP390_SENSORTIME_1    = 0x0D,          // Sensor time register 1 sub-address
  BMP390_SENSORTIME_2    = 0x0E,          // Sensor time register 2 sub-address
  BMP390_EVENT           = 0x10,          // Event register sub-address
  BMP390_INT_STATUS      = 0x11,          // Interrupt Status register sub-address
  BMP390_FIFO_LENGTH_0   = 0x12,          // FIFO Length Least Significant Byte (LSB) register sub-address
  BMP390_FIFO_LENGTH_1   = 0x13,          // FIFO Length Most Significant Byte (MSB) register sub-address
  BMP390_FIFO_DATA       = 0x14,          // FIFO Data register sub-address
  BMP390_FIFO_WTM_0      = 0x15,          // FIFO Water Mark Least Significant Byte (LSB) register sub-address
  BMP390_FIFO_WTM_1      = 0x16,          // FIFO Water Mark Most Significant Byte (MSB) register sub-address
  BMP390_FIFO_CONFIG_1   = 0x17,          // FIFO Configuration 1 register sub-address
  BMP390_FIFO_CONFIG_2   = 0x18,          // FIFO Configuration 2 register sub-address
  BMP390_INT_CTRL        = 0x19,          // Interrupt Control register sub-address
  BMP390_IF_CONFIG       = 0x1A,          // Interface Configuration register sub-address
  BMP390_PWR_CTRL        = 0x1B,          // Power Control register sub-address
  BMP390_OSR             = 0x1C,          // Oversampling register sub-address
  BMP390_ODR             = 0x1D,          // Output Data Rate register sub-address
  BMP390_CONFIG          = 0x1F,          // Configuration register sub-address
  BMP390_TRIM_PARAMS     = 0x31,          // Trim parameter registers' base sub-address
  BMP390_CMD             = 0x7E           // Command register sub-address
};
 struct {                                                    
      uint16_t param_T1;
      uint16_t param_T2;
      int8_t   param_T3;
      int16_t  param_P1;
      int16_t  param_P2;
      int8_t   param_P3;
      int8_t   param_P4;
      uint16_t param_P5;
      uint16_t param_P6;
      int8_t   param_P7;
      int8_t   param_P8;
      int16_t  param_P9;
      int8_t   param_P10;
      int8_t   param_P11;
 }params;
    
    struct FloatParams {                                        
      float param_T1;
      float param_T2;
      float param_T3;
      float param_P1;
      float param_P2;
      float param_P3;
      float param_P4;
      float param_P5;
      float param_P6;
      float param_P7;
      float param_P8;
      float param_P9;
      float param_P10;
      float param_P11;
    } floatParams;          

void setup() {
  Wire.begin();
    Serial.begin(115200);
  Wire.begin();

//  //register 0x1F config 
//  Wire.beginTransmission(BMPAdd);
//  Wire.write(0x1F);
//  Wire.write();
//  Wire.endTransmission(true);

  //Register 0x1C OSR
  Wire.beginTransmission(BMPAdd);
  Wire.write(0x1C);
  Wire.write(0b00000000);
  Wire.endTransmission(true);

  //Register 0x1D ODR
  Wire.beginTransmission(BMPAdd);
  Wire.write(0x1D);
  Wire.write(0b00000000);

  //register pwr_ctrl 0x1B
  Wire.beginTransmission(BMPAdd);
  Wire.write(0x1B);
  Wire.write(0b00110011);
  Wire.endTransmission(true);

  //register FIFO_CONFIG2 0x18
  Wire.beginTransmission(BMPAdd);
  Wire.write(0x18);
  Wire.write(0b00001000);
  Wire.endTransmission(true);

  //register FIFO_CONFIG1 0x17
  Wire.beginTransmission(BMPAdd);
  Wire.write(0x17);
  Wire.write(0b00011001);
  Wire.endTransmission(true);


  //register INT_STATUS 0x11
  Wire.beginTransmission(BMPAdd);
  Wire.write(0x11);
  Wire.write(0b00000011);
  Wire.endTransmission();

  //register EVENT 0x10
  Wire.beginTransmission(BMPAdd);
  Wire.write(0x10);
  Wire.write(0b00000011);
  Wire.endTransmission(true);


 //register CHIP_ID
 Wire.beginTransmission(BMPAdd);
 Wire.write(0x00);
 Wire.write(0x60);
 Wire.endTransmission(true); 


  
  long address=BMP390_I2C_ADDR;
   readBytes(BMP390_TRIM_PARAMS, (uint8_t*)&params, sizeof(params)); // Read the trim parameters into the params structure
  floatParams.param_T1 = (float)params.param_T1 / powf(2.0f, -8.0f); // Calculate the floating point trim parameters
  floatParams.param_T2 = (float)params.param_T2 / powf(2.0f, 30.0f);
  floatParams.param_T3 = (float)params.param_T3 / powf(2.0f, 48.0f);
  floatParams.param_P1 = ((float)params.param_P1 - powf(2.0f, 14.0f)) / powf(2.0f, 20.0f);
  floatParams.param_P2 = ((float)params.param_P2 - powf(2.0f, 14.0f)) / powf(2.0f, 29.0f);
  floatParams.param_P3 = (float)params.param_P3 / powf(2.0f, 32.0f);
  floatParams.param_P4 = (float)params.param_P4 / powf(2.0f, 37.0f);
  floatParams.param_P5 = (float)params.param_P5 / powf(2.0f, -3.0f);
  floatParams.param_P6 = (float)params.param_P6 / powf(2.0f, 6.0f);
  floatParams.param_P7 = (float)params.param_P7 / powf(2.0f, 8.0f);
  floatParams.param_P8 = (float)params.param_P8 / powf(2.0f, 15.0f);
  floatParams.param_P9 = (float)params.param_P9 / powf(2.0f, 48.0f);
  floatParams.param_P10 = (float)params.param_P10 / powf(2.0f, 48.0f);
  floatParams.param_P11 = (float)params.param_P11 / powf(2.0f, 65.0f);
  Serial.begin(115200);
 
  
}

void loop() {


getTempPres();

}
// read single byte........................................................................................................................................................................
uint8_t readByte(uint8_t subAddress)                        // Read a byte from the sub-address using I2C
{
  uint8_t data = 0x00;
  
    Wire.beginTransmission(Address);         
    Wire.write(subAddress);                  
    Wire.endTransmission(false);             
    Wire.requestFrom(Address, (uint8_t)1);   
    data = Wire.read();                      
 
  
  return data;                                                      // Return data read from sub-address register
}
// read multiple bytes from I2C...........................................................................................................................................................
void readBytes(uint8_t subAddress, uint8_t* data, uint16_t count)
{  
    // Read "count" bytes into the "data" buffer using I2C
  
    Wire.beginTransmission(Address);          
    Wire.write(subAddress);                   
    Wire.endTransmission(false);              
    uint8_t i = 0;
    Wire.requestFrom(Address, (uint8_t)count);  
    while (Wire.available()) 
    {
      data[i++] = Wire.read(); 
      delay(4.5);
//      Serial.println(data[i]);         
    }
}


//...........................................................Read temperature and pressure.............................................................................................
  uint8_t getTempPres(){

  float temperature;
  float pressure;
  uint8_t data[6];
  
  // Create a data buffer
  
  readBytes(BMP390_DATA_0, &data[0], 6);                            // Read the temperature and pressure data
  int32_t adcTemp = (int32_t)data[5] << 16 | (int32_t)data[4] << 8 | (int32_t)data[3];  // Copy the temperature and pressure data into the adc variables
  int32_t adcPres = (int32_t)data[2] << 16 | (int32_t)data[1] << 8 | (int32_t)data[0];
  
  
  temperature = bmp390_compensate_temp((float)adcTemp);             // Temperature compensation (function from BMP388 datasheet)
  pressure = bmp390_compensate_press((float)adcPres, temperature);  // Pressure compensation (function from BMP388 datasheet)
  pressure /= 100.0f;                                               // Calculate the pressure in millibar/hPa
  Serial.print("temperature : ");
  Serial.println(temperature);
  Serial.print("pressure : ");
  Serial.println(pressure);
}




//.................................................................compensate raw temperature algorithm from datasheet...................................................................

float bmp390_compensate_temp(float uncomp_temp)
{
  float partial_data1 = uncomp_temp - floatParams.param_T1;
  float partial_data2 = partial_data1 * floatParams.param_T2;
  return partial_data2 + partial_data1 * partial_data1 * floatParams.param_T3;  
}

float bmp390_compensate_press(float uncomp_press, float t_lin)
{
  float partial_data1 = floatParams.param_P6 * t_lin;
  float partial_data2 = floatParams.param_P7 * t_lin * t_lin;
  float partial_data3 = floatParams.param_P8 * t_lin * t_lin * t_lin;
  float partial_out1 = floatParams.param_P5 + partial_data1 + partial_data2 + partial_data3;
  partial_data1 = floatParams.param_P2 * t_lin;
  partial_data2 = floatParams.param_P3 * t_lin * t_lin;
  partial_data3 = floatParams.param_P4 * t_lin * t_lin * t_lin;
  float partial_out2 = uncomp_press * (floatParams.param_P1 +
    partial_data1 + partial_data2 + partial_data3);
  partial_data1 = uncomp_press * uncomp_press;
  partial_data2 = floatParams.param_P9 + floatParams.param_P10 * t_lin;
  partial_data3 = partial_data1 * partial_data2;
  float partial_data4 = partial_data3 + uncomp_press * uncomp_press * uncomp_press * floatParams.param_P11;
  return partial_out1 + partial_out2 + partial_data4;
  
}
