
/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/


#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>

#define VL6180X_ADDRESS 0x29
#define VL6180x_FAILURE_RESET  -1

#define VL6180X_IDENTIFICATION_MODEL_ID              0x0000
#define VL6180X_IDENTIFICATION_MODEL_REV_MAJOR       0x0001
#define VL6180X_IDENTIFICATION_MODEL_REV_MINOR       0x0002
#define VL6180X_IDENTIFICATION_MODULE_REV_MAJOR      0x0003
#define VL6180X_IDENTIFICATION_MODULE_REV_MINOR      0x0004
#define VL6180X_IDENTIFICATION_DATE                  0x0006 //16bit value
#define VL6180X_IDENTIFICATION_TIME                  0x0008 //16bit value

#define VL6180X_SYSTEM_MODE_GPIO0                    0x0010
#define VL6180X_SYSTEM_MODE_GPIO1                    0x0011
#define VL6180X_SYSTEM_HISTORY_CTRL                  0x0012
#define VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO         0x0014
#define VL6180X_SYSTEM_INTERRUPT_CLEAR               0x0015
#define VL6180X_SYSTEM_FRESH_OUT_OF_RESET            0x0016
#define VL6180X_SYSTEM_GROUPED_PARAMETER_HOLD        0x0017

#define VL6180X_SYSRANGE_START                       0x0018
#define VL6180X_SYSRANGE_THRESH_HIGH                 0x0019
#define VL6180X_SYSRANGE_THRESH_LOW                  0x001A
#define VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD     0x001B
#define VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME        0x001C
#define VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE 0x001E
#define VL6180X_SYSRANGE_CROSSTALK_VALID_HEIGHT      0x0021
#define VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE  0x0022
#define VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET   0x0024
#define VL6180X_SYSRANGE_RANGE_IGNORE_VALID_HEIGHT   0x0025
#define VL6180X_SYSRANGE_RANGE_IGNORE_THRESHOLD      0x0026
#define VL6180X_SYSRANGE_MAX_AMBIENT_LEVEL_MULT      0x002C
#define VL6180X_SYSRANGE_RANGE_CHECK_ENABLES         0x002D
#define VL6180X_SYSRANGE_VHV_RECALIBRATE             0x002E
#define VL6180X_SYSRANGE_VHV_REPEAT_RATE             0x0031

#define VL6180X_SYSALS_START                         0x0038
#define VL6180X_SYSALS_THRESH_HIGH                   0x003A
#define VL6180X_SYSALS_THRESH_LOW                    0x003C
#define VL6180X_SYSALS_INTERMEASUREMENT_PERIOD       0x003E
#define VL6180X_SYSALS_ANALOGUE_GAIN                 0x003F
#define VL6180X_SYSALS_INTEGRATION_PERIOD            0x0040

#define VL6180X_RESULT_RANGE_STATUS                  0x004D
#define VL6180X_RESULT_ALS_STATUS                    0x004E
#define VL6180X_RESULT_INTERRUPT_STATUS_GPIO         0x004F
#define VL6180X_RESULT_ALS_VAL                       0x0050
#define VL6180X_RESULT_HISTORY_BUFFER                0x0052 
#define VL6180X_RESULT_RANGE_VAL                     0x0062
#define VL6180X_RESULT_RANGE_RAW                     0x0064
#define VL6180X_RESULT_RANGE_RETURN_RATE             0x0066
#define VL6180X_RESULT_RANGE_REFERENCE_RATE          0x0068
#define VL6180X_RESULT_RANGE_RETURN_SIGNAL_COUNT     0x006C
#define VL6180X_RESULT_RANGE_REFERENCE_SIGNAL_COUNT  0x0070
#define VL6180X_RESULT_RANGE_RETURN_AMB_COUNT        0x0074
#define VL6180X_RESULT_RANGE_REFERENCE_AMB_COUNT     0x0078
#define VL6180X_RESULT_RANGE_RETURN_CONV_TIME        0x007C
#define VL6180X_RESULT_RANGE_REFERENCE_CONV_TIME     0x0080

#define VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD      0x010A
#define VL6180X_FIRMWARE_BOOTUP                      0x0119
#define VL6180X_FIRMWARE_RESULT_SCALER               0x0120
#define VL6180X_I2C_SLAVE_DEVICE_ADDRESS             0x0212
#define VL6180X_INTERLEAVED_MODE_ENABLE              0x02A3

#define cardSelect 4 

char filename[15];
File logfile;
char CardExists;
String lFile;
const int  RTC_cs=11;
boolean SerialStat = true;
boolean SDStat = true;
boolean RTC_Valid = 0;   

RTC_DS3231 rtc;

enum vl6180x_als_gain { //Data sheet shows gain values as binary list

GAIN_20 = 0, // Actual ALS Gain of 20
GAIN_10,     // Actual ALS Gain of 10.32
GAIN_5,      // Actual ALS Gain of 5.21
GAIN_2_5,    // Actual ALS Gain of 2.60
GAIN_1_67,   // Actual ALS Gain of 1.72
GAIN_1_25,   // Actual ALS Gain of 1.28
GAIN_1 ,     // Actual ALS Gain of 1.01
GAIN_40,     // Actual ALS Gain of 40

};

/*
 * Some file dump and directory routines.
 */
File root;
File entry;
File dataFile;

int _i2caddress;

int reading[4];


void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200); //Start Serial at 115200bps
  Wire.begin(); //Start I2C library
  for (int numSensor = 0; numSensor < 4; numSensor++){
    TCA9548A(numSensor);
    _i2caddress = VL6180X_ADDRESS; 
  
    VL6180xInit();
    if(VL6180xInit() != 0){
      Serial.println("FAILED TO INITALIZED"); //Initialize device and check for errors
    } 
  VL6180xDefautSettings();
  }
  delay(5000); 

  Serial.println("Welcome to distance measurements ");

    if ( !rtc.begin())
  {
    Serial.println ("RTC not correct. Will reset.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  pinMode (cardSelect, OUTPUT);
  digitalWrite (cardSelect, HIGH);

    // **************** Create an output file. ****************************

  CardExists = true;

  if (SerialStat) Serial.print ("SD card ");
  if (SD.begin(cardSelect))
  {
     if (SerialStat) Serial.println ("found");
     CardExists=true; 
     //digitalWrite(greenLED, HIGH);
  }
  else
  {
    if (SerialStat) Serial.println ("not found.");
    CardExists = false;
  }

  int j;
  strcpy(filename, "SC_000.TXT");

  if (CardExists)
  {
    for (uint8_t i = 0; i < 1000; i++) 
    {
      j=i/100;
      filename[3] = '0' + j;
      j=i%100;
      filename[4] = '0' + j/10;
      j=j%10;
      filename[5] = '0' + j%10;
      // create if does not exist, do not open existing, write, sync after write
      if (! SD.exists(filename)) break;
    }
  
    logfile = SD.open(filename, FILE_WRITE);
    if( ! logfile ) 
    { SDStat = false;
      if (SerialStat) 
      {
        Serial.print("Couldn't create "); 
        Serial.println(filename);
        //digitalWrite (greenLED, LOW);        
      }
    }
    else
    {
      SDStat = true;
      logfile.close();
      if (SerialStat) Serial.print("Writing to "); 
      if (SerialStat) Serial.println(filename);
    }
  }

  if (SerialStat) Serial.println ("============================================");

  //getIdentification(&identification); // Retrieve manufacture info from device memory
  //printIdentification(&identification);
  
  //OffsetCalibration();
  //after running ofset calibration PartToPartOffset == 27
  //VL6180x_setRegister(VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET, 10);
  //CrossTalkCalibration();

  //VL6180x_setRegister(VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE, 0);
  //int Initial_CrossTalkRate = VL6180x_getRegister16bit(VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE);
  //Serial.print("Initial cross talk rate is ");  
  //Serial.println(Initial_CrossTalkRate); 
  
  //setRangeIgnore();
  if (SDStat)
  {

    Serial.print ("*");
    logfile = SD.open(filename, FILE_WRITE);
    logfile.print ("UnixTime"); 
    logfile.print (" "); 
    logfile.print ("Data"); 
    logfile.print (" "); 
    logfile.print ("Time"); 
    logfile.print (" "); 
    logfile.print ("sensor 0, mm"); logfile.print (" "); 
    logfile.print ("sensor 1, mm"); logfile.print (" ");
    logfile.print ("sensor 2, mm"); logfile.print (" ");
    logfile.print ("sensor 3, mm"); logfile.println (" ");
    
    logfile.flush();
    logfile.close();

  }

}

// the loop function runs over and over again forever
void loop() {
  
  DateTime now = rtc.now();
  //Get Ambient Light level and report in LUX
  //Serial.print("Ambient Light Level (Lux) = ");
  
  //Input GAIN for light levels, 
  // GAIN_20     // Actual ALS Gain of 20
  // GAIN_10     // Actual ALS Gain of 10.32
  // GAIN_5      // Actual ALS Gain of 5.21
  // GAIN_2_5    // Actual ALS Gain of 2.60
  // GAIN_1_67   // Actual ALS Gain of 1.72
  // GAIN_1_25   // Actual ALS Gain of 1.28
  // GAIN_1      // Actual ALS Gain of 1.01
  // GAIN_40     // Actual ALS Gain of 40
  
  //Serial.println(getAmbientLight(GAIN_1) );
  //Get Distance and report in mm
  for (int numSensor = 0; numSensor < 4; numSensor++){
    TCA9548A(numSensor);
    Serial.print("Sensor num ");
    Serial.print(numSensor);
    Serial.print(" Distance measured (mm) = ");
    reading[numSensor] = getDistance();
    
    Serial.println(reading[numSensor]); 
  }
  Serial.println (ReadTimeDate());

if (SDStat)
  {
    digitalWrite(LED_BUILTIN, HIGH);  
    Serial.print ("*");
    logfile = SD.open(filename, FILE_WRITE);
    logfile.print (ReadTimeDate()); 

    logfile.print (" "); 
    logfile.print(reading[0]);  logfile.print(" ");
    logfile.print(reading[1]);  logfile.print(" ");
    logfile.print(reading[2]);  logfile.print(" ");
    logfile.print(reading[3]);  logfile.println(" ");
    
    logfile.flush();
    logfile.close();
    digitalWrite(LED_BUILTIN, LOW);

  }

  SyncTimeRTC();
  
 delay(1000);  
}

uint8_t VL6180x_getRegister(uint16_t registerAddr)
{
  uint8_t data;

  Wire.beginTransmission( _i2caddress ); // Address set on class instantiation
  Wire.write((registerAddr >> 8) & 0xFF); //MSB of register address
  Wire.write(registerAddr & 0xFF); //LSB of register address
  Wire.endTransmission(false); //Send address and register address bytes
  Wire.requestFrom( _i2caddress , 1);
  data = Wire.read(); //Read Data from selected register

  return data;
}
uint16_t VL6180x_getRegister16bit(uint16_t registerAddr)
{
  uint8_t data_low;
  uint8_t data_high;
  uint16_t data;

  Wire.beginTransmission( _i2caddress ); // Address set on class instantiation
  Wire.write((registerAddr >> 8) & 0xFF); //MSB of register address
  Wire.write(registerAddr & 0xFF); //LSB of register address
  Wire.endTransmission(false); //Send address and register address bytes

  Wire.requestFrom( _i2caddress, 2);
  data_high = Wire.read(); //Read Data from selected register
  data_low = Wire.read(); //Read Data from selected register
  data = (data_high << 8)|data_low;

  return data;
}

void VL6180x_setRegister(uint16_t registerAddr, uint8_t data)
{
  Wire.beginTransmission( _i2caddress ); // Address set on class instantiation
  Wire.write((registerAddr >> 8) & 0xFF); //MSB of register address
  Wire.write(registerAddr & 0xFF); //LSB of register address
  Wire.write(data); // Data/setting to be sent to device.
  Wire.endTransmission(); //Send address and register address bytes
}
void VL6180x_setRegister16bit(uint16_t registerAddr, uint16_t data)
{
  Wire.beginTransmission( _i2caddress ); // Address set on class instantiation
  Wire.write((registerAddr >> 8) & 0xFF); //MSB of register address
  Wire.write(registerAddr & 0xFF); //LSB of register address
  uint8_t temp;
  temp = (data >> 8) & 0xff;
  Wire.write(temp); // Data/setting to be sent to device
  temp = data & 0xff;
  Wire.write(temp); // Data/setting to be sent to device
  Wire.endTransmission(); //Send address and register address bytes
}

uint8_t VL6180xInit(void){
  uint8_t data; //for temp data storage

  data = VL6180x_getRegister(VL6180X_SYSTEM_FRESH_OUT_OF_RESET);
  //Serial.println(data);
  if(data != 1){ return VL6180x_FAILURE_RESET;
  
  //Serial.println("FAILED TO DATA"); //Initialize device and check for errors
  }
  //Required by datasheet
  //http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf
  VL6180x_setRegister(0x0207, 0x01);
  VL6180x_setRegister(0x0208, 0x01);
  VL6180x_setRegister(0x0096, 0x00);
  VL6180x_setRegister(0x0097, 0xfd);
  VL6180x_setRegister(0x00e3, 0x00);
  VL6180x_setRegister(0x00e4, 0x04);
  VL6180x_setRegister(0x00e5, 0x02);
  VL6180x_setRegister(0x00e6, 0x01);
  VL6180x_setRegister(0x00e7, 0x03);
  VL6180x_setRegister(0x00f5, 0x02);
  VL6180x_setRegister(0x00d9, 0x05);
  VL6180x_setRegister(0x00db, 0xce);
  VL6180x_setRegister(0x00dc, 0x03);
  VL6180x_setRegister(0x00dd, 0xf8);
  VL6180x_setRegister(0x009f, 0x00);
  VL6180x_setRegister(0x00a3, 0x3c);
  VL6180x_setRegister(0x00b7, 0x00);
  VL6180x_setRegister(0x00bb, 0x3c);
  VL6180x_setRegister(0x00b2, 0x09);
  VL6180x_setRegister(0x00ca, 0x09);  
  VL6180x_setRegister(0x0198, 0x01);
  VL6180x_setRegister(0x01b0, 0x17);
  VL6180x_setRegister(0x01ad, 0x00);
  VL6180x_setRegister(0x00ff, 0x05);
  VL6180x_setRegister(0x0100, 0x05);
  VL6180x_setRegister(0x0199, 0x05);
  VL6180x_setRegister(0x01a6, 0x1b);
  VL6180x_setRegister(0x01ac, 0x3e);
  VL6180x_setRegister(0x01a7, 0x1f);
  VL6180x_setRegister(0x0030, 0x00);

  return 0;
}
void VL6180xDefautSettings(void){
  //Recommended settings from datasheet
  //http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf
  //Enable Interrupts on Conversion Complete (any source)
  VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, (4 << 3)|(4) ); // Set GPIO1 high when sample complete
  VL6180x_setRegister(VL6180X_SYSTEM_MODE_GPIO1, 0x10); // Set GPIO1 high when sample complete
  VL6180x_setRegister(VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD, 0x30); //Set Avg sample period
  VL6180x_setRegister(VL6180X_SYSALS_ANALOGUE_GAIN, 0x46); // Set the ALS gain
  VL6180x_setRegister(VL6180X_SYSRANGE_VHV_REPEAT_RATE, 0xFF); // Set auto calibration period (Max = 255)/(OFF = 0)
  VL6180x_setRegister(VL6180X_SYSALS_INTEGRATION_PERIOD, 0x63); // Set ALS integration time to 100ms
  VL6180x_setRegister(VL6180X_SYSRANGE_VHV_RECALIBRATE, 0x01); // perform a single temperature calibration
  //Optional settings from datasheet
  //http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf
  VL6180x_setRegister(VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD, 0x09); // Set default ranging inter-measurement period to 100ms
  VL6180x_setRegister(VL6180X_SYSALS_INTERMEASUREMENT_PERIOD, 0x0A); // Set default ALS inter-measurement period to 100ms
  VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x24); // Configures interrupt on ‘New Sample Ready threshold event’ 
  //Additional settings defaults from community
  VL6180x_setRegister(VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME, 0x32);
  VL6180x_setRegister(VL6180X_SYSRANGE_RANGE_CHECK_ENABLES, 0x10 | 0x01);
  VL6180x_setRegister16bit(VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE, 0x7B );
  VL6180x_setRegister16bit(VL6180X_SYSALS_INTEGRATION_PERIOD, 0x64);
  VL6180x_setRegister(VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD,0x30);
  VL6180x_setRegister(VL6180X_SYSALS_ANALOGUE_GAIN,0x40);
  VL6180x_setRegister(VL6180X_FIRMWARE_RESULT_SCALER,0x01);
}
//void getIdentification(struct VL6180xIdentification *temp){
//
//  temp->idModel =  VL6180x_getRegister(VL6180X_IDENTIFICATION_MODEL_ID);
//  temp->idModelRevMajor = VL6180x_getRegister(VL6180X_IDENTIFICATION_MODEL_REV_MAJOR);
//  temp->idModelRevMinor = VL6180x_getRegister(VL6180X_IDENTIFICATION_MODEL_REV_MINOR);
//  temp->idModuleRevMajor = VL6180x_getRegister(VL6180X_IDENTIFICATION_MODULE_REV_MAJOR);
//  temp->idModuleRevMinor = VL6180x_getRegister(VL6180X_IDENTIFICATION_MODULE_REV_MINOR);
//  temp->idDate = VL6180x_getRegister16bit(VL6180X_IDENTIFICATION_DATE);
//  temp->idTime = VL6180x_getRegister16bit(VL6180X_IDENTIFICATION_TIME);
//}

//void printIdentification(struct VL6180xIdentification *temp){
//  Serial.print("Model ID = ");
//  Serial.println(temp->idModel);
//
//  Serial.print("Model Rev = ");
//  Serial.print(temp->idModelRevMajor);
//  Serial.print(".");
//  Serial.println(temp->idModelRevMinor);
//
//  Serial.print("Module Rev = ");
//  Serial.print(temp->idModuleRevMajor);
//  Serial.print(".");
//  Serial.println(temp->idModuleRevMinor);  
//
//  Serial.print("Manufacture Date = ");
//  Serial.print((temp->idDate >> 3) & 0x001F);
//  Serial.print("/");
//  Serial.print((temp->idDate >> 8) & 0x000F);
//  Serial.print("/1");
//  Serial.print((temp->idDate >> 12) & 0x000F);
//  Serial.print(" Phase: ");
//  Serial.println(temp->idDate & 0x0007);
//
//  Serial.print("Manufacture Time (s)= ");
//  Serial.println(temp->idTime * 2);
//  Serial.println();
//  Serial.println();
//}
uint8_t getDistance()
{
  VL6180x_setRegister(VL6180X_SYSRANGE_START, 0x01); //Start Single shot mode
  delay(10);
  VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);
  return VL6180x_getRegister(VL6180X_RESULT_RANGE_VAL);
  //  return distance;
}

float getAmbientLight(vl6180x_als_gain VL6180X_ALS_GAIN)
{
  //First load in Gain we are using, do it everytime incase someone changes it on us.
  //Note: Upper nibble shoudl be set to 0x4 i.e. for ALS gain of 1.0 write 0x46
  VL6180x_setRegister(VL6180X_SYSALS_ANALOGUE_GAIN, (0x40 | VL6180X_ALS_GAIN)); // Set the ALS gain

  //Start ALS Measurement 
  VL6180x_setRegister(VL6180X_SYSALS_START, 0x01);

    delay(100); //give it time... 

  VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);

  //Retrieve the Raw ALS value from the sensoe
  unsigned int alsRaw = VL6180x_getRegister16bit(VL6180X_RESULT_ALS_VAL);
  
  //Get Integration Period for calculation, we do this everytime incase someone changes it on us.
  unsigned int alsIntegrationPeriodRaw = VL6180x_getRegister16bit(VL6180X_SYSALS_INTEGRATION_PERIOD);
  
  float alsIntegrationPeriod = 100.0 / alsIntegrationPeriodRaw ;

  //Calculate actual LUX from Appnotes

  float alsGain = 0.0;
  
  switch (VL6180X_ALS_GAIN){
    case GAIN_20: alsGain = 20.0; break;
    case GAIN_10: alsGain = 10.32; break;
    case GAIN_5: alsGain = 5.21; break;
    case GAIN_2_5: alsGain = 2.60; break;
    case GAIN_1_67: alsGain = 1.72; break;
    case GAIN_1_25: alsGain = 1.28; break;
    case GAIN_1: alsGain = 1.01; break;
    case GAIN_40: alsGain = 40.0; break;
  }

//Calculate LUX from formula in AppNotes
  
  float alsCalculated = (float)0.32 * ((float)alsRaw / alsGain) * alsIntegrationPeriod;

  return alsCalculated;
}

int OffsetCalibration(){
  int status = 0;
  // Position a white target (88% reflectance(g)) at a distance of 50 mm from the top of the cover glass.
  //Perform a minimum of 10 range measurements and compute the average range (from RESULT__RANGE_VAL{0x62})
  int Average_range = 0;
  int8_t  Offset = 0;
  int8_t Initial_Offset = VL6180x_getRegister(VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET);
  Serial.print("Initial offset is "); 
  Serial.println(Initial_Offset); 
  int Initial_CrossTalkRate = VL6180x_getRegister(VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE);
  if (Initial_CrossTalkRate != 0){
     VL6180x_setRegister(VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE, 0);
  }
  for (int i = 0; i<10; i++){
    Average_range += getDistance();
  }
  Average_range = Average_range/10; // in mm
  // If the average range is within the 50 ± 3 mm, offset calibration is not required. 
  //Otherwise, complete this calibration procedure.
  if (Average_range >= 47 && Average_range <= 53 ){
    Serial.println("offset calibration is not required");  
    status = 1;
  }
  else {
    Serial.println("offset calibration is required"); 
    // Set SYSRANGE__PART_TO_PART_RANGE_OFFSET{0x24} = 0.
    VL6180x_setRegister(VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET, 0);
      for (int i = 0; i<10; i++){
        Average_range += getDistance();
      }
      Average_range = Average_range/10;
      //part-to-part offset = 50 mm  -  average range
      int8_t PartToPartOffset = 50 - Average_range;
      // Write the part-to-part offset result (in two’s complement notation) to SYSRANGE__PART_TO_PART_RANGE_OFFSE
      VL6180x_setRegister(VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET, PartToPartOffset);
      Serial.print("new part to part offset is ");  
      Serial.println(PartToPartOffset); 
      status = 1;
  }
  return status;  
}

int CrossTalkCalibration(){
  
  int status = 0;
  int Average_range = 0;
  int Average_return_rate = 0;
  
  //Perform offset calibration if required
  //Position a black target (3% reflectance(h)) at a distance of 100 mm from the top of the cover glass.     
  // Ensure     SYSRANGE__CROSSTALK_COMPENSATION_RATE{0x1E} = 0.
  
  int Initial_CrossTalkRate = VL6180x_getRegister16bit(VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE);
  Serial.print("Initial cross talk rate is ");  
  Serial.println(Initial_CrossTalkRate); 
  if (Initial_CrossTalkRate != 0){
     VL6180x_setRegister(VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE, 0);
  }
     // Perform a minimum of 10 range measurements and compute the average return rate (from RESULT__RANGE_RETURN_RATE{0x66})
     // VL6180X_RESULT_RANGE_RETURN_RATE 
     //and the average range (from RESULT__RANGE_VAL{0x62}).
  for (int i = 0; i<10; i++){
     Average_range = Average_range + getDistance();
     Average_return_rate = VL6180x_getRegister(VL6180X_RESULT_RANGE_RETURN_RATE);
        
  }

  Average_range = Average_range/10;
  Average_return_rate = Average_return_rate/10;
  
  // Calculate the cross-talk factor as follows
  int16_t crossTalk = Average_return_rate * (1 - Average_range/100); //(in Mcps)
  Serial.print("Calculated cross talk rate is ");  
  Serial.print(crossTalk); 
  Serial.println(" in Mcps"); 
  crossTalk = round(crossTalk * 128);
  Serial.print("Calculated cross talk rate is ");  
  Serial.print(crossTalk); 
  Serial.println(" in 9.7 format"); 
  status = 1;
  // Write the cross-talk result in 9.7 format to SYSRANGE__CROSSTALK_COMPENSATION_RATE.
  VL6180x_setRegister16bit(VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE, crossTalk);
  
  return status;
}
void setRangeIgnore(){
  Serial.print("VL6180X_SYSRANGE_RANGE_CHECK_ENABLES ");  
  Serial.print(VL6180x_getRegister(VL6180X_SYSRANGE_RANGE_CHECK_ENABLES));  
}
void TCA9548A(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

void DumpFile (void)
{
    Serial.print ("Dump file ");
    Serial.println (lFile);

  
 // if (!SD.begin (cardSelect)) {Serial.println ("No SD"); return;}
  dataFile = SD.open(lFile);

  
  // if the file is available, write to it:
  if (dataFile) 
  {
    while (dataFile.available()) {
      Serial.write(dataFile.read());
    }
    dataFile.close();
  }
  else Serial.println ("failed to open");

  Serial.println("============================================");
}

//*******************************************
void printDirectory(File dir, int numTabs) {
  Serial.println ("============================================");
  Serial.println ("Current Directory Listing");
  dir.rewindDirectory();
  while (true) 
  {
    entry =  dir.openNextFile();
    if (! entry) 
    {
      // no more files
      break;
    }

    for (uint8_t i = 0; i < numTabs; i++) 
    {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } 
    else 
    {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
      lFile = entry.name();
    entry.close();

    
  }
}
void ListDirectory (void)
{
/*
  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
*/
  //SD.begin(4);
  root = SD.open("/");

  printDirectory(root, 0);
  root.close();

  Serial.println("============================================");
}
int SetTimeDate(int d, int mo, int y, int h, int mi, int s){ 
  int TimeDate [7]={s,mi,h,0,d,mo,y};
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);
  for(int i=0; i<=6;i++){
    if(i==3)
      i++;
    int b= TimeDate[i]/10;
    int a= TimeDate[i]-b*10;
    if(i==2){
      if (b==2)
        b=B00000010;
      else if (b==1)
        b=B00000001;
    } 
    TimeDate[i]= a+(b<<4);
      
    digitalWrite(RTC_cs, LOW);
    SPI.transfer(i+0x80); 
    SPI.transfer(TimeDate[i]);        
    digitalWrite(RTC_cs, HIGH);
  }
}
int RTC_init(){ 
    Serial.println ("RTC_init started");
    pinMode(RTC_cs,OUTPUT); // chip select
    // start the SPI library:
    digitalWrite (RTC_cs, LOW);
    //SPI.begin(RTC_cs);
    SPI.setBitOrder(MSBFIRST); 
    SPI.setDataMode(SPI_MODE1); // both mode 1 & 3 should work 
    //set control register 
    digitalWrite(RTC_cs, LOW);  
    SPI.transfer(0x8E);
    SPI.transfer(0x60); //60= disable Oscillator and Battery SQ wave @1hz, temp compensation, Alarms disabled
    digitalWrite(RTC_cs, HIGH);
    delay(10);
    ReadTimeDate();
    Serial.print ("RTC Valid = "); Serial.println (RTC_Valid);
    if (RTC_Valid == 0)
    {
      Serial.println ("Preparing to set time and date.");
      SetTimeDate(18,8,16,12,0,0);
    }
    if (SerialStat) Serial.println (ReadTimeDate());
    digitalWrite (RTC_cs, HIGH);

//    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

String ReadTimeDate()
{
  String temp;
  int kIndex;
  DateTime now = rtc.now();
  //DateTime nowUnix = rtc.unixtime();

  temp = "";
  temp.concat(now.unixtime());
  temp.concat(" ");
  temp.concat(now.year());
  temp.concat(".") ;
  if (now.month()<10) temp.concat ("0");
  temp.concat(now.month());
  temp.concat(".") ;
  if (now.day()<10) temp.concat ("0");
  temp.concat(now.day());
  temp.concat(" ") ;
  if (now.hour()<10) temp.concat ("0");
  temp.concat(now.hour());
  temp.concat(":") ;
  if (now.minute()<10) temp.concat ("0");
  temp.concat(now.minute());
  temp.concat(":") ;
  if (now.second()<10) temp.concat ("0");
  temp.concat(now.second());  

  //temp = temp.concat(now.year());
  return(temp);
  
}

void ShowHelp (void)
{
  Serial.println ("******************************************");
  Serial.println ("   Available Commands:");
  Serial.println ("     L List files");
  Serial.println ("     D Dump Current File");
  Serial.println ("     S Test the Serial (USB) connection");
  Serial.println ("     F Show file name");
  Serial.println ("     T");
  Serial.println ("     V");
  Serial.println ("     C");
  Serial.println ("******************************************");
  
}
void SyncTimeRTC()
{
  if(Serial.available()) 
    {
     // char inChar = (char)Serial.read();
     String incoming = "";
      while (Serial.available())
      {
         incoming = Serial.readString();
         Serial.println("here");
         Serial.println(incoming);
      if ((incoming[0] == 's') && (incoming[1] == 't'))
        {
         Serial.println("yowza"); 
         String STyear = "";
         String STmonth = "";
         String STday = "";
         String SThour = "";
         String STminute = "";
         String STsecond = "";
         STyear = STyear + incoming[2] + incoming[3]+incoming[4]+incoming[5];
         int STyearInt = STyear.toInt();
         
         STmonth = STmonth + incoming[6] + incoming[7];
         int STmonthInt = STmonth.toInt();

         STday = STday + incoming[8] + incoming[9];
         int STdayInt = STday.toInt();

         SThour = SThour + incoming[10] + incoming[11];
         int SThourInt = SThour.toInt();

         STminute = STminute + incoming[12] + incoming[13];
         int STminuteInt = STminute.toInt();

         STsecond = STsecond + incoming[14] + incoming[15];
         int STsecondInt = STsecond.toInt();
         
         
         //int STyear = incoming[3],.toInt())
         Serial.println(STyearInt); 
         rtc.adjust(DateTime(STyearInt, STmonthInt, STdayInt, SThourInt, STminuteInt, STsecondInt));
         Serial.print ("adjusted");
        }
        
      }
  }
}
