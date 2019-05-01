#include "KX022.h"

void KX022::init(uint8_t range, uint8_t rate)
{
   _range = range;
   
   Wire.begin();

   //uint8_t cntl1 = CNTL1_PC1_STANDBY | CNTL1_RES_HIGHCURRENT | range | CNTL1_DRDYE | CNTL1_TDTE | CNTL1_WUFE | CNTL1_TPE;
   uint8_t cntl1 = CNTL1_PC1_STANDBY | range | CNTL1_DRDYE;    // enkel data ready ints hier
    
   writeRegister(KX022_CNTL1,cntl1);

   writeRegister(KX022_CNTL2,CNTL2_DEFAULT);
   writeRegister(KX022_CNTL3,CNTL3_DEFAULT);
   writeRegister(KX022_ODCNTL,RATE_400_HZ);
   writeRegister(KX022_TILT_TIMER, KX022_TILT_TIMER_VALUE);

   // interrupts, enable all interrupts on PIN1
   writeRegister(KX022_INC1, INC1_IEN_ENABLED | INC1_IEA_ACTIVEHIGH | INC1_IEL_PINLATCHING);
   //writeRegister(KX022_INC2, INC2_DEFAULT);
   //writeRegister(KX022_INC3, INC3_DEFAULT);
   writeRegister(KX022_INC4, INC4_BFI1_ENABLE | INC4_WMI1_ENABLE |
                            INC4_DRDYI1_ENABLE | INC4_TDTI1_ENABLE |
                            INC4_WUFI1_ENABLE | INC4_TPI1_ENABLE);


   // standby -> operating
   writeRegister(KX022_CNTL1, cntl1 | CNTL1_PC1_OPERATING);

   // sds : delay moet na de activatie (datasheet p7)
   delay(20);
   //delay((uint32_t)(1200.0f / KX022_ODR[rate]));

} // init


void KX022::poll(void)
{
    // TODO
}


void KX022::writeRegister(uint8_t regAddress, uint8_t regValue)
{
   Wire.beginTransmission(_i2c_address);
   Wire.write(regAddress);
   Wire.write(regValue);
   Wire.endTransmission();
}


uint8_t KX022::readRegister(uint8_t regAddress)
{
   uint8_t value_ = 0;
   Wire.beginTransmission(_i2c_address);
   Wire.write(regAddress);
   Wire.endTransmission();
   Wire.requestFrom(_i2c_address, 1);
   if (Wire.available())
   {
      value_ = Wire.read();
   }
   return value_;
}


void KX022::getRawXYZ(int16_t (&xyz)[3])
{
   Wire.beginTransmission(_i2c_address);
   Wire.write(DATA_OUT_BASE);
   Wire.endTransmission();
   Wire.requestFrom(_i2c_address, 6);
   if (Wire.available())
   {
      xyz[0] = static_cast<int16_t>(Wire.read() | (Wire.read() << 8));
      xyz[1] = static_cast<int16_t>(Wire.read() | (Wire.read() << 8));
      xyz[2] = static_cast<int16_t>(Wire.read() | (Wire.read() << 8));
   }
}

float KX022::getAccel(uint8_t channelNum)
{
   int16_t measurement =
       static_cast<int16_t>((readRegister(DATA_OUT_BASE + 1 + 2 * channelNum) << 8) |
                            readRegister(DATA_OUT_BASE + 2 * channelNum));
   return static_cast<float>(measurement) / KX022_ACC_SENSITIVITY[_range >> 3];
}

void KX022::getAccelXYZ(float (&xyz)[3])
{
   int16_t xyz_[3] = {0};
   getRawXYZ(xyz_);
   xyz[0] = static_cast<float>(xyz_[0])  / KX022_ACC_SENSITIVITY[_range >> 3];
   xyz[1] = static_cast<float>(xyz_[1])  / KX022_ACC_SENSITIVITY[_range >> 3];
   xyz[2] = static_cast<float>(xyz_[2])  / KX022_ACC_SENSITIVITY[_range >> 3];
}
