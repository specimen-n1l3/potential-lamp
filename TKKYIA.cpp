#include "Arduino.h"
#include "TKKYIA.h"

Ultrasensor::Ultrasensor(byte trigger, byte echo, unsigned short maxdistcm, unsigned long maxtimeoutmicros) {
  this->trigger = trigger;
  this->echo = echo;
  this->maxdistcm = maxdistcm;
  this->maxtimeoutmicros = maxtimeoutmicros;
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
}

float Ultrasensor::getdistcm() {
  return getdistcm(19.307);
}

float Ultrasensor::getdistcm(float tempkelvin) {
  unsigned long maxdistmicros;

  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);

  float vsoundcmmicros = sqrt(1.4 * 8.31 * tempkelvin / 28.97); //need to convert this to m/s

  //25% margin in micros measurememt
  maxdistmicros = 2.5 * maxdistcm / vsoundcmmicros;
  if (maxtimeoutmicros > 0) {
    maxdistmicros = min(maxdistmicros, maxtimeoutmicros);
  }

  unsigned long durationmicros = pulseIn(echo, HIGH, maxdistmicros);

  float distcm = durationmicros / 2.0 * vsoundcmmicros;
  if (distcm == 0 || distcm > maxdistcm) {
    return 0.0;
  } else {
    return distcm;
  }
}

MPU9250::MPU9250() {
    devAddr = MPU9250_DEFAULT_ADDRESS;
}

MPU9250::MPU9250(uint8_t address) {
    devAddr = address;
}

void MPU9250::initialize() {
    setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!
    setClockSource(MPU9250_CLOCK_PLL_XGYRO);
    setFullScaleGyroRange(MPU9250_GYRO_FS_250);
    setFullScaleAccelRange(MPU9250_ACCEL_FS_2);
}

bool MPU9250::testConnection() {
    return getDeviceID() == 0x71;
}

uint8_t MPU9250::getAuxVDDIOLevel() {
    I2Cdev::readBit(devAddr, MPU9250_RA_YG_OFFS_TC, MPU9250_TC_PWR_MODE_BIT, buffer);
    return buffer[0];
}

void MPU9250::setAuxVDDIOLevel(uint8_t level) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_YG_OFFS_TC, MPU9250_TC_PWR_MODE_BIT, level);
}

uint8_t MPU9250::getRate() {
    I2Cdev::readByte(devAddr, MPU9250_RA_SMPLRT_DIV, buffer);
    return buffer[0];
}

void MPU9250::setRate(uint8_t rate) {
    I2Cdev::writeByte(devAddr, MPU9250_RA_SMPLRT_DIV, rate);
}

uint8_t MPU9250::getExternalFrameSync() {
    I2Cdev::readBits(devAddr, MPU9250_RA_CONFIG, MPU9250_CFG_EXT_SYNC_SET_BIT, MPU9250_CFG_EXT_SYNC_SET_LENGTH, buffer);
    return buffer[0];
}

void MPU9250::setExternalFrameSync(uint8_t sync) {
    I2Cdev::writeBits(devAddr, MPU9250_RA_CONFIG, MPU9250_CFG_EXT_SYNC_SET_BIT, MPU9250_CFG_EXT_SYNC_SET_LENGTH, sync);
}

uint8_t MPU9250::getDLPFMode() {
    I2Cdev::readBits(devAddr, MPU9250_RA_CONFIG, MPU9250_CFG_DLPF_CFG_BIT, MPU9250_CFG_DLPF_CFG_LENGTH, buffer);
    return buffer[0];
}

void MPU9250::setDLPFMode(uint8_t mode) {
    I2Cdev::writeBits(devAddr, MPU9250_RA_CONFIG, MPU9250_CFG_DLPF_CFG_BIT, MPU9250_CFG_DLPF_CFG_LENGTH, mode);
}

uint8_t MPU9250::getFullScaleGyroRange() {
    I2Cdev::readBits(devAddr, MPU9250_RA_GYRO_CONFIG, MPU9250_GCONFIG_FS_SEL_BIT, MPU9250_GCONFIG_FS_SEL_LENGTH, buffer);
    return buffer[0];
}

void MPU9250::setFullScaleGyroRange(uint8_t range) {
    I2Cdev::writeBits(devAddr, MPU9250_RA_GYRO_CONFIG, MPU9250_GCONFIG_FS_SEL_BIT, MPU9250_GCONFIG_FS_SEL_LENGTH, range);
}

bool MPU9250::getAccelXSelfTest() {
    I2Cdev::readBit(devAddr, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_XA_ST_BIT, buffer);
    return buffer[0];
}

void MPU9250::setAccelXSelfTest(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_XA_ST_BIT, enabled);
}

bool MPU9250::getAccelYSelfTest() {
    I2Cdev::readBit(devAddr, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_YA_ST_BIT, buffer);
    return buffer[0];
}

void MPU9250::setAccelYSelfTest(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_YA_ST_BIT, enabled);
}

bool MPU9250::getAccelZSelfTest() {
    I2Cdev::readBit(devAddr, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_ZA_ST_BIT, buffer);
    return buffer[0];
}

void MPU9250::setAccelZSelfTest(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_ZA_ST_BIT, enabled);
}

uint8_t MPU9250::getFullScaleAccelRange() {
    I2Cdev::readBits(devAddr, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_AFS_SEL_BIT, MPU9250_ACONFIG_AFS_SEL_LENGTH, buffer);
    return buffer[0];
}

void MPU9250::setFullScaleAccelRange(uint8_t range) {
    I2Cdev::writeBits(devAddr, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_AFS_SEL_BIT, MPU9250_ACONFIG_AFS_SEL_LENGTH, range);
}

uint8_t MPU9250::getDHPFMode() {
    I2Cdev::readBits(devAddr, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_ACCEL_HPF_BIT, MPU9250_ACONFIG_ACCEL_HPF_LENGTH, buffer);
    return buffer[0];
}

void MPU9250::setDHPFMode(uint8_t bandwidth) {
    I2Cdev::writeBits(devAddr, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_ACCEL_HPF_BIT, MPU9250_ACONFIG_ACCEL_HPF_LENGTH, bandwidth);
}

uint8_t MPU9250::getFreefallDetectionThreshold() {
    I2Cdev::readByte(devAddr, MPU9250_RA_FF_THR, buffer);
    return buffer[0];
}

void MPU9250::setFreefallDetectionThreshold(uint8_t threshold) {
    I2Cdev::writeByte(devAddr, MPU9250_RA_FF_THR, threshold);
}

uint8_t MPU9250::getFreefallDetectionDuration() {
    I2Cdev::readByte(devAddr, MPU9250_RA_FF_DUR, buffer);
    return buffer[0];
}

uint8_t MPU9250::getMotionDetectionThreshold() {
    I2Cdev::readByte(devAddr, MPU9250_RA_MOT_THR, buffer);
    return buffer[0];
}

void MPU9250::setMotionDetectionThreshold(uint8_t threshold) {
    I2Cdev::writeByte(devAddr, MPU9250_RA_MOT_THR, threshold);
}

uint8_t MPU9250::getMotionDetectionDuration() {
    I2Cdev::readByte(devAddr, MPU9250_RA_MOT_DUR, buffer);
    return buffer[0];
}

void MPU9250::setMotionDetectionDuration(uint8_t duration) {
    I2Cdev::writeByte(devAddr, MPU9250_RA_MOT_DUR, duration);
}

uint8_t MPU9250::getZeroMotionDetectionThreshold() {
    I2Cdev::readByte(devAddr, MPU9250_RA_ZRMOT_THR, buffer);
    return buffer[0];
}

void MPU9250::setZeroMotionDetectionThreshold(uint8_t threshold) {
    I2Cdev::writeByte(devAddr, MPU9250_RA_ZRMOT_THR, threshold);
}

uint8_t MPU9250::getZeroMotionDetectionDuration() {
    I2Cdev::readByte(devAddr, MPU9250_RA_ZRMOT_DUR, buffer);
    return buffer[0];
}

void MPU9250::setZeroMotionDetectionDuration(uint8_t duration) {
    I2Cdev::writeByte(devAddr, MPU9250_RA_ZRMOT_DUR, duration);
}

bool MPU9250::getTempFIFOEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_TEMP_FIFO_EN_BIT, buffer);
    return buffer[0];
}

void MPU9250::setTempFIFOEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_TEMP_FIFO_EN_BIT, enabled);
}

bool MPU9250::getAccelFIFOEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_ACCEL_FIFO_EN_BIT, buffer);
    return buffer[0];
}

void MPU9250::setAccelFIFOEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_ACCEL_FIFO_EN_BIT, enabled);
}

bool MPU9250::getSlave2FIFOEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_SLV2_FIFO_EN_BIT, buffer);
    return buffer[0];
}

void MPU9250::setSlave2FIFOEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_SLV2_FIFO_EN_BIT, enabled);
}

bool MPU9250::getSlave1FIFOEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_SLV1_FIFO_EN_BIT, buffer);
    return buffer[0];
}

void MPU9250::setSlave1FIFOEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_SLV1_FIFO_EN_BIT, enabled);
}

bool MPU9250::getSlave0FIFOEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_SLV0_FIFO_EN_BIT, buffer);
    return buffer[0];
}

void MPU9250::setSlave0FIFOEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_SLV0_FIFO_EN_BIT, enabled);
}

bool MPU9250::getMultiMasterEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_I2C_MST_CTRL, MPU9250_MULT_MST_EN_BIT, buffer);
    return buffer[0];
}

void MPU9250::setMultiMasterEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_I2C_MST_CTRL, MPU9250_MULT_MST_EN_BIT, enabled);
}

bool MPU9250::getWaitForExternalSensorEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_I2C_MST_CTRL, MPU9250_WAIT_FOR_ES_BIT, buffer);
    return buffer[0];
}

void MPU9250::setWaitForExternalSensorEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_I2C_MST_CTRL, MPU9250_WAIT_FOR_ES_BIT, enabled);
}

bool MPU9250::getSlave3FIFOEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_I2C_MST_CTRL, MPU9250_SLV_3_FIFO_EN_BIT, buffer);
    return buffer[0];
}

void MPU9250::setSlave3FIFOEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_I2C_MST_CTRL, MPU9250_SLV_3_FIFO_EN_BIT, enabled);
}

bool MPU9250::getSlaveReadWriteTransitionEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_I2C_MST_CTRL, MPU9250_I2C_MST_P_NSR_BIT, buffer);
    return buffer[0];
}

void MPU9250::setSlaveReadWriteTransitionEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_I2C_MST_CTRL, MPU9250_I2C_MST_P_NSR_BIT, enabled);
}

uint8_t MPU9250::getMasterClockSpeed() {
    I2Cdev::readBits(devAddr, MPU9250_RA_I2C_MST_CTRL, MPU9250_I2C_MST_CLK_BIT, MPU9250_I2C_MST_CLK_LENGTH, buffer);
    return buffer[0];
}

void MPU9250::setMasterClockSpeed(uint8_t speed) {
    I2Cdev::writeBits(devAddr, MPU9250_RA_I2C_MST_CTRL, MPU9250_I2C_MST_CLK_BIT, MPU9250_I2C_MST_CLK_LENGTH, speed);
}

uint8_t MPU9250::getSlaveAddress(uint8_t num) {
    if (num > 3) return 0;
    I2Cdev::readByte(devAddr, MPU9250_RA_I2C_SLV0_ADDR + num*3, buffer);
    return buffer[0];
}

void MPU9250::setSlaveAddress(uint8_t num, uint8_t address) {
    if (num > 3) return;
    I2Cdev::writeByte(devAddr, MPU9250_RA_I2C_SLV0_ADDR + num*3, address);
}

uint8_t MPU9250::getSlaveRegister(uint8_t num) {
    if (num > 3) return 0;
    I2Cdev::readByte(devAddr, MPU9250_RA_I2C_SLV0_REG + num*3, buffer);
    return buffer[0];
}

void MPU9250::setSlaveRegister(uint8_t num, uint8_t reg) {
    if (num > 3) return;
    I2Cdev::writeByte(devAddr, MPU9250_RA_I2C_SLV0_REG + num*3, reg);
}

bool MPU9250::getSlaveEnabled(uint8_t num) {
    if (num > 3) return 0;
    I2Cdev::readBit(devAddr, MPU9250_RA_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV_EN_BIT, buffer);
    return buffer[0];
}

void MPU9250::setSlaveEnabled(uint8_t num, bool enabled) {
    if (num > 3) return;
    I2Cdev::writeBit(devAddr, MPU9250_RA_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV_EN_BIT, enabled);
}

bool MPU9250::getSlaveWordByteSwap(uint8_t num) {
    if (num > 3) return 0;
    I2Cdev::readBit(devAddr, MPU9250_RA_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV_BYTE_SW_BIT, buffer);
    return buffer[0];
}

void MPU9250::setSlaveWordByteSwap(uint8_t num, bool enabled) {
    if (num > 3) return;
    I2Cdev::writeBit(devAddr, MPU9250_RA_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV_BYTE_SW_BIT, enabled);
}

bool MPU9250::getSlaveWriteMode(uint8_t num) {
    if (num > 3) return 0;
    I2Cdev::readBit(devAddr, MPU9250_RA_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV_REG_DIS_BIT, buffer);
    return buffer[0];
}

void MPU9250::setSlaveWriteMode(uint8_t num, bool mode) {
    if (num > 3) return;
    I2Cdev::writeBit(devAddr, MPU9250_RA_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV_REG_DIS_BIT, mode);
}

bool MPU9250::getSlaveWordGroupOffset(uint8_t num) {
    if (num > 3) return 0;
    I2Cdev::readBit(devAddr, MPU9250_RA_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV_GRP_BIT, buffer);
    return buffer[0];
}

void MPU9250::setSlaveWordGroupOffset(uint8_t num, bool enabled) {
    if (num > 3) return;
    I2Cdev::writeBit(devAddr, MPU9250_RA_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV_GRP_BIT, enabled);
}

uint8_t MPU9250::getSlaveDataLength(uint8_t num) {
    if (num > 3) return 0;
    I2Cdev::readBits(devAddr, MPU9250_RA_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV_LEN_BIT, MPU9250_I2C_SLV_LEN_LENGTH, buffer);
    return buffer[0];
}

void MPU9250::setSlaveDataLength(uint8_t num, uint8_t length) {
    if (num > 3) return;
    I2Cdev::writeBits(devAddr, MPU9250_RA_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV_LEN_BIT, MPU9250_I2C_SLV_LEN_LENGTH, length);
}

uint8_t MPU9250::getSlave4Address() {
    I2Cdev::readByte(devAddr, MPU9250_RA_I2C_SLV4_ADDR, buffer);
    return buffer[0];
}

void MPU9250::setSlave4Address(uint8_t address) {
    I2Cdev::writeByte(devAddr, MPU9250_RA_I2C_SLV4_ADDR, address);
}

uint8_t MPU9250::getSlave4Register() {
    I2Cdev::readByte(devAddr, MPU9250_RA_I2C_SLV4_REG, buffer);
    return buffer[0];
}

void MPU9250::setSlave4Register(uint8_t reg) {
    I2Cdev::writeByte(devAddr, MPU9250_RA_I2C_SLV4_REG, reg);
}

void MPU9250::setSlave4OutputByte(uint8_t data) {
    I2Cdev::writeByte(devAddr, MPU9250_RA_I2C_SLV4_DO, data);
}

bool MPU9250::getSlave4Enabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_EN_BIT, buffer);
    return buffer[0];
}

void MPU9250::setSlave4Enabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_EN_BIT, enabled);
}

bool MPU9250::getSlave4InterruptEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_INT_EN_BIT, buffer);
    return buffer[0];
}

void MPU9250::setSlave4InterruptEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_INT_EN_BIT, enabled);
}

bool MPU9250::getSlave4WriteMode() {
    I2Cdev::readBit(devAddr, MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_REG_DIS_BIT, buffer);
    return buffer[0];
}

void MPU9250::setSlave4WriteMode(bool mode) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_REG_DIS_BIT, mode);
}

uint8_t MPU9250::getSlave4MasterDelay() {
    I2Cdev::readBits(devAddr, MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_MST_DLY_BIT, MPU9250_I2C_SLV4_MST_DLY_LENGTH, buffer);
    return buffer[0];
}

void MPU9250::setSlave4MasterDelay(uint8_t delay) {
    I2Cdev::writeBits(devAddr, MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_MST_DLY_BIT, MPU9250_I2C_SLV4_MST_DLY_LENGTH, delay);
}

uint8_t MPU9250::getSlate4InputByte() {
    I2Cdev::readByte(devAddr, MPU9250_RA_I2C_SLV4_DI, buffer);
    return buffer[0];
}

bool MPU9250::getPassthroughStatus() {
    I2Cdev::readBit(devAddr, MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_PASS_THROUGH_BIT, buffer);
    return buffer[0];
}

bool MPU9250::getSlave4IsDone() {
    I2Cdev::readBit(devAddr, MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_I2C_SLV4_DONE_BIT, buffer);
    return buffer[0];
}

bool MPU9250::getLostArbitration() {
    I2Cdev::readBit(devAddr, MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_I2C_LOST_ARB_BIT, buffer);
    return buffer[0];
}

bool MPU9250::getSlave4Nack() {
    I2Cdev::readBit(devAddr, MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_I2C_SLV4_NACK_BIT, buffer);
    return buffer[0];
}

bool MPU9250::getSlave3Nack() {
    I2Cdev::readBit(devAddr, MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_I2C_SLV3_NACK_BIT, buffer);
    return buffer[0];
}

bool MPU9250::getSlave2Nack() {
    I2Cdev::readBit(devAddr, MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_I2C_SLV2_NACK_BIT, buffer);
    return buffer[0];
}

bool MPU9250::getSlave1Nack() {
    I2Cdev::readBit(devAddr, MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_I2C_SLV1_NACK_BIT, buffer);
    return buffer[0];
}

bool MPU9250::getSlave0Nack() {
    I2Cdev::readBit(devAddr, MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_I2C_SLV0_NACK_BIT, buffer);
    return buffer[0];
}

bool MPU9250::getInterruptMode() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_INT_LEVEL_BIT, buffer);
    return buffer[0];
}

void MPU9250::setInterruptMode(bool mode) {
   I2Cdev::writeBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_INT_LEVEL_BIT, mode);
}

bool MPU9250::getInterruptDrive() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_INT_OPEN_BIT, buffer);
    return buffer[0];
}

void MPU9250::setInterruptDrive(bool drive) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_INT_OPEN_BIT, drive);
}

bool MPU9250::getInterruptLatch() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_LATCH_INT_EN_BIT, buffer);
    return buffer[0];
}

void MPU9250::setInterruptLatch(bool latch) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_LATCH_INT_EN_BIT, latch);
}

bool MPU9250::getInterruptLatchClear() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_INT_RD_CLEAR_BIT, buffer);
    return buffer[0];
}

void MPU9250::setInterruptLatchClear(bool clear) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_INT_RD_CLEAR_BIT, clear);
}

bool MPU9250::getFSyncInterruptLevel() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_FSYNC_INT_LEVEL_BIT, buffer);
    return buffer[0];
}

void MPU9250::setFSyncInterruptLevel(bool level) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_FSYNC_INT_LEVEL_BIT, level);
}

bool MPU9250::getFSyncInterruptEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_FSYNC_INT_EN_BIT, buffer);
    return buffer[0];
}

void MPU9250::setFSyncInterruptEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_FSYNC_INT_EN_BIT, enabled);
}

bool MPU9250::getI2CBypassEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_I2C_BYPASS_EN_BIT, buffer);
    return buffer[0];
}

void MPU9250::setI2CBypassEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

bool MPU9250::getClockOutputEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_CLKOUT_EN_BIT, buffer);
    return buffer[0];
}

void MPU9250::setClockOutputEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_CLKOUT_EN_BIT, enabled);
}

uint8_t MPU9250::getIntEnabled() {
    I2Cdev::readByte(devAddr, MPU9250_RA_INT_ENABLE, buffer);
    return buffer[0];
}

void MPU9250::setIntEnabled(uint8_t enabled) {
    I2Cdev::writeByte(devAddr, MPU9250_RA_INT_ENABLE, enabled);
}

bool MPU9250::getIntFreefallEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_FF_BIT, buffer);
    return buffer[0];
}

void MPU9250::setIntFreefallEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_FF_BIT, enabled);
}

bool MPU9250::getIntMotionEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_MOT_BIT, buffer);
    return buffer[0];
}

void MPU9250::setIntMotionEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_MOT_BIT, enabled);
}

bool MPU9250::getIntZeroMotionEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_ZMOT_BIT, buffer);
    return buffer[0];
}

void MPU9250::setIntZeroMotionEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_ZMOT_BIT, enabled);
}

bool MPU9250::getIntFIFOBufferOverflowEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_FIFO_OFLOW_BIT, buffer);
    return buffer[0];
}

void MPU9250::setIntFIFOBufferOverflowEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_FIFO_OFLOW_BIT, enabled);
}

bool MPU9250::getIntI2CMasterEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_I2C_MST_INT_BIT, buffer);
    return buffer[0];
}

void MPU9250::setIntI2CMasterEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_I2C_MST_INT_BIT, enabled);
}

bool MPU9250::getIntDataReadyEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_DATA_RDY_BIT, buffer);
    return buffer[0];
}

void MPU9250::setIntDataReadyEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_DATA_RDY_BIT, enabled);
}

uint8_t MPU9250::getIntStatus() {
    I2Cdev::readByte(devAddr, MPU9250_RA_INT_STATUS, buffer);
    return buffer[0];
}

bool MPU9250::getIntFreefallStatus() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_FF_BIT, buffer);
    return buffer[0];
}

bool MPU9250::getIntMotionStatus() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_MOT_BIT, buffer);
    return buffer[0];
}

bool MPU9250::getIntZeroMotionStatus() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_ZMOT_BIT, buffer);
    return buffer[0];
}

bool MPU9250::getIntFIFOBufferOverflowStatus() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_FIFO_OFLOW_BIT, buffer);
    return buffer[0];
}

bool MPU9250::getIntI2CMasterStatus() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_I2C_MST_INT_BIT, buffer);
    return buffer[0];
}

bool MPU9250::getIntDataReadyStatus() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_DATA_RDY_BIT, buffer);
    return buffer[0];
}

void MPU9250::getAcceleration(int16_t* x, int16_t* y, int16_t* z) {
    I2Cdev::readBytes(devAddr, MPU9250_RA_ACCEL_XOUT_H, 6, buffer);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}

int16_t MPU9250::getTemperature() {
    I2Cdev::readBytes(devAddr, MPU9250_RA_TEMP_OUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

uint8_t MPU9250::getExternalSensorByte(int position) {
    I2Cdev::readByte(devAddr, MPU9250_RA_EXT_SENS_DATA_00 + position, buffer);
    return buffer[0];
}

uint16_t MPU9250::getExternalSensorWord(int position) {
    I2Cdev::readBytes(devAddr, MPU9250_RA_EXT_SENS_DATA_00 + position, 2, buffer);
    return (((uint16_t)buffer[0]) << 8) | buffer[1];
}

uint32_t MPU9250::getExternalSensorDWord(int position) {
    I2Cdev::readBytes(devAddr, MPU9250_RA_EXT_SENS_DATA_00 + position, 4, buffer);
    return (((uint32_t)buffer[0]) << 24) | (((uint32_t)buffer[1]) << 16) | (((uint16_t)buffer[2]) << 8) | buffer[3];
}

bool MPU9250::getXNegMotionDetected() {
    I2Cdev::readBit(devAddr, MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_XNEG_BIT, buffer);
    return buffer[0];
}

bool MPU9250::getXPosMotionDetected() {
    I2Cdev::readBit(devAddr, MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_XPOS_BIT, buffer);
    return buffer[0];
}

bool MPU9250::getYNegMotionDetected() {
    I2Cdev::readBit(devAddr, MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_YNEG_BIT, buffer);
    return buffer[0];
}

bool MPU9250::getYPosMotionDetected() {
    I2Cdev::readBit(devAddr, MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_YPOS_BIT, buffer);
    return buffer[0];
}

bool MPU9250::getZNegMotionDetected() {
    I2Cdev::readBit(devAddr, MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_ZNEG_BIT, buffer);
    return buffer[0];
}

bool MPU9250::getZPosMotionDetected() {
    I2Cdev::readBit(devAddr, MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_ZPOS_BIT, buffer);
    return buffer[0];
}

bool MPU9250::getZeroMotionDetected() {
    I2Cdev::readBit(devAddr, MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_ZRMOT_BIT, buffer);
    return buffer[0];
}

void MPU9250::setSlaveOutputByte(uint8_t num, uint8_t data) {
    if (num > 3) return;
    I2Cdev::writeByte(devAddr, MPU9250_RA_I2C_SLV0_DO + num, data);
}

bool MPU9250::getExternalShadowDelayEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_I2C_MST_DELAY_CTRL, MPU9250_DELAYCTRL_DELAY_ES_SHADOW_BIT, buffer);
    return buffer[0];
}

void MPU9250::setExternalShadowDelayEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_I2C_MST_DELAY_CTRL, MPU9250_DELAYCTRL_DELAY_ES_SHADOW_BIT, enabled);
}

bool MPU9250::getSlaveDelayEnabled(uint8_t num) {
    // MPU9250_DELAYCTRL_I2C_SLV4_DLY_EN_BIT is 4, SLV3 is 3, etc.
    if (num > 4) return 0;
    I2Cdev::readBit(devAddr, MPU9250_RA_I2C_MST_DELAY_CTRL, num, buffer);
    return buffer[0];
}

void MPU9250::setSlaveDelayEnabled(uint8_t num, bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_I2C_MST_DELAY_CTRL, num, enabled);
}

void MPU9250::resetAccelerometerPath() {
    I2Cdev::writeBit(devAddr, MPU9250_RA_SIGNAL_PATH_RESET, MPU9250_PATHRESET_ACCEL_RESET_BIT, true);
}

void MPU9250::resetTemperaturePath() {
    I2Cdev::writeBit(devAddr, MPU9250_RA_SIGNAL_PATH_RESET, MPU9250_PATHRESET_TEMP_RESET_BIT, true);
}

uint8_t MPU9250::getAccelerometerPowerOnDelay() {
    I2Cdev::readBits(devAddr, MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_ACCEL_ON_DELAY_BIT, MPU9250_DETECT_ACCEL_ON_DELAY_LENGTH, buffer);
    return buffer[0];
}

void MPU9250::setAccelerometerPowerOnDelay(uint8_t delay) {
    I2Cdev::writeBits(devAddr, MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_ACCEL_ON_DELAY_BIT, MPU9250_DETECT_ACCEL_ON_DELAY_LENGTH, delay);
}

uint8_t MPU9250::getMotionDetectionCounterDecrement() {
    I2Cdev::readBits(devAddr, MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_MOT_COUNT_BIT, MPU9250_DETECT_MOT_COUNT_LENGTH, buffer);
    return buffer[0];
}

void MPU9250::setMotionDetectionCounterDecrement(uint8_t decrement) {
    I2Cdev::writeBits(devAddr, MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_MOT_COUNT_BIT, MPU9250_DETECT_MOT_COUNT_LENGTH, decrement);
}

bool MPU9250::getFIFOEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_FIFO_EN_BIT, buffer);
    return buffer[0];
}

void MPU9250::setFIFOEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_FIFO_EN_BIT, enabled);
}

bool MPU9250::getI2CMasterModeEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_I2C_MST_EN_BIT, buffer);
    return buffer[0];
}

void MPU9250::setI2CMasterModeEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_I2C_MST_EN_BIT, enabled);
}

void MPU9250::switchSPIEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_I2C_IF_DIS_BIT, enabled);
}

void MPU9250::resetFIFO() {
    I2Cdev::writeBit(devAddr, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_FIFO_RESET_BIT, true);
}

void MPU9250::resetI2CMaster() {
    I2Cdev::writeBit(devAddr, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_I2C_MST_RESET_BIT, true);
}

void MPU9250::resetSensors() {
    I2Cdev::writeBit(devAddr, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_SIG_COND_RESET_BIT, true);
}

void MPU9250::reset() {
    I2Cdev::writeBit(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_DEVICE_RESET_BIT, true);
}

bool MPU9250::getSleepEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_SLEEP_BIT, buffer);
    return buffer[0];
}

void MPU9250::setSleepEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_SLEEP_BIT, enabled);
}

bool MPU9250::getWakeCycleEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_CYCLE_BIT, buffer);
    return buffer[0];
}

void MPU9250::setWakeCycleEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_CYCLE_BIT, enabled);
}

bool MPU9250::getTempSensorEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_TEMP_DIS_BIT, buffer);
    return buffer[0] == 0; // 1 is actually disabled here
}

void MPU9250::setTempSensorEnabled(bool enabled) {
    // 1 is actually disabled here
    I2Cdev::writeBit(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_TEMP_DIS_BIT, !enabled);
}

uint8_t MPU9250::getClockSource() {
    I2Cdev::readBits(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_CLKSEL_BIT, MPU9250_PWR1_CLKSEL_LENGTH, buffer);
    return buffer[0];
}

void MPU9250::setClockSource(uint8_t source) {
    I2Cdev::writeBits(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_CLKSEL_BIT, MPU9250_PWR1_CLKSEL_LENGTH, source);
}

uint8_t MPU9250::getWakeFrequency() {
    I2Cdev::readBits(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_LP_WAKE_CTRL_BIT, MPU9250_PWR2_LP_WAKE_CTRL_LENGTH, buffer);
    return buffer[0];
}

void MPU9250::setWakeFrequency(uint8_t frequency) {
    I2Cdev::writeBits(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_LP_WAKE_CTRL_BIT, MPU9250_PWR2_LP_WAKE_CTRL_LENGTH, frequency);
}

bool MPU9250::getStandbyXAccelEnabled() {
        I2Cdev::readBit(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_XA_BIT, buffer);
        return buffer[0];
}

void MPU9250::setStandbyXAccelEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_XA_BIT, enabled);
}

bool MPU9250::getStandbyYAccelEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_YA_BIT, buffer);
    return buffer[0];
}

void MPU9250::setStandbyYAccelEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_YA_BIT, enabled);
}

bool MPU9250::getStandbyZAccelEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_ZA_BIT, buffer);
    return buffer[0];
}

void MPU9250::setStandbyZAccelEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_ZA_BIT, enabled);
}

uint16_t MPU9250::getFIFOCount() {
    I2Cdev::readBytes(devAddr, MPU9250_RA_FIFO_COUNTH, 2, buffer);
    return (((uint16_t)buffer[0]) << 8) | buffer[1];
}

uint8_t MPU9250::getFIFOByte() {
    I2Cdev::readByte(devAddr, MPU9250_RA_FIFO_R_W, buffer);
    return buffer[0];
}
void MPU9250::getFIFOBytes(uint8_t *data, uint8_t length) {
    I2Cdev::readBytes(devAddr, MPU9250_RA_FIFO_R_W, length, data);
}

void MPU9250::setFIFOByte(uint8_t data) {
    I2Cdev::writeByte(devAddr, MPU9250_RA_FIFO_R_W, data);
}

uint8_t MPU9250::getDeviceID() {
//	#define MPU9250_RA_WHO_AM_I         0x75
// 	#define MPU9250_WHO_AM_I_BIT        6
//	#define MPU9250_WHO_AM_I_LENGTH     8
//    I2Cdev::readBits(devAddr, MPU9250_RA_WHO_AM_I, MPU9250_WHO_AM_I_BIT, MPU9250_WHO_AM_I_LENGTH, buffer);
//    return buffer[0];
    I2Cdev::readByte(devAddr, MPU9250_RA_WHO_AM_I, buffer);
    return buffer[0];
}

void MPU9250::setDeviceID(uint8_t id) {
    I2Cdev::writeBits(devAddr, MPU9250_RA_WHO_AM_I, MPU9250_WHO_AM_I_BIT, MPU9250_WHO_AM_I_LENGTH, id);
}

uint8_t MPU9250::getOTPBankValid() {
    I2Cdev::readBit(devAddr, MPU9250_RA_XG_OFFS_TC, MPU9250_TC_OTP_BNK_VLD_BIT, buffer);
    return buffer[0];
}
void MPU9250::setOTPBankValid(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_XG_OFFS_TC, MPU9250_TC_OTP_BNK_VLD_BIT, enabled);
}
int8_t MPU9250::getXGyroOffset() {
    I2Cdev::readBits(devAddr, MPU9250_RA_XG_OFFS_TC, MPU9250_TC_OFFSET_BIT, MPU9250_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}
void MPU9250::setXGyroOffset(int8_t offset) {
    I2Cdev::writeBits(devAddr, MPU9250_RA_XG_OFFS_TC, MPU9250_TC_OFFSET_BIT, MPU9250_TC_OFFSET_LENGTH, offset);
}

int8_t MPU9250::getYGyroOffset() {
    I2Cdev::readBits(devAddr, MPU9250_RA_YG_OFFS_TC, MPU9250_TC_OFFSET_BIT, MPU9250_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}
void MPU9250::setYGyroOffset(int8_t offset) {
    I2Cdev::writeBits(devAddr, MPU9250_RA_YG_OFFS_TC, MPU9250_TC_OFFSET_BIT, MPU9250_TC_OFFSET_LENGTH, offset);
}

// ZG_OFFS_TC register

int8_t MPU9250::getZGyroOffset() {
    I2Cdev::readBits(devAddr, MPU9250_RA_ZG_OFFS_TC, MPU9250_TC_OFFSET_BIT, MPU9250_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}
void MPU9250::setZGyroOffset(int8_t offset) {
    I2Cdev::writeBits(devAddr, MPU9250_RA_ZG_OFFS_TC, MPU9250_TC_OFFSET_BIT, MPU9250_TC_OFFSET_LENGTH, offset);
}

int8_t MPU9250::getXFineGain() {
    I2Cdev::readByte(devAddr, MPU9250_RA_X_FINE_GAIN, buffer);
    return buffer[0];
}
void MPU9250::setXFineGain(int8_t gain) {
    I2Cdev::writeByte(devAddr, MPU9250_RA_X_FINE_GAIN, gain);
}

// Y_FINE_GAIN register

int8_t MPU9250::getYFineGain() {
    I2Cdev::readByte(devAddr, MPU9250_RA_Y_FINE_GAIN, buffer);
    return buffer[0];
}
void MPU9250::setYFineGain(int8_t gain) {
    I2Cdev::writeByte(devAddr, MPU9250_RA_Y_FINE_GAIN, gain);
}

int8_t MPU9250::getZFineGain() {
    I2Cdev::readByte(devAddr, MPU9250_RA_Z_FINE_GAIN, buffer);
    return buffer[0];
}
void MPU9250::setZFineGain(int8_t gain) {
    I2Cdev::writeByte(devAddr, MPU9250_RA_Z_FINE_GAIN, gain);
}

// XA_OFFS_* registers

int16_t MPU9250::getXAccelOffset() {
    I2Cdev::readBytes(devAddr, MPU9250_RA_XA_OFFS_H, 2, buffer);
    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}
void MPU9250::setXAccelOffset(int16_t offset) {
    int16_t bit0 = getXAccelOffset() & 1;
    int16_t value = offset & 0xFFFE | bit0;
    
    I2Cdev::writeWord(devAddr, MPU9250_RA_XA_OFFS_H, value);
}

int16_t MPU9250::getYAccelOffset() {
    I2Cdev::readBytes(devAddr, MPU9250_RA_YA_OFFS_H, 2, buffer);
    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}
void MPU9250::setYAccelOffset(int16_t offset) {
    int16_t bit0 = getYAccelOffset() & 1;
    int16_t value = offset & 0xFFFE | bit0;

    I2Cdev::writeWord(devAddr, MPU9250_RA_YA_OFFS_H, value);
}

int16_t MPU9250::getZAccelOffset() {
    I2Cdev::readBytes(devAddr, MPU9250_RA_ZA_OFFS_H, 2, buffer);
    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}
void MPU9250::setZAccelOffset(int16_t offset) {
    int16_t bit0 = getZAccelOffset() & 1;
    int16_t value = offset & 0xFFFE | bit0;

    I2Cdev::writeWord(devAddr, MPU9250_RA_ZA_OFFS_H, value);
}

int16_t MPU9250::getXGyroOffsetUser() {
    I2Cdev::readBytes(devAddr, MPU9250_RA_XG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU9250::setXGyroOffsetUser(int16_t offset) {
    I2Cdev::writeWord(devAddr, MPU9250_RA_XG_OFFS_USRH, offset);
}

int16_t MPU9250::getYGyroOffsetUser() {
    I2Cdev::readBytes(devAddr, MPU9250_RA_YG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU9250::setYGyroOffsetUser(int16_t offset) {
    I2Cdev::writeWord(devAddr, MPU9250_RA_YG_OFFS_USRH, offset);
}

int16_t MPU9250::getZGyroOffsetUser() {
    I2Cdev::readBytes(devAddr, MPU9250_RA_ZG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU9250::setZGyroOffsetUser(int16_t offset) {
    I2Cdev::writeWord(devAddr, MPU9250_RA_ZG_OFFS_USRH, offset);
}

bool MPU9250::getIntPLLReadyEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_PLL_RDY_INT_BIT, buffer);
    return buffer[0];
}
void MPU9250::setIntPLLReadyEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_PLL_RDY_INT_BIT, enabled);
}
bool MPU9250::getIntDMPEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_DMP_INT_BIT, buffer);
    return buffer[0];
}
void MPU9250::setIntDMPEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_DMP_INT_BIT, enabled);
}

bool MPU9250::getDMPInt5Status() {
    I2Cdev::readBit(devAddr, MPU9250_RA_DMP_INT_STATUS, MPU9250_DMPINT_5_BIT, buffer);
    return buffer[0];
}
bool MPU9250::getDMPInt4Status() {
    I2Cdev::readBit(devAddr, MPU9250_RA_DMP_INT_STATUS, MPU9250_DMPINT_4_BIT, buffer);
    return buffer[0];
}
bool MPU9250::getDMPInt3Status() {
    I2Cdev::readBit(devAddr, MPU9250_RA_DMP_INT_STATUS, MPU9250_DMPINT_3_BIT, buffer);
    return buffer[0];
}
bool MPU9250::getDMPInt2Status() {
    I2Cdev::readBit(devAddr, MPU9250_RA_DMP_INT_STATUS, MPU9250_DMPINT_2_BIT, buffer);
    return buffer[0];
}
bool MPU9250::getDMPInt1Status() {
    I2Cdev::readBit(devAddr, MPU9250_RA_DMP_INT_STATUS, MPU9250_DMPINT_1_BIT, buffer);
    return buffer[0];
}
bool MPU9250::getDMPInt0Status() {
    I2Cdev::readBit(devAddr, MPU9250_RA_DMP_INT_STATUS, MPU9250_DMPINT_0_BIT, buffer);
    return buffer[0];
}

bool MPU9250::getIntPLLReadyStatus() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_PLL_RDY_INT_BIT, buffer);
    return buffer[0];
}
bool MPU9250::getIntDMPStatus() {
    I2Cdev::readBit(devAddr, MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_DMP_INT_BIT, buffer);
    return buffer[0];
}

bool MPU9250::getDMPEnabled() {
    I2Cdev::readBit(devAddr, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_DMP_EN_BIT, buffer);
    return buffer[0];
}
void MPU9250::setDMPEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_DMP_EN_BIT, enabled);
}
void MPU9250::resetDMP() {
    I2Cdev::writeBit(devAddr, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_DMP_RESET_BIT, true);
}

void MPU9250::setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank) {
    bank &= 0x1F;
    if (userBank) bank |= 0x20;
    if (prefetchEnabled) bank |= 0x40;
    I2Cdev::writeByte(devAddr, MPU9250_RA_BANK_SEL, bank);
}

void MPU9250::setMemoryStartAddress(uint8_t address) {
    I2Cdev::writeByte(devAddr, MPU9250_RA_MEM_START_ADDR, address);
}

uint8_t MPU9250::readMemoryByte() {
    I2Cdev::readByte(devAddr, MPU9250_RA_MEM_R_W, buffer);
    return buffer[0];
}
void MPU9250::writeMemoryByte(uint8_t data) {
    I2Cdev::writeByte(devAddr, MPU9250_RA_MEM_R_W, data);
}
void MPU9250::readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address) {
    setMemoryBank(bank);
    setMemoryStartAddress(address);
    uint8_t chunkSize;
    for (uint16_t i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU9250_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        // read the chunk of data as specified
        I2Cdev::readBytes(devAddr, MPU9250_RA_MEM_R_W, chunkSize, data + i);
        
        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            setMemoryBank(bank);
            setMemoryStartAddress(address);
        }
    }
}
bool MPU9250::writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem) {
    setMemoryBank(bank);
    setMemoryStartAddress(address);
    uint8_t chunkSize;
    uint8_t *verifyBuffer;
    uint8_t *progBuffer;
    uint16_t i;
    uint8_t j;
    if (verify) verifyBuffer = (uint8_t *)malloc(MPU9250_DMP_MEMORY_CHUNK_SIZE);
    if (useProgMem) progBuffer = (uint8_t *)malloc(MPU9250_DMP_MEMORY_CHUNK_SIZE);
    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU9250_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;
        
        if (useProgMem) {
            // write the chunk of data as specified
            for (j = 0; j < chunkSize; j++) progBuffer[j] = pgm_read_byte(data + i + j);
        } else {
            // write the chunk of data as specified
            progBuffer = (uint8_t *)data + i;
        }

        I2Cdev::writeBytes(devAddr, MPU9250_RA_MEM_R_W, chunkSize, progBuffer);

        // verify data if needed
        if (verify && verifyBuffer) {
            setMemoryBank(bank);
            setMemoryStartAddress(address);
            I2Cdev::readBytes(devAddr, MPU9250_RA_MEM_R_W, chunkSize, verifyBuffer);
            if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
                /*Serial.print("Block write verification error, bank ");
                Serial.print(bank, DEC);
                Serial.print(", address ");
                Serial.print(address, DEC);
                Serial.print("!\nExpected:");
                for (j = 0; j < chunkSize; j++) {
                    Serial.print(" 0x");
                    if (progBuffer[j] < 16) Serial.print("0");
                    Serial.print(progBuffer[j], HEX);
                }
                Serial.print("\nReceived:");
                for (uint8_t j = 0; j < chunkSize; j++) {
                    Serial.print(" 0x");
                    if (verifyBuffer[i + j] < 16) Serial.print("0");
                    Serial.print(verifyBuffer[i + j], HEX);
                }
                Serial.print("\n");*/
                free(verifyBuffer);
                if (useProgMem) free(progBuffer);
                return false; // uh oh.
            }
        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            setMemoryBank(bank);
            setMemoryStartAddress(address);
        }
    }
    if (verify) free(verifyBuffer);
    if (useProgMem) free(progBuffer);
    return true;
}
bool MPU9250::writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify) {
    return writeMemoryBlock(data, dataSize, bank, address, verify, true);
}
bool MPU9250::writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, bool useProgMem) {
    uint8_t *progBuffer, success, special;
    uint16_t i, j;
    if (useProgMem) {
        progBuffer = (uint8_t *)malloc(8); // assume 8-byte blocks, realloc later if necessary
    }

    // config set data is a long string of blocks with the following structure:
    // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
    uint8_t bank, offset, length;
    for (i = 0; i < dataSize;) {
        if (useProgMem) {
            bank = pgm_read_byte(data + i++);
            offset = pgm_read_byte(data + i++);
            length = pgm_read_byte(data + i++);
        } else {
            bank = data[i++];
            offset = data[i++];
            length = data[i++];
        }

        // write data or perform special action
        if (length > 0) {
            // regular block of data to write
            /*Serial.print("Writing config block to bank ");
            Serial.print(bank);
            Serial.print(", offset ");
            Serial.print(offset);
            Serial.print(", length=");
            Serial.println(length);*/
            if (useProgMem) {
                if (sizeof(progBuffer) < length) progBuffer = (uint8_t *)realloc(progBuffer, length);
                for (j = 0; j < length; j++) progBuffer[j] = pgm_read_byte(data + i + j);
            } else {
                progBuffer = (uint8_t *)data + i;
            }
            success = writeMemoryBlock(progBuffer, length, bank, offset, true);
            i += length;
        } else {
            // special instruction
            // NOTE: this kind of behavior (what and when to do certain things)
            // is totally undocumented. This code is in here based on observed
            // behavior only, and exactly why (or even whether) it has to be here
            // is anybody's guess for now.
            if (useProgMem) {
                special = pgm_read_byte(data + i++);
            } else {
                special = data[i++];
            }
            /*Serial.print("Special command code ");
            Serial.print(special, HEX);
            Serial.println(" found...");*/
            if (special == 0x01) {
                // enable DMP-related interrupts
                
                //setIntZeroMotionEnabled(true);
                //setIntFIFOBufferOverflowEnabled(true);
                //setIntDMPEnabled(true);
                I2Cdev::writeByte(devAddr, MPU9250_RA_INT_ENABLE, 0x32);  // single operation

                success = true;
            } else {
                // unknown special command
                success = false;
            }
        }
        
        if (!success) {
            if (useProgMem) free(progBuffer);
            return false; // uh oh
        }
    }
    if (useProgMem) free(progBuffer);
    return true;
}
bool MPU9250::writeProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize) {
    return writeDMPConfigurationSet(data, dataSize, true);
}

uint8_t MPU9250::getDMPConfig1() {
    I2Cdev::readByte(devAddr, MPU9250_RA_DMP_CFG_1, buffer);
    return buffer[0];
}
void MPU9250::setDMPConfig1(uint8_t config) {
    I2Cdev::writeByte(devAddr, MPU9250_RA_DMP_CFG_1, config);
}

uint8_t MPU9250::getDMPConfig2() {
    I2Cdev::readByte(devAddr, MPU9250_RA_DMP_CFG_2, buffer);
    return buffer[0];
}
void MPU9250::setDMPConfig2(uint8_t config) {
    I2Cdev::writeByte(devAddr, MPU9250_RA_DMP_CFG_2, config);
}

void BMP180::init(void)
{
    Wire.begin();
    ac1 = bmp180ReadInt(0xAA);
    ac2 = bmp180ReadInt(0xAC);
    ac3 = bmp180ReadInt(0xAE);
    ac4 = bmp180ReadInt(0xB0);
    ac5 = bmp180ReadInt(0xB2);
    ac6 = bmp180ReadInt(0xB4);
    b1 = bmp180ReadInt(0xB6);
    b2 = bmp180ReadInt(0xB8);
    mb = bmp180ReadInt(0xBA);
    mc = bmp180ReadInt(0xBC);
    md = bmp180ReadInt(0xBE);
}
// Read 1 byte from the BMP085 at 'address'
// Return: the read byte;
char BMP180::bmp180Read(unsigned char address)
{
    //Wire.begin();
    unsigned char data;
    Wire.beginTransmission(BMP180_ADDRESS);
    Wire.write(address);
    Wire.endTransmission();

    Wire.requestFrom(BMP180_ADDRESS, 1);
    while(!Wire.available());
    return Wire.read();
}
// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int BMP180::bmp180ReadInt(unsigned char address)
{
    unsigned char msb, lsb;
    Wire.beginTransmission(BMP180_ADDRESS);
    Wire.write(address);
    Wire.endTransmission();
    Wire.requestFrom(BMP180_ADDRESS, 2);
    while(Wire.available()<2);
    msb = Wire.read();
    lsb = Wire.read();
    return (int) msb<<8 | lsb;
}
// Read the uncompensated temperature value
unsigned int BMP180::bmp180ReadUT()
{
  unsigned int ut;
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  delay(5);
  ut = bmp180ReadInt(0xF6);
  return ut;
}
// Read the uncompensated pressure value
unsigned long BMP180::bmp180ReadUP()
{
    unsigned char msb, lsb, xlsb;
    unsigned long up = 0;
    Wire.beginTransmission(BMP180_ADDRESS);
    Wire.write(0xF4);
    Wire.write(0x34 + (OSS<<6));
    Wire.endTransmission();
    delay(2 + (3<<OSS));

    // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
    msb = bmp180Read(0xF6);
    lsb = bmp180Read(0xF7);
    xlsb = bmp180Read(0xF8);
    up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
    return up;
}
void BMP180::writeRegister(int deviceAddress, byte address, byte val)
{
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}
int BMP180::readRegister(int deviceAddress, byte address)
{
    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
    // waiting
    }

    v = Wire.read();
    return v;
}
float BMP180::calcAltitude(float pressure)
{
    float A = pressure/101325;
    float B = 1/5.25588;
    float C = pow(A,B);
    C = 1 - C;
    C = C /0.0000225577;
    return C;
}
float BMP180::bmp180GetTemperature(unsigned int ut)
{
    long x1, x2;

    x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
    x2 = ((long)mc << 11)/(x1 + md);
    PressureCompensate = x1 + x2;

    float temp = ((PressureCompensate + 8)>>4);
    temp = temp /10;

    return temp;
}
long BMP180::bmp180GetPressure(unsigned long up)
{
    long x1, x2, x3, b3, b6, p;
    unsigned long b4, b7;
    b6 = PressureCompensate - 4000;
    x1 = (b2 * (b6 * b6)>>12)>>11;
    x2 = (ac2 * b6)>>11;
    x3 = x1 + x2;
    b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

    // Calculate B4
    x1 = (ac3 * b6)>>13;
    x2 = (b1 * ((b6 * b6)>>12))>>16;
    x3 = ((x1 + x2) + 2)>>2;
    b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

    b7 = ((unsigned long)(up - b3) * (50000>>OSS));
    if (b7 < 0x80000000)
    p = (b7<<1)/b4;
    else
    p = (b7/b4)<<1;

    x1 = (p>>8) * (p>>8);
    x1 = (x1 * 3038)>>16;
    x2 = (-7357 * p)>>16;
    p += (x1 + x2 + 3791)>>4;

    long temp = p;
    return temp;


}

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_SBWIRE

    #ifdef I2CDEV_IMPLEMENTATION_WARNINGS
        #if ARDUINO < 100
            #warning Using outdated Arduino IDE with Wire library is functionally limiting.
            #warning Arduino IDE v1.6.5+ with I2Cdev Fastwire implementation is recommended.
            #warning This I2Cdev implementation does not support:
            #warning - Repeated starts conditions
            #warning - Timeout detection (some Wire requests block forever)
        #elif ARDUINO == 100
            #warning Using outdated Arduino IDE with Wire library is functionally limiting.
            #warning Arduino IDE v1.6.5+ with I2Cdev Fastwire implementation is recommended.
            #warning This I2Cdev implementation does not support:
            #warning - Repeated starts conditions
            #warning - Timeout detection (some Wire requests block forever)
        #elif ARDUINO > 100
        #endif
    #endif

#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE

    //#error The I2CDEV_BUILTIN_FASTWIRE implementation is known to be broken right now. Patience, Iago!

#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE

    #ifdef I2CDEV_IMPLEMENTATION_WARNINGS
        #warning Using I2CDEV_BUILTIN_NBWIRE implementation may adversely affect interrupt detection.
        #warning This I2Cdev implementation does not support:
        #warning - Repeated starts conditions
    #endif

    TwoWire Wire;

#endif

I2Cdev::I2Cdev() {}

int8_t I2Cdev::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout, void *wireObj) {
    uint8_t b;
    uint8_t count = readByte(devAddr, regAddr, &b, timeout, wireObj);
    *data = b & (1 << bitNum);
    return count;
}

int8_t I2Cdev::readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout, void *wireObj) {
    uint16_t b;
    uint8_t count = readWord(devAddr, regAddr, &b, timeout, wireObj);
    *data = b & (1 << bitNum);
    return count;
}

int8_t I2Cdev::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout, void *wireObj) {
    uint8_t count, b;
    if ((count = readByte(devAddr, regAddr, &b, timeout, wireObj)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

int8_t I2Cdev::readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout, void *wireObj) {
    uint8_t count;
    uint16_t w;
    if ((count = readWord(devAddr, regAddr, &w, timeout, wireObj)) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        w &= mask;
        w >>= (bitStart - length + 1);
        *data = w;
    }
    return count;
}

int8_t I2Cdev::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout, void *wireObj) {
    return readBytes(devAddr, regAddr, 1, data, timeout, wireObj);
}

int8_t I2Cdev::readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout, void *wireObj) {
    return readWords(devAddr, regAddr, 1, data, timeout, wireObj);
}

int8_t I2Cdev::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout, void *wireObj) {
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print("I2C (0x");
        Serial.print(devAddr, HEX);
        Serial.print(") reading ");
        Serial.print(length, DEC);
        Serial.print(" bytes from 0x");
        Serial.print(regAddr, HEX);
        Serial.print("...");
    #endif

    int8_t count = 0;
    uint32_t t1 = millis();

    #if (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_SBWIRE || I2CDEV_IMPLEMENTATION == I2CDEV_TEENSY_3X_WIRE)
        TwoWire *useWire = &Wire;
        if (wireObj) useWire = (TwoWire *)wireObj;

        #if (ARDUINO < 100)
            for (int k = 0; k < length; k += min((int)length, I2CDEVLIB_WIRE_BUFFER_LENGTH)) {
                useWire->beginTransmission(devAddr);
                useWire->send(regAddr);
                useWire->endTransmission();
                useWire->requestFrom((uint8_t)devAddr, (uint8_t)min((int)length - k, I2CDEVLIB_WIRE_BUFFER_LENGTH));
                for (; useWire->available() && (timeout == 0 || millis() - t1 < timeout); count++) {
                    data[count] = useWire->receive();
                    #ifdef I2CDEV_SERIAL_DEBUG
                        Serial.print(data[count], HEX);
                        if (count + 1 < length) Serial.print(" ");
                    #endif
                }
            }
        #elif (ARDUINO == 100)
            for (int k = 0; k < length; k += min((int)length, I2CDEVLIB_WIRE_BUFFER_LENGTH)) {
                useWire->beginTransmission(devAddr);
                useWire->write(regAddr);
                useWire->endTransmission();
                useWire->requestFrom((uint8_t)devAddr, (uint8_t)min((int)length - k, I2CDEVLIB_WIRE_BUFFER_LENGTH));
                for (; useWire->available() && (timeout == 0 || millis() - t1 < timeout); count++) {
                    data[count] = useWire->read();
                    #ifdef I2CDEV_SERIAL_DEBUG
                        Serial.print(data[count], HEX);
                        if (count + 1 < length) Serial.print(" ");
                    #endif
                }
            }
        #elif (ARDUINO > 100)
            for (int k = 0; k < length; k += min((int)length, I2CDEVLIB_WIRE_BUFFER_LENGTH)) {
                useWire->beginTransmission(devAddr);
                useWire->write(regAddr);
                useWire->endTransmission();
                useWire->requestFrom((uint8_t)devAddr, (uint8_t)min((int)length - k, I2CDEVLIB_WIRE_BUFFER_LENGTH));
                for (; useWire->available() && (timeout == 0 || millis() - t1 < timeout); count++) {
                    data[count] = useWire->read();
                    #ifdef I2CDEV_SERIAL_DEBUG
                        Serial.print(data[count], HEX);
                        if (count + 1 < length) Serial.print(" ");
                    #endif
                }
            }
        #endif

    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)

        // Fastwire library
        // no loop required for fastwire
        uint8_t status = Fastwire::readBuf(devAddr << 1, regAddr, data, length);
        if (status == 0) {
            count = length; // success
        } else {
            count = -1; // error
        }

    #endif

    // check for timeout
    if (timeout > 0 && millis() - t1 >= timeout && count < length) count = -1; // timeout

    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print(". Done (");
        Serial.print(count, DEC);
        Serial.println(" read).");
    #endif

    return count;
}

int8_t I2Cdev::readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout, void *wireObj) {
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print("I2C (0x");
        Serial.print(devAddr, HEX);
        Serial.print(") reading ");
        Serial.print(length, DEC);
        Serial.print(" words from 0x");
        Serial.print(regAddr, HEX);
        Serial.print("...");
    #endif

    int8_t count = 0;
    uint32_t t1 = millis();

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_SBWIRE || I2CDEV_IMPLEMENTATION == I2CDEV_TEENSY_3X_WIRE
        TwoWire *useWire = &Wire;
        if (wireObj) useWire = (TwoWire *)wireObj;

        #if (ARDUINO < 100)
            // Arduino v00xx (before v1.0), Wire library

            // I2C/TWI subsystem uses internal buffer that breaks with large data requests
            // so if user requests more than I2CDEVLIB_WIRE_BUFFER_LENGTH bytes, we have to do it in
            // smaller chunks instead of all at once
            for (uint8_t k = 0; k < length * 2; k += min(length * 2, I2CDEVLIB_WIRE_BUFFER_LENGTH)) {
                useWire->beginTransmission(devAddr);
                useWire->send(regAddr);
                useWire->endTransmission();
                useWire->requestFrom(devAddr, (uint8_t)(length * 2)); // length=words, this wants bytes
    
                bool msb = true; // starts with MSB, then LSB
                for (; useWire->available() && count < length && (timeout == 0 || millis() - t1 < timeout);) {
                    if (msb) {
                        // first byte is bits 15-8 (MSb=15)
                        data[count] = useWire->receive() << 8;
                    } else {
                        // second byte is bits 7-0 (LSb=0)
                        data[count] |= useWire->receive();
                        #ifdef I2CDEV_SERIAL_DEBUG
                            Serial.print(data[count], HEX);
                            if (count + 1 < length) Serial.print(" ");
                        #endif
                        count++;
                    }
                    msb = !msb;
                }
            }
        #elif (ARDUINO == 100)
            for (uint8_t k = 0; k < length * 2; k += min(length * 2, I2CDEVLIB_WIRE_BUFFER_LENGTH)) {
                useWire->beginTransmission(devAddr);
                useWire->write(regAddr);
                useWire->endTransmission();
                useWire->requestFrom(devAddr, (uint8_t)(length * 2)); // length=words, this wants bytes
    
                bool msb = true; // starts with MSB, then LSB
                for (; useWire->available() && count < length && (timeout == 0 || millis() - t1 < timeout);) {
                    if (msb) {
                        // first byte is bits 15-8 (MSb=15)
                        data[count] = useWire->read() << 8;
                    } else {
                        // second byte is bits 7-0 (LSb=0)
                        data[count] |= useWire->read();
                        #ifdef I2CDEV_SERIAL_DEBUG
                            Serial.print(data[count], HEX);
                            if (count + 1 < length) Serial.print(" ");
                        #endif
                        count++;
                    }
                    msb = !msb;
                }
            }
        #elif (ARDUINO > 100)
            for (uint8_t k = 0; k < length * 2; k += min(length * 2, I2CDEVLIB_WIRE_BUFFER_LENGTH)) {
                useWire->beginTransmission(devAddr);
                useWire->write(regAddr);
                useWire->endTransmission();
                useWire->requestFrom(devAddr, (uint8_t)(length * 2)); // length=words, this wants bytes
        
                bool msb = true; // starts with MSB, then LSB
                for (; useWire->available() && count < length && (timeout == 0 || millis() - t1 < timeout);) {
                    if (msb) {
                        // first byte is bits 15-8 (MSb=15)
                        data[count] = useWire->read() << 8;
                    } else {
                        // second byte is bits 7-0 (LSb=0)
                        data[count] |= useWire->read();
                        #ifdef I2CDEV_SERIAL_DEBUG
                            Serial.print(data[count], HEX);
                            if (count + 1 < length) Serial.print(" ");
                        #endif
                        count++;
                    }
                    msb = !msb;
                }
            }
        #endif

    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)

        // Fastwire library
        // no loop required for fastwire
        uint8_t intermediate[(uint8_t)length*2];
        uint8_t status = Fastwire::readBuf(devAddr << 1, regAddr, intermediate, (uint8_t)(length * 2));
        if (status == 0) {
            count = length; // success
            for (uint8_t i = 0; i < length; i++) {
                data[i] = (intermediate[2*i] << 8) | intermediate[2*i + 1];
            }
        } else {
            count = -1; // error
        }

    #endif

    if (timeout > 0 && millis() - t1 >= timeout && count < length) count = -1; // timeout

    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print(". Done (");
        Serial.print(count, DEC);
        Serial.println(" read).");
    #endif
    
    return count;
}

bool I2Cdev::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data, void *wireObj) {
    uint8_t b;
    readByte(devAddr, regAddr, &b, I2Cdev::readTimeout, wireObj);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(devAddr, regAddr, b, wireObj);
}

bool I2Cdev::writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data, void *wireObj) {
    uint16_t w;
    readWord(devAddr, regAddr, &w, I2Cdev::readTimeout, wireObj);
    w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
    return writeWord(devAddr, regAddr, w, wireObj);
}

bool I2Cdev::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data, void *wireObj) {
    uint8_t b;
    if (readByte(devAddr, regAddr, &b, I2Cdev::readTimeout, wireObj) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return writeByte(devAddr, regAddr, b, wireObj);
    } else {
        return false;
    }
}

bool I2Cdev::writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data, void *wireObj) {
    uint16_t w;
    if (readWord(devAddr, regAddr, &w, I2Cdev::readTimeout, wireObj) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        w &= ~(mask); // zero all important bits in existing word
        w |= data; // combine data with existing word
        return writeWord(devAddr, regAddr, w, wireObj);
    } else {
        return false;
    }
}

bool I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data, void *wireObj) {
    return writeBytes(devAddr, regAddr, 1, &data, wireObj);
}

bool I2Cdev::writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data, void *wireObj) {
    return writeWords(devAddr, regAddr, 1, &data, wireObj);
}

bool I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data, void *wireObj) {
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print("I2C (0x");
        Serial.print(devAddr, HEX);
        Serial.print(") writing ");
        Serial.print(length, DEC);
        Serial.print(" bytes to 0x");
        Serial.print(regAddr, HEX);
        Serial.print("...");
    #endif
    uint8_t status = 0;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_SBWIRE || I2CDEV_IMPLEMENTATION == I2CDEV_TEENSY_3X_WIRE
    TwoWire *useWire = &Wire;
    if (wireObj) useWire = (TwoWire *)wireObj;
#endif

    #if ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO < 100) || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE)
        useWire->beginTransmission(devAddr);
        useWire->send((uint8_t) regAddr); // send address
    #elif ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO >= 100) \
            || (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_SBWIRE && ARDUINO >= 100) \
            || I2CDEV_IMPLEMENTATION == I2CDEV_TEENSY_3X_WIRE)
        useWire->beginTransmission(devAddr);
        useWire->write((uint8_t) regAddr); // send address
    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)
        Fastwire::beginTransmission(devAddr);
        Fastwire::write(regAddr);
    #endif
    for (uint8_t i = 0; i < length; i++) {
        #ifdef I2CDEV_SERIAL_DEBUG
            Serial.print(data[i], HEX);
            if (i + 1 < length) Serial.print(" ");
        #endif
        #if ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO < 100) || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE)
            useWire->send((uint8_t) data[i]);
        #elif ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO >= 100) \
                || (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_SBWIRE && ARDUINO >= 100) \
                || I2CDEV_IMPLEMENTATION == I2CDEV_TEENSY_3X_WIRE)
            useWire->write((uint8_t) data[i]);
        #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)
            Fastwire::write((uint8_t) data[i]);
        #endif
    }
    #if ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO < 100) || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE)
        useWire->endTransmission();
    #elif ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO >= 100) \
            || (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_SBWIRE && ARDUINO >= 100) \
            || I2CDEV_IMPLEMENTATION == I2CDEV_TEENSY_3X_WIRE)
        status = useWire->endTransmission();
    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)
        Fastwire::stop();
        //status = Fastwire::endTransmission();
    #endif
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.println(". Done.");
    #endif
    return status == 0;
}

bool I2Cdev::writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data, void *wireObj) {
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print("I2C (0x");
        Serial.print(devAddr, HEX);
        Serial.print(") writing ");
        Serial.print(length, DEC);
        Serial.print(" words to 0x");
        Serial.print(regAddr, HEX);
        Serial.print("...");
    #endif
    uint8_t status = 0;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_SBWIRE || I2CDEV_IMPLEMENTATION == I2CDEV_TEENSY_3X_WIRE
    TwoWire *useWire = &Wire;
    if (wireObj) useWire = (TwoWire *)wireObj;
#endif

    #if ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO < 100) || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE)
        useWire->beginTransmission(devAddr);
        useWire->send(regAddr); // send address
    #elif ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO >= 100) \
            || (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_SBWIRE && ARDUINO >= 100) \
            || I2CDEV_IMPLEMENTATION == I2CDEV_TEENSY_3X_WIRE)
        useWire->beginTransmission(devAddr);
        useWire->write(regAddr); // send address
    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)
        Fastwire::beginTransmission(devAddr);
        Fastwire::write(regAddr);
    #endif
    for (uint8_t i = 0; i < length; i++) { 
        #ifdef I2CDEV_SERIAL_DEBUG
            Serial.print(data[i], HEX);
            if (i + 1 < length) Serial.print(" ");
        #endif
        #if ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO < 100) || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE)
            useWire->send((uint8_t)(data[i] >> 8));     // send MSB
            useWire->send((uint8_t)data[i]);          // send LSB
        #elif ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO >= 100) \
                || (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_SBWIRE && ARDUINO >= 100) \
                || I2CDEV_IMPLEMENTATION == I2CDEV_TEENSY_3X_WIRE)
            useWire->write((uint8_t)(data[i] >> 8));    // send MSB
            useWire->write((uint8_t)data[i]);         // send LSB
        #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)
            Fastwire::write((uint8_t)(data[i] >> 8));       // send MSB
            status = Fastwire::write((uint8_t)data[i]);   // send LSB
            if (status != 0) break;
        #endif
    }
    #if ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO < 100) || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE)
        useWire->endTransmission();
    #elif ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO >= 100) \
            || (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_SBWIRE && ARDUINO >= 100) \
            || I2CDEV_IMPLEMENTATION == I2CDEV_TEENSY_3X_WIRE)
        status = useWire->endTransmission();
    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)
        Fastwire::stop();
        //status = Fastwire::endTransmission();
    #endif
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.println(". Done.");
    #endif
    return status == 0;
}

uint16_t I2Cdev::readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;

#if I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    // I2C library

    boolean Fastwire::waitInt() {
        int l = 250;
        while (!(TWCR & (1 << TWINT)) && l-- > 0);
        return l > 0;
    }

    void Fastwire::setup(int khz, boolean pullup) {
        TWCR = 0;
        #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
            // activate internal pull-ups for twi (PORTC bits 4 & 5)
            // as per note from atmega8 manual pg167
            if (pullup) PORTC |= ((1 << 4) | (1 << 5));
            else        PORTC &= ~((1 << 4) | (1 << 5));
        #elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)
            // activate internal pull-ups for twi (PORTC bits 0 & 1)
            if (pullup) PORTC |= ((1 << 0) | (1 << 1));
            else        PORTC &= ~((1 << 0) | (1 << 1));
        #else
            // activate internal pull-ups for twi (PORTD bits 0 & 1)
            // as per note from atmega128 manual pg204
            if (pullup) PORTD |= ((1 << 0) | (1 << 1));
            else        PORTD &= ~((1 << 0) | (1 << 1));
        #endif

        TWSR = 0; // no prescaler => prescaler = 1
        TWBR = F_CPU / 2000 / khz - 8; // change the I2C clock rate
        TWCR = 1 << TWEN; // enable twi module, no interrupt
    }

    byte Fastwire::beginTransmission(byte device) {
        byte twst, retry;
        retry = 2;
        do {
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO) | (1 << TWSTA);
            if (!waitInt()) return 1;
            twst = TWSR & 0xF8;
            if (twst != TW_START && twst != TW_REP_START) return 2;

            //Serial.print(device, HEX);
            //Serial.print(" ");
            TWDR = device << 1; // send device address without read bit (1)
            TWCR = (1 << TWINT) | (1 << TWEN);
            if (!waitInt()) return 3;
            twst = TWSR & 0xF8;
        } while (twst == TW_MT_SLA_NACK && retry-- > 0);
        if (twst != TW_MT_SLA_ACK) return 4;
        return 0;
    }

    byte Fastwire::writeBuf(byte device, byte address, byte *data, byte num) {
        byte twst, retry;

        retry = 2;
        do {
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO) | (1 << TWSTA);
            if (!waitInt()) return 1;
            twst = TWSR & 0xF8;
            if (twst != TW_START && twst != TW_REP_START) return 2;

            //Serial.print(device, HEX);
            //Serial.print(" ");
            TWDR = device & 0xFE; // send device address without read bit (1)
            TWCR = (1 << TWINT) | (1 << TWEN);
            if (!waitInt()) return 3;
            twst = TWSR & 0xF8;
        } while (twst == TW_MT_SLA_NACK && retry-- > 0);
        if (twst != TW_MT_SLA_ACK) return 4;

        //Serial.print(address, HEX);
        //Serial.print(" ");
        TWDR = address; // send data to the previously addressed device
        TWCR = (1 << TWINT) | (1 << TWEN);
        if (!waitInt()) return 5;
        twst = TWSR & 0xF8;
        if (twst != TW_MT_DATA_ACK) return 6;

        for (byte i = 0; i < num; i++) {
            //Serial.print(data[i], HEX);
            //Serial.print(" ");
            TWDR = data[i]; // send data to the previously addressed device
            TWCR = (1 << TWINT) | (1 << TWEN);
            if (!waitInt()) return 7;
            twst = TWSR & 0xF8;
            if (twst != TW_MT_DATA_ACK) return 8;
        }
        //Serial.print("\n");

        return 0;
    }

    byte Fastwire::write(byte value) {
        byte twst;
        //Serial.println(value, HEX);
        TWDR = value; // send data
        TWCR = (1 << TWINT) | (1 << TWEN);
        if (!waitInt()) return 1;
        twst = TWSR & 0xF8;
        if (twst != TW_MT_DATA_ACK) return 2;
        return 0;
    }

    byte Fastwire::readBuf(byte device, byte address, byte *data, byte num) {
        byte twst, retry;

        retry = 2;
        do {
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO) | (1 << TWSTA);
            if (!waitInt()) return 16;
            twst = TWSR & 0xF8;
            if (twst != TW_START && twst != TW_REP_START) return 17;

            //Serial.print(device, HEX);
            //Serial.print(" ");
            TWDR = device & 0xfe; // send device address to write
            TWCR = (1 << TWINT) | (1 << TWEN);
            if (!waitInt()) return 18;
            twst = TWSR & 0xF8;
        } while (twst == TW_MT_SLA_NACK && retry-- > 0);
        if (twst != TW_MT_SLA_ACK) return 19;

        //Serial.print(address, HEX);
        //Serial.print(" ");
        TWDR = address; // send data to the previously addressed device
        TWCR = (1 << TWINT) | (1 << TWEN);
        if (!waitInt()) return 20;
        twst = TWSR & 0xF8;
        if (twst != TW_MT_DATA_ACK) return 21;

        /***/

        retry = 2;
        do {
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO) | (1 << TWSTA);
            if (!waitInt()) return 22;
            twst = TWSR & 0xF8;
            if (twst != TW_START && twst != TW_REP_START) return 23;

            //Serial.print(device, HEX);
            //Serial.print(" ");
            TWDR = device | 0x01; // send device address with the read bit (1)
            TWCR = (1 << TWINT) | (1 << TWEN);
            if (!waitInt()) return 24;
            twst = TWSR & 0xF8;
        } while (twst == TW_MR_SLA_NACK && retry-- > 0);
        if (twst != TW_MR_SLA_ACK) return 25;

        for (uint8_t i = 0; i < num; i++) {
            if (i == num - 1)
                TWCR = (1 << TWINT) | (1 << TWEN);
            else
                TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
            if (!waitInt()) return 26;
            twst = TWSR & 0xF8;
            if (twst != TW_MR_DATA_ACK && twst != TW_MR_DATA_NACK) return twst;
            data[i] = TWDR;
            //Serial.print(data[i], HEX);
            //Serial.print(" ");
        }
        //Serial.print("\n");
        stop();

        return 0;
    }

    void Fastwire::reset() {
        TWCR = 0;
    }

    byte Fastwire::stop() {
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
        if (!waitInt()) return 1;
        return 0;
    }
#endif

#if I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE
    
    uint8_t TwoWire::rxBuffer[NBWIRE_BUFFER_LENGTH];
    uint8_t TwoWire::rxBufferIndex = 0;
    uint8_t TwoWire::rxBufferLength = 0;
    
    uint8_t TwoWire::txAddress = 0;
    uint8_t TwoWire::txBuffer[NBWIRE_BUFFER_LENGTH];
    uint8_t TwoWire::txBufferIndex = 0;
    uint8_t TwoWire::txBufferLength = 0;
    
    //uint8_t TwoWire::transmitting = 0;
    void (*TwoWire::user_onRequest)(void);
    void (*TwoWire::user_onReceive)(int);
    
    static volatile uint8_t twi_transmitting;
    static volatile uint8_t twi_state;
    static uint8_t twi_slarw;
    static volatile uint8_t twi_error;
    static uint8_t twi_masterBuffer[TWI_BUFFER_LENGTH];
    static volatile uint8_t twi_masterBufferIndex;
    static uint8_t twi_masterBufferLength;
    static uint8_t twi_rxBuffer[TWI_BUFFER_LENGTH];
    static volatile uint8_t twi_rxBufferIndex;
    //static volatile uint8_t twi_Interrupt_Continue_Command;
    static volatile uint8_t twi_Return_Value;
    static volatile uint8_t twi_Done;
    void (*twi_cbendTransmissionDone)(int);
    void (*twi_cbreadFromDone)(int);
    
    void twi_init() {
        // initialize state
        twi_state = TWI_READY;

        // activate internal pull-ups for twi
        // as per note from atmega8 manual pg167
        sbi(PORTC, 4);
        sbi(PORTC, 5);

        // initialize twi prescaler and bit rate
        cbi(TWSR, TWPS0); // TWI Status Register - Prescaler bits
        cbi(TWSR, TWPS1);

        TWBR = ((CPU_FREQ / TWI_FREQ) - 16) / 2; // bitrate register
        // enable twi module, acks, and twi interrupt

        TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
    }
    
    typedef struct {
        uint8_t address;
        uint8_t* data;
        uint8_t length;
        uint8_t wait;
        uint8_t i;
    } twi_Write_Vars;

    twi_Write_Vars *ptwv = 0;
    static void (*fNextInterruptFunction)(void) = 0;

    void twi_Finish(byte bRetVal) {
        if (ptwv) {
            free(ptwv);
            ptwv = 0;
        }
        twi_Done = 0xFF;
        twi_Return_Value = bRetVal;
        fNextInterruptFunction = 0;
    }
    
    uint8_t twii_WaitForDone(uint16_t timeout) {
        uint32_t endMillis = millis() + timeout;
        while (!twi_Done && (timeout == 0 || millis() < endMillis)) continue;
        return twi_Return_Value;
    }
    
    void twii_SetState(uint8_t ucState) {
        twi_state = ucState;
    }

    void twii_SetError(uint8_t ucError) {
        twi_error = ucError ;
    }

    void twii_InitBuffer(uint8_t ucPos, uint8_t ucLength) {
        twi_masterBufferIndex = 0;
        twi_masterBufferLength = ucLength;
    }

    void twii_CopyToBuf(uint8_t* pData, uint8_t ucLength) {
        uint8_t i;
        for (i = 0; i < ucLength; ++i) {
            twi_masterBuffer[i] = pData[i];
        }
    }

    void twii_CopyFromBuf(uint8_t *pData, uint8_t ucLength) {
        uint8_t i;
        for (i = 0; i < ucLength; ++i) {
            pData[i] = twi_masterBuffer[i];
        }
    }

    void twii_SetSlaRW(uint8_t ucSlaRW) {
        twi_slarw = ucSlaRW;
    }

    void twii_SetStart() {
        TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWINT) | (1 << TWSTA);
    }

    void twi_write01() {
        if (TWI_MTX == twi_state) return; // blocking test
        twi_transmitting = 0 ;
        if (twi_error == 0xFF)
            twi_Finish (0);    // success
        else if (twi_error == TW_MT_SLA_NACK)
            twi_Finish (2);    // error: address send, nack received
        else if (twi_error == TW_MT_DATA_NACK)
            twi_Finish (3);    // error: data send, nack received
        else
            twi_Finish (4);    // other twi error
        if (twi_cbendTransmissionDone) return twi_cbendTransmissionDone(twi_Return_Value);
        return;
    }
    
    
    void twi_write00() {
        if (TWI_READY != twi_state) return; // blocking test
        if (TWI_BUFFER_LENGTH < ptwv -> length) {
            twi_Finish(1); // end write with error 1
            return;
        }
        twi_Done = 0x00; // show as working
        twii_SetState(TWI_MTX); // to transmitting
        twii_SetError(0xFF); // to No Error
        twii_InitBuffer(0, ptwv -> length); // pointer and length
        twii_CopyToBuf(ptwv -> data, ptwv -> length); // get the data
        twii_SetSlaRW((ptwv -> address << 1) | TW_WRITE); // write command
        twii_SetStart(); // start the cycle
        fNextInterruptFunction = twi_write01; // next routine
        return twi_write01();
    }
    
    void twi_writeTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait) {
        uint8_t i;
        ptwv = (twi_Write_Vars *)malloc(sizeof(twi_Write_Vars));
        ptwv -> address = address;
        ptwv -> data = data;
        ptwv -> length = length;
        ptwv -> wait = wait;
        fNextInterruptFunction = twi_write00;
        return twi_write00();
    }

    void twi_read01() {
        if (TWI_MRX == twi_state) return; // blocking test
        if (twi_masterBufferIndex < ptwv -> length) ptwv -> length = twi_masterBufferIndex;
        twii_CopyFromBuf(ptwv -> data, ptwv -> length);
        twi_Finish(ptwv -> length);
        if (twi_cbreadFromDone) return twi_cbreadFromDone(twi_Return_Value);
        return;
    }
    
    void twi_read00() {
        if (TWI_READY != twi_state) return; // blocking test
        if (TWI_BUFFER_LENGTH < ptwv -> length) twi_Finish(0); // error return
        twi_Done = 0x00; // show as working
        twii_SetState(TWI_MRX); // reading
        twii_SetError(0xFF); // reset error
        twii_InitBuffer(0, ptwv -> length - 1); // init to one less than length
        twii_SetSlaRW((ptwv -> address << 1) | TW_READ); // read command
        twii_SetStart(); // start cycle
        fNextInterruptFunction = twi_read01;
        return twi_read01();
    }

    void twi_readFrom(uint8_t address, uint8_t* data, uint8_t length) {
        uint8_t i;

        ptwv = (twi_Write_Vars *)malloc(sizeof(twi_Write_Vars));
        ptwv -> address = address;
        ptwv -> data = data;
        ptwv -> length = length;
        fNextInterruptFunction = twi_read00;
        return twi_read00();
    }

    void twi_reply(uint8_t ack) {
        // transmit master read ready signal, with or without ack
        if (ack){
            TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA);
        } else {
            TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT);
        }
    }
    
    void twi_stop(void) {
        // send stop condition
        TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWINT) | (1 << TWSTO);
    
        // wait for stop condition to be exectued on bus
        // TWINT is not set after a stop condition!
        while (TWCR & (1 << TWSTO)) {
            continue;
        }
    
        // update twi state
        twi_state = TWI_READY;
    }

    void twi_releaseBus(void) {
        // release bus
        TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWINT);
    
        // update twi state
        twi_state = TWI_READY;
    }
    
    SIGNAL(TWI_vect) {
        switch (TW_STATUS) {
            // All Master
            case TW_START:     // sent start condition
            case TW_REP_START: // sent repeated start condition
                // copy device address and r/w bit to output register and ack
                TWDR = twi_slarw;
                twi_reply(1);
                break;
    
            // Master Transmitter
            case TW_MT_SLA_ACK:  // slave receiver acked address
            case TW_MT_DATA_ACK: // slave receiver acked data
                // if there is data to send, send it, otherwise stop
                if (twi_masterBufferIndex < twi_masterBufferLength) {
                    // copy data to output register and ack
                    TWDR = twi_masterBuffer[twi_masterBufferIndex++];
                    twi_reply(1);
                } else {
                    twi_stop();
                }
                break;

            case TW_MT_SLA_NACK:  // address sent, nack received
                twi_error = TW_MT_SLA_NACK;
                twi_stop();
                break;

            case TW_MT_DATA_NACK: // data sent, nack received
                twi_error = TW_MT_DATA_NACK;
                twi_stop();
                break;

            case TW_MT_ARB_LOST: // lost bus arbitration
                twi_error = TW_MT_ARB_LOST;
                twi_releaseBus();
                break;
    
            // Master Receiver
            case TW_MR_DATA_ACK: // data received, ack sent
                // put byte into buffer
                twi_masterBuffer[twi_masterBufferIndex++] = TWDR;

            case TW_MR_SLA_ACK:  // address sent, ack received
                // ack if more bytes are expected, otherwise nack
                if (twi_masterBufferIndex < twi_masterBufferLength) {
                    twi_reply(1);
                } else {
                    twi_reply(0);
                }
                break;

            case TW_MR_DATA_NACK: // data received, nack sent
                // put final byte into buffer
                twi_masterBuffer[twi_masterBufferIndex++] = TWDR;

            case TW_MR_SLA_NACK: // address sent, nack received
                twi_stop();
                break;

        // TW_MR_ARB_LOST handled by TW_MT_ARB_LOST case
        // Slave Receiver (NOT IMPLEMENTED YET)

            // all
            case TW_NO_INFO:   // no state information
                break;

            case TW_BUS_ERROR: // bus error, illegal stop/start
                twi_error = TW_BUS_ERROR;
                twi_stop();
                break;
        }

        if (fNextInterruptFunction) return fNextInterruptFunction();
    }

    TwoWire::TwoWire() { }
    
    void TwoWire::begin(void) {
        rxBufferIndex = 0;
        rxBufferLength = 0;
    
        txBufferIndex = 0;
        txBufferLength = 0;

        twi_init();
    }
    
    void TwoWire::beginTransmission(uint8_t address) {
        //beginTransmission((uint8_t)address);

        // indicate that we are transmitting
        twi_transmitting = 1;
        
        // set address of targeted slave
        txAddress = address;
        
        // reset tx buffer iterator vars
        txBufferIndex = 0;
        txBufferLength = 0;
    }

    uint8_t TwoWire::endTransmission(uint16_t timeout) {
        // transmit buffer (blocking)
        //int8_t ret =
        twi_cbendTransmissionDone = NULL;
        twi_writeTo(txAddress, txBuffer, txBufferLength, 1);
        int8_t ret = twii_WaitForDone(timeout);

        // reset tx buffer iterator vars
        txBufferIndex = 0;
        txBufferLength = 0;

        // indicate that we are done transmitting
        // twi_transmitting = 0;
        return ret;
    }

    void TwoWire::nbendTransmission(void (*function)(int)) {
        twi_cbendTransmissionDone = function;
        twi_writeTo(txAddress, txBuffer, txBufferLength, 1);
        return;
    }
    
    void TwoWire::send(uint8_t data) {
        if (twi_transmitting) {
            // in master transmitter mode
            // don't bother if buffer is full
            if (txBufferLength >= NBWIRE_BUFFER_LENGTH) {
                return;
            }

            // put byte in tx buffer
            txBuffer[txBufferIndex] = data;
            ++txBufferIndex;

            // update amount in buffer
            txBufferLength = txBufferIndex;
        } else {
        }
    }
    
    uint8_t TwoWire::receive(void) {
        // default to returning null char
        // for people using with char strings
        uint8_t value = 0;
      
        // get each successive byte on each call
        if (rxBufferIndex < rxBufferLength) {
            value = rxBuffer[rxBufferIndex];
            ++rxBufferIndex;
        }
    
        return value;
    }
    
    uint8_t TwoWire::requestFrom(uint8_t address, int quantity, uint16_t timeout) {
        // clamp to buffer length
        if (quantity > NBWIRE_BUFFER_LENGTH) {
            quantity = NBWIRE_BUFFER_LENGTH;
        }

        // perform blocking read into buffer
        twi_cbreadFromDone = NULL;
        twi_readFrom(address, rxBuffer, quantity);
        uint8_t read = twii_WaitForDone(timeout);

        // set rx buffer iterator vars
        rxBufferIndex = 0;
        rxBufferLength = read;
    
        return read;
    }
    
    void TwoWire::nbrequestFrom(uint8_t address, int quantity, void (*function)(int)) {
        // clamp to buffer length
        if (quantity > NBWIRE_BUFFER_LENGTH) {
            quantity = NBWIRE_BUFFER_LENGTH;
        }

        // perform blocking read into buffer
        twi_cbreadFromDone = function;
        twi_readFrom(address, rxBuffer, quantity);
        //uint8_t read = twii_WaitForDone();

        // set rx buffer iterator vars
        //rxBufferIndex = 0;
        //rxBufferLength = read;

        rxBufferIndex = 0;
        rxBufferLength = quantity; // this is a hack

        return; //read;
    }

    uint8_t TwoWire::available(void) {
        return rxBufferLength - rxBufferIndex;
    }

#endif
