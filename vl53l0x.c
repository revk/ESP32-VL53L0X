// VL53L0X control
// Copyright © 2019 Adrian Kennard, Andrews & Arnold Ltd. See LICENCE file for details. GPL 3.0
// Based on https://github.com/pololu/vl53l0x-arduino
static const char __attribute__((unused)) TAG[] = "ranger";

#include "vl53l0x.h"
#include "esp_log.h"
#include <driver/i2c.h>

#define TIMEOUT	(10/portTICK_PERIOD_MS) // I2C command timeout

//#define tBUF  1300            // tBUF=1.3ms from data sheet (but seems unnecessary)
//#define VL53L0X_LOG   ESP_LOGI        // Set to allow I2C logginc

#ifndef VL53L0X_LOG
#define VL53L0X_LOG(tag,...) err=err;
#endif

enum
{
   SYSRANGE_START = 0x00,

   SYSTEM_THRESH_HIGH = 0x0C,
   SYSTEM_THRESH_LOW = 0x0E,

   SYSTEM_SEQUENCE_CONFIG = 0x01,
   SYSTEM_RANGE_CONFIG = 0x09,
   SYSTEM_INTERMEASUREMENT_PERIOD = 0x04,

   SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A,

   GPIO_HV_MUX_ACTIVE_HIGH = 0x84,

   SYSTEM_INTERRUPT_CLEAR = 0x0B,

   RESULT_INTERRUPT_STATUS = 0x13,
   RESULT_RANGE_STATUS = 0x14,

   RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC,
   RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0,
   RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0,
   RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4,
   RESULT_PEAK_SIGNAL_RATE_REF = 0xB6,

   ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28,

   MSRC_CONFIG_CONTROL = 0x60,

   PRE_RANGE_CONFIG_MIN_SNR = 0x27,
   PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56,
   PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57,
   PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64,

   FINAL_RANGE_CONFIG_MIN_SNR = 0x67,
   FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47,
   FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48,
   FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

   PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61,
   PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62,

   PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50,
   PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51,
   PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52,

   SYSTEM_HISTOGRAM_BIN = 0x81,
   HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33,
   HISTOGRAM_CONFIG_READOUT_CTRL = 0x55,

   FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70,
   FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71,
   FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72,
   CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20,

   MSRC_CONFIG_TIMEOUT_MACROP = 0x46,

   I2C_SLAVE_DEVICE_ADDRESS = 0x8A,

   SOFT_RESET_GO2_SOFT_RESET_N = 0xBF,
   IDENTIFICATION_MODEL_ID = 0xC0,
   IDENTIFICATION_REVISION_ID = 0xC2,

   OSC_CALIBRATE_VAL = 0xF8,

   GLOBAL_CONFIG_VCSEL_WIDTH = 0x32,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5,

   GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6,
   DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E,
   DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F,
   POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80,

   VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89,

   ALGO_PHASECAL_LIM = 0x30,
   ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30,
};

struct vl53l0x_s
{
   uint8_t port;
   uint8_t address;
   int8_t xshut;
   uint16_t io_timeout;
   uint8_t io_2v8:1;
   uint8_t did_timeout:1;
   uint8_t i2c_fail:1;
};

typedef struct
{
   uint8_t tcc:1;
   uint8_t msrc:1;
   uint8_t dss:1;
   uint8_t pre_range:1;
   uint8_t final_range:1;
} SequenceStepEnables;

typedef struct
{
   uint16_t pre_range_vcsel_period_pclks,
     final_range_vcsel_period_pclks;
   uint16_t msrc_dss_tcc_mclks,
     pre_range_mclks,
     final_range_mclks;
   uint32_t msrc_dss_tcc_us,
     pre_range_us,
     final_range_us;
}
SequenceStepTimeouts;

static uint8_t stop_variable;
static uint16_t timeout_start_ms;
static uint32_t measurement_timing_budget_us;
#define millis() (esp_timer_get_time()/1000LL)

// Record the current time to check an upcoming timeout against
#define startTimeout() (timeout_start_ms = millis())

// Check if timeout is enabled (set to nonzero value) and has expired
#define checkTimeoutExpired() (v->io_timeout > 0 && ((uint16_t)(millis() - timeout_start_ms)) > v->io_timeout)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

static esp_err_t
Done (vl53l0x_t * v, i2c_cmd_handle_t i)
{
   i2c_master_stop (i);
   esp_err_t err = i2c_master_cmd_begin (v->port, i, TIMEOUT);
   if (err)
      v->i2c_fail = 1;
   i2c_cmd_link_delete (i);
#ifdef tBUF
   usleep (tBUF);
#endif
   return err;
}

static i2c_cmd_handle_t
Read (vl53l0x_t * v, uint8_t reg)
{                               // Set up for read
   i2c_cmd_handle_t i = i2c_cmd_link_create ();
   i2c_master_start (i);
   i2c_master_write_byte (i, (v->address << 1), 1);
   i2c_master_write_byte (i, reg, 1);
   Done (v, i);
   i = i2c_cmd_link_create ();
   i2c_master_start (i);
   i2c_master_write_byte (i, (v->address << 1) + 1, 1);
   return i;
}

static i2c_cmd_handle_t
Write (vl53l0x_t * v, uint8_t reg)
{                               // Set up for write
   i2c_cmd_handle_t i = i2c_cmd_link_create ();
   i2c_master_start (i);
   i2c_master_write_byte (i, (v->address << 1), 1);
   i2c_master_write_byte (i, reg, 1);
   return i;
}

void
vl53l0x_writeReg8Bit (vl53l0x_t * v, uint8_t reg, uint8_t val)
{
   i2c_cmd_handle_t i = Write (v, reg);
   i2c_master_write_byte (i, val, 1);
   esp_err_t err = Done (v, i);
   VL53L0X_LOG (TAG, "W %02X=%02X %s", reg, val, esp_err_to_name (err));
}

void
vl53l0x_writeReg16Bit (vl53l0x_t * v, uint8_t reg, uint16_t val)
{
   i2c_cmd_handle_t i = Write (v, reg);
   i2c_master_write_byte (i, val >> 8, 1);
   i2c_master_write_byte (i, val, 1);
   esp_err_t err = Done (v, i);
   VL53L0X_LOG (TAG, "W %02X=%04X %s", reg, val, esp_err_to_name (err));
}

void
vl53l0x_writeReg32Bit (vl53l0x_t * v, uint8_t reg, uint32_t val)
{
   i2c_cmd_handle_t i = Write (v, reg);
   i2c_master_write_byte (i, val >> 24, 1);
   i2c_master_write_byte (i, val >> 16, 1);
   i2c_master_write_byte (i, val >> 8, 1);
   i2c_master_write_byte (i, val, 1);
   esp_err_t err = Done (v, i);
   VL53L0X_LOG (TAG, "W %02X=%08X %s", reg, val, esp_err_to_name (err));
}

uint8_t
vl53l0x_readReg8Bit (vl53l0x_t * v, uint8_t reg)
{
   uint8_t buf[1] = { };
   i2c_cmd_handle_t i = Read (v, reg);
   i2c_master_read_byte (i, buf + 0, I2C_MASTER_LAST_NACK);
   esp_err_t err = Done (v, i);
   VL53L0X_LOG (TAG, "R %02X=%02X %s", reg, buf[0], esp_err_to_name (err));
   return buf[0];
}

uint16_t
vl53l0x_readReg16Bit (vl53l0x_t * v, uint8_t reg)
{
   uint8_t buf[2] = { };
   i2c_cmd_handle_t i = Read (v, reg);
   i2c_master_read_byte (i, buf + 0, I2C_MASTER_ACK);
   i2c_master_read_byte (i, buf + 1, I2C_MASTER_LAST_NACK);
   esp_err_t err = Done (v, i);
   VL53L0X_LOG (TAG, "R %02X=%02X%02X %s", reg, buf[0], buf[1], esp_err_to_name (err));
   return (buf[0] << 8) + buf[1];
}

uint32_t
vl53l0x_readReg32Bit (vl53l0x_t * v, uint8_t reg)
{
   uint8_t buf[4] = { };
   i2c_cmd_handle_t i = Read (v, reg);
   i2c_master_read_byte (i, buf + 0, I2C_MASTER_ACK);
   i2c_master_read_byte (i, buf + 1, I2C_MASTER_ACK);
   i2c_master_read_byte (i, buf + 2, I2C_MASTER_ACK);
   i2c_master_read_byte (i, buf + 3, I2C_MASTER_LAST_NACK);
   esp_err_t err = Done (v, i);
   VL53L0X_LOG (TAG, "R %02X=%02X%02X%02X%02X %s", reg, buf[0], buf[1], buf[2], buf[3], esp_err_to_name (err));
   return (buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3];
}

// Get the return signal rate limit check value in MCPS

// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
void
vl53l0x_readMulti (vl53l0x_t * v, uint8_t reg, uint8_t * dst, uint8_t count)
{
   i2c_cmd_handle_t i = Read (v, reg);
   if (count > 1)
      i2c_master_read (i, dst + 0, count - 1, I2C_MASTER_ACK);
   i2c_master_read_byte (i, dst + count - 1, I2C_MASTER_LAST_NACK);
   esp_err_t err = Done (v, i);
   VL53L0X_LOG (TAG, "R %02X (%d) %s", reg, count, esp_err_to_name (err));
}

// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
void
vl53l0x_writeMulti (vl53l0x_t * v, uint8_t reg, uint8_t const *src, uint8_t count)
{
   i2c_cmd_handle_t i = Write (v, reg);
   i2c_master_write (i, (uint8_t *) src, count, 1);
   esp_err_t err = Done (v, i);
   VL53L0X_LOG (TAG, "W %02X (%d) %s", reg, count, esp_err_to_name (err));
}

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
static uint16_t
decodeTimeout (uint16_t reg_val)
{
   // format: "(LSByte * 2^MSByte) + 1"
   return (uint16_t) ((reg_val & 0x00FF) << (uint16_t) ((reg_val & 0xFF00) >> 8)) + 1;
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
static uint32_t
timeoutMclksToMicroseconds (uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
   uint32_t macro_period_ns = calcMacroPeriod (vcsel_period_pclks);
   return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// based on VL53L0X_perform_single_ref_calibration()
static const char *
performSingleRefCalibration (vl53l0x_t * v, uint8_t vhv_init_byte)
{
   vl53l0x_writeReg8Bit (v, SYSRANGE_START, 0x01 | vhv_init_byte);      // VL53L0X_REG_SYSRANGE_MODE_START_STOP
   startTimeout ();
   while ((vl53l0x_readReg8Bit (v, RESULT_INTERRUPT_STATUS) & 0x07) == 0)
   {
      if (checkTimeoutExpired ())
         return "CAL Timeout";
   }
   vl53l0x_writeReg8Bit (v, SYSTEM_INTERRUPT_CLEAR, 0x01);
   vl53l0x_writeReg8Bit (v, SYSRANGE_START, 0x00);
   return NULL;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
static uint16_t
encodeTimeout (uint16_t timeout_mclks)
{
   // format: "(LSByte * 2^MSByte) + 1"

   uint32_t ls_byte = 0;
   uint16_t ms_byte = 0;

   if (timeout_mclks > 0)
   {
      ls_byte = timeout_mclks - 1;

      while ((ls_byte & 0xFFFFFF00) > 0)
      {
         ls_byte >>= 1;
         ms_byte++;
      }

      return (ms_byte << 8) | (ls_byte & 0xFF);
   } else
   {
      return 0;
   }
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
static uint8_t
getVcselPulsePeriod (vl53l0x_t * v, vl53l0x_vcselPeriodType type)
{
   if (type == VcselPeriodPreRange)
   {
      return decodeVcselPeriod (vl53l0x_readReg8Bit (v, PRE_RANGE_CONFIG_VCSEL_PERIOD));
   } else if (type == VcselPeriodFinalRange)
   {
      return decodeVcselPeriod (vl53l0x_readReg8Bit (v, FINAL_RANGE_CONFIG_VCSEL_PERIOD));
   } else
   {
      return 255;
   }
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
static void
getSequenceStepEnables (vl53l0x_t * v, SequenceStepEnables * enables)
{
   uint8_t sequence_config = vl53l0x_readReg8Bit (v, SYSTEM_SEQUENCE_CONFIG);

   enables->tcc = (sequence_config >> 4) & 0x1;
   enables->dss = (sequence_config >> 3) & 0x1;
   enables->msrc = (sequence_config >> 2) & 0x1;
   enables->pre_range = (sequence_config >> 6) & 0x1;
   enables->final_range = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
static void
getSequenceStepTimeouts (vl53l0x_t * v, SequenceStepEnables const *enables, SequenceStepTimeouts * timeouts)
{
   timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod (v, VcselPeriodPreRange);

   timeouts->msrc_dss_tcc_mclks = vl53l0x_readReg8Bit (v, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
   timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds (timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);

   timeouts->pre_range_mclks = decodeTimeout (vl53l0x_readReg16Bit (v, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
   timeouts->pre_range_us = timeoutMclksToMicroseconds (timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

   timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod (v, VcselPeriodFinalRange);

   timeouts->final_range_mclks = decodeTimeout (vl53l0x_readReg16Bit (v, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

   if (enables->pre_range)
   {
      timeouts->final_range_mclks -= timeouts->pre_range_mclks;
   }

   timeouts->final_range_us = timeoutMclksToMicroseconds (timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
static uint32_t
timeoutMicrosecondsToMclks (uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
   uint32_t macro_period_ns = calcMacroPeriod (vcsel_period_pclks);

   return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
const char *
vl53l0x_setSignalRateLimit (vl53l0x_t * v, float limit_Mcps)
{
   if (limit_Mcps < 0 || limit_Mcps > 511.99)
      return "Bad rate";
   // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
   vl53l0x_writeReg16Bit (v, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
   return NULL;
}

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
const char *
vl53l0x_getSpadInfo (vl53l0x_t * v, uint8_t * count, int *type_is_aperture)
{
   uint8_t tmp;

   vl53l0x_writeReg8Bit (v, 0x80, 0x01);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x00, 0x00);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x06);
   vl53l0x_writeReg8Bit (v, 0x83, vl53l0x_readReg8Bit (v, 0x83) | 0x04);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x07);
   vl53l0x_writeReg8Bit (v, 0x81, 0x01);

   vl53l0x_writeReg8Bit (v, 0x80, 0x01);

   vl53l0x_writeReg8Bit (v, 0x94, 0x6b);
   vl53l0x_writeReg8Bit (v, 0x83, 0x00);
   startTimeout ();
   while (vl53l0x_readReg8Bit (v, 0x83) == 0x00)
   {
      if (checkTimeoutExpired ())
         return "SPAD Timeout";
   }
   vl53l0x_writeReg8Bit (v, 0x83, 0x01);
   tmp = vl53l0x_readReg8Bit (v, 0x92);

   *count = tmp & 0x7f;
   *type_is_aperture = (tmp >> 7) & 0x01;

   vl53l0x_writeReg8Bit (v, 0x81, 0x00);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x06);
   vl53l0x_writeReg8Bit (v, 0x83, vl53l0x_readReg8Bit (v, 0x83) & ~0x04);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x00, 0x01);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x80, 0x00);

   return NULL;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t
vl53l0x_getMeasurementTimingBudget (vl53l0x_t * v)
{
   SequenceStepEnables enables;
   SequenceStepTimeouts timeouts;

   uint16_t const StartOverhead = 1910; // note that this is different than the value in set_
   uint16_t const EndOverhead = 960;
   uint16_t const MsrcOverhead = 660;
   uint16_t const TccOverhead = 590;
   uint16_t const DssOverhead = 690;
   uint16_t const PreRangeOverhead = 660;
   uint16_t const FinalRangeOverhead = 550;

   // "Start and end overhead times always present"
   uint32_t budget_us = StartOverhead + EndOverhead;

   getSequenceStepEnables (v, &enables);
   getSequenceStepTimeouts (v, &enables, &timeouts);

   if (enables.tcc)
   {
      budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
   }

   if (enables.dss)
   {
      budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
   } else if (enables.msrc)
   {
      budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
   }

   if (enables.pre_range)
   {
      budget_us += (timeouts.pre_range_us + PreRangeOverhead);
   }

   if (enables.final_range)
   {
      budget_us += (timeouts.final_range_us + FinalRangeOverhead);
   }

   measurement_timing_budget_us = budget_us;    // store for internal reuse
   return budget_us;
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
const char *
vl53l0x_setMeasurementTimingBudget (vl53l0x_t * v, uint32_t budget_us)
{
   SequenceStepEnables enables;
   SequenceStepTimeouts timeouts;

   uint16_t const StartOverhead = 1320; // note that this is different than the value in get_
   uint16_t const EndOverhead = 960;
   uint16_t const MsrcOverhead = 660;
   uint16_t const TccOverhead = 590;
   uint16_t const DssOverhead = 690;
   uint16_t const PreRangeOverhead = 660;
   uint16_t const FinalRangeOverhead = 550;

   uint32_t const MinTimingBudget = 20000;

   if (budget_us < MinTimingBudget)
      return "Low budget";

   uint32_t used_budget_us = StartOverhead + EndOverhead;

   getSequenceStepEnables (v, &enables);
   getSequenceStepTimeouts (v, &enables, &timeouts);

   if (enables.tcc)
      used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);

   if (enables.dss)
      used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
   else if (enables.msrc)
      used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);

   if (enables.pre_range)
      used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);

   if (enables.final_range)
   {
      used_budget_us += FinalRangeOverhead;

      // "Note that the final range timeout is determined by the timing
      // budget and the sum of all other timeouts within the sequence.
      // If there is no room for the final range timeout, then an error
      // will be set. Otherwise the remaining time will be applied to
      // the final range."

      if (used_budget_us > budget_us)
         return "High budget";  // "Requested timeout too big."

      uint32_t final_range_timeout_us = budget_us - used_budget_us;

      // set_sequence_step_timeout() begin
      // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

      // "For the final range timeout, the pre-range timeout
      //  must be added. To do this both final and pre-range
      //  timeouts must be expressed in macro periods MClks
      //  because they have different vcsel periods."

      uint16_t
         final_range_timeout_mclks = timeoutMicrosecondsToMclks (final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

      if (enables.pre_range)
         final_range_timeout_mclks += timeouts.pre_range_mclks;

      vl53l0x_writeReg16Bit (v, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout (final_range_timeout_mclks));

      // set_sequence_step_timeout() end

      measurement_timing_budget_us = budget_us; // store for internal reuse
   }
   return NULL;
}

vl53l0x_t *
vl53l0x_config (int8_t port, int8_t scl, int8_t sda, int8_t xshut, uint8_t address, uint8_t io_2v8)
{
   if (port < 0 || scl < 0 || sda < 0 || scl == sda)
      return NULL;
   if (!GPIO_IS_VALID_OUTPUT_GPIO (scl) || !GPIO_IS_VALID_OUTPUT_GPIO (sda) || (xshut >= 0 && !GPIO_IS_VALID_OUTPUT_GPIO (xshut)))
      return 0;
   if (i2c_driver_install (port, I2C_MODE_MASTER, 0, 0, 0))
      return NULL;              // Uh?
   i2c_config_t config = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = sda,
      .scl_io_num = scl,
      .sda_pullup_en = true,
      .scl_pullup_en = true,
      .master.clk_speed = 100000,
   };
   if (i2c_param_config (port, &config))
   {                            // Config failed
      i2c_driver_delete (port);
      return NULL;
   }
   i2c_set_timeout (port, 80000);       // Clock stretching
   i2c_filter_enable (port, 5);
   if (xshut >= 0)
   {
      gpio_reset_pin (xshut);
      gpio_set_level (xshut, 0);        // Off
      gpio_set_drive_capability (xshut, GPIO_DRIVE_CAP_3);
      gpio_set_direction (xshut, GPIO_MODE_OUTPUT);
   }
   vl53l0x_t *v = malloc (sizeof (*v));
   if (!v)
   {                            // Uh?
      i2c_driver_delete (port);
      return v;
   }
   memset (v, 0, sizeof (*v));
   v->xshut = xshut;
   v->io_2v8 = io_2v8;
   v->port = port;
   v->address = address;
   v->io_timeout = 100;
   return v;
}

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
const char *
vl53l0x_init (vl53l0x_t * v)
{
   const char *err;
   // Set up the VL53L0X
   if (v->xshut >= 0)
   {                            // XSHUT or power control
      gpio_set_level (v->xshut, 0);     // Off
      usleep (100000);
      gpio_set_level (v->xshut, 0);     // On
      usleep (10000);           // Plenty of time to boot (data sheet says 1.2ms)
   }
   // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
   if (v->io_2v8)
      vl53l0x_writeReg8Bit (v, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, vl53l0x_readReg8Bit (v, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);   // set bit 0
   // "Set I2C standard mode"
   vl53l0x_writeReg8Bit (v, 0x88, 0x00);

   vl53l0x_writeReg8Bit (v, 0x80, 0x01);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x00, 0x00);
   stop_variable = vl53l0x_readReg8Bit (v, 0x91);
   vl53l0x_writeReg8Bit (v, 0x00, 0x01);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x80, 0x00);

   // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
   vl53l0x_writeReg8Bit (v, MSRC_CONFIG_CONTROL, vl53l0x_readReg8Bit (v, MSRC_CONFIG_CONTROL) | 0x12);

   // set final range signal rate limit to 0.25 MCPS (million counts per second)
   if ((err = vl53l0x_setSignalRateLimit (v, 0.25)))
      return err;

   vl53l0x_writeReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, 0xFF);

   // VL53L0X_DataInit() end

   // VL53L0X_StaticInit() begin

   uint8_t spad_count;
   int spad_type_is_aperture;
   if ((err = vl53l0x_getSpadInfo (v, &spad_count, &spad_type_is_aperture)))
      return err;
   // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
   // the API, but the same data seems to be more easily readable from
   // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
   uint8_t ref_spad_map[6];
   vl53l0x_readMulti (v, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

   // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
   vl53l0x_writeReg8Bit (v, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

   uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;       // 12 is the first aperture spad
   uint8_t spads_enabled = 0;

   for (uint8_t i = 0; i < 48; i++)
   {
      if (i < first_spad_to_enable || spads_enabled == spad_count)
      {
         // This bit is lower than the first one that should be enabled, or
         // (reference_spad_count) bits have already been enabled, so zero this bit
         ref_spad_map[i / 8] &= ~(1 << (i % 8));
      } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
      {
         spads_enabled++;
      }
   }

   vl53l0x_writeMulti (v, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

   // -- VL53L0X_set_reference_spads() end

   // -- VL53L0X_load_tuning_settings() begin
   // DefaultTuningSettings from vl53l0x_tuning.h

   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x00, 0x00);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x09, 0x00);
   vl53l0x_writeReg8Bit (v, 0x10, 0x00);
   vl53l0x_writeReg8Bit (v, 0x11, 0x00);

   vl53l0x_writeReg8Bit (v, 0x24, 0x01);
   vl53l0x_writeReg8Bit (v, 0x25, 0xFF);
   vl53l0x_writeReg8Bit (v, 0x75, 0x00);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x4E, 0x2C);
   vl53l0x_writeReg8Bit (v, 0x48, 0x00);
   vl53l0x_writeReg8Bit (v, 0x30, 0x20);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x30, 0x09);
   vl53l0x_writeReg8Bit (v, 0x54, 0x00);
   vl53l0x_writeReg8Bit (v, 0x31, 0x04);
   vl53l0x_writeReg8Bit (v, 0x32, 0x03);
   vl53l0x_writeReg8Bit (v, 0x40, 0x83);
   vl53l0x_writeReg8Bit (v, 0x46, 0x25);
   vl53l0x_writeReg8Bit (v, 0x60, 0x00);
   vl53l0x_writeReg8Bit (v, 0x27, 0x00);
   vl53l0x_writeReg8Bit (v, 0x50, 0x06);
   vl53l0x_writeReg8Bit (v, 0x51, 0x00);
   vl53l0x_writeReg8Bit (v, 0x52, 0x96);
   vl53l0x_writeReg8Bit (v, 0x56, 0x08);
   vl53l0x_writeReg8Bit (v, 0x57, 0x30);
   vl53l0x_writeReg8Bit (v, 0x61, 0x00);
   vl53l0x_writeReg8Bit (v, 0x62, 0x00);
   vl53l0x_writeReg8Bit (v, 0x64, 0x00);
   vl53l0x_writeReg8Bit (v, 0x65, 0x00);
   vl53l0x_writeReg8Bit (v, 0x66, 0xA0);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x22, 0x32);
   vl53l0x_writeReg8Bit (v, 0x47, 0x14);
   vl53l0x_writeReg8Bit (v, 0x49, 0xFF);
   vl53l0x_writeReg8Bit (v, 0x4A, 0x00);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x7A, 0x0A);
   vl53l0x_writeReg8Bit (v, 0x7B, 0x00);
   vl53l0x_writeReg8Bit (v, 0x78, 0x21);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x23, 0x34);
   vl53l0x_writeReg8Bit (v, 0x42, 0x00);
   vl53l0x_writeReg8Bit (v, 0x44, 0xFF);
   vl53l0x_writeReg8Bit (v, 0x45, 0x26);
   vl53l0x_writeReg8Bit (v, 0x46, 0x05);
   vl53l0x_writeReg8Bit (v, 0x40, 0x40);
   vl53l0x_writeReg8Bit (v, 0x0E, 0x06);
   vl53l0x_writeReg8Bit (v, 0x20, 0x1A);
   vl53l0x_writeReg8Bit (v, 0x43, 0x40);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x34, 0x03);
   vl53l0x_writeReg8Bit (v, 0x35, 0x44);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x31, 0x04);
   vl53l0x_writeReg8Bit (v, 0x4B, 0x09);
   vl53l0x_writeReg8Bit (v, 0x4C, 0x05);
   vl53l0x_writeReg8Bit (v, 0x4D, 0x04);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x44, 0x00);
   vl53l0x_writeReg8Bit (v, 0x45, 0x20);
   vl53l0x_writeReg8Bit (v, 0x47, 0x08);
   vl53l0x_writeReg8Bit (v, 0x48, 0x28);
   vl53l0x_writeReg8Bit (v, 0x67, 0x00);
   vl53l0x_writeReg8Bit (v, 0x70, 0x04);
   vl53l0x_writeReg8Bit (v, 0x71, 0x01);
   vl53l0x_writeReg8Bit (v, 0x72, 0xFE);
   vl53l0x_writeReg8Bit (v, 0x76, 0x00);
   vl53l0x_writeReg8Bit (v, 0x77, 0x00);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x0D, 0x01);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x80, 0x01);
   vl53l0x_writeReg8Bit (v, 0x01, 0xF8);

   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x8E, 0x01);
   vl53l0x_writeReg8Bit (v, 0x00, 0x01);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x80, 0x00);

   // -- VL53L0X_load_tuning_settings() end

   // "Set interrupt config to new sample ready"
   // -- VL53L0X_SetGpioConfig() begin

   vl53l0x_writeReg8Bit (v, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
   vl53l0x_writeReg8Bit (v, GPIO_HV_MUX_ACTIVE_HIGH, vl53l0x_readReg8Bit (v, GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
   vl53l0x_writeReg8Bit (v, SYSTEM_INTERRUPT_CLEAR, 0x01);

   // -- VL53L0X_SetGpioConfig() end

   measurement_timing_budget_us = vl53l0x_getMeasurementTimingBudget (v);

   // "Disable MSRC and TCC by default"
   // MSRC = Minimum Signal Rate Check
   // TCC = Target CentreCheck
   // -- VL53L0X_SetSequenceStepEnable() begin

   vl53l0x_writeReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, 0xE8);

   // -- VL53L0X_SetSequenceStepEnable() end

   // "Recalculate timing budget"
   if ((err = vl53l0x_setMeasurementTimingBudget (v, measurement_timing_budget_us)))
      return err;

   // VL53L0X_StaticInit() end

   // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

   // -- VL53L0X_perform_vhv_calibration() begin

   vl53l0x_writeReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, 0x01);
   if ((err = performSingleRefCalibration (v, 0x40)))
      return err;
   // -- VL53L0X_perform_vhv_calibration() end

   // -- VL53L0X_perform_phase_calibration() begin

   vl53l0x_writeReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, 0x02);
   if ((err = performSingleRefCalibration (v, 0x00)))
      return err;
   // -- VL53L0X_perform_phase_calibration() end

   // "restore the previous Sequence Config"
   vl53l0x_writeReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, 0xE8);

   // VL53L0X_PerformRefCalibration() end
   if (vl53l0x_i2cFail (v))
      return "I2C fail";

   return NULL;
}

void
vl53l0x_end (vl53l0x_t * v)
{
   if (!v)
      return;
   i2c_driver_delete (v->port);
   free (v);
}

void
vl53l0x_setAddress (vl53l0x_t * v, uint8_t new_addr)
{
   vl53l0x_writeReg8Bit (v, I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
   v->address = new_addr;
}

uint8_t
vl53l0x_getAddress (vl53l0x_t * v)
{
   return v->address;
}

void
vl53l0x_setTimeout (vl53l0x_t * v, uint16_t new_timeout)
{
   v->io_timeout = new_timeout;
}

uint16_t
vl53l0x_getTimeout (vl53l0x_t * v)
{
   return v->io_timeout;
}

int
vl53l0x_timeoutOccurred (vl53l0x_t * v)
{
   int tmp = v->did_timeout;
   v->did_timeout = 0;
   return tmp;
}

int
vl53l0x_i2cFail (vl53l0x_t * v)
{
   int tmp = v->i2c_fail;
   v->i2c_fail = 0;
   return tmp;
}

float
vl53l0x_getSignalRateLimit (vl53l0x_t * v)
{
   return (float) vl53l0x_readReg16Bit (v, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
const char *
vl53l0x_setVcselPulsePeriod (vl53l0x_t * v, vl53l0x_vcselPeriodType type, uint8_t period_pclks)
{
   uint8_t vcsel_period_reg = encodeVcselPeriod (period_pclks);

   SequenceStepEnables enables;
   SequenceStepTimeouts timeouts;

   getSequenceStepEnables (v, &enables);
   getSequenceStepTimeouts (v, &enables, &timeouts);

   // "Apply specific settings for the requested clock period"
   // "Re-calculate and apply timeouts, in macro periods"

   // "When the VCSEL period for the pre or final range is changed,
   // the corresponding timeout must be read from the device using
   // the current VCSEL period, then the new VCSEL period can be
   // applied. The timeout then must be written back to the device
   // using the new VCSEL period.
   //
   // For the MSRC timeout, the same applies - this timeout being
   // dependant on the pre-range vcsel period."


   if (type == VcselPeriodPreRange)
   {
      // "Set phase check limits"
      switch (period_pclks)
      {
      case 12:
         vl53l0x_writeReg8Bit (v, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
         break;

      case 14:
         vl53l0x_writeReg8Bit (v, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
         break;

      case 16:
         vl53l0x_writeReg8Bit (v, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
         break;

      case 18:
         vl53l0x_writeReg8Bit (v, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
         break;

      default:
         // invalid period
         return "Invalid period";
      }
      vl53l0x_writeReg8Bit (v, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

      // apply new VCSEL period
      vl53l0x_writeReg8Bit (v, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

      // update timeouts

      // set_sequence_step_timeout() begin
      // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

      uint16_t new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks (timeouts.pre_range_us, period_pclks);

      vl53l0x_writeReg16Bit (v, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout (new_pre_range_timeout_mclks));

      // set_sequence_step_timeout() end

      // set_sequence_step_timeout() begin
      // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

      uint16_t new_msrc_timeout_mclks = timeoutMicrosecondsToMclks (timeouts.msrc_dss_tcc_us, period_pclks);

      vl53l0x_writeReg8Bit (v, MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

      // set_sequence_step_timeout() end
   } else if (type == VcselPeriodFinalRange)
   {
      switch (period_pclks)
      {
      case 8:
         vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
         vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
         vl53l0x_writeReg8Bit (v, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
         vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
         vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
         vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_LIM, 0x30);
         vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
         break;

      case 10:
         vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
         vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
         vl53l0x_writeReg8Bit (v, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
         vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
         vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
         vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_LIM, 0x20);
         vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
         break;

      case 12:
         vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
         vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
         vl53l0x_writeReg8Bit (v, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
         vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
         vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
         vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_LIM, 0x20);
         vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
         break;

      case 14:
         vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
         vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
         vl53l0x_writeReg8Bit (v, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
         vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
         vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
         vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_LIM, 0x20);
         vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
         break;

      default:
         // invalid period
         return "Invalid period";
      }

      // apply new VCSEL period
      vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

      // update timeouts

      // set_sequence_step_timeout() begin
      // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

      // "For the final range timeout, the pre-range timeout
      //  must be added. To do this both final and pre-range
      //  timeouts must be expressed in macro periods MClks
      //  because they have different vcsel periods."

      uint16_t new_final_range_timeout_mclks = timeoutMicrosecondsToMclks (timeouts.final_range_us, period_pclks);

      if (enables.pre_range)
         new_final_range_timeout_mclks += timeouts.pre_range_mclks;

      vl53l0x_writeReg16Bit (v, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout (new_final_range_timeout_mclks));

      // set_sequence_step_timeout end
   } else
      return "Invalid type";

   // "Finally, the timing budget must be re-applied"
   const char *err;
   if ((err = vl53l0x_setMeasurementTimingBudget (v, measurement_timing_budget_us)))
      return err;

   // "Perform the phase calibration. This is needed after changing on vcsel period."
   // VL53L0X_perform_phase_calibration() begin

   uint8_t sequence_config = vl53l0x_readReg8Bit (v, SYSTEM_SEQUENCE_CONFIG);
   vl53l0x_writeReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, 0x02);
   performSingleRefCalibration (v, 0x0);
   vl53l0x_writeReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, sequence_config);

   // VL53L0X_perform_phase_calibration() end

   return NULL;
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void
vl53l0x_startContinuous (vl53l0x_t * v, uint32_t period_ms)
{
   vl53l0x_writeReg8Bit (v, 0x80, 0x01);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x00, 0x00);
   vl53l0x_writeReg8Bit (v, 0x91, stop_variable);
   vl53l0x_writeReg8Bit (v, 0x00, 0x01);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x80, 0x00);

   if (period_ms != 0)
   {
      // continuous timed mode

      // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

      uint16_t osc_calibrate_val = vl53l0x_readReg16Bit (v, OSC_CALIBRATE_VAL);

      if (osc_calibrate_val != 0)
      {
         period_ms *= osc_calibrate_val;
      }

      vl53l0x_writeReg32Bit (v, SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

      // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

      vl53l0x_writeReg8Bit (v, SYSRANGE_START, 0x04);   // VL53L0X_REG_SYSRANGE_MODE_TIMED
   } else
   {
      // continuous back-to-back mode
      vl53l0x_writeReg8Bit (v, SYSRANGE_START, 0x02);   // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
   }
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void
vl53l0x_stopContinuous (vl53l0x_t * v)
{
   vl53l0x_writeReg8Bit (v, SYSRANGE_START, 0x01);      // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x00, 0x00);
   vl53l0x_writeReg8Bit (v, 0x91, 0x00);
   vl53l0x_writeReg8Bit (v, 0x00, 0x01);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
uint16_t
vl53l0x_readRangeContinuousMillimeters (vl53l0x_t * v)
{
   startTimeout ();
   while ((vl53l0x_readReg8Bit (v, RESULT_INTERRUPT_STATUS) & 0x07) == 0)
   {
      if (checkTimeoutExpired ())
      {
         v->did_timeout = 1;
         return 65535;
      }
   }
   // assumptions: Linearity Corrective Gain is 1000 (default);
   // fractional ranging is not enabled
   uint16_t range = vl53l0x_readReg16Bit (v, RESULT_RANGE_STATUS + 10);
   vl53l0x_writeReg8Bit (v, SYSTEM_INTERRUPT_CLEAR, 0x01);
   return range;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
uint16_t
vl53l0x_readRangeSingleMillimeters (vl53l0x_t * v)
{
   vl53l0x_writeReg8Bit (v, 0x80, 0x01);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x01);
   vl53l0x_writeReg8Bit (v, 0x00, 0x00);
   vl53l0x_writeReg8Bit (v, 0x91, stop_variable);
   vl53l0x_writeReg8Bit (v, 0x00, 0x01);
   vl53l0x_writeReg8Bit (v, 0xFF, 0x00);
   vl53l0x_writeReg8Bit (v, 0x80, 0x00);

   vl53l0x_writeReg8Bit (v, SYSRANGE_START, 0x01);

   // "Wait until start bit has been cleared"
   startTimeout ();
   while (vl53l0x_readReg8Bit (v, SYSRANGE_START) & 0x01)
   {
      if (checkTimeoutExpired ())
      {
         v->did_timeout = 1;
         return 65535;
      }
   }

   return vl53l0x_readRangeContinuousMillimeters (v);
}
