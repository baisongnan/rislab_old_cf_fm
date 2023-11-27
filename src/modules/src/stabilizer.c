/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#define DEBUG_MODULE "STAB"

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "motors.h"
#include "pm.h"

#include "stabilizer.h"

#include "sensors.h"
#include "commander.h"
#include "crtp_localization_service.h"
#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"
// #include "collision_avoidance.h"

#include "estimator.h"
#include "usddeck.h"
#include "quatcompress.h"
#include "statsCnt.h"
#include "static_mem.h"
#include "rateSupervisor.h"


//my code servo control..start
static uint8_t servoRatio_stabilizer = 243;
uint8_t getservoRatio()
{
  return servoRatio_stabilizer;
}
//my code servo control..end

static bool isInit;
static bool emergencyStop = false;
// static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;

static bool checkStops;

#define PROPTEST_NBR_OF_VARIANCE_VALUES 100
static bool startPropTest = false;

uint32_t inToOutLatency;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

static float attitude_control_limit;
static float idle_thrust;
static float autorotate_thrust;
static float autorotate_thrust_2;

static StateEstimatorType estimatorType;
static ControllerType controllerType;

// ----- my variables start -----
static float attitude_control_limit;
bool thrust_flag;

// static float control_kp;
// static float control_kd;
// static float control_kdz;

static float kp_xy = 6000;
static float kp_xy_temp = 6000;
static float kp_z = 6000;
static float kp_z_temp = 6000;

// static float angle_error_threshold = 1.57f;
// static float angle_error_velocity = 300.0f;

static float kd_xy = 10;
static float kd_z = 10;

static float tau_x_offset = 0.0f;
static float tau_y_offset = 0.0f;
static float tau_z_offset = 0.0f;

static float tau_x = 0.0f;
static float tau_y = 0.0f;
static float tau_z = 0.0f;

static float omega_x = 0.0f;
static float omega_y = 0.0f;
static float omega_z = 0.0f;

static float qw_desired = 1.0f;
static float qx_desired = 0.0f;
static float qy_desired = 0.0f;
static float qz_desired = 0.0f;

static float qw_desired_delay = 1.0f;
static float qx_desired_delay = 0.0f;
static float qy_desired_delay = 0.0f;
static float qz_desired_delay = 0.0f;

uint32_t timestamp_setpoint = 0;

static float external_loop_freq = 100.0f;

float limint16(float in)
{
  if (in > 32000.0f)
    return 32000.0f;
  else if (in < -32000.0f)
    return -32000.0f;
  else
    return in;
}

float lim_num(float in, float num)
{
  if (in > num)
    return num;
  else if (in < -num)
    return -num;
  else
    return in;
}

// Function to determine if two quaternions represent the same attitude
bool same_attitude(float w, float x, float y, float z, float w_d, float x_d, float y_d, float z_d)
{
  // Calculate the dot product of the two quaternions
  float dot_product = w * w_d + x * x_d + y * y_d + z * z_d;
  // Check if the absolute value of the dot product is close to 1
  return fabsf(dot_product) > 0.999999f;
}
// ----- my variables end   -----

void pcontrol(float w, float x, float y, float z, float w_d, float x_d,
              float y_d, float z_d, float *tau_x, float *tau_y, float *tau_z)
{

  // if (w*w_d<0.0f)
  // {
  //   w_d = -w_d;
  //   x_d = -x_d;
  //   y_d = -y_d;
  //   z_d = -z_d;
  // }

  if (same_attitude(w, x, y, z, w_d, x_d, y_d, z_d))
  {
    *tau_x = 0.0F;
    *tau_y = 0.0F;
    *tau_z = 0.0F;
  }
  else
  {
    float axang1;
    float axang2;
    float axang3;
    float axang4;
    float rot1;
    float temp_1;

    temp_1 = ((w * w_d + x * x_d) + y * y_d) + z * z_d;

    if (temp_1 == 1.0F)
    {
      /*  axang = [0, 0, 1, 0]; */
      axang1 = 0.0F;
      axang2 = 0.0F;
      axang3 = 1.0F;
      axang4 = 0.0F;
    }
    else
    {

      axang4 = sqrtf(1.0F - temp_1 * temp_1);
      axang1 = (((w * x_d - w_d * x) + y * z_d) - y_d * z) / axang4;
      axang2 = (((w * y_d - w_d * y) - x * z_d) + x_d * z) / axang4;
      axang3 = (((w * z_d - w_d * z) + x * y_d) - x_d * y) / axang4;
      axang4 = 2.0F * acosf(temp_1);
    }

    if (axang4 > 3.1415926535897931f)
    {
      axang4 = 6.28318548F - axang4;
      axang1 = -axang1;
      axang2 = -axang2;
      axang3 = -axang3;
    }
    else if (axang4 < -3.1415926535897931f)
    {
      axang4 += 6.28318548F + axang4;
      axang1 = -axang1;
      axang2 = -axang2;
      axang3 = -axang3;
    }

    rot1 = axang1 * axang4;
    axang1 = axang2 * axang4;
    temp_1 = axang3 * axang4;

    axang4 = (rot1 * x + axang1 * y) + temp_1 * z;
    axang2 = (rot1 * w - temp_1 * y) + axang1 * z;
    axang3 = (axang1 * w + temp_1 * x) - rot1 * z;
    temp_1 = (temp_1 * w - axang1 * x) + rot1 * y;

    *tau_x = ((w * axang2 - y * temp_1) + x * axang4) + z * axang3;
    *tau_y = ((w * axang3 + x * temp_1) + y * axang4) - z * axang2;
    *tau_z = ((w * temp_1 - x * axang3) + y * axang2) + z * axang4;
  }
}

void eul2quat_my(float yaw, float pitch, float roll,
                 float *w, float *x, float *y, float *z)
{
  float c1c2;
  float c_idx_0;
  float c_idx_1;
  float c_idx_2;
  float s1s2;
  float s_idx_0;
  float s_idx_1;
  float s_idx_2;
  s_idx_0 = yaw / 2.0F;
  s_idx_1 = pitch / 2.0F;
  s_idx_2 = roll / 2.0F;
  c_idx_0 = cosf(s_idx_0);
  s_idx_0 = sinf(s_idx_0);
  c_idx_1 = cosf(s_idx_1);
  s_idx_1 = sinf(s_idx_1);
  c_idx_2 = cosf(s_idx_2);
  s_idx_2 = sinf(s_idx_2);
  c1c2 = c_idx_0 * c_idx_1;
  s1s2 = s_idx_0 * s_idx_1;
  c_idx_0 *= s_idx_1;
  s_idx_1 = s_idx_0 * c_idx_1;
  *w = c1c2 * c_idx_2 + s1s2 * s_idx_2;
  *x = c1c2 * s_idx_2 - s1s2 * c_idx_2;
  *y = c_idx_0 * c_idx_2 + s_idx_1 * s_idx_2;
  *z = s_idx_1 * c_idx_2 - c_idx_0 * s_idx_2;
}


typedef enum
{
  configureAcc,
  measureNoiseFloor,
  measureProp,
  testBattery,
  restartBatTest,
  evaluateResult,
  testDone
} TestState;
#ifdef RUN_PROP_TEST_AT_STARTUP
static TestState testState = configureAcc;
#else
static TestState testState = testDone;
#endif

static STATS_CNT_RATE_DEFINE(stabilizerRate, 500);
static rateSupervisor_t rateSupervisorContext;
static bool rateWarningDisplayed = false;

static struct
{
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
  // compressed quaternion, see quatcompress.h
  int32_t quat;
  // angular velocity - milliradians / sec
  int16_t rateRoll;
  int16_t ratePitch;
  int16_t rateYaw;
} stateCompressed;

static struct
{
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
} setpointCompressed;

static float accVarX[NBR_OF_MOTORS];
static float accVarY[NBR_OF_MOTORS];
static float accVarZ[NBR_OF_MOTORS];
// Bit field indicating if the motors passed the motor test.
// Bit 0 - 1 = M1 passed
// Bit 1 - 1 = M2 passed
// Bit 2 - 1 = M3 passed
// Bit 3 - 1 = M4 passed
static uint8_t motorPass = 0;
static uint16_t motorTestCount = 0;

STATIC_MEM_TASK_ALLOC(stabilizerTask, STABILIZER_TASK_STACKSIZE);

static void stabilizerTask(void *param);
static void testProps(sensorData_t *sensors);

// ----- my functions start -----

// ----- my functions end   -----

static void calcSensorToOutputLatency(const sensorData_t *sensorData)
{
  uint64_t outTimestamp = usecTimestamp();
  inToOutLatency = outTimestamp - sensorData->interruptTimestamp;
}


static void compressState()
{
  // stateCompressed.x = state.position.x * 1000.0f;
  // stateCompressed.y = state.position.y * 1000.0f;
  // stateCompressed.z = state.position.z * 1000.0f;

  // stateCompressed.vx = state.velocity.x * 1000.0f;
  // stateCompressed.vy = state.velocity.y * 1000.0f;
  // stateCompressed.vz = state.velocity.z * 1000.0f;

  // stateCompressed.ax = state.acc.x * 9.81f * 1000.0f;
  // stateCompressed.ay = state.acc.y * 9.81f * 1000.0f;
  // stateCompressed.az = (state.acc.z + 1) * 9.81f * 1000.0f;
  stateCompressed.ax = sensorData.acc.x * 9810.0f;
  stateCompressed.ay = sensorData.acc.y * 9810.0f;
  stateCompressed.az = (sensorData.acc.z - 1) * 9810.0f;

  float const q[4] = {
      state.attitudeQuaternion.x,
      state.attitudeQuaternion.y,
      state.attitudeQuaternion.z,
      state.attitudeQuaternion.w};
  stateCompressed.quat = quatcompress(q);

  float const deg2millirad = ((float)M_PI * 1000.0f) / 180.0f;
  stateCompressed.rateRoll = sensorData.gyro.x * deg2millirad;
  stateCompressed.ratePitch = -sensorData.gyro.y * deg2millirad;
  // stateCompressed.rateYaw = sensorData.gyro.z * deg2millirad;
}

// static void compressSetpoint()
// {
//   setpointCompressed.x = setpoint.position.x * 1000.0f;
//   setpointCompressed.y = setpoint.position.y * 1000.0f;
//   setpointCompressed.z = setpoint.position.z * 1000.0f;
//   setpointCompressed.vx = setpoint.velocity.x * 1000.0f;
//   setpointCompressed.vy = setpoint.velocity.y * 1000.0f;
//   setpointCompressed.vz = setpoint.velocity.z * 1000.0f;
//   setpointCompressed.ax = setpoint.acceleration.x * 1000.0f;
//   setpointCompressed.ay = setpoint.acceleration.y * 1000.0f;
//   setpointCompressed.az = setpoint.acceleration.z * 1000.0f;
// }

void stabilizerInit(StateEstimatorType estimator)
{
  if (isInit)
    return;

  sensorsInit();
  stateEstimatorInit(estimator);
  controllerInit(ControllerTypeAny);
  powerDistributionInit();
  sitAwInit();
  // collisionAvoidanceInit();
  estimatorType = 1;
  controllerType = getControllerType();

  STATIC_MEM_TASK_CREATE(stabilizerTask, stabilizerTask, STABILIZER_TASK_NAME, NULL, STABILIZER_TASK_PRI);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  // pass &= controllerTest();
  pass &= powerDistributionTest();
  // pass &= collisionAvoidanceTest();

  return pass;
}

// static void checkEmergencyStopTimeout()
// {
//   if (emergencyStopTimeout >= 0)
//   {
//     emergencyStopTimeout -= 1;
//     if (emergencyStopTimeout == 0)
//     {
//       emergencyStop = true;
//     }
//   }
// }

/* The stabilizer loop runs at 1kHz (stock) or 500Hz (kalman). It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */

static void stabilizerTask(void *param)
{
  uint32_t tick;
  uint32_t lastWakeTime;
  vTaskSetApplicationTaskTag(0, (void *)TASK_STABILIZER_ID_NBR);

  // Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  DEBUG_PRINT("Wait for sensor calibration...\n");

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount();
  while (!sensorsAreCalibrated())
  {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }
  // Initialize tick to something else then 0
  tick = 1;

  rateSupervisorInit(&rateSupervisorContext, xTaskGetTickCount(), M2T(1000), 997, 1003, 1);

  DEBUG_PRINT("Ready to fly.\n");

  idle_thrust = 1500.0f;
  autorotate_thrust = 1550.0f;
  autorotate_thrust_2 = 1580.0f;

  attitude_control_limit = 1300.0f;
  thrust_flag = true;

  while (1)
  {
    // The sensor should unlock at 1kHz
    sensorsWaitDataReady();

    if (startPropTest != false)
    {
      // TODO: What happens with estimator when we run tests after startup?
      testState = configureAcc;
      startPropTest = false;
    }

    if (testState != testDone)
    {
      sensorsAcquire(&sensorData, tick);
      testProps(&sensorData);
    }
    else
    {
      // allow to update estimator dynamically
      if (getStateEstimator() != estimatorType)
      {
        stateEstimatorSwitchTo(estimatorType);
        estimatorType = getStateEstimator();
      }
      // allow to update controller dynamically
      // if (getControllerType() != controllerType)
      // {
      //   controllerInit(controllerType);
      //   controllerType = getControllerType();
      // }

      stateEstimator(&state, &sensorData, &control, tick);
      compressState();

      commanderGetSetpoint(&setpoint, &state);
      // compressSetpoint();

      sitAwUpdateSetpoint(&setpoint, &sensorData, &state);

      // collisionAvoidanceUpdateSetpoint(&setpoint, &sensorData, &state, tick);
      
      if (setpoint.attitudeRate.yaw > 180.0f) 
      {
        servoRatio_stabilizer=243;
        setpoint.attitudeRate.yaw = 0.0f;
      }
      else if (setpoint.attitudeRate.yaw < -180.0f) 
      {
        servoRatio_stabilizer=230;
        setpoint.attitudeRate.yaw = 0.0f;
      }


      // controller(&control, &setpoint, &sensorData, &state, tick);

      // disable P controller when thrust is equal to attitude_control_limit
      if (fabsf(setpoint.thrust - attitude_control_limit) < 10.0f)
      {
        kp_xy_temp = 0.0f;
        kp_z_temp = 0.0f;
      }
      else
      {
        kp_xy_temp = kp_xy;
        kp_z_temp = kp_z;
      }

      if (timestamp_setpoint == setpoint.timestamp)
      {
        // no control input is received
        ;
      }
      else
      {
        // control input is received
        if (setpoint.timestamp > timestamp_setpoint)
          // time_gap_setpoint = setpoint.timestamp - timestamp_setpoint;

          timestamp_setpoint = setpoint.timestamp;

        qw_desired_delay = qw_desired;
        qx_desired_delay = qx_desired;
        qy_desired_delay = qy_desired;
        qz_desired_delay = qz_desired;

        // compute desired quat
        if (fabsf(autorotate_thrust - setpoint.thrust) < 2.0f)
          ;
        else
        {
          eul2quat_my(setpoint.attitudeRate.yaw * -0.0174532925199433f,
                      setpoint.attitude.pitch * -0.0174532925199433f,
                      setpoint.attitude.roll * 0.0174532925199433f,
                      &qw_desired,
                      &qx_desired,
                      &qy_desired,
                      &qz_desired);
        }

        pcontrol(qw_desired_delay,
                 qx_desired_delay,
                 qy_desired_delay,
                 qz_desired_delay,
                 qw_desired,
                 qx_desired,
                 qy_desired,
                 qz_desired,
                 &omega_x, &omega_y, &omega_z);

        // // desired angular rate in degrees
        omega_x = omega_x * 57.2957795130823f * external_loop_freq;
        omega_y = omega_y * 57.2957795130823f * external_loop_freq;
        omega_z = omega_z * 57.2957795130823f * external_loop_freq;

        omega_x = lim_num(omega_x, 300);
        omega_y = lim_num(omega_y, 300);
        omega_z = lim_num(omega_z, 300);
      }
      if (fabsf(setpoint.thrust - idle_thrust) < 10.0f)
      {
        control.thrust = 500.0f;
        control.roll = 0.0f;
        control.pitch = 0.0f;
        control.yaw = 0.0f;
      }
      else if (setpoint.thrust >= 10.0f)
      {

        pcontrol(state.attitudeQuaternion.w,
                 state.attitudeQuaternion.x,
                 state.attitudeQuaternion.y,
                 state.attitudeQuaternion.z,
                 qw_desired,
                 qx_desired,
                 qy_desired,
                 qz_desired,
                 &tau_x, &tau_y, &tau_z);

        // float angle_error = sqrtf(tau_x * tau_x + tau_y * tau_y + tau_z * tau_z);

        // if (angle_error > angle_error_threshold)
        // {
        //   omega_x = (tau_x/angle_error) * angle_error_velocity;
        //   omega_y = (tau_y/angle_error) * angle_error_velocity;
        //   omega_z = (tau_z/angle_error) * angle_error_velocity;
        // }
        // else
        // {
        //   omega_x = 0.0f;
        //   omega_y = 0.0f;
        //   omega_z = 0.0f;
        // }

        tau_x = tau_x + tau_x_offset;
        tau_y = tau_y + tau_y_offset;
        tau_z = tau_z + tau_z_offset;

        if (fabsf(autorotate_thrust - setpoint.thrust) < 2.0f)
        {
          control.thrust = setpoint.attitude.roll * 100.0f;
          control.roll = 0.0f;
          control.pitch = 0.0f;
          control.yaw = 0.0f;
        }
        // else if (fabsf(autorotate_thrust_2 - setpoint.thrust) < 2.0f)
        // {
        //   control.thrust = setpoint.thrust;
        //   control.roll = (int16_t)limint16(tau_x * kp_xy_temp + (omega_x - sensorData.gyro.x) * kd_xy);
        //   control.pitch = -(int16_t)limint16(tau_y * kp_xy_temp + (omega_y - sensorData.gyro.y) * kd_xy);
        //   control.yaw = -(int16_t)limint16(tau_z * kp_z + (omega_z - sensorData.gyro.z) * kd_z);
        // }
        else
        {
          control.thrust = setpoint.thrust;
          control.roll = (int16_t)limint16(tau_x * kp_xy_temp + (omega_x - sensorData.gyro.x) * kd_xy);
          control.pitch = -(int16_t)limint16(tau_y * kp_xy_temp + (omega_y - sensorData.gyro.y) * kd_xy);
          control.yaw = -(int16_t)limint16(tau_z * kp_z + (omega_z - sensorData.gyro.z) * kd_z);
        }
      }
      else
      {
        control.thrust = 0.0f;
        control.roll = 0.0f;
        control.pitch = 0.0f;
        control.yaw = 0.0f;
      }

      

      // checkEmergencyStopTimeout(); // a timer, is this useful? remove it?
      // stabilizerResetEmergencyStop();

      checkStops = systemIsArmed();
      if (emergencyStop || (systemIsArmed() == false))
      {
        ;
        // powerStop();
      }
      else
      {
        powerDistribution(&control);
      }

    }
    calcSensorToOutputLatency(&sensorData);
    tick++;
    STATS_CNT_RATE_EVENT(&stabilizerRate);

    if (!rateSupervisorValidate(&rateSupervisorContext, xTaskGetTickCount()))
    {
      if (!rateWarningDisplayed)
      {
        DEBUG_PRINT("WARNING: stabilizer loop rate is off (%lu)\n", rateSupervisorLatestCount(&rateSupervisorContext));
        rateWarningDisplayed = true;
      }
    }
  }
}

void stabilizerSetEmergencyStop()
{
  emergencyStop = false;
}

void stabilizerResetEmergencyStop()
{
  emergencyStop = false;
}

// void stabilizerSetEmergencyStopTimeout(int timeout)
// {
//   emergencyStop = false;
//   emergencyStopTimeout = timeout;
// }

static float variance(float *buffer, uint32_t length)
{
  uint32_t i;
  float sum = 0;
  float sumSq = 0;

  for (i = 0; i < length; i++)
  {
    sum += buffer[i];
    sumSq += buffer[i] * buffer[i];
  }

  return sumSq - (sum * sum) / length;
}

/** Evaluate the values from the propeller test
 * @param low The low limit of the self test
 * @param high The high limit of the self test
 * @param value The value to compare with.
 * @param string A pointer to a string describing the value.
 * @return True if self test within low - high limit, false otherwise
 */
static bool evaluateTest(float low, float high, float value, uint8_t motor)
{
  if (value < low || value > high)
  {
    DEBUG_PRINT("Propeller test on M%d [FAIL]. low: %0.2f, high: %0.2f, measured: %0.2f\n",
                motor + 1, (double)low, (double)high, (double)value);
    return false;
  }

  motorPass |= (1 << motor);

  return true;
}

static void testProps(sensorData_t *sensors)
{
  static uint32_t i = 0;
  NO_DMA_CCM_SAFE_ZERO_INIT static float accX[PROPTEST_NBR_OF_VARIANCE_VALUES];
  NO_DMA_CCM_SAFE_ZERO_INIT static float accY[PROPTEST_NBR_OF_VARIANCE_VALUES];
  NO_DMA_CCM_SAFE_ZERO_INIT static float accZ[PROPTEST_NBR_OF_VARIANCE_VALUES];
  static float accVarXnf;
  static float accVarYnf;
  static float accVarZnf;
  static int motorToTest = 0;
  static uint8_t nrFailedTests = 0;
  static float idleVoltage;
  static float minSingleLoadedVoltage[NBR_OF_MOTORS];
  static float minLoadedVoltage;

  if (testState == configureAcc)
  {
    motorPass = 0;
    sensorsSetAccMode(ACC_MODE_PROPTEST);
    testState = measureNoiseFloor;
    minLoadedVoltage = idleVoltage = pmGetBatteryVoltage();
    minSingleLoadedVoltage[MOTOR_M1] = minLoadedVoltage;
    minSingleLoadedVoltage[MOTOR_M2] = minLoadedVoltage;
    minSingleLoadedVoltage[MOTOR_M3] = minLoadedVoltage;
    minSingleLoadedVoltage[MOTOR_M4] = minLoadedVoltage;
  }
  if (testState == measureNoiseFloor)
  {
    accX[i] = sensors->acc.x;
    accY[i] = sensors->acc.y;
    accZ[i] = sensors->acc.z;

    if (++i >= PROPTEST_NBR_OF_VARIANCE_VALUES)
    {
      i = 0;
      accVarXnf = variance(accX, PROPTEST_NBR_OF_VARIANCE_VALUES);
      accVarYnf = variance(accY, PROPTEST_NBR_OF_VARIANCE_VALUES);
      accVarZnf = variance(accZ, PROPTEST_NBR_OF_VARIANCE_VALUES);
      DEBUG_PRINT("Acc noise floor variance X+Y:%f, (Z:%f)\n",
                  (double)accVarXnf + (double)accVarYnf, (double)accVarZnf);
      testState = measureProp;
    }
  }
  else if (testState == measureProp)
  {
    if (i < PROPTEST_NBR_OF_VARIANCE_VALUES)
    {
      accX[i] = sensors->acc.x;
      accY[i] = sensors->acc.y;
      accZ[i] = sensors->acc.z;
      if (pmGetBatteryVoltage() < minSingleLoadedVoltage[motorToTest])
      {
        minSingleLoadedVoltage[motorToTest] = pmGetBatteryVoltage();
      }
    }
    i++;

    if (i == 1)
    {
      motorsSetRatio(motorToTest, 0xFFFF);
    }
    else if (i == 50)
    {
      motorsSetRatio(motorToTest, 0);
    }
    else if (i == PROPTEST_NBR_OF_VARIANCE_VALUES)
    {
      accVarX[motorToTest] = variance(accX, PROPTEST_NBR_OF_VARIANCE_VALUES);
      accVarY[motorToTest] = variance(accY, PROPTEST_NBR_OF_VARIANCE_VALUES);
      accVarZ[motorToTest] = variance(accZ, PROPTEST_NBR_OF_VARIANCE_VALUES);
      DEBUG_PRINT("Motor M%d variance X+Y:%f (Z:%f)\n",
                  motorToTest + 1, (double)accVarX[motorToTest] + (double)accVarY[motorToTest],
                  (double)accVarZ[motorToTest]);
    }
    else if (i >= 1000)
    {
      i = 0;
      motorToTest++;
      if (motorToTest >= NBR_OF_MOTORS)
      {
        i = 0;
        motorToTest = 0;
        testState = evaluateResult;
        sensorsSetAccMode(ACC_MODE_FLIGHT);
      }
    }
  }
  else if (testState == testBattery)
  {
    if (i == 0)
    {
      minLoadedVoltage = idleVoltage = pmGetBatteryVoltage();
    }
    if (i == 1)
    {
      motorsSetRatio(MOTOR_M1, 0xFFFF);
      motorsSetRatio(MOTOR_M2, 0xFFFF);
      motorsSetRatio(MOTOR_M3, 0xFFFF);
      motorsSetRatio(MOTOR_M4, 0xFFFF);
    }
    else if (i < 50)
    {
      if (pmGetBatteryVoltage() < minLoadedVoltage)
        minLoadedVoltage = pmGetBatteryVoltage();
    }
    else if (i == 50)
    {
      motorsSetRatio(MOTOR_M1, 0);
      motorsSetRatio(MOTOR_M2, 0);
      motorsSetRatio(MOTOR_M3, 0);
      motorsSetRatio(MOTOR_M4, 0);
      //      DEBUG_PRINT("IdleV: %f, minV: %f, M1V: %f, M2V: %f, M3V: %f, M4V: %f\n", (double)idleVoltage,
      //                  (double)minLoadedVoltage,
      //                  (double)minSingleLoadedVoltage[MOTOR_M1],
      //                  (double)minSingleLoadedVoltage[MOTOR_M2],
      //                  (double)minSingleLoadedVoltage[MOTOR_M3],
      //                  (double)minSingleLoadedVoltage[MOTOR_M4]);
      DEBUG_PRINT("%f %f %f %f %f %f\n", (double)idleVoltage,
                  (double)(idleVoltage - minLoadedVoltage),
                  (double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M1]),
                  (double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M2]),
                  (double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M3]),
                  (double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M4]));
      testState = restartBatTest;
      i = 0;
    }
    i++;
  }
  else if (testState == restartBatTest)
  {
    if (i++ > 2000)
    {
      testState = configureAcc;
      i = 0;
    }
  }
  else if (testState == evaluateResult)
  {
    for (int m = 0; m < NBR_OF_MOTORS; m++)
    {
      if (!evaluateTest(0, PROPELLER_BALANCE_TEST_THRESHOLD, accVarX[m] + accVarY[m], m))
      {
        nrFailedTests++;
        for (int j = 0; j < 3; j++)
        {
          motorsBeep(m, true, testsound[m], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4) / 20);
          vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
          motorsBeep(m, false, 0, 0);
          vTaskDelay(M2T(100));
        }
      }
    }
#ifdef PLAY_STARTUP_MELODY_ON_MOTORS
    if (nrFailedTests == 0)
    {
      for (int m = 0; m < NBR_OF_MOTORS; m++)
      {
        motorsBeep(m, true, testsound[m], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4) / 20);
        vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
        motorsBeep(m, false, 0, 0);
        vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
      }
    }
#endif
    motorTestCount++;
    testState = testDone;
  }
}
PARAM_GROUP_START(health)
PARAM_ADD(PARAM_UINT8, startPropTest, &startPropTest)
PARAM_GROUP_STOP(health)

PARAM_GROUP_START(stabilizer)
PARAM_ADD(PARAM_UINT8, estimator, &estimatorType)
// PARAM_ADD(PARAM_UINT8, controller, &controllerType)
PARAM_ADD(PARAM_UINT8, stop, &emergencyStop)
PARAM_ADD(PARAM_FLOAT, acl, &attitude_control_limit)

PARAM_ADD(PARAM_FLOAT, kpxy, &kp_xy)
PARAM_ADD(PARAM_FLOAT, kpz, &kp_z)
PARAM_ADD(PARAM_FLOAT, kdxy, &kd_xy)
PARAM_ADD(PARAM_FLOAT, kdz, &kd_z)
PARAM_ADD(PARAM_FLOAT, exfreq, &external_loop_freq)

// PARAM_ADD(PARAM_FLOAT, aet, &angle_error_threshold)
// PARAM_ADD(PARAM_FLOAT, aev, &angle_error_velocity)

PARAM_ADD(PARAM_FLOAT, qxo, &tau_x_offset)
PARAM_ADD(PARAM_FLOAT, qyo, &tau_y_offset)
PARAM_ADD(PARAM_FLOAT, qzo, &tau_z_offset)

PARAM_GROUP_STOP(stabilizer)

LOG_GROUP_START(health)
LOG_ADD(LOG_FLOAT, motorVarXM1, &accVarX[0])
LOG_ADD(LOG_FLOAT, motorVarYM1, &accVarY[0])
LOG_ADD(LOG_FLOAT, motorVarXM2, &accVarX[1])
LOG_ADD(LOG_FLOAT, motorVarYM2, &accVarY[1])
LOG_ADD(LOG_FLOAT, motorVarXM3, &accVarX[2])
LOG_ADD(LOG_FLOAT, motorVarYM3, &accVarY[2])
LOG_ADD(LOG_FLOAT, motorVarXM4, &accVarX[3])
LOG_ADD(LOG_FLOAT, motorVarYM4, &accVarY[3])
LOG_ADD(LOG_UINT8, motorPass, &motorPass)
LOG_ADD(LOG_UINT16, motorTestCount, &motorTestCount)
LOG_ADD(LOG_UINT8, checkStops, &checkStops)
LOG_GROUP_STOP(health)

LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, x, &setpoint.position.x)
LOG_ADD(LOG_FLOAT, y, &setpoint.position.y)
LOG_ADD(LOG_FLOAT, z, &setpoint.position.z)

LOG_ADD(LOG_FLOAT, vx, &setpoint.velocity.x)
LOG_ADD(LOG_FLOAT, vy, &setpoint.velocity.y)
LOG_ADD(LOG_FLOAT, vz, &setpoint.velocity.z)

LOG_ADD(LOG_FLOAT, ax, &setpoint.acceleration.x)
LOG_ADD(LOG_FLOAT, ay, &setpoint.acceleration.y)
LOG_ADD(LOG_FLOAT, az, &setpoint.acceleration.z)

LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)

LOG_GROUP_START(ctrltargetZ)
LOG_ADD(LOG_INT16, x, &setpointCompressed.x) // position - mm
LOG_ADD(LOG_INT16, y, &setpointCompressed.y)
LOG_ADD(LOG_INT16, z, &setpointCompressed.z)

LOG_ADD(LOG_INT16, vx, &setpointCompressed.vx) // velocity - mm / sec
LOG_ADD(LOG_INT16, vy, &setpointCompressed.vy)
LOG_ADD(LOG_INT16, vz, &setpointCompressed.vz)

LOG_ADD(LOG_INT16, ax, &setpointCompressed.ax) // acceleration - mm / sec^2
LOG_ADD(LOG_INT16, ay, &setpointCompressed.ay)
LOG_ADD(LOG_INT16, az, &setpointCompressed.az)
LOG_GROUP_STOP(ctrltargetZ)

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
LOG_ADD(LOG_FLOAT, thrust, &control.thrust)
LOG_ADD(LOG_UINT8, servo, &servoRatio_stabilizer)



STATS_CNT_RATE_LOG_ADD(rtStab, &stabilizerRate)
LOG_ADD(LOG_UINT32, intToOut, &inToOutLatency)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(accSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.accSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.accSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.accSec.z)
LOG_GROUP_STOP(accSec)
#endif

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &sensorData.baro.asl)
LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)
LOG_ADD(LOG_FLOAT, pressure, &sensorData.baro.pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(gyroSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyroSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyroSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyroSec.z)
LOG_GROUP_STOP(gyroSec)
#endif

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &sensorData.mag.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.mag.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(controller)
LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
LOG_GROUP_STOP(controller)

LOG_GROUP_START(stateEstimate)
LOG_ADD(LOG_FLOAT, x, &state.position.x)
LOG_ADD(LOG_FLOAT, y, &state.position.y)
LOG_ADD(LOG_FLOAT, z, &state.position.z)

LOG_ADD(LOG_FLOAT, vx, &state.velocity.x)
LOG_ADD(LOG_FLOAT, vy, &state.velocity.y)
LOG_ADD(LOG_FLOAT, vz, &state.velocity.z)

LOG_ADD(LOG_FLOAT, ax, &state.acc.x)
LOG_ADD(LOG_FLOAT, ay, &state.acc.y)
LOG_ADD(LOG_FLOAT, az, &state.acc.z)

LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)

LOG_ADD(LOG_FLOAT, qx, &state.attitudeQuaternion.x)
LOG_ADD(LOG_FLOAT, qy, &state.attitudeQuaternion.y)
LOG_ADD(LOG_FLOAT, qz, &state.attitudeQuaternion.z)
LOG_ADD(LOG_FLOAT, qw, &state.attitudeQuaternion.w)
LOG_GROUP_STOP(stateEstimate)

LOG_GROUP_START(stateEstimateZ)
LOG_ADD(LOG_INT16, x, &stateCompressed.x) // position - mm
LOG_ADD(LOG_INT16, y, &stateCompressed.y)
LOG_ADD(LOG_INT16, z, &stateCompressed.z)

LOG_ADD(LOG_INT16, vx, &stateCompressed.vx) // velocity - mm / sec
LOG_ADD(LOG_INT16, vy, &stateCompressed.vy)
LOG_ADD(LOG_INT16, vz, &stateCompressed.vz)

LOG_ADD(LOG_INT16, ax, &stateCompressed.ax) // acceleration - mm / sec^2
LOG_ADD(LOG_INT16, ay, &stateCompressed.ay)
LOG_ADD(LOG_INT16, az, &stateCompressed.az)

LOG_ADD(LOG_UINT32, quat, &stateCompressed.quat) // compressed quaternion, see quatcompress.h

LOG_ADD(LOG_INT16, rateRoll, &stateCompressed.rateRoll) // angular velocity - milliradians / sec
LOG_ADD(LOG_INT16, ratePitch, &stateCompressed.ratePitch)
LOG_ADD(LOG_INT16, rateYaw, &stateCompressed.rateYaw)
LOG_GROUP_STOP(stateEstimateZ)
