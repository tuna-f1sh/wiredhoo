#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "main.h"
#include "utilities.h"
#include "antmessage.h"
#include "antdefines.h"
#include "ant.h"
#include "trainer.h"
#include "tim.h"
#include "adc.h"

Trainer_t trainer;
UserConfig_t user;

// print buffer
static char print[64];

inline uint32_t calculate_flywheel_rps(uint32_t light_freq) {
  // divide the light gate frequency by the number of segments per revolution for the revolutions per second of the flywheel
  return light_freq / FLYWHEEL_TICKS_REVOLUTION;
}

inline float calculate_flywheel_angular_velocity(uint32_t rps) {
  // 2 * pi * rps /60
  return (float) ((float) rps * 2 * M_PI) / 60;
}

inline float calculate_system_kinetic_energy(float angular_v) {
  // 0.5 * I av^2 * k_comp
  return 0.5 * SYSTEM_INERTIA * (angular_v * angular_v) * SYSTEM_INERTIA_COMP_GAIN;
}

// TODO calculate spindown lookup and emf input
float calculate_rider_power(float system_ke, uint16_t update_freq) {
  static float last_rider_ke;
  float rider_ke;
  float delta_ke;
  float power;

  // use kinetic energy in system and take away known losses to get rider input
  rider_ke = gsystem.ke /* - spindown lookup - emf control */;

  // calculate change since last sample
  delta_ke = rider_ke - last_rider_ke;

  // power is the change in ke with respect to time (tick rate of task); change in energy with time
  power = delta_ke * update_freq;

  // set for next calculation
  last_rider_ke = rider_ke;

  // return with any defined compensation gain
  return power * RIDER_POWER_COMP_GAIN;
}

// TODO spindown routine: do a system power calculation from 36 km/h down to ~0
float calculate_system_power(float system_ke, uint16_t update_freq) {
  static float last_ke;
  float delta_ke;
  float power;

  // calculate change since last sample
  delta_ke = system_ke - last_ke;

  // power is the change in ke with respect to time (tick rate of task); change in energy with time
  power = delta_ke * update_freq;

  // set for next calculation
  last_ke = system_ke;

  return power;
}

// returns tick rate of driven _wheel_
inline uint16_t calculate_driven_rps(uint32_t rps) {
  return (rps * WHEEL_FLYWHEEL_RATIO_100) / 100;
}

// returns mm/s
inline uint16_t calculate_trainer_speed(uint32_t rps) {
  // return speed of driven _wheel_ and the defined circumference for implied speed
  return calculate_driven_rps(rps) * trainer.wheel_circumference;
}

void trainer_init(void) {
  // adc for sense channels
  setup_adc_channels();

  // tim2 to capture light gate
  tim2_capture_setup();

  // tim3 for emf control
  tim3_pwm_init();
  tim3_pwm_set_duty(TIM_CHANNEL_1, 50);

  // setup trainer vars
  memset(&trainer, 0, sizeof(Trainer_t));
  trainer.state = READY;
  /* trainer.status.resistance_calibration = 1; */
  trainer.wheel_diameter = WHEEL_DIAMETER;
  trainer.wheel_circumference = (trainer.wheel_diameter * M_PI);
}

void trainer_run(void *argument) {
  TickType_t last_tick;
  uint16_t freq = 1000;
  uint16_t len = 0;

  printf("Trainer Task Starting\r\n");

  // Initialise the xLastWakeTime variable with the current time.
  last_tick = xTaskGetTickCount();

  // initialise the hardware being used and state
  trainer_init();

  if (DEBUG_TRAINER) {
    len = sprintf(print, "rps,omega,ke,rider_power,speed\r\n");
    thread_printf((uint8_t*) print, len, 10);
  }

  for(;;)
  {
    // execute at the update tick
    vTaskDelayUntil(&last_tick, TRAINER_TASK_UPDATE_MS);

    // get revs per second using frequency capture
    gsystem.rps = calculate_flywheel_rps(tim2_calc_frequency());
    // then angular velocity (rad/s) using this
    gsystem.omega = calculate_flywheel_angular_velocity(gsystem.rps);
    // and calculate the kinetic energy in system
    gsystem.ke = calculate_system_kinetic_energy(gsystem.omega);

    // TODO actual timing not constant
    trainer.power = (uint16_t) calculate_rider_power(gsystem.ke, TRAINER_TASK_UPDATE_FREQ);

    // TODO peak detection (high pass filter power?) for cadence

    // TODO speed, elapsed time when IN_USE state, distance, wheel_period
    trainer.speed = calculate_trainer_speed(gsystem.rps);
    trainer.wheel_period = 2048 / calculate_driven_rps(gsystem.rps); // this is the average period of a wheel rev in 1/2048 s - should be average since last update but we'll do that later...

    // elasped is 0.25 s update
    if (last_tick % 250 == 0) {
      trainer.elapsed_time += 1;

      // update distance on each second
      if (trainer.elapsed_time % 4 == 0) {
        // this will roll over as defined by profile
        trainer.distance_traveled = trainer.speed * (trainer.elapsed_time / 4); // m/s * s / 4 because time is 0.25 s
      }
    }

    // not currently used but get them anyway since it's just a buffered value
    gsystem.vin = get_vsense();
    gsystem.csense = get_csense();
    gsystem.emf = get_emfsense();

    if (DEBUG_TRAINER) {
      len = sprintf(print, "%u,%u,%u,%d,%d\r\n",
          (unsigned int) gsystem.rps,
          (unsigned int) (gsystem.omega * 1000),
          (unsigned int) (gsystem.ke * 1000),
          trainer.power,
          trainer.speed
      );
      thread_printf((uint8_t*) print, len, 0);
    }

    if (freq < (1000 * 10)) {
      freq += 100;
    } else {
      freq = 1000;
    }

    tim3_pwm_set_freq(freq);
    tim3_pwm_set_duty(TIM_CHANNEL_1, 50);
  }
}



uint8_t trainer_process_request(uint8_t *request, uint8_t *page) {
  uint8_t ret = 0; // !0 means didn't add anything to return page
  uint8_t dpage = request[0];

  switch (dpage) {

    case ANT_FEC_CALIBRATION_REQ:
      if ((request[1] & ANT_FEC_SPIN_DOWN_MASK) == ANT_FEC_SPIN_DOWN_MASK) {
        trainer.fsm = SPIN_DOWN;
        ret = 1;
      } else if ((request[1] & ANT_FEC_ZERO_OFFSET_MASK) == ANT_FEC_ZERO_OFFSET_MASK) {
        // TODO send back now to say it's done since we don't do it
        trainer.calibration_status = 0x00;
        trainer.spin_down_period = 0xFFFF;
        trainer_generate_page(ANT_FEC_CALIBRATION_REQ, page);
      }
      break;

    case ANT_FEC_DP_BASIC_RESISTANCE:
      trainer.resistance = request[7]; // 0-100%
      trainer.last_control = ANT_FEC_DP_BASIC_RESISTANCE;
      trainer.control_counter++;
      ret = 1;
      break;

    case ANT_FEC_DP_TARGET_POWER:
      trainer.target_power = request[6] | (request[7] << 8); // 0-4000 W
      trainer.last_control = ANT_FEC_DP_TARGET_POWER;
      trainer.control_counter++;
      ret = 1;
      break;

    case ANT_FEC_DP_WIND_RESISTANCE:
      trainer.wind_resistance = request[5]; // 0.01 kg/m 0-1.86 W
      trainer.wind_speed = request[6]; // -127/+127
      trainer.draft_factor = request[7]; // 0.01 0-1.00
      trainer.last_control = ANT_FEC_DP_WIND_RESISTANCE;
      trainer.control_counter++;
      ret = 1;
      break;

    case ANT_FEC_DP_TRACK_RESISTANCE:
      trainer.grade = request[5] | (request[6] << 8); // 0.01 % -200.00 - +200.00
      trainer.coef_rolling = request[6]; // 5e-5 0.0-0.0127
      trainer.last_control = ANT_FEC_DP_TRACK_RESISTANCE;
      trainer.control_counter++;
      ret = 1;
      break;

    case ANT_FEC_DP_USER_CONFIG:
      user.weight = (request[1] | (request[2] << 8)) * 10; // 0.01 kg 0-655.34 kg * 10 to convert to g
      user.wheel_offset = request[4] & 0x0F; // 1 mm 0 - 10 mm
      user.bike_weight = (((request[4] & 0xF0) >> 4) | (request[5] << 8)); // 0.05 kg 0 - 50 kg * 20 to convert to g
      trainer.wheel_diameter = request[6] * 10; // 0.01 m 0 - 2.54 m * 10 to convert to mm
      trainer.wheel_circumference = (trainer.wheel_diameter * M_PI); // set circumference now too
      user.gear_ratio = request[7]; // 0.03 0.03 - 7.65
      ret = 1;
      break;

    case ANT_FEC_DP_FE_CAPABILITES:
      trainer_generate_page(ANT_FEC_DP_FE_CAPABILITES, page);
      break;

    case ANT_COMMON_PAGE1:
    case ANT_COMMON_PAGE2:
      ant_generate_common_page(dpage, page);
      break;

    case ANT_COMMON_REQ_PAGE:
      // this also contains number of times to send bits 0-6 and 7 until ack if 0x80 but ignore for now
      trainer_generate_page(request[6], page);
      break;

    default:
      ret = 1;
      break;

  }

  return ret;
}

uint8_t trainer_generate_page(uint8_t data_page, uint8_t *page) {
  uint8_t ret = 0;

  switch(data_page) {

    case ANT_FEC_CALIBRATION_REQ:
      page[0] = ANT_FEC_CALIBRATION_REQ;
      page[1] = trainer.calibration_status;
      page[2] = 0x00;
      page[3] = 0xFF; // temperature invalid 0.5 deg
      page[4] = 0xFF; // zero offset lsb
      page[5] = 0xFF; // zero offset hsb
      page[6] = LOW_BYTE(trainer.spin_down_period);
      page[7] = HIGH_BYTE(trainer.spin_down_period);
      break;

    case ANT_FEC_CALIBRATION_PROG:
      page[0] = ANT_FEC_CALIBRATION_PROG;
      page[1] = ANT_FEC_SPIN_DOWN_MASK; // send we are doing spin down
      page[2] = trainer.calib_condition.byte; // condition status
      page[3] = 0xff; // temperature 0.5 deg
      page[4] = LOW_BYTE(SPIN_DOWN_TARGET_SPEED); // target speed lsb 0.001 m/s
      page[5] = HIGH_BYTE(SPIN_DOWN_TARGET_SPEED); // target speed hsb
      page[6] = 0xff; // target spin down time lsb ms
      page[7] = 0xff; // target spin down time hsb ms
      break;

    case ANT_FEC_DP_FE_CAPABILITES:
      page[0] = ANT_FEC_DP_FE_CAPABILITES;
      page[1] = 0xFF;
      page[2] = 0xFF;
      page[3] = 0xFF;
      page[4] = 0xFF;
      page[5] = 0xFF; // max resistance lsb
      page[6] = 0xFF; // max resistance hsb
      page[7] = ANT_FEC_CAPABILITES;
      break;

    case ANT_COMMON_CMD_STATUS:
      page[0] = ANT_COMMON_CMD_STATUS;
      page[1] = (trainer.last_control == 0) ? 0xFF : trainer.last_control;
      page[2] = (trainer.control_counter == 0) ? 0xFF : trainer.control_counter;
      page[3] = 0x00; // pass
      page[4] = 0xFF;
      page[5] = 0xFF;
      page[6] = 0xFF;
      page[7] = 0xFF;
      if (trainer.last_control == ANT_FEC_DP_BASIC_RESISTANCE) {
        page[7] = trainer.resistance;
      } else if (trainer.last_control == ANT_FEC_DP_TARGET_POWER) {
        page[6] = LOW_BYTE(trainer.target_power);
        page[7] = HIGH_BYTE(trainer.target_power);
      }
      break;

    case ANT_FEC_GENERAL_PAGE:
      page[0] = ANT_FEC_GENERAL_PAGE;
      page[1] = ANT_FEC_TYPE;
      page[2] = trainer.elapsed_time;
      page[3] = trainer.distance_traveled; // distance not used but same as above
      page[4] = LOW_BYTE(trainer.speed); // speed lsb (mm/s)
      page[5] = HIGH_BYTE(trainer.speed); // speed hsb (mm/s)
      page[6] = 0xff; // heart rate
      page[7] = ANT_FEC_GEN_CAPABILITES | (trainer.state << 4);
      break;

    case ANT_FEC_GENERAL_SET_PAGE:
      page[0] = ANT_FEC_GENERAL_SET_PAGE;
      page[1] = 0xff;
      page[2] = 0xff;
      page[3] = (uint8_t) (trainer.wheel_circumference / 10); // cycle length is wheel circumference in 0.01 m
      page[4] = LOW_BYTE(trainer.grade);
      page[5] = HIGH_BYTE(trainer.grade);
      page[6] = trainer.resistance;
      page[7] = trainer.state << 4;
      break;

      // this is handled in calling function
    /* case ANT_FEC_TRAINER_DATA_PAGE: */
    /*     page[0] = ANT_FEC_TRAINER_DATA_PAGE; */
    /*     page[1] = trainer.accumulated_power_counter; */
    /*     page[2] = trainer.cadence; */
    /*     page[3] = LOW_BYTE(trainer.accumulated_power); // accumulated power lsb */
    /*     page[4] = HIGH_BYTE(trainer.accumulated_power); // accumulated power hsb */
    /*     page[5] = LOW_BYTE(trainer.power); // instananous power lsb */
    /*     page[6] = (HIGH_BYTE(trainer.power) & 0x0F) | (trainer.status.byte << 4); // instananous power hsb 0:3 status 4:7 */
    /*     page[7] = (trainer.flag & 0x0F) | (trainer.state << 4); */
    /*   break; */

      // this is currently not sent...
    case ANT_FEC_TRAINER_TORQUE_PAGE:
      trainer.accumulated_torque += trainer.torque;
      trainer.accumulated_torque_counter++;

      page[0] = ANT_FEC_TRAINER_TORQUE_PAGE;
      page[1] = ++trainer.accumulated_torque_counter;
      page[2] = 0x00; // TODO: wheel ticks field increments with each wheel revolution and is used to calculate linear distance traveled
      page[3] = LOW_BYTE(trainer.wheel_period);
      page[4] = HIGH_BYTE(trainer.wheel_period);
      page[5] = LOW_BYTE(trainer.accumulated_torque);
      page[6] = HIGH_BYTE(trainer.accumulated_torque);
      page[7] = trainer.state << 4;
      break;

    case ANT_FEC_KICKR_SIG:
      page[0] = ANT_FEC_KICKR_SIG;
      page[1] = 0xFF;
      page[2] = 0xFF;
      page[3] = 0xFF;
      page[4] = 0xFF;
      page[5] = 0xFF;
      page[6] = 0xFF;
      page[7] = 0x00;
      break;
    default:
      ret = 1;
      break;
  }

  return ret;
}
