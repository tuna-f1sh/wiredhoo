#include <math.h>
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

uint32_t calculate_flywheel_rps(void) {
  uint32_t freq = tim2_calc_frequency();

  // divide the light gate frequency by the number of segments per revolution for the revolutions per second of the flywheel
  return freq / FLYWHEEL_TICKS_REVOLUTION;
}

float calculate_flywheel_angular_velocity(void) {
  uint32_t rps = calculate_flywheel_rps();

  // set global system var
  gsystem.rps = rps;

  // 2 * pi * rps /60
  return (float) ((float) rps * 2 * M_PI) / 60;
}

float calculate_system_kinetic_energy(void) {
  float angular_v = calculate_flywheel_angular_velocity();

  // 0.5 * I av^2
  return 0.5 * SYSTEM_INERTIA * (angular_v * angular_v);
}

// TODO calculate spindown lookup and emf input
float calculate_rider_power(uint16_t update_freq) {
  static float last_ke;
  float delta_ke;
  float power;

  // get current kinetic energy in system and take away known losses to get rider input
  gsystem.ke = calculate_system_kinetic_energy(); // - spindown lookup - emf control

  // calculate change since last sample
  delta_ke = gsystem.ke - last_ke;

  // power is the change in ke with respect to time (tick rate of task); change in energy with time
  power = delta_ke * update_freq;

  // set for next calculation
  last_ke = gsystem.ke;

  return power;
}

// TODO spindown routine: do a system power calculation from 36 km/h down to ~0
float calculate_system_power(uint16_t update_freq) {
  static float last_ke;
  float new_ke, delta_ke;
  float power;

  // get current kinetic energy in system and take away known losses to get rider input
  new_ke = calculate_system_kinetic_energy();

  // calculate change since last sample
  delta_ke = new_ke - last_ke;

  // power is the change in ke with respect to time (tick rate of task); change in energy with time
  power = delta_ke * update_freq;

  // set for next calculation
  last_ke = new_ke;

  return power;
}

void trainer_init(void) {
  // adc for sense channels
  setup_adc_channels();

  // tim2 to capture light gate
  tim2_capture_setup();

  // tim3 for emf control
  tim3_pwm_init();
  tim3_pwm_set_duty(TIM_CHANNEL_1, 50);

  memset(&trainer, 0, sizeof(Trainer_t));
  trainer.state = READY;
  /* trainer.status.resistance_calibration = 1; */
  trainer.wheel_dia = WHEEL_DIAMETER;
}

void trainer_run(void *argument) {
  uint16_t freq = 1000;

  // initialise the hardware being used and state
  trainer_init();

  for(;;)
  {
    // TODO actual timing not constant
    trainer.power = calculate_rider_power(TRAINER_TASK_UPDATE_FREQ);

    gsystem.vin = get_vsense();
    gsystem.csense = get_csense();
    gsystem.emf = get_emfsense();

    if (freq < (1000 * 10)) {
      freq += 100;
    } else {
      freq = 1000;
    }

    tim3_pwm_set_freq(freq);
    tim3_pwm_set_duty(TIM_CHANNEL_1, 50);

    osDelay(TRAINER_TASK_UPDATE_MS);
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
        // send back now to say it's done since we don't do it
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
      user.weight = request[1] | (request[2] << 8); // 0.01 kg 0-655.34 kg
      user.wheel_offset = request[4] & 0x0F; // 1 mm 0 - 10 mm
      user.bike_weight = (request[4] & 0xF0) | request[5]; // 0.05 kg 0 - 50 kg
      user.wheel_diameter = request[6]; // 0.01 m 0 - 2.54 m
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
      page[2] = 0x00; // elasped time not used
      page[3] = 0x00; // distance not used
      page[4] = LOW_BYTE(trainer.speed); // speed lsb
      page[5] = HIGH_BYTE(trainer.speed); // speed hsb
      page[6] = 0xff; // heart rate
      page[7] = ANT_FEC_GEN_CAPABILITES | (trainer.state << 4);
      break;

    case ANT_FEC_GENERAL_SET_PAGE:
      page[0] = ANT_FEC_GENERAL_SET_PAGE;
      page[1] = 0xff;
      page[2] = 0xff;
      page[3] = (uint8_t) (trainer.wheel_dia * M_PI); // cycle length is wheel circumference
      page[4] = LOW_BYTE(trainer.grade);
      page[5] = HIGH_BYTE(trainer.grade);
      page[6] = trainer.resistance;
      page[7] = trainer.state << 4;
      break;

    case ANT_FEC_TRAINER_TORQUE_PAGE:
      page[0] = ANT_FEC_TRAINER_TORQUE_PAGE;
      page[1] = 0x00;
      page[2] = WHEEL_TICKS_REVOLUTION;
      page[3] = LOW_BYTE(trainer.accumulated_wheel);
      page[4] = HIGH_BYTE(trainer.accumulated_wheel);
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
