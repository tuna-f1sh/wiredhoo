#define ANT_FEC_TYPE                        25 // stationary trainer
#define ANT_FEC_HR_SOURCE                   0 // 0 for no heart rate
#define ANT_FEC_DISTANCE                    0 // 0 for no distance
#define ANT_FEC_SPEED                       0 // 1 for virtual speed (0 since simulated not enabled)
#define ANT_FEC_BASIC_RESISTANCE            1
#define ANT_FEC_TARGET_POWER                1
#define ANT_FEC_SIMULATION                  0
#define ANT_FEC_GEN_CAPABILITES             (ANT_FEC_HR_SOURCE & 0x03) | (ANT_FEC_DISTANCE << 2) | (ANT_FEC_SPEED << 3)
#define ANT_FEC_CAPABILITES                 (ANT_FEC_BASIC_RESISTANCE << 0) | (ANT_FEC_TARGET_POWER << 1) | (ANT_FEC_SIMULATION << 2)

#define SPIN_DOWN_TARGET_SPEED              10000U // 0.001 m/s (mm/s)
#define WHEEL_DIAMETER                      670 // 0.001 m (mm) default wheel diameter (used in ANT+ profile)
#define WHEEL_FLYWHEEL_RATIO_100            10 // ratio between flywheel and the belt driven _wheel_ for speed infer (0.01)
#define FLYWHEEL_TICKS_REVOLUTION           1 // the number of transistions per rev of light gate
#define SYSTEM_INERTIA                      0.024664f // the inertia in the system (calculated based on Kickr V4 first principle measurements) (km/m^2)

#define CALIB_NA                            0
#define CALIB_TOO_LOW                       1
#define CALIB_OK                            2

#define TRAINER_TASK_UPDATE_MS              100
#define TRAINER_TASK_UPDATE_FREQ            1000/TRAINER_TASK_UPDATE_MS

// Fudge factors
#define SYSTEM_INERTIA_COMP_GAIN            1.0f // applied to ke calculation
#define RIDER_POWER_COMP_GAIN               1.0f // apllied to rider power reading

typedef enum {
  RESERVED,
  ASLEEP,
  READY,
  IN_USE,
  FINISHED,
} TrainerState_t;

typedef enum {
  TARGET_POWER,
  USER_TOO_LOW,
  USER_TOO_HIGH,
  UNDETERMINED,
} TrainerFlag_t;

typedef enum {
  IDLE,
  SPIN_DOWN,
  ZERO_OFFSET,
  TERMINATE,
} TrainerFSM_t;

typedef union {
  struct {
    uint8_t power_calibration: 1;
    uint8_t resistance_calibration: 1;
    uint8_t user_configuration: 1;
  };
  uint8_t byte;
} TrainerStatus_t;

typedef union {
  struct {
    uint8_t reserved:4;
    uint8_t temperature:2;
    uint8_t speed:2;
  };
  uint8_t byte;
} TrainerCalibCond_t;

typedef struct {
  uint16_t power;
  uint16_t torque;
  uint16_t accumulated_power;
  uint8_t accumulated_power_counter; // update counter - shared with power profile
  uint16_t speed;
  uint16_t wheel_period;
  uint16_t accumulated_torque;
  uint8_t accumulated_torque_counter; // update counter for torque page
  uint8_t cadence;
  uint16_t target_power;
  uint16_t wheel_diameter;
  uint16_t wheel_circumference;
  uint8_t resistance;
  uint8_t wind_resistance;
  int8_t wind_speed;
  uint8_t draft_factor;
  int16_t grade;
  uint8_t coef_rolling;
  uint16_t spin_down_period;
  uint8_t calibration_status;
  uint8_t last_control;
  uint8_t control_counter;
  uint8_t distance_traveled; // m updates when active
  uint8_t elapsed_time; // 0.25 s updates when active
  TrainerStatus_t status;
  TrainerState_t state; // this is state according to ANT FEC type
  TrainerFlag_t flag;
  TrainerCalibCond_t calib_condition;
  uint8_t fsm; // this is finite state machine
} Trainer_t;

typedef struct {
  uint32_t weight; // in g
  uint8_t wheel_offset;
  uint32_t bike_weight; // in g
  uint8_t gear_ratio; // 0.03 scaled
} UserConfig_t;

uint8_t trainer_process_request(uint8_t *request, uint8_t *page);
uint8_t trainer_generate_page(uint8_t data_page, uint8_t *page);
void trainer_run(void *argument);
