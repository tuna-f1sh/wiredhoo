#define ANT_FEC_TYPE                        25 // stationary trainer
#define ANT_FEC_HR_SOURCE                   0 // 0 for no heart rate
#define ANT_FEC_DISTANCE                    0 // 0 for no distance
#define ANT_FEC_SPEED                       0 // 1 for virtual speed (0 since simulated not enabled)
#define ANT_FEC_BASIC_RESISTANCE            1
#define ANT_FEC_TARGET_POWER                1
#define ANT_FEC_SIMULATION                  0
#define ANT_FEC_GEN_CAPABILITES             (ANT_FEC_HR_SOURCE & 0x03) | (ANT_FEC_DISTANCE << 2) | (ANT_FEC_SPEED << 3)
#define ANT_FEC_CAPABILITES                 (ANT_FEC_BASIC_RESISTANCE << 0) | (ANT_FEC_TARGET_POWER << 1) | (ANT_FEC_SIMULATION << 2)

#define SPIN_DOWN_TARGET_SPEED              10000U // 0.001 m/s

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
  uint8_t power_calibration: 1;
  uint8_t resistance_calibration: 1;
  uint8_t user_configuration: 1;
  uint8_t byte;
} TrainerStatus_t;

typedef struct {
  uint16_t power;
  uint32_t accumulated_power;
  uint16_t speed;
  uint8_t cadence;
  uint16_t target_power;
  uint16_t wheel_dia;
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
  TrainerStatus_t status;
  TrainerState_t state; // this is state according to ANT FEC type
  TrainerFlag_t flag;
  uint8_t fsm; // this is finite state machine
} Trainer_t;

typedef struct {
  uint16_t weight;
  uint8_t wheel_offset;
  uint16_t bike_weight;
  uint8_t wheel_diameter;
  uint8_t gear_ratio;
} UserConfig_t;

uint8_t trainer_process_request(uint8_t *request, uint8_t *page);
uint8_t trainer_generate_page(uint8_t data_page, uint8_t *page);
