#pragma once

class MotorModuleArm;

namespace gripper_cfg {

// Turntable positions for house pickup/dropoff.
// 0.0 = back, 1.0 = front
constexpr float TUNNEL_TURN               = 0.8f;
constexpr float RED_YELLOW_TURN           = 0.62f;
constexpr float BLUE_GREEN_TURN           = 0.29f;

// Arm steering angles for house pickup/dropoff.
// 0.0 = up, 1.0 = down
constexpr float TUNNEL_STEER              = 0.3f;
constexpr float RED_YELLOW_STEER          = 0.36f;
constexpr float BLUE_GREEN_STEER          = 0.15f;
// Safety angle for vertical arm alignment
constexpr float VERT_SAFETY_STEER         = 1.0f;

// Storage slot turntable positions.
constexpr float SLOT_1_TURN = 0.0;
constexpr float SLOT_2_TURN = 0.0f;
constexpr float SLOT_3_TURN = 0.0f;
constexpr float SLOT_4_TURN = 0.0f;

// Steering angles for storage slots.
constexpr float SLOT_1_STEER = 0.0f;
constexpr float SLOT_2_STEER = 0.0f;
constexpr float SLOT_3_STEER = 0.0f;
constexpr float SLOT_4_STEER = 0.0f;
// false: single package; true: also manage in storage.
extern bool use_storage;

// Rope turns for house and storage.
constexpr float ROPE_RED_YELLOW  = 5.0f;
constexpr float ROPE_BLUE_GREEN  = 6.2f;
constexpr float ROPE_SLOT_1      = 2.5f;
constexpr float ROPE_SLOT_2      = 2.5f;
constexpr float ROPE_SLOT_3      = 2.5f;
constexpr float ROPE_SLOT_4      = 2.5f;

} // namespace gripper_cfg

static constexpr float SERVO_MAX_SPEED = 1.5f;

namespace gripper_actuators {
void initTurnServo();
void initSteerServo();
void initArmMotor();
void initAll();
MotorModuleArm &getArmMotor();
void disableTurnServo();
void disableSteerServo();
void disableArmMotor();
void disableAll();
void enableFastMode();
void returnSlow();
void testPositionSafety();
} // namespace gripper_actuators

static constexpr int K_SLOTS  = 4;
static constexpr int K_EMPTY  = 0;
static constexpr int K_RED    = 1;
static constexpr int K_BLUE   = 2;
static constexpr int K_YELLOW = 3;
static constexpr int K_GREEN  = 4;

namespace storage {
// Slot state: 0=empty, 1=red, 2=blue, 3=yellow, 4=green.
extern int g_slot_1;
extern int g_slot_2;
extern int g_slot_3;
extern int g_slot_4;

bool maybeStore(int color);
bool maybeRetrieve(int color);
bool isFull();
} // namespace storage

namespace pickup {
class PickupModule
{
public:
    PickupModule();

    void pickupRed();
    void pickupBlue();
    void pickupYellow();
    void pickupGreen();
};
} // namespace pickup

namespace dropoff {
class DropoffModule
{
public:
    DropoffModule();

    void dropoffRed();
    void dropoffBlue();
    void dropoffYellow();
    void dropoffGreen();
};
} // namespace dropoff
