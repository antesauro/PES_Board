#include "greifmechanismus_module.h"

#include "actuators/motor_module_Arm.h"
#include "actuators/servo_module_Arm.h"

namespace {
arm_turn::ServoModule &turnServo()
{
    static arm_turn::ServoModule instance;
    return instance;
}

arm_steer::ServoModule &steerServo()
{
    static arm_steer::ServoModule instance;
    return instance;
}

MotorModuleArm &armMotor()
{
    static MotorModuleArm instance;
    return instance;
}

float &armBase()
{
    static float base = 0.0f;
    return base;
}

void initActuators()
{
    (void)turnServo();
    (void)steerServo();
    (void)armMotor();
}

constexpr int SERVO_REACTION_TIME_MS = 400;
constexpr int SERVO_REACTION_TIME_RETURN_MS = 200; // fast return after rope raised

void servoWait() { thread_sleep_for(SERVO_REACTION_TIME_MS); }
void servoWaitFast() { thread_sleep_for(SERVO_REACTION_TIME_RETURN_MS); }

/* void moveToTunnelPosition()
{
    turnServo().setSteeringAngle(gripper_cfg::TUNNEL_TURN);
    servoWait();
    steerServo().setSteeringAngle(gripper_cfg::TUNNEL_STEER);
    servoWait();
} */

void moveToTunnelFast()
{
    turnServo().setSteeringAngle(gripper_cfg::TUNNEL_TURN);
    servoWaitFast();
    steerServo().setSteeringAngle(gripper_cfg::TUNNEL_STEER);
    servoWaitFast();
}

void moveToWorkPos(float turn_angle, float steer_angle, bool via_tunnel)
{
    // Move turn servo to target position first
    if (via_tunnel) {
        turnServo().setSteeringAngle(gripper_cfg::TUNNEL_TURN);
        servoWait();
    }

    // Only then lower steer servo to working position
    steerServo().setSteeringAngle(steer_angle);
    servoWait();

    turnServo().setSteeringAngle(turn_angle);
    servoWait();
}
} // namespace

namespace gripper_actuators {
void initTurnServo()
{
    initActuators();
    turnServo().initialize();
    turnServo().enable();
}

void initSteerServo()
{
    initActuators();
    steerServo().initialize();
    steerServo().enable();
}

void initArmMotor()
{
    initActuators();
    armMotor().initialize();
}

void initAll()
{
    initTurnServo();
    initSteerServo();
    initArmMotor();
    armBase() = armMotor().get();
}

MotorModuleArm &getArmMotor()
{
    initActuators();
    return armMotor();
}

void disableTurnServo()
{
    initActuators();
    turnServo().disable();
}

void disableSteerServo()
{
    initActuators();
    steerServo().disable();
}

void disableArmMotor()
{
    initActuators();
    armMotor().disableMotors();
}

void disableAll()
{
    disableTurnServo();
    disableSteerServo();
    disableArmMotor();
}

void enableFastMode()
{
    initActuators();
    turnServo().setSpeed(SERVO_MAX_SPEED);
    steerServo().setSpeed(SERVO_MAX_SPEED);
}

void returnSlow()
{
    initActuators();
    // Switch back to slow reset speed
    turnServo().setSpeed(0.4f);
    steerServo().setSpeed(0.4f);

    // Gently return to start positions
    turnServo().center();
    steerServo().center();
}

void testPositionSafety()
{
    struct Pose {
        float turn;
        float steer;
    };

    // Step through positions slowly and verify no collision
    const Pose test_positions[] = {
        {0.29f, 0.15f}, // BLUE
        {0.62f, 0.36f}, // YELLOW
    };

    for (const Pose &pos : test_positions) {
        gripper_actuators::getArmMotor(); // make sure arm is at safe height first
        turnServo().setSteeringAngle(pos.turn);
        steerServo().setSteeringAngle(pos.steer);
        thread_sleep_for(10000); // give servo time to reach position
        // visually inspect for collision before pressing button
    }
}
} // namespace gripper_actuators

namespace gripper_cfg {
bool use_storage = false;
} // namespace gripper_cfg

namespace storage {
// Slot state: 0=empty, 1=red, 2=blue, 3=yellow, 4=green.
int g_slot_1 = 0;
int g_slot_2 = 0;
int g_slot_3 = 0;
int g_slot_4 = 0;

// Internal storage functions
namespace {
void goToSlot(int pos)
{
    if (pos == 1) {
        turnServo().setSteeringAngle(gripper_cfg::SLOT_1_TURN);
        steerServo().setSteeringAngle(gripper_cfg::SLOT_1_STEER);
    } else if (pos == 2) {
        turnServo().setSteeringAngle(gripper_cfg::SLOT_2_TURN);
        steerServo().setSteeringAngle(gripper_cfg::SLOT_2_STEER);
    } else if (pos == 3) {
        turnServo().setSteeringAngle(gripper_cfg::SLOT_3_TURN);
        steerServo().setSteeringAngle(gripper_cfg::SLOT_3_STEER);
    } else if (pos == 4) {
        turnServo().setSteeringAngle(gripper_cfg::SLOT_4_TURN);
        steerServo().setSteeringAngle(gripper_cfg::SLOT_4_STEER);
    }
    thread_sleep_for(500);
}

void pickDropSlot(int pos, float rope_turns)
{
    goToSlot(pos);
    const float base = armBase();
    armMotor().setAndWait(base - rope_turns);
    armMotor().setAndWait(base);
}

float ropeForSlot(int pos)
{
    if (pos == 1) {
        return gripper_cfg::ROPE_SLOT_1;
    }
    if (pos == 2) {
        return gripper_cfg::ROPE_SLOT_2;
    }
    if (pos == 3) {
        return gripper_cfg::ROPE_SLOT_3;
    }
    if (pos == 4) {
        return gripper_cfg::ROPE_SLOT_4;
    }
    return 0.0f;
}

void storeAtSlot(int pos) { pickDropSlot(pos, ropeForSlot(pos)); }

void retrieveFromSlot(int pos) { pickDropSlot(pos, ropeForSlot(pos)); }
} // namespace

bool maybeStore(int color)
{
    if (!gripper_cfg::use_storage) {
        return true;
    }

    if (g_slot_1 == K_EMPTY) {
        storeAtSlot(1);
        g_slot_1 = color;
        return true;
    } else if (g_slot_2 == K_EMPTY) {
        storeAtSlot(2);
        g_slot_2 = color;
        return true;
    } else if (g_slot_3 == K_EMPTY) {
        storeAtSlot(3);
        g_slot_3 = color;
        return true;
    } else if (g_slot_4 == K_EMPTY) {
        storeAtSlot(4);
        g_slot_4 = color;
        return true;
    }

    return false;
}

bool maybeRetrieve(int color)
{
    if (!gripper_cfg::use_storage) {
        return true;
    }

    if (g_slot_1 == color) {
        retrieveFromSlot(1);
        g_slot_1 = K_EMPTY;
        return true;
    } else if (g_slot_2 == color) {
        retrieveFromSlot(2);
        g_slot_2 = K_EMPTY;
        return true;
    } else if (g_slot_3 == color) {
        retrieveFromSlot(3);
        g_slot_3 = K_EMPTY;
        return true;
    } else if (g_slot_4 == color) {
        retrieveFromSlot(4);
        g_slot_4 = K_EMPTY;
        return true;
    }

    return false;
}

bool isFull()
{
    return g_slot_1 != K_EMPTY && g_slot_2 != K_EMPTY && g_slot_3 != K_EMPTY &&
           g_slot_4 != K_EMPTY;
}
} // namespace storage

namespace pickup {
namespace {
void moveToTunnelAfterPickup() { moveToTunnelFast(); }

void performPickupAt(float turn_angle, float steer_angle, float rope_turns, bool via_tunnel)
{
    moveToWorkPos(turn_angle, steer_angle, via_tunnel);
    const float base = armBase();
    thread_sleep_for(400);
    armMotor().setAndWait(base - rope_turns);
    armMotor().setAndWait(base);
}
} // namespace

PickupModule::PickupModule() { gripper_actuators::initAll(); }

void PickupModule::pickupRed()
{
    performPickupAt(gripper_cfg::RED_YELLOW_TURN,
                    gripper_cfg::RED_YELLOW_STEER,
                    gripper_cfg::ROPE_RED_YELLOW,
                    true);
    if (!gripper_cfg::use_storage) {
        moveToTunnelAfterPickup();
        return;
    }
    if (storage::maybeStore(K_RED) && storage::isFull()) {
        moveToTunnelAfterPickup();
    }
}

void PickupModule::pickupBlue()
{
    performPickupAt(gripper_cfg::BLUE_GREEN_TURN,
                    gripper_cfg::BLUE_GREEN_STEER,
                    gripper_cfg::ROPE_BLUE_GREEN,
                    false);
    if (!gripper_cfg::use_storage) {
        moveToTunnelAfterPickup();
        return;
    }
    if (storage::maybeStore(K_BLUE) && storage::isFull()) {
        moveToTunnelAfterPickup();
    }
}

void PickupModule::pickupYellow()
{
    performPickupAt(gripper_cfg::RED_YELLOW_TURN,
                    gripper_cfg::RED_YELLOW_STEER,
                    gripper_cfg::ROPE_RED_YELLOW,
                    true);

    if (!gripper_cfg::use_storage) {
        moveToTunnelAfterPickup();
        return;
    }
    if (storage::maybeStore(K_YELLOW) && storage::isFull()) {
        moveToTunnelAfterPickup();
    }
}

void PickupModule::pickupGreen()
{
    performPickupAt(gripper_cfg::BLUE_GREEN_TURN,
                    gripper_cfg::BLUE_GREEN_STEER,
                    gripper_cfg::ROPE_BLUE_GREEN,
                    false);
    if (!gripper_cfg::use_storage) {
        moveToTunnelAfterPickup();
        return;
    }
    if (storage::maybeStore(K_GREEN) && storage::isFull()) {
        moveToTunnelAfterPickup();
    }
}
} // namespace pickup

namespace dropoff {
namespace {
void moveToStartPositionAfterDropoff() { moveToTunnelFast(); }

void performDropoffAt(float turn_angle, float steer_angle, float rope_turns, bool via_tunnel)
{
    moveToWorkPos(turn_angle, steer_angle, via_tunnel);
    const float base = armBase();
    thread_sleep_for(400);
    armMotor().setAndWait(base - rope_turns);
    armMotor().setAndWait(base);
}
} // namespace

DropoffModule::DropoffModule() { gripper_actuators::initAll(); }

void DropoffModule::dropoffRed()
{
    if (!storage::maybeRetrieve(K_RED)) {
        return;
    }
    performDropoffAt(gripper_cfg::RED_YELLOW_TURN,
                     gripper_cfg::RED_YELLOW_STEER,
                     gripper_cfg::ROPE_RED_YELLOW,
                     true);

    moveToStartPositionAfterDropoff();
}

void DropoffModule::dropoffBlue()
{
    if (!storage::maybeRetrieve(K_BLUE)) {
        return;
    }
    performDropoffAt(gripper_cfg::BLUE_GREEN_TURN,
                     gripper_cfg::BLUE_GREEN_STEER,
                     gripper_cfg::ROPE_BLUE_GREEN,
                     false);
    moveToStartPositionAfterDropoff();
}

void DropoffModule::dropoffYellow()
{
    if (!storage::maybeRetrieve(K_YELLOW)) {
        return;
    }
    performDropoffAt(gripper_cfg::RED_YELLOW_TURN,
                     gripper_cfg::RED_YELLOW_STEER,
                     gripper_cfg::ROPE_RED_YELLOW,
                     true);
    moveToStartPositionAfterDropoff();
}

void DropoffModule::dropoffGreen()
{
    if (!storage::maybeRetrieve(K_GREEN)) {
        return;
    }
    performDropoffAt(gripper_cfg::BLUE_GREEN_TURN,
                     gripper_cfg::BLUE_GREEN_STEER,
                     gripper_cfg::ROPE_BLUE_GREEN,
                     false);
    moveToStartPositionAfterDropoff();
}
} // namespace dropoff