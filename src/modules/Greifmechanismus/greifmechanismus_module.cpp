#include "greifmechanismus_module.h"

#include "actuators/servo_module_Arm.h"
#include "actuators/motor_module_Arm.h"

namespace {
arm_drehkranz::ServoModule& drehkranzServo()
{
    static arm_drehkranz::ServoModule instance;
    return instance;
}

arm_lenkung::ServoModule& lenkungServo()
{
    static arm_lenkung::ServoModule instance;
    return instance;
}

MotorModuleArm& armMotor()
{
    static MotorModuleArm instance;
    return instance;
}

float& armBasePosition()
{
    static float base = 0.0f;
    return base;
}

void initActuators()
{
    (void)drehkranzServo();
    (void)lenkungServo();
    (void)armMotor();
}

constexpr int SERVO_REACTION_TIME_MS = 750;

void waitServoReaction()
{
    thread_sleep_for(SERVO_REACTION_TIME_MS);
}

void moveToTunnelPosition()
{
    drehkranzServo().setSteeringAngle(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_tunnel_D);
    waitServoReaction();
    lenkungServo().setSteeringAngle(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_tunnel_L);
    waitServoReaction();
}

void moveToHouseWorkPosition(float drehkranz_angle,
                             float lenkung_angle,
                             bool via_tunnel)
{
    lenkungServo().setSteeringAngle(lenkung_angle);
    waitServoReaction();

    if (via_tunnel) {
        drehkranzServo().setSteeringAngle(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_tunnel_D);
        waitServoReaction();
    }

    drehkranzServo().setSteeringAngle(drehkranz_angle);
    waitServoReaction();
}
}

namespace gripper_actuators
{
void initializeDrehkranzServo()
{
    initActuators();
    drehkranzServo().initialize();
    drehkranzServo().enable();
}

void initializeLenkungServo()
{
    initActuators();
    lenkungServo().initialize();
    lenkungServo().enable();
}

void initializeArmMotor()
{
    initActuators();
    armMotor().initialize();
}

void initializeAll()
{
    initializeDrehkranzServo();
    initializeLenkungServo();
    initializeArmMotor();
    armBasePosition() = armMotor().get();
}

MotorModuleArm& getArmMotor()
{
    initActuators();
    return armMotor();
}

void disableDrehkranzServo()
{
    initActuators();
    drehkranzServo().disable();
}

void disableLenkungServo()
{
    initActuators();
    lenkungServo().disable();
}

void disableArmMotor()
{
    initActuators();
    armMotor().disableMotors();
}

void disableAll()
{
    disableDrehkranzServo();
    disableLenkungServo();
    disableArmMotor();
}
}

namespace gripper_cfg
{
bool lager = false;
} // namespace gripper_cfg

namespace lagern
{
// Shared storage state: 0=leer, 1=rot, 2=blau, 3=gelb, 4=gruen.
int g_lager_pos_1 = 0;
int g_lager_pos_2 = 0;
int g_lager_pos_3 = 0;
int g_lager_pos_4 = 0;

// Internal storage functions
namespace {
void goToStoragePosition(int pos)
{
    if (pos == 1) {
        drehkranzServo().setSteeringAngle(gripper_cfg::LAGER_POS_1_D);
        lenkungServo().setSteeringAngle(gripper_cfg::LAGER_POS_1_L);
    } else if (pos == 2) {
        drehkranzServo().setSteeringAngle(gripper_cfg::LAGER_POS_2_D);
        lenkungServo().setSteeringAngle(gripper_cfg::LAGER_POS_2_L);
    } else if (pos == 3) {
        drehkranzServo().setSteeringAngle(gripper_cfg::LAGER_POS_3_D);
        lenkungServo().setSteeringAngle(gripper_cfg::LAGER_POS_3_L);
    } else if (pos == 4) {
        drehkranzServo().setSteeringAngle(gripper_cfg::LAGER_POS_4_D);
        lenkungServo().setSteeringAngle(gripper_cfg::LAGER_POS_4_L);
    }
    thread_sleep_for(500);
}

void pickOrDropStorage(int pos, float seil_umdrehungen)
{
    goToStoragePosition(pos);
    const float base = armBasePosition();
    armMotor().setAndWait(base - seil_umdrehungen);
    armMotor().setAndWait(base);
}

float getStorageRopeTurnsForPos(int pos)
{
    if (pos == 1) {
        return gripper_cfg::SEIL_ROTATIONEN_LAGER_POS_1;
    }
    if (pos == 2) {
        return gripper_cfg::SEIL_ROTATIONEN_LAGER_POS_2;
    }
    if (pos == 3) {
        return gripper_cfg::SEIL_ROTATIONEN_LAGER_POS_3;
    }
    if (pos == 4) {
        return gripper_cfg::SEIL_ROTATIONEN_LAGER_POS_4;
    }
    return 0.0f;
}

void einlagernposition(int pos)
{
    pickOrDropStorage(pos, getStorageRopeTurnsForPos(pos));
}

void auslagernposition(int pos)
{
    pickOrDropStorage(pos, getStorageRopeTurnsForPos(pos));
}
}

bool maybeEinlagernFarbe(int farbe)
{
    if (!gripper_cfg::lager) {
        return true;
    }

    if (g_lager_pos_1 == K_LAGER_LEER) {
        einlagernposition(1);
        g_lager_pos_1 = farbe;
        return true;
    } else if (g_lager_pos_2 == K_LAGER_LEER) {
        einlagernposition(2);
        g_lager_pos_2 = farbe;
        return true;
    } else if (g_lager_pos_3 == K_LAGER_LEER) {
        einlagernposition(3);
        g_lager_pos_3 = farbe;
        return true;
    } else if (g_lager_pos_4 == K_LAGER_LEER) {
        einlagernposition(4);
        g_lager_pos_4 = farbe;
        return true;
    }

    return false;
}

bool maybeAuslagernFarbe(int farbe)
{
    if (!gripper_cfg::lager) {
        return true;
    }

    if (g_lager_pos_1 == farbe) {
        auslagernposition(1);
        g_lager_pos_1 = K_LAGER_LEER;
        return true;
    } else if (g_lager_pos_2 == farbe) {
        auslagernposition(2);
        g_lager_pos_2 = K_LAGER_LEER;
        return true;
    } else if (g_lager_pos_3 == farbe) {
        auslagernposition(3);
        g_lager_pos_3 = K_LAGER_LEER;
        return true;
    } else if (g_lager_pos_4 == farbe) {
        auslagernposition(4);
        g_lager_pos_4 = K_LAGER_LEER;
        return true;
    }

    return false;
}

bool isLagerVoll()
{
    return g_lager_pos_1 != K_LAGER_LEER && g_lager_pos_2 != K_LAGER_LEER
           && g_lager_pos_3 != K_LAGER_LEER && g_lager_pos_4 != K_LAGER_LEER;
}
} // namespace lagern

namespace aufnehmen
{
namespace {
void moveToTunnelAfterPickup()
{
    moveToTunnelPosition();
}

void performPickupAt(float drehkranz_angle,
                     float lenkung_angle,
                     float seil_umdrehungen,
                     bool via_tunnel)
{
    moveToHouseWorkPosition(drehkranz_angle, lenkung_angle, via_tunnel);
    const float base = armBasePosition();
    armMotor().setAndWait(base - seil_umdrehungen);
    armMotor().setAndWait(base);
}
}

AufnehmenModule::AufnehmenModule()
{
    gripper_actuators::initializeAll();
}

void AufnehmenModule::aufnehmenRot()
{
    performPickupAt(
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D,
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_L,
        gripper_cfg::SEIL_ROTATIONEN_HAUS_ROT_GELB,
        true);
    if (!gripper_cfg::lager) {
        moveToTunnelAfterPickup();
        return;
    }
    if (lagern::maybeEinlagernFarbe(K_FARBE_ROT) && lagern::isLagerVoll()) {
        moveToTunnelAfterPickup();
    }
}

void AufnehmenModule::aufnehmenBlau()
{
    performPickupAt(
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D,
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_L,
        gripper_cfg::SEIL_ROTATIONEN_HAUS_BLAU_GRUEN,
        false);
    if (!gripper_cfg::lager) {
        moveToTunnelAfterPickup();
        return;
    }
    if (lagern::maybeEinlagernFarbe(K_FARBE_BLAU) && lagern::isLagerVoll()) {
        moveToTunnelAfterPickup();
    }
}

void AufnehmenModule::aufnehmenGelb()
{
    performPickupAt(
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D,
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_L,
        gripper_cfg::SEIL_ROTATIONEN_HAUS_ROT_GELB,
        true);

    if (!gripper_cfg::lager) {
        moveToTunnelAfterPickup();
        return;
    }
    if (lagern::maybeEinlagernFarbe(K_FARBE_GELB) && lagern::isLagerVoll()) {
        moveToTunnelAfterPickup();
    }
}

void AufnehmenModule::aufnehmenGruen()
{
    performPickupAt(
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D,
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_L,
        gripper_cfg::SEIL_ROTATIONEN_HAUS_BLAU_GRUEN,
        false);
    if (!gripper_cfg::lager) {
        moveToTunnelAfterPickup();
        return;
    }
    if (lagern::maybeEinlagernFarbe(K_FARBE_GRUEN) && lagern::isLagerVoll()) {
        moveToTunnelAfterPickup();
    }
}
} // namespace aufnehmen

namespace abladen
{
namespace {
void moveToStartPositionAfterDropoff()
{
    moveToTunnelPosition();
}

void performDropoffAt(float drehkranz_angle,
                      float lenkung_angle,
                      float seil_umdrehungen,
                      bool via_tunnel)
{
    moveToHouseWorkPosition(drehkranz_angle, lenkung_angle, via_tunnel);
    const float base = armBasePosition();
    armMotor().setAndWait(base - seil_umdrehungen);
    armMotor().setAndWait(base);
}
}

AbladenModule::AbladenModule()
{
    gripper_actuators::initializeAll();
}

void AbladenModule::abladenRot()
{
    if (!lagern::maybeAuslagernFarbe(K_FARBE_ROT)) {
        return;
    }
    performDropoffAt(
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D,
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_L,
        gripper_cfg::SEIL_ROTATIONEN_HAUS_ROT_GELB,
        true);
    moveToStartPositionAfterDropoff();
}

void AbladenModule::abladenBlau()
{
    if (!lagern::maybeAuslagernFarbe(K_FARBE_BLAU)) {
        return;
    }
    performDropoffAt(
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D,
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_L,
        gripper_cfg::SEIL_ROTATIONEN_HAUS_BLAU_GRUEN,
        false);
    moveToStartPositionAfterDropoff();
}

void AbladenModule::abladenGelb()
{
    if (!lagern::maybeAuslagernFarbe(K_FARBE_GELB)) {
        return;
    }
    performDropoffAt(
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D,
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_L,
        gripper_cfg::SEIL_ROTATIONEN_HAUS_ROT_GELB,
        true);
    moveToStartPositionAfterDropoff();
}

void AbladenModule::abladenGruen()
{
    if (!lagern::maybeAuslagernFarbe(K_FARBE_GRUEN)) {
        return;
    }
    performDropoffAt(
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D,
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_L,
        gripper_cfg::SEIL_ROTATIONEN_HAUS_BLAU_GRUEN,
        false);
    moveToStartPositionAfterDropoff();
}
}