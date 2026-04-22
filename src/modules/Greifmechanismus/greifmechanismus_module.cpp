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

void initActuators()
{
    (void)drehkranzServo();
    (void)lenkungServo();
    (void)armMotor();
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
    armMotor().setAndWait(seil_umdrehungen);
    armMotor().setAndWait(seil_umdrehungen * -1.0f);
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

void maybeEinlagernFarbe(int farbe)
{
    if (!gripper_cfg::lager) {
        return;
    }

    if (g_lager_pos_1 == K_LAGER_LEER) {
        einlagernposition(1);
        g_lager_pos_1 = farbe;
    } else if (g_lager_pos_2 == K_LAGER_LEER) {
        einlagernposition(2);
        g_lager_pos_2 = farbe;
    } else if (g_lager_pos_3 == K_LAGER_LEER) {
        einlagernposition(3);
        g_lager_pos_3 = farbe;
    } else if (g_lager_pos_4 == K_LAGER_LEER) {
        einlagernposition(4);
        g_lager_pos_4 = farbe;
    }
}

void maybeAuslagernFarbe(int farbe)
{
    if (!gripper_cfg::lager) {
        return;
    }

    if (g_lager_pos_1 == farbe) {
        auslagernposition(1);
        g_lager_pos_1 = K_LAGER_LEER;
    } else if (g_lager_pos_2 == farbe) {
        auslagernposition(2);
        g_lager_pos_2 = K_LAGER_LEER;
    } else if (g_lager_pos_3 == farbe) {
        auslagernposition(3);
        g_lager_pos_3 = K_LAGER_LEER;
    } else if (g_lager_pos_4 == farbe) {
        auslagernposition(4);
        g_lager_pos_4 = K_LAGER_LEER;
    }
}
} // namespace lagern

namespace aufnehmen
{
namespace {
void moveToTunnelAfterPickup()
{
    drehkranzServo().setSteeringAngle(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_tunnel_D);
    lenkungServo().setSteeringAngle(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_tunnel_L);
    thread_sleep_for(500);
}

void performPickupAt(float drehkranz_angle, float lenkung_angle, float seil_umdrehungen)
{
    drehkranzServo().setSteeringAngle(-drehkranz_angle);
    lenkungServo().setSteeringAngle(-lenkung_angle);
    thread_sleep_for(500);
    armMotor().setAndWait(seil_umdrehungen);
    armMotor().setAndWait(seil_umdrehungen * -1.0f);
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
        gripper_cfg::SEIL_ROTATIONEN_HAUS_ROT_GELB
                    );
    if (!gripper_cfg::lager) {
        moveToTunnelAfterPickup();
        return;
    }
    lagern::maybeEinlagernFarbe(K_FARBE_ROT);
    moveToTunnelAfterPickup();
}

void AufnehmenModule::aufnehmenBlau()
{
    performPickupAt(
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D,
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_L,
        gripper_cfg::SEIL_ROTATIONEN_HAUS_BLAU_GRUEN
            );
    if (!gripper_cfg::lager) {
        moveToTunnelAfterPickup();
        return;
    }
    lagern::maybeEinlagernFarbe(K_FARBE_BLAU);
    moveToTunnelAfterPickup();
}

void AufnehmenModule::aufnehmenGelb()
{
    performPickupAt(
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D,
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_L,
        gripper_cfg::SEIL_ROTATIONEN_HAUS_ROT_GELB
                    );
    if (!gripper_cfg::lager) {
        moveToTunnelAfterPickup();
        return;
    }
    lagern::maybeEinlagernFarbe(K_FARBE_GELB);
    moveToTunnelAfterPickup();
}

void AufnehmenModule::aufnehmenGruen()
{
    performPickupAt(
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D,
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_L,
        gripper_cfg::SEIL_ROTATIONEN_HAUS_BLAU_GRUEN
                    );
    if (!gripper_cfg::lager) {
        moveToTunnelAfterPickup();
        return;
    }
    lagern::maybeEinlagernFarbe(K_FARBE_GRUEN);
    moveToTunnelAfterPickup();
}
} // namespace aufnehmen

namespace abladen
{
namespace {
void performDropoffAt(float drehkranz_angle, float lenkung_angle, float seil_umdrehungen)
{
    drehkranzServo().setSteeringAngle(-drehkranz_angle);
    lenkungServo().setSteeringAngle(-lenkung_angle);
    thread_sleep_for(500);
    armMotor().setAndWait(seil_umdrehungen);
    armMotor().setAndWait(seil_umdrehungen * -1.0f);
}
}

AbladenModule::AbladenModule()
{
    gripper_actuators::initializeAll();
}

void AbladenModule::abladenRot()
{
    lagern::maybeAuslagernFarbe(K_FARBE_ROT);
    performDropoffAt(
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D,
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_L,
        gripper_cfg::SEIL_ROTATIONEN_HAUS_ROT_GELB
                    );
}

void AbladenModule::abladenBlau()
{
    lagern::maybeAuslagernFarbe(K_FARBE_BLAU);
    performDropoffAt(
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D,
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_L,
        gripper_cfg::SEIL_ROTATIONEN_HAUS_BLAU_GRUEN
                    );
}

void AbladenModule::abladenGelb()
{
    lagern::maybeAuslagernFarbe(K_FARBE_GELB);
    performDropoffAt(
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D,
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_L,
        gripper_cfg::SEIL_ROTATIONEN_HAUS_ROT_GELB
                    );
}

void AbladenModule::abladenGruen()
{
    lagern::maybeAuslagernFarbe(K_FARBE_GRUEN);
    performDropoffAt(
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D,
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_L,
        gripper_cfg::SEIL_ROTATIONEN_HAUS_BLAU_GRUEN
                    );
}
}