#include "greifmechanismus_module.h"

#include "servo_module_Arm.h"
#include "motor_module_Arm.h"

namespace {
arm_drehkranz::ServoModule* g_servo_drehkranz = nullptr;
arm_lenkung::ServoModule* g_servo_lenkung = nullptr;
MotorModuleArm* g_motor_arm = nullptr;

void initActuators()
{
    if (!g_servo_drehkranz) g_servo_drehkranz = new arm_drehkranz::ServoModule();
    if (!g_servo_lenkung) g_servo_lenkung = new arm_lenkung::ServoModule();
    if (!g_motor_arm) g_motor_arm = new MotorModuleArm();
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
        g_servo_drehkranz->setSteeringAngle(gripper_cfg::LAGER_POS_1_D);
        g_servo_lenkung->setSteeringAngle(gripper_cfg::LAGER_POS_1_L);
    } else if (pos == 2) {
        g_servo_drehkranz->setSteeringAngle(gripper_cfg::LAGER_POS_2_D);
        g_servo_lenkung->setSteeringAngle(gripper_cfg::LAGER_POS_2_L);
    } else if (pos == 3) {
        g_servo_drehkranz->setSteeringAngle(gripper_cfg::LAGER_POS_3_D);
        g_servo_lenkung->setSteeringAngle(gripper_cfg::LAGER_POS_3_L);
    } else if (pos == 4) {
        g_servo_drehkranz->setSteeringAngle(gripper_cfg::LAGER_POS_4_D);
        g_servo_lenkung->setSteeringAngle(gripper_cfg::LAGER_POS_4_L);
    }
}

void pickOrDropStorage(int pos, float seil_umdrehungen)
{
    goToStoragePosition(pos);
    g_motor_arm->setAndWait(seil_umdrehungen);
    g_motor_arm->setAndWait(seil_umdrehungen * -1.0f);
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
    g_servo_drehkranz->setSteeringAngle(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_tunnel);
    g_servo_lenkung->setSteeringAngle(0.25f);
}

void performPickupAt(float drehkranz_angle, float lenkung_angle, float seil_umdrehungen)
{
    g_servo_drehkranz->setSteeringAngle(drehkranz_angle);
    g_servo_lenkung->setSteeringAngle(lenkung_angle);
    g_motor_arm->setAndWait(seil_umdrehungen);
    g_motor_arm->setAndWait(seil_umdrehungen * -1.0f);
}
}

AufnehmenModule::AufnehmenModule()
{
    initActuators();
    g_servo_drehkranz->initialize();
    g_servo_lenkung->initialize();
}

void AufnehmenModule::aufnehmenRot()
{
    performPickupAt(
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D,
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_L,
        gripper_cfg::SEIL_ROTATIONEN_HAUS_ROT_GELB);
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
        gripper_cfg::SEIL_ROTATIONEN_HAUS_BLAU_GRUEN);
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
        gripper_cfg::SEIL_ROTATIONEN_HAUS_ROT_GELB);
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
        gripper_cfg::SEIL_ROTATIONEN_HAUS_BLAU_GRUEN);
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
    g_servo_drehkranz->setSteeringAngle(drehkranz_angle);
    g_servo_lenkung->setSteeringAngle(lenkung_angle);
    g_motor_arm->setAndWait(seil_umdrehungen);
    g_motor_arm->setAndWait(seil_umdrehungen * -1.0f);
}
}

AbladenModule::AbladenModule()
{
    initActuators();
    g_servo_drehkranz->initialize();
    g_servo_lenkung->initialize();
}

void AbladenModule::abladenRot()
{
    lagern::maybeAuslagernFarbe(K_FARBE_ROT);
    performDropoffAt(
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D,
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_L,
        gripper_cfg::SEIL_ROTATIONEN_HAUS_ROT_GELB);
}

void AbladenModule::abladenBlau()
{
    lagern::maybeAuslagernFarbe(K_FARBE_BLAU);
    performDropoffAt(
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D,
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_L,
        gripper_cfg::SEIL_ROTATIONEN_HAUS_BLAU_GRUEN);
}

void AbladenModule::abladenGelb()
{
    lagern::maybeAuslagernFarbe(K_FARBE_GELB);
    performDropoffAt(
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D,
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_L,
        gripper_cfg::SEIL_ROTATIONEN_HAUS_ROT_GELB);
}

void AbladenModule::abladenGruen()
{
    lagern::maybeAuslagernFarbe(K_FARBE_GRUEN);
    performDropoffAt(
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D,
        gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_L,
        gripper_cfg::SEIL_ROTATIONEN_HAUS_BLAU_GRUEN);
}
} // namespace abladen