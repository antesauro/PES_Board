#include "greifmechanismus_module.h"

#include "servo_module_Arm.h"
#include "motor_module_Arm.h"

namespace {
arm_drehkranz::ServoModule g_servo_drehkranz;
arm_lenkung::ServoModule g_servo_lenkung;
MotorModuleArm g_motor_arm;
}

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
        g_servo_drehkranz.setSteeringAngle(gripper_cfg::LAGER_POS_1_D);
    } else if (pos == 2) {
        g_servo_drehkranz.setSteeringAngle(gripper_cfg::LAGER_POS_2_D);
    } else if (pos == 3) {
        g_servo_drehkranz.setSteeringAngle(gripper_cfg::LAGER_POS_3_D);
    } else if (pos == 4) {
        g_servo_drehkranz.setSteeringAngle(gripper_cfg::LAGER_POS_4_D);
    }
    g_servo_lenkung.setSteeringAngle(gripper_cfg::LAGER_LENKUNG);
}

void einlagernposition(int pos)
{
    goToStoragePosition(pos);
    g_motor_arm.setAndWait(gripper_cfg::SEIL_HERUNTER_ROTATIONEN_LAGER);
    g_motor_arm.setAndWait(gripper_cfg::SEIL_HOCH_ROTATIONEN_LAGER);
}

void auslagernposition(int pos)
{
    goToStoragePosition(pos);
    g_motor_arm.setAndWait(gripper_cfg::SEIL_HERUNTER_ROTATIONEN_LAGER);
    g_motor_arm.setAndWait(gripper_cfg::SEIL_HOCH_ROTATIONEN_LAGER);
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
    g_servo_drehkranz.setSteeringAngle(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_tunnel);
    g_servo_lenkung.setSteeringAngle(0.25f);
}

void performPickupAt(float drehkranzAngle)
{
    g_servo_drehkranz.setSteeringAngle(drehkranzAngle);
    g_servo_lenkung.setSteeringAngle(gripper_cfg::AUFNEHMEN_ABLEGEN_LENKUNG);
    g_motor_arm.setAndWait(gripper_cfg::SEIL_HERUNTER_ROTATIONEN_HAUS);
    g_motor_arm.setAndWait(gripper_cfg::SEIL_HOCH_ROTATIONEN_HAUS);
}
}

AufnehmenModule::AufnehmenModule()
{
    g_servo_drehkranz.initialize();
    g_servo_lenkung.initialize();
}

void AufnehmenModule::aufnehmenRot()
{
    performPickupAt(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D);
    if (!gripper_cfg::lager) {
        moveToTunnelAfterPickup();
        return;
    }
    lagern::maybeEinlagernFarbe(K_FARBE_ROT);
    moveToTunnelAfterPickup();
}

void AufnehmenModule::aufnehmenBlau()
{
    performPickupAt(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D);
    if (!gripper_cfg::lager) {
        moveToTunnelAfterPickup();
        return;
    }
    lagern::maybeEinlagernFarbe(K_FARBE_BLAU);
    moveToTunnelAfterPickup();
}

void AufnehmenModule::aufnehmenGelb()
{
    performPickupAt(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D);
    if (!gripper_cfg::lager) {
        moveToTunnelAfterPickup();
        return;
    }
    lagern::maybeEinlagernFarbe(K_FARBE_GELB);
    moveToTunnelAfterPickup();
}

void AufnehmenModule::aufnehmenGruen()
{
    performPickupAt(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D);
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
void performDropoffAt(float drehkranzAngle)
{
    g_servo_drehkranz.setSteeringAngle(drehkranzAngle);
    g_servo_lenkung.setSteeringAngle(gripper_cfg::AUFNEHMEN_ABLEGEN_LENKUNG);
    g_motor_arm.setAndWait(gripper_cfg::SEIL_HERUNTER_ROTATIONEN_HAUS);
    g_motor_arm.setAndWait(gripper_cfg::SEIL_HOCH_ROTATIONEN_HAUS);
}
}

AbladenModule::AbladenModule()
{
    g_servo_drehkranz.initialize();
    g_servo_lenkung.initialize();
}

void AbladenModule::abladenRot()
{
    lagern::maybeAuslagernFarbe(K_FARBE_ROT);
    performDropoffAt(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D);
}

void AbladenModule::abladenBlau()
{
    lagern::maybeAuslagernFarbe(K_FARBE_BLAU);
    performDropoffAt(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D);
}

void AbladenModule::abladenGelb()
{
    lagern::maybeAuslagernFarbe(K_FARBE_GELB);
    performDropoffAt(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D);
}

void AbladenModule::abladenGruen()
{
    lagern::maybeAuslagernFarbe(K_FARBE_GRUEN);
    performDropoffAt(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D);
}
} // namespace abladen
