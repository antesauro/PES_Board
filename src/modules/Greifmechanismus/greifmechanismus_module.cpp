#include "greifmechanismus_module.h"

#include "actuators/motor_module_Arm.h"
#include "actuators/servo_module_Arm.h"

namespace {
arm_drehkranz::ServoModule g_servo_drehkranz;
arm_lenkung::ServoModule g_servo_lenkung;
MotorModuleArm g_motor_arm;

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

void pickOrDropHouse(float angle)
{
    g_servo_drehkranz.setSteeringAngle(angle);
    g_servo_lenkung.setSteeringAngle(gripper_cfg::AUFNEHMEN_ABLEGEN_LENKUNG);
    g_motor_arm.setAndWait(gripper_cfg::SEIL_HERUNTER_ROTATIONEN_HAUS);
    g_motor_arm.setAndWait(gripper_cfg::SEIL_HOCH_ROTATIONEN_HAUS);
}

void pickOrDropStorage(int pos)
{
    goToStoragePosition(pos);
    g_motor_arm.setAndWait(gripper_cfg::SEIL_HERUNTER_ROTATIONEN_LAGER);
    g_motor_arm.setAndWait(gripper_cfg::SEIL_HOCH_ROTATIONEN_LAGER);
}

void storeColorInFirstFreeSlot(int farbe)
{
    if (lagern::g_lager_pos_1 == K_LAGER_LEER) {
        pickOrDropStorage(1);
        lagern::g_lager_pos_1 = farbe;
    } else if (lagern::g_lager_pos_2 == K_LAGER_LEER) {
        pickOrDropStorage(2);
        lagern::g_lager_pos_2 = farbe;
    } else if (lagern::g_lager_pos_3 == K_LAGER_LEER) {
        pickOrDropStorage(3);
        lagern::g_lager_pos_3 = farbe;
    } else if (lagern::g_lager_pos_4 == K_LAGER_LEER) {
        pickOrDropStorage(4);
        lagern::g_lager_pos_4 = farbe;
    }
}

void unloadColorFromSlotIfPresent(int farbe)
{
    if (lagern::g_lager_pos_1 == farbe) {
        pickOrDropStorage(1);
        lagern::g_lager_pos_1 = K_LAGER_LEER;
    } else if (lagern::g_lager_pos_2 == farbe) {
        pickOrDropStorage(2);
        lagern::g_lager_pos_2 = K_LAGER_LEER;
    } else if (lagern::g_lager_pos_3 == farbe) {
        pickOrDropStorage(3);
        lagern::g_lager_pos_3 = K_LAGER_LEER;
    } else if (lagern::g_lager_pos_4 == farbe) {
        pickOrDropStorage(4);
        lagern::g_lager_pos_4 = K_LAGER_LEER;
    }
}
} // namespace

namespace lagern
{
int g_lager_pos_1 = 0;
int g_lager_pos_2 = 0;
int g_lager_pos_3 = 0;
int g_lager_pos_4 = 0;
} // namespace lagern

namespace aufnehmen
{
AufnehmenModule::AufnehmenModule()
{
    g_servo_drehkranz.initialize();
    g_servo_lenkung.initialize();
}

void AufnehmenModule::aufnehmenRot()
{
    pickOrDropHouse(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D);
    if (gripper_cfg::lager) {
        storeColorInFirstFreeSlot(K_FARBE_ROT);
    }
}

void AufnehmenModule::aufnehmenBlau()
{
    pickOrDropHouse(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D);
    if (gripper_cfg::lager) {
        storeColorInFirstFreeSlot(K_FARBE_BLAU);
    }
}

void AufnehmenModule::aufnehmenGelb()
{
    pickOrDropHouse(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D);
    if (gripper_cfg::lager) {
        storeColorInFirstFreeSlot(K_FARBE_GELB);
    }
}

void AufnehmenModule::aufnehmenGruen()
{
    pickOrDropHouse(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D);
    if (gripper_cfg::lager) {
        storeColorInFirstFreeSlot(K_FARBE_GRUEN);
    }
}
} // namespace aufnehmen

namespace abladen
{
AbladenModule::AbladenModule()
{
    g_servo_drehkranz.initialize();
    g_servo_lenkung.initialize();
}

void AbladenModule::abladenRot()
{
    if (gripper_cfg::lager) {
        unloadColorFromSlotIfPresent(K_FARBE_ROT);
    }
    pickOrDropHouse(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D);
}

void AbladenModule::abladenBlau()
{
    if (gripper_cfg::lager) {
        unloadColorFromSlotIfPresent(K_FARBE_BLAU);
    }
    pickOrDropHouse(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D);
}

void AbladenModule::abladenGelb()
{
    if (gripper_cfg::lager) {
        unloadColorFromSlotIfPresent(K_FARBE_GELB);
    }
    pickOrDropHouse(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D);
}

void AbladenModule::abladenGruen()
{
    if (gripper_cfg::lager) {
        unloadColorFromSlotIfPresent(K_FARBE_GRUEN);
    }
    pickOrDropHouse(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D);
}
} // namespace abladen
