#include "abladen_module.h"

#include "servo_module_Arm.h"
#include "motor_module_Arm.h"
#include "greifmechanismus_config.h"
bool schon_abgelegt_rot=false;
bool schon_abgelegt_blau=false;
bool schon_abgelegt_gelb=false;
bool schon_abgelegt_gruen=false;

namespace {
arm_drehkranz::ServoModule g_servo_drehkranz;
arm_lenkung::ServoModule g_servo_lenkung;
MotorModuleArm g_motor_arm;

int findSlotByColor(int farbe)
{
    if (g_lager_pos_1 == farbe) return 1;
    if (g_lager_pos_2 == farbe) return 2;
    if (g_lager_pos_3 == farbe) return 3;
    if (g_lager_pos_4 == farbe) return 4;
    return 0;
}

float drehkranzPosForSlot(int slot)
{
	if (slot == 1) return gripper_cfg::LAGER_POS_1_D;
	if (slot == 2) return gripper_cfg::LAGER_POS_2_D;
	if (slot == 3) return gripper_cfg::LAGER_POS_3_D;
	return gripper_cfg::LAGER_POS_4_D;
}

void clearSlot(int slot)
{
    if (slot == 1) g_lager_pos_1 = 0;
    else if (slot == 2) g_lager_pos_2 = 0;
    else if (slot == 3) g_lager_pos_3 = 0;
    else if (slot == 4) g_lager_pos_4 = 0;
}

float drehkranzPosForHouseColor(int farbe)
{
    if (farbe == K_FARBE_ROT || farbe == K_FARBE_GELB)
		return gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D;
	return gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D;
}

bool unloadFromSlotToHouse(int slot, int farbe)
{
	if (slot == 0)
		return false;

	// 1) Zum passenden Lagerplatz fahren und Paket aufnehmen.
	g_servo_drehkranz.setSteeringAngle(drehkranzPosForSlot(slot));
	g_servo_lenkung.setSteeringAngle(gripper_cfg::LAGER_LENKUNG);
	g_motor_arm.setAndWait(gripper_cfg::SEIL_HERUNTER_ROTATIONEN_LAGER);
	g_motor_arm.setAndWait(gripper_cfg::SEIL_HOCH_ROTATIONEN_LAGER);

	// 2) Zum Haus fahren und Paket ablegen.
	g_servo_drehkranz.setSteeringAngle(drehkranzPosForHouseColor(farbe));
	g_servo_lenkung.setSteeringAngle(gripper_cfg::AUFNEHMEN_ABLEGEN_LENKUNG);
	g_motor_arm.setAndWait(gripper_cfg::SEIL_HERUNTER_ROTATIONEN_HAUS);
	g_motor_arm.setAndWait(gripper_cfg::SEIL_HOCH_ROTATIONEN_HAUS);

	// 3) Lagerplatz freigeben.
	clearSlot(slot);
	return true;
}
}

AbladenModule::AbladenModule()
{
    g_servo_drehkranz.initialize();
    g_servo_lenkung.initialize();
}

bool AbladenModule::auslagern(int farbe)
{
	if (farbe == K_FARBE_ROT) {
		abladenRot();
		return true;
	}
	if (farbe == K_FARBE_BLAU) {
		abladenBlau();
		return true;
	}
	if (farbe == K_FARBE_GELB) {
		abladenGelb();
		return true;
	}
	if (farbe == K_FARBE_GRUEN) {
		abladenGruen();
		return true;
	}
	return false;
}

int AbladenModule::belegtePlaetze() const
{
	int belegt = 0;
	if (g_lager_pos_1 != 0) belegt++;
	if (g_lager_pos_2 != 0) belegt++;
	if (g_lager_pos_3 != 0) belegt++;
	if (g_lager_pos_4 != 0) belegt++;
	return belegt;
}

void AbladenModule::abladenRot()
{
	if (g_lager_pos_1 == K_FARBE_ROT) {
		(void)unloadFromSlotToHouse(1, K_FARBE_ROT);
	} else if (g_lager_pos_2 == K_FARBE_ROT) {
		(void)unloadFromSlotToHouse(2, K_FARBE_ROT);
	} else if (g_lager_pos_3 == K_FARBE_ROT) {
		(void)unloadFromSlotToHouse(3, K_FARBE_ROT);
	} else if (g_lager_pos_4 == K_FARBE_ROT) {
		(void)unloadFromSlotToHouse(4, K_FARBE_ROT);
	}
}

void AbladenModule::abladenBlau()
{
	if (g_lager_pos_1 == K_FARBE_BLAU) {
		(void)unloadFromSlotToHouse(1, K_FARBE_BLAU);
	} else if (g_lager_pos_2 == K_FARBE_BLAU) {
		(void)unloadFromSlotToHouse(2, K_FARBE_BLAU);
	} else if (g_lager_pos_3 == K_FARBE_BLAU) {
		(void)unloadFromSlotToHouse(3, K_FARBE_BLAU);
	} else if (g_lager_pos_4 == K_FARBE_BLAU) {
		(void)unloadFromSlotToHouse(4, K_FARBE_BLAU);
	}
}

void AbladenModule::abladenGelb()
{
	if (g_lager_pos_1 == K_FARBE_GELB) {
		(void)unloadFromSlotToHouse(1, K_FARBE_GELB);
	} else if (g_lager_pos_2 == K_FARBE_GELB) {
		(void)unloadFromSlotToHouse(2, K_FARBE_GELB);
	} else if (g_lager_pos_3 == K_FARBE_GELB) {
		(void)unloadFromSlotToHouse(3, K_FARBE_GELB);
	} else if (g_lager_pos_4 == K_FARBE_GELB) {
		(void)unloadFromSlotToHouse(4, K_FARBE_GELB);
	}
}

void AbladenModule::abladenGruen()
{
	if (g_lager_pos_1 == K_FARBE_GRUEN) {
		(void)unloadFromSlotToHouse(1, K_FARBE_GRUEN);
	} else if (g_lager_pos_2 == K_FARBE_GRUEN) {
		(void)unloadFromSlotToHouse(2, K_FARBE_GRUEN);
	} else if (g_lager_pos_3 == K_FARBE_GRUEN) {
		(void)unloadFromSlotToHouse(3, K_FARBE_GRUEN);
	} else if (g_lager_pos_4 == K_FARBE_GRUEN) {
		(void)unloadFromSlotToHouse(4, K_FARBE_GRUEN);
	}
}
