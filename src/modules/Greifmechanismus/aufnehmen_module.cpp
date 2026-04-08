#include "aufnehmen_module.h"
#include "servo_module_Arm.h"
#include "motor_module_Arm.h"
#include "greifmechanismus_config.h"
// Shared storage state: 0=leer, 1=rot, 2=blau, 3=gelb, 4=gruen.
int g_lager_pos_1 = 0;
int g_lager_pos_2 = 0;
int g_lager_pos_3 = 0;
int g_lager_pos_4 = 0;

namespace {
arm_drehkranz::ServoModule g_servo_drehkranz;
arm_lenkung::ServoModule g_servo_lenkung;
MotorModuleArm g_motor_arm;
}

AufnehmenModule::AufnehmenModule()
{
	g_servo_drehkranz.initialize();
	g_servo_lenkung.initialize();
}

int AufnehmenModule::einlagernposition1()
{//1. servo auf lager pos 1 fahren
g_servo_drehkranz.setSteeringAngle(gripper_cfg::LAGER_POS_1_D); // drehkranz auf pos fahren
g_servo_lenkung.setSteeringAngle(gripper_cfg::LAGER_LENKUNG); // lenkung auf pos fahren
//2. magnet herunter fahren 
g_motor_arm.setAndWait(gripper_cfg::SEIL_HERUNTER_ROTATIONEN_LAGER);
//3. magnet ausschalten

//3. magnet hochfahren
g_motor_arm.setAndWait(gripper_cfg::SEIL_HOCH_ROTATIONEN_LAGER);
	return true;
}
int AufnehmenModule::einlagernposition2()
{//1. servo auf lager pos 2 fahren
g_servo_drehkranz.setSteeringAngle(gripper_cfg::LAGER_POS_2_D); // drehkranz auf pos fahren
g_servo_lenkung.setSteeringAngle(gripper_cfg::LAGER_LENKUNG); // lenkung auf pos fahren
//2. magnet herunter fahren 
g_motor_arm.setAndWait(gripper_cfg::SEIL_HERUNTER_ROTATIONEN_LAGER);
//3. magnet ausschalten

//3. magnet hochfahren
g_motor_arm.setAndWait(gripper_cfg::SEIL_HOCH_ROTATIONEN_LAGER);
	return true;
}
int AufnehmenModule::einlagernposition3()
{//1. servo auf lager pos 3 fahren
g_servo_drehkranz.setSteeringAngle(gripper_cfg::LAGER_POS_3_D); // drehkranz auf pos fahren
g_servo_lenkung.setSteeringAngle(gripper_cfg::LAGER_LENKUNG); // lenkung auf pos fahren
//2. magnet herunter fahren 
g_motor_arm.setAndWait(gripper_cfg::SEIL_HERUNTER_ROTATIONEN_LAGER);
//3. magnet ausschalten

//3. magnet hochfahren
g_motor_arm.setAndWait(gripper_cfg::SEIL_HOCH_ROTATIONEN_LAGER);
	return true;
}
int AufnehmenModule::einlagernposition4()
{//1. servo auf lager pos 4 fahren
g_servo_drehkranz.setSteeringAngle(gripper_cfg::LAGER_POS_4_D); // drehkranz auf pos fahren
g_servo_lenkung.setSteeringAngle(gripper_cfg::LAGER_LENKUNG); // lenkung auf pos fahren
//2. magnet herunter fahren 
g_motor_arm.setAndWait(gripper_cfg::SEIL_HERUNTER_ROTATIONEN_LAGER);
//3. magnet ausschalten

//3. magnet hochfahren
g_motor_arm.setAndWait(gripper_cfg::SEIL_HOCH_ROTATIONEN_LAGER);
	return true;
}

void AufnehmenModule::aufnehmenRot()
{
//1. servo auf pos paket rot fahren
g_servo_drehkranz.setSteeringAngle(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D); // drehkranz auf pos fahren
g_servo_lenkung.setSteeringAngle(gripper_cfg::AUFNEHMEN_ABLEGEN_LENKUNG); // lenkung auf pos fahren
//2. elektro magnet einschalten
//2. magnet herunter fahren 
g_motor_arm.setAndWait(gripper_cfg::SEIL_HERUNTER_ROTATIONEN_LAGER);
//3. magnet hochfahren
g_motor_arm.setAndWait(gripper_cfg::SEIL_HOCH_ROTATIONEN_LAGER);
//4. naechste ferie lager pos abfrggen 
if (g_lager_pos_1 == 0)
{
	einlagernposition1();
	g_lager_pos_1 = 1;
	
}
else if (g_lager_pos_2 == 0)
{
	einlagernposition2();
	g_lager_pos_2 = 1;
	
}
else if (g_lager_pos_3 == 0)
{
	einlagernposition3();
	g_lager_pos_3 = 1;
	
}
else if (g_lager_pos_4 == 0)
{
	einlagernposition4();
	g_lager_pos_4 = 1;
	
}
}

void AufnehmenModule::aufnehmenBlau()
{
//1. servo auf pos paket rot fahren
g_servo_drehkranz.setSteeringAngle(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D); // drehkranz auf pos fahren
g_servo_lenkung.setSteeringAngle(gripper_cfg::AUFNEHMEN_ABLEGEN_LENKUNG); // lenkung auf pos fahren
//2. elektro magnet einschalten
//2. magnet herunter fahren 
g_motor_arm.setAndWait(gripper_cfg::SEIL_HERUNTER_ROTATIONEN_LAGER);
//3. magnet hochfahren
g_motor_arm.setAndWait(gripper_cfg::SEIL_HOCH_ROTATIONEN_LAGER);
//4. naechste ferie lager pos abfrggen 
if (g_lager_pos_1 == 0)
{
	einlagernposition1();
	g_lager_pos_1 = 2;
	
}
else if (g_lager_pos_2 == 0)
{
	einlagernposition2();
	g_lager_pos_2 = 2;
	
}
else if (g_lager_pos_3 == 0)
{
	einlagernposition3();
	g_lager_pos_3 = 2;
	
}
else if (g_lager_pos_4 == 0)
{
	einlagernposition4();
	g_lager_pos_4 = 2;
	
}
	
}

void AufnehmenModule::aufnehmenGelb()
{
//1. servo auf pos paket rot fahren
g_servo_drehkranz.setSteeringAngle(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D); // drehkranz auf pos fahren
g_servo_lenkung.setSteeringAngle(gripper_cfg::AUFNEHMEN_ABLEGEN_LENKUNG); // lenkung auf pos fahren
//2. elektro magnet einschalten
//2. magnet herunter fahren 
g_motor_arm.setAndWait(gripper_cfg::SEIL_HERUNTER_ROTATIONEN_LAGER);
//3. magnet hochfahren
g_motor_arm.setAndWait(gripper_cfg::SEIL_HOCH_ROTATIONEN_LAGER);
//4. naechste ferie lager pos abfrggen 
if (g_lager_pos_1 == 0)
{
	einlagernposition1();
	g_lager_pos_1 = 3;
	
}
else if (g_lager_pos_2 == 0)
{
	einlagernposition2();
	g_lager_pos_2 = 3;
	
}
else if (g_lager_pos_3 == 0)
{
	einlagernposition3();
	g_lager_pos_3 = 3;
	
}
else if (g_lager_pos_4 == 0)
{
	einlagernposition4();
	g_lager_pos_4 = 3;
	
}

}

void AufnehmenModule::aufnehmenGruen()
{
	
//1. servo auf pos paket rot fahren
g_servo_drehkranz.setSteeringAngle(gripper_cfg::AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D); // drehkranz auf pos fahren
g_servo_lenkung.setSteeringAngle(gripper_cfg::AUFNEHMEN_ABLEGEN_LENKUNG); // lenkung auf pos fahren
//2. elektro magnet einschalten
//2. magnet herunter fahren 
g_motor_arm.setAndWait(gripper_cfg::SEIL_HERUNTER_ROTATIONEN_LAGER);
//3. magnet hochfahren
g_motor_arm.setAndWait(gripper_cfg::SEIL_HOCH_ROTATIONEN_LAGER);
//4. naechste ferie lager pos abfrggen 
if (g_lager_pos_1 == 0)
{
	einlagernposition1();
	g_lager_pos_1 = 4;
	
}
else if (g_lager_pos_2 == 0)
{
	einlagernposition2();
	g_lager_pos_2 = 4;
	
}
else if (g_lager_pos_3 == 0)
{
	einlagernposition3();
	g_lager_pos_3 = 4;
	
}
else if (g_lager_pos_4 == 0)
{
	einlagernposition4();
	g_lager_pos_4 = 4;
	
}
}
