#pragma once

class MotorModuleArm;

namespace gripper_cfg {

// Drehkranz-Positionen fuer Farben am Haus/Aufnahme.
constexpr float AUFNEHMEN_ABLEGEN_POS_tunnel = 0.0f;
constexpr float AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D = 0.083f;
constexpr float AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D = 0.344f;

// Lenkung bei Aufnahme/Ablage am Haus fuer.
constexpr float AUFNEHMEN_ABLEGEN_POS_ROT_GELB_L = 0.15f;
constexpr float AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_L = 0.25f;

// Lagerplatz-Positionen am Drehkranz.
constexpr float LAGER_POS_1_D = 0.10f;
constexpr float LAGER_POS_2_D = 0.20f;
constexpr float LAGER_POS_3_D = 0.30f;
constexpr float LAGER_POS_4_D = 0.40f;

// Lenkung bei Lagerplaetzen.
constexpr float LAGER_POS_1_L = 0.12f;
constexpr float LAGER_POS_2_L = 0.18f;
constexpr float LAGER_POS_3_L = 0.22f;
constexpr float LAGER_POS_4_L = 0.28f;
// false: einzelnes Paket; true: Paket wird zusaetzlich im Lager verwaltet.
extern bool lager;

// Seilhub fuer Haus und Lager.
//rot_gelb
constexpr float SEIL_ROTATIONEN_HAUS_ROT_GELB = 1.5f;
//blau_gruen
constexpr float SEIL_ROTATIONEN_HAUS_BLAU_GRUEN = 1.8f;
//Lagerpos1
constexpr float SEIL_ROTATIONEN_LAGER_POS_1 = 1.2f;
//Lagerpos2
constexpr float SEIL_ROTATIONEN_LAGER_POS_2 = 1.4f;
//Lagerpos3
constexpr float SEIL_ROTATIONEN_LAGER_POS_3 = 1.6f;
//Lagerpos4
constexpr float SEIL_ROTATIONEN_LAGER_POS_4 = 2.0f;

}

namespace gripper_actuators
{
void initializeDrehkranzServo();
void initializeLenkungServo();
void initializeArmMotor();
void initializeAll();
MotorModuleArm& getArmMotor();
void disableDrehkranzServo();
void disableLenkungServo();
void disableArmMotor();
void disableAll();
}

static constexpr int K_LAGER_PLAETZE = 4;
static constexpr int K_LAGER_LEER = 0;
static constexpr int K_FARBE_ROT = 1;
static constexpr int K_FARBE_BLAU = 2;
static constexpr int K_FARBE_GELB = 3;
static constexpr int K_FARBE_GRUEN = 4;

namespace lagern
{
// Shared storage state: 0=leer, 1=rot, 2=blau, 3=gelb, 4=gruen.
extern int g_lager_pos_1;
extern int g_lager_pos_2;
extern int g_lager_pos_3;
extern int g_lager_pos_4;
} // namespace lagern

namespace aufnehmen
{
class AufnehmenModule
{
public:
    AufnehmenModule();

    void aufnehmenRot();
    void aufnehmenBlau();
    void aufnehmenGelb();
    void aufnehmenGruen();
};
} // namespace aufnehmen

namespace abladen
{
class AbladenModule
{
public:
    AbladenModule();

    void abladenRot();
    void abladenBlau();
    void abladenGelb();
    void abladenGruen();
};
} // namespace abladen
