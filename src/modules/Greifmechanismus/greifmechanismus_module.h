#pragma once

class MotorModuleArm;

namespace gripper_cfg {

// Drehkranz-Positionen fuer Farben am Haus/Aufnahme.
constexpr float AUFNEHMEN_ABLEGEN_POS_tunnel_D = 0.7f;
constexpr float AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D = 0.45f;
constexpr float AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D = 0.18f;

// Lenkung bei Aufnahme/Ablage am Haus fuer.
constexpr float AUFNEHMEN_ABLEGEN_POS_tunnel_L = 0.5f;
constexpr float AUFNEHMEN_ABLEGEN_POS_ROT_GELB_L = 0.4f;
constexpr float AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_L = 0.2f;
// Sicherheitswinkel fuer vertikale Ausrichtung des Arms
constexpr float VERTIKAL_SICHERHEITS_WINKEL_L = 1.0f;

// Lagerplatz-Positionen am Drehkranz.
constexpr float LAGER_POS_1_D = 0.0;
constexpr float LAGER_POS_2_D = 0.0f;
constexpr float LAGER_POS_3_D = 0.0f;
constexpr float LAGER_POS_4_D = 0.0f;

// Lenkung bei Lagerplaetzen.
constexpr float LAGER_POS_1_L = 0.0f;
constexpr float LAGER_POS_2_L = 0.0f;
constexpr float LAGER_POS_3_L = 0.0f;
constexpr float LAGER_POS_4_L = 0.0f;
// false: einzelnes Paket; true: Paket wird zusaetzlich im Lager verwaltet.
extern bool lager;

// Seilhub fuer Haus und Lager.
// ROT_GRUEN
constexpr float SEIL_ROTATIONEN_HAUS_ROT_GRUEN = 2.5f;
// blau_gruen
constexpr float SEIL_ROTATIONEN_HAUS_BLAU_GELB = 4.3f;
// Lagerpos1
constexpr float SEIL_ROTATIONEN_LAGER_POS_1 = 2.5f;
// Lagerpos2
constexpr float SEIL_ROTATIONEN_LAGER_POS_2 = 2.5f;
// Lagerpos3
constexpr float SEIL_ROTATIONEN_LAGER_POS_3 = 2.5f;
// Lagerpos4
constexpr float SEIL_ROTATIONEN_LAGER_POS_4 = 2.5f;

} // namespace gripper_cfg

namespace gripper_actuators {
void initializeDrehkranzServo();
void initializeLenkungServo();
void initializeArmMotor();
void initializeAll();
MotorModuleArm &getArmMotor();
void disableDrehkranzServo();
void disableLenkungServo();
void disableArmMotor();
void disableAll();
} // namespace gripper_actuators

static constexpr int K_LAGER_PLAETZE = 4;
static constexpr int K_LAGER_LEER = 0;
static constexpr int K_FARBE_ROT = 1;
static constexpr int K_FARBE_BLAU = 2;
static constexpr int K_FARBE_GELB = 3;
static constexpr int K_FARBE_GRUEN = 4;

namespace lagern {
// Shared storage state: 0=leer, 1=rot, 2=blau, 3=gelb, 4=gruen.
extern int g_lager_pos_1;
extern int g_lager_pos_2;
extern int g_lager_pos_3;
extern int g_lager_pos_4;
} // namespace lagern

namespace aufnehmen {
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

namespace abladen {
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
