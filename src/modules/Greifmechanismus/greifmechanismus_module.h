#pragma once

namespace gripper_cfg {

// Drehkranz-Positionen fuer Farben am Haus/Aufnahme.
inline constexpr float AUFNEHMEN_ABLEGEN_POS_ROT_GELB_D = 0.083f;
inline constexpr float AUFNEHMEN_ABLEGEN_POS_BLAU_GRUEN_D = 0.344f;

// Lenkung bei Aufnahme/Ablage am Haus.
inline constexpr float AUFNEHMEN_ABLEGEN_LENKUNG = 0.0f;

// Lagerplatz-Positionen am Drehkranz.
inline constexpr float LAGER_POS_1_D = 0.0f;
inline constexpr float LAGER_POS_2_D = 0.0f;
inline constexpr float LAGER_POS_3_D = 0.0f;
inline constexpr float LAGER_POS_4_D = 0.0f;

// Lenkung bei Lagerplaetzen.
inline constexpr float LAGER_LENKUNG = 0.0f;

// false: einzelnes Paket; true: Paket wird zusaetzlich im Lager verwaltet.
inline bool lager = false;

// Seilhub fuer Haus und Lager.
inline constexpr float SEIL_HERUNTER_ROTATIONEN_HAUS = 0.0f;
inline constexpr float SEIL_HOCH_ROTATIONEN_HAUS = -0.0f;
inline constexpr float SEIL_HERUNTER_ROTATIONEN_LAGER = 0.0f;
inline constexpr float SEIL_HOCH_ROTATIONEN_LAGER = -0.0f;

}

static constexpr int K_LAGER_PLAETZE = 4;
static constexpr int K_LAGER_LEER = 0;
static constexpr int K_FARBE_ROT = 1;
static constexpr int K_FARBE_BLAU = 2;
static constexpr int K_FARBE_GELB = 3;
static constexpr int K_FARBE_GRUEN = 4;

namespace lagern
{
extern int g_lager_pos_1;
extern int g_lager_pos_2;
extern int g_lager_pos_3;
extern int g_lager_pos_4;
}

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
}

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
}

using AufnehmenModule = aufnehmen::AufnehmenModule;
using AbladenModule = abladen::AbladenModule;
