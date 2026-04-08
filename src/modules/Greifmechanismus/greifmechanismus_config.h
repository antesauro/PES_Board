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

// Seilhub fuer Haus und Lager.
inline constexpr float SEIL_HERUNTER_ROTATIONEN_HAUS = 0.0f;
inline constexpr float SEIL_HOCH_ROTATIONEN_HAUS = -0.0f;
inline constexpr float SEIL_HERUNTER_ROTATIONEN_LAGER = 0.0f;
inline constexpr float SEIL_HOCH_ROTATIONEN_LAGER = -0.0f;

}