#pragma once

static constexpr int K_LAGER_PLAETZE = 4;

// Shared storage state: one color variable per storage position.
extern int g_lager_pos_1;
extern int g_lager_pos_2;
extern int g_lager_pos_3;
extern int g_lager_pos_4;

class AufnehmenModule
{
public:
	AufnehmenModule();

	bool einlagern(int farbe);
	int freiePlaetze() const;
	bool istVoll() const;
};
