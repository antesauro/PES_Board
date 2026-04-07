#pragma once

static constexpr int K_LAGER_PLAETZE = 4;
static constexpr int K_LAGER_LEER = 0;
static constexpr int K_FARBE_ROT = 1;
static constexpr int K_FARBE_BLAU = 2;
static constexpr int K_FARBE_GELB = 3;
static constexpr int K_FARBE_GRUEN = 4;

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

	void aufnehmenRot();
	void aufnehmenBlau();
	void aufnehmenGelb();
	void aufnehmenGruen();
};
