#include "aufnehmen_module.h"

int g_lager_pos_1 = 0;
int g_lager_pos_2 = 0;
int g_lager_pos_3 = 0;
int g_lager_pos_4 = 0;

AufnehmenModule::AufnehmenModule()
{
}

bool AufnehmenModule::einlagern(int farbe)
{
	(void)farbe;
	return false;
}

int AufnehmenModule::freiePlaetze() const
{
	return 0;
}

bool AufnehmenModule::istVoll() const
{
	return freiePlaetze() == 0;
}

void AufnehmenModule::aufnehmenRot()
{
}

void AufnehmenModule::aufnehmenBlau()
{
}

void AufnehmenModule::aufnehmenGelb()
{
}

void AufnehmenModule::aufnehmenGruen()
{
}
