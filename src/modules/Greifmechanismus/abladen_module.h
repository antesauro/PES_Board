#pragma once

#include "aufnehmen_module.h"

class AbladenModule
{
public:
	AbladenModule();

	bool auslagern(int farbe);
	int belegtePlaetze() const;
};
