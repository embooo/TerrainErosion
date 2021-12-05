#pragma once

#include "ScalarField.h"

class Heightfield : public ScalarField
{
public:
    Heightfield();
    void UpdateMinMaxElevation(double elevation);

    ScalarField slopeField;
};
