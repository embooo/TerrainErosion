#include "ScalarField.h"

ScalarField::ScalarField()
	: extent(0, 0, 100, 100), nx(400), nz(400)
{
	values.resize(nx * nz);
	InitValues();
}

ScalarField::ScalarField(int xmin, int zmin, int xmax, int zmax, int nx, int nz, const std::string& name)
	: extent(xmin, zmin, xmax, zmax), nx(nx), nz(nz), name(name)
{
	values.resize(nx * nz);
	InitValues();
}

void ScalarField::InitValues()
{
	dx = (extent.xmax - extent.xmin) / (double)(nx - 1);
	dy = 10.0;
	dz = (extent.zmax - extent.zmin) / (double)(nz - 1);

	invDelta.x = 1.0 / dx;
	invDelta.z = 1.0 / dz;

	inv2Delta.x = 1.0 / (2.0 * dx);
	inv2Delta.z = 1.0 / (2.0 * dz);
}