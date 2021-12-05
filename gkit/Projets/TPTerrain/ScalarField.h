#pragma once

#include "vec.h"
#include <vector>
#include "color.h"

struct Box2
{
    Box2(double xmin, double zmin, double xmax, double zmax)
        : xmin(xmin), xmax(xmax), zmin(zmin), zmax(zmax)
    {

    }

    double xmin, xmax;
    double zmin, zmax;
};

class ScalarField
{
public:
    ScalarField();

    ScalarField(int xmin, int zmin, int xmax, int zmax, int nx, int ny, const std::string& name);

    void InitValues();
    
    // Set in ctor
    Box2 extent;

    double dx, dy, dz; // delta in X and Z
    int nx, nz; // subdivision in X and Z
    Vector delta;
    Vector invDelta;
    Vector inv2Delta;

    double Slope(int i, int j) const;
    Vector Normal(int i, int j) const;
    Vector Gradient(int i, int j) const;
    double Laplacian(int i, int j) const;

    // TODO : move to Heightmap class
    double Normalize(double value) const;
    void UpdateMinMaxValues(double value);
    void UpdateMinMaxValues();
    void UpdateLaplacians();
    void UpdateSlopes();
    double minValue = std::numeric_limits<double>::max(), maxValue = std::numeric_limits<double>::min();
    double m;

    void ExportTexture(const std::string& filename) const;
    const std::string& GetName() const { return name; }

    virtual double& at(int i, int j) { return values.at(index(i, j)); }
    virtual const double& at(int i, int j) const { return values.at(index(i, j)); }
    const std::vector<double>& GetArray() const { return values; }
    int index(int i, int j) { return (i * nz) + j; }
    int index(int i, int j) const { return (i * nz) + j; }

    virtual ~ScalarField() {};
protected:
    std::vector<double> values;
    std::string name;
};