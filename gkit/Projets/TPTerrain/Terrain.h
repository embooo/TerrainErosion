#include <chrono>

#include "mat.h"
#include "program.h"
#include "uniforms.h"
#include "wavefront.h"
#include "texture.h"

#include "orbiter.h"
#include "draw.h"

#include "app.h"        // classe Application a deriver
#include "text.h"

#include <array>
#include "ScalarField.h"
#include "Heightfield.h"



class Util
{
public:
    Util() = delete;

    template<typename T>
    static T clamp(const T& lo, const T& hi, const T& val)
    { 
        return std::min(std::max(0.0, val), hi);
    }

    /// https://stackoverflow.com/questions/7706339/grayscale-to-red-green-blue-matlab-jet-color-scale
    static Color ColorMapValue(double v, double vmin = 0.0, double vmax = 1.0)
    {
        Color c (1.0,1.0,1.0); // white

        if (v < vmin) { v = vmin; }
        if (v > vmax) { v = vmax; }

        double dv = vmax - vmin;

        if (v < (vmin + 0.25 * dv)) 
        {
            c.r = 0;
            c.g = 4 * (v - vmin) / dv;
        }
        else if (v < (vmin + 0.5 * dv)) 
        {
            c.r = 0;
            c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
        }
        else if (v < (vmin + 0.75 * dv))
        {
            c.r = 4 * (v - vmin - 0.5 * dv) / dv;
            c.b = 0;
        }
        else 
        {
            c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
            c.b = 0;
        }

        return c;
    }
};

class Terrain;

class TerrainApp : public App
{
public:
    TerrainApp();

    int init()   override;
    int quit()   override;
    int render() override;
    int update(const float time, const float delta) override;
    std::unique_ptr<Terrain> terrain;

protected:
    Orbiter m_Camera;
    GLuint m_Program;
};

class TerrainData
{
public:
    enum SF
    {
        HEIGHT, SLOPE, LAPLACIAN, DRAIN, NUM_SCALAR_FIELDS
    };

    const ScalarField& GetHeightSF()    const { return scalarField[SF::HEIGHT]; }
    const ScalarField& GetSlopeSF()     const { return scalarField[SF::SLOPE]; }
    const ScalarField& GetLaplacianSF() const { return scalarField[SF::LAPLACIAN]; }
    const ScalarField& GetDrainageSF()  const { return scalarField[SF::DRAIN]; }

    ScalarField& GetHeightSF()    { return scalarField[SF::HEIGHT];    }
    ScalarField& GetSlopeSF()     { return scalarField[SF::SLOPE];     }
    ScalarField& GetLaplacianSF() { return scalarField[SF::LAPLACIAN]; }
    ScalarField& GetDrainageSF()  { return scalarField[SF::DRAIN]; }

    const std::array<ScalarField, 4>& GetArray() { return scalarField;  }
protected:
    std::array<ScalarField, 4> scalarField;

};

class Terrain
{
public:
    Terrain() {}
    TerrainData data;
    void Polygonize();
    Mesh& GetMesh() { return mesh;  }
    Transform& GetModel() { return model; }
    void ComputeDrainage();
    void SimulateErosion();
    Mesh mesh;

public:
    void InitFromTexture(const Image& im);
public:
    void StreamPowerErosion(double k); 
    void HillSlopeErosion(double k);
protected:
    Transform model;
};


int main(int argc, char** argv)
{
    TerrainApp tp;
    tp.run();

    return 0;
}