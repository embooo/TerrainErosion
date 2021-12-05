#include "Terrain.h"

#include <sstream>

TerrainApp::TerrainApp()
	: App(1280, 720)
{
	glClearColor(0.5f, 0.0f, 0.5f, 1.f);

	glClearDepth(1.f);
	glDepthFunc(GL_LESS);
	glEnable(GL_DEPTH_TEST);

	glCullFace(GL_BACK);
}

int TerrainApp::init()
{
	terrain = std::make_unique<Terrain>();
	m_Program = read_program("data/shaders/mesh_color.glsl");
	program_print_errors(m_Program);
	Image hm = read_image("data/hm2.jpg");
	terrain->InitFromTexture(hm);
	terrain->Polygonize();

	// Export scalar field to textures
	for (const ScalarField& sf : terrain->data.GetArray())
	{
		sf.ExportTexture(sf.GetName());
	}

	Point pmin, pmax;
	terrain->GetMesh().bounds(pmin, pmax);
	m_Camera.lookat(pmin, pmax);
	
	return 0;
}

void Terrain::SimulateErosion()
{
	static int nbIteration = 0;
	auto start = std::chrono::steady_clock::now();
	StreamPowerErosion(15.0);
	//StreamPowerErosion(0.5);
	//StreamPowerErosion(0.5);

	HillSlopeErosion(10.0);

	//HillSlopeErosion(0.5);
	//HillSlopeErosion(0.05);
	//HillSlopeErosion(0.08);

	// Update laplacian and slope fields
	data.GetHeightSF().UpdateMinMaxValues();
	data.GetLaplacianSF().UpdateMinMaxValues();
	data.GetSlopeSF().UpdateMinMaxValues();
	data.GetDrainageSF().UpdateMinMaxValues();
	ComputeDrainage();
	auto end = std::chrono::steady_clock::now();
	std::chrono::duration<double> elapsed_seconds = end - start;

	Polygonize();
	nbIteration++;

	printf("Time: %f ms [iteration:%d]\n", elapsed_seconds.count() * 1000.0, nbIteration);


}

int TerrainApp::quit()
{
	return 0;
}

int TerrainApp::render()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	draw(terrain->GetMesh(), terrain->GetModel(), m_Camera);
	return 1;
}

int TerrainApp::update(const float time, const float delta)
{
	//hf->model = RotationY(10.0/delta)(hf->model);
	// deplace la camera
	int mx, my;
	unsigned int mb = SDL_GetRelativeMouseState(&mx, &my);
	if (mb & SDL_BUTTON(1))              // le bouton gauche est enfonce
		m_Camera.rotation(mx / (float)delta, my / (float)delta);
	else if (mb & SDL_BUTTON(3))         // le bouton droit est enfonce
		m_Camera.move(mx / (float)delta);
	else if (mb & SDL_BUTTON(2))         // le bouton du milieu est enfonce
		m_Camera.translation((float)mx / (float)window_width(), (float)my / (float)window_height());

	if (key_state(SDLK_p))
	{
		terrain->SimulateErosion();
	}

	return 0;
}


/////////////////////////////////////////////////////////////

void Terrain::InitFromTexture(const Image& im)
{
	data.GetHeightSF()    = ScalarField(0, 0, 5000, 5000, im.width(), im.height(), "HeightField");
	data.GetSlopeSF()     = ScalarField(0, 0, 5000, 5000, im.width(), im.height(), "SlopeField");
	data.GetLaplacianSF() = ScalarField(0, 0, 5000, 5000, im.width(), im.height(), "LaplacianField");
	data.GetDrainageSF()  = ScalarField(0, 0, 5000, 5000, im.width(), im.height(), "Drainage");
	
	ScalarField& HF = data.GetHeightSF();
	ScalarField& SlopeField = data.GetSlopeSF();
	ScalarField& LaplacianField = data.GetLaplacianSF();

	HF.dy = 1500.0;
	for (int i = 0; i < HF.nx ; i++)
	{
		for (int j = 0; j < HF.nz; j++)
		{
			double elevation = (im(i, j).r/255.0) * 100.0 * HF.dy;
			HF.at(i, j) = elevation;

			double slope = HF.Slope(i, j);

			double laplacian = HF.Laplacian(i, j);

			SlopeField.at(i, j) =  slope;
			LaplacianField.at(i, j) = laplacian;
		}
	}
	ComputeDrainage();

	HF.UpdateMinMaxValues();
	SlopeField.UpdateMinMaxValues();
	LaplacianField.UpdateMinMaxValues();
}

void Terrain::StreamPowerErosion(double k)
{
	// Trier les niveau d'eau selon Z décroissant
	ScalarField& Heightfield                  = data.GetHeightSF();
	ScalarField& SlopeField                   = data.GetSlopeSF();
	ScalarField& DrainageField                = data.GetDrainageSF();
	const std::vector<double>& elevations     = data.GetHeightSF().GetArray();
	const std::vector<double>& arrayDrainage  = DrainageField.GetArray();

	std::vector<std::pair<Vector, double>> cells(elevations.size()); // {position, elevation}, water level
	assert(elevations.size() == arrayDrainage.size());
	for (int i = 0; i < Heightfield.nx; i++)
	{
		for (int j = 0; j < Heightfield.nz; j++)
		{
			int index1D = Heightfield.index(i, j);
			cells.at(index1D) = std::make_pair(Vector(i, Heightfield.at(i, j), j), 1.0);
		}
	}

	// sort in water level in descending order of elevation
	std::sort(cells.begin(), cells.end(), [](std::pair<Vector, double> a, std::pair<Vector, double> b) -> bool { return a.first.y > b.first.y; });

	for (int i = 1; i < Heightfield.nx - 1; i++)
	{
		for (int j = 1; j < Heightfield.nz - 1; j++)
		{
			int index1D = Heightfield.index(i, j);

			const std::pair<Vector, double>& cell = cells.at(index1D);

			cells.at(Heightfield.index(i,     j - 1)).second += cell.second * arrayDrainage.at(index1D);
			cells.at(Heightfield.index(i,     j + 1)).second += cell.second * arrayDrainage.at(index1D);
			cells.at(Heightfield.index(i - 1, j)).second     += cell.second * arrayDrainage.at(index1D);
			cells.at(Heightfield.index(i + 1, j)).second     += cell.second * arrayDrainage.at(index1D);
		}
	}

	//// Update elevation
	for (int i = 0; i < Heightfield.nx; i++)
	{
		for (int j = 0; j < Heightfield.nz; j++)
		{
			SlopeField.at(i, j) = Heightfield.Slope(i, j);
			//double k = 0.01;
			double drainage = pow(DrainageField.at(i, j), 0.5);
			double dzdt = SlopeField.Normalize(SlopeField.at(i, j)) * drainage;

			Heightfield.at(i, j) = Heightfield.at(i, j) - (k * dzdt);
		}
	}
	// Update slopes

	//printf("Slope Min/Max : %f / %f ", (SlopeField.minValue), (SlopeField.maxValue));
}

void Terrain::HillSlopeErosion(double k)
{
	ScalarField& Heightfield    = data.GetHeightSF();
	ScalarField& LaplacianField = data.GetLaplacianSF();
	ScalarField& SlopeField     = data.GetSlopeSF();

	// Update elevation based on laplacian value
	for (int i = 0; i < Heightfield.nx; i++)
	{
		for (int j = 0; j < Heightfield.nz; j++)
		{
			double laplacianNormalized = LaplacianField.Normalize(LaplacianField.at(i, j));
			Heightfield.at(i, j) += (k * laplacianNormalized); // on retire de la matiere mais le laplacien peut être négatif
			LaplacianField.at(i, j) = Heightfield.Laplacian(i, j);
		}
	}

}

double ScalarField::Slope(int i, int j) const
{
	return length(Gradient(i, j));
}

Vector ScalarField::Normal(int i, int j) const
{
	Vector g = Gradient(i, j);
	
	return normalize(Vector(-g.x , 1.0, -g.z));
}

Vector ScalarField::Gradient(int i, int j) const
{
	double dFdx = 0.0;
	double dFdz = 0.0;

	//// difference de dénivelé en X
	//if(i > 0 && i < (nx - 1) && (j > 0 && j < (nz - 1)))
	//{
	//	dFdx = (values.at(index(i + 1, j)) - values.at(index(i - 1, j))) / (2.0 * dx);
	//	dFdz = (values.at(index(i, j + 1)) - values.at(index(i, j - 1))) / (2.0 * dz); // difference de dénivelé sur Z

	//	return Vector(dFdx, 0.0, dFdz);
	//}

	if	(i == 0)          { dFdx = (values.at(index(i + 1, j)) - values.at(index(0, j))) / dx; }
	else if (i == nx - 1) { dFdx = (values.at(index(i, j)) - values.at(index(i - 1, j))) / dx; }
	else                  { dFdx = (values.at(index(i + 1, j)) - values.at(index(i - 1, j))) / (2.0 * dx); }

	if (j == 0)           { dFdz = (values.at(index(i, j + 1)) - values.at(index(i, 0))) / dz; }
	else if (j == nz - 1) { dFdz = (values.at(index(i, j)) - values.at(index(i, j - 1))) / dz; }
	else                  { dFdz = (values.at(index(i, j + 1)) - values.at(index(i, j - 1))) / (2.0 * dz); }
		
	return Vector(dFdx, 0.0, dFdz);
}

double ScalarField::Laplacian(int i, int j) const
{
	double a = 0.0, b = 0.0;
	//if (i > 0 && i < nx - 1)
	//{
	//	a = (((values.at(index(i + 1, j)) - 2.0 * values.at(index(i, j)) + values.at(index(i - 1, j))))) / (dx * dx);
	//}

	//if (j > 0 && j < nz - 1)
	//{
	//	b = ((values.at(index(i, j + 1)) - 2.0 * values.at(index(i, j)) + values.at(index(i, j - 1))) ) / (dz * dz);
	//}

	if (i == 0)           { a = (((values.at(index(i + 1, j)) + values.at(index(0, j)))) - 2.0 * values.at(index(i, j))) / dx; }
	else if (i == nx - 1) { a = (((values.at(index(i, j)) + values.at(index(i - 1, j)))) - 2.0 * values.at(index(i, j))) / dx; }
	else                  { a = (((values.at(index(i + 1, j)) + values.at(index(i - 1, j)))) - 2.0 * values.at(index(i, j))) / (dx * dx); }

	if (j == 0)           { b = ((values.at(index(i, j + 1)) + values.at(index(i, 0))) - 2.0 * values.at(index(i, j))) / dz; }
	else if (j == nz - 1) { b = ((values.at(index(i, j)) + values.at(index(i, j - 1))) - 2.0 * values.at(index(i, j))) / dz; }
	else                  { b = ((values.at(index(i, j + 1)) + values.at(index(i, j - 1))) - 2.0 * values.at(index(i, j))) / (dz * dz); }

	return a + b;
}



// SF2::Normalize : hMin, hMax m = max(abs(hMin), abs(hMax)) y = x/m (relocalisation linéaire)
// 
// [a,  b]
// [0,  1] : 
// [-1, 1] : 
// deltaH -> [-1, 1] : K=5, on enlève 5 (5*-1) dans le pire des cas ou on rajoute 5 : K = coeff de contrôle 
// intervalle [0, 1] ou [-1, 1] (pour le laplacien)
// Hi+1 = Hi - k*deltaHNormalisé (laplacien normalisé entre -1 et 1)

// pente : 1 ~ 45 deg 
// delta T : 1 cm / 100 ans
 
//dzdt = -k*Laplacien(z)

void Terrain::Polygonize()
{
	mesh.release();
	mesh = Mesh(GL_TRIANGLE_STRIP);

	double dx = data.GetHeightSF().dx;
	double dy = data.GetHeightSF().dy;
	double dz = data.GetHeightSF().dz;

	ScalarField& HF			    = data.GetHeightSF();
	ScalarField& LaplacianField = data.GetLaplacianSF();
	ScalarField& SlopeField	    = data.GetSlopeSF();
	ScalarField& Drainage	    = data.GetDrainageSF();
	
	for (int i = 0; i < data.GetHeightSF().nx - 1; i++)
	{
		for (int j = 0; j < data.GetHeightSF().nz - 1; j++)
		{
			int x = i ;
			int z = j ;
			int xsuiv = (i + 1);
			int zsuiv = (j + 1);
			
			//Vector n1 = HF.Normal(xsuiv, z);
			//double l1 = LaplacianField.Normalize(LaplacianField.at(xsuiv, z));
			//double d1 = Drainage.at(xsuiv, z);
			Vector light(-1, 0, 1);

			double hs1 = 0.5 + 0.25 * (dot(normalize(HF.Gradient(xsuiv, z)), light) / 2.0) + 0.15 * (LaplacianField.Normalize(-LaplacianField.at(xsuiv, z))); // hillshade
			
			//mesh.color(Color(n1.x, n1.y, n1.z));


			//Vector n2 = HF.Normal(x, z);
			//double s2 = SlopeField.Normalize( SlopeField.at(x, z) );
			//double d2 = Drainage.at(x, z);
			double hs2 = 0.5 + 0.25 * (dot(normalize(HF.Gradient(x, z)), light) / 2.0) + 0.15 * (LaplacianField.Normalize(-LaplacianField.at(x, z))); // hillshade

			//SlopeField.at(i, j) = HF.Slope(i, j);
			//LaplacianField.at(i, j) = HF.Laplacian(i, j);
			//mesh.color(Color(n2.x , n2.y, n2.z));

			const Vector& n1 = HF.Normal(xsuiv, z);
			double s1 = SlopeField.at(xsuiv, z);
			//double d1 = Drainage.at(xsuiv, z);
			//double l1 = LaplacianField.at(xsuiv, z);
			//mesh.color(Util::ColorMapValue(s1, SlopeField.minValue, SlopeField.maxValue));
			mesh.color(Color(hs1, hs1, hs1));
			mesh.vertex(Point(xsuiv * dx, HF.at(xsuiv, z), z * dz));

			const Vector& n2 = HF.Normal(x, z);
			double s2 = SlopeField.at(x, z);
			//double d2 = Drainage.at(x, z);
			//double l2 = LaplacianField.at(x, z);
			//mesh.color(Util::ColorMapValue(s2, SlopeField.minValue, SlopeField.maxValue));
			mesh.color(Color(hs2, hs2, hs2));
			mesh.vertex(Point(x * dx, HF.at(x, z) , z * dz));

		}

		mesh.restart_strip();
	}

}

void Terrain::ComputeDrainage()
{
	// Pondération / répartition
	// Look-up 4 neighbors : 
	
	int offset = 1; // 

	ScalarField& Drainage = data.GetDrainageSF();

	ScalarField& SlopeField  = data.GetSlopeSF();
	ScalarField& Heightfield = data.GetHeightSF();
	for (int i = offset; i < data.GetHeightSF().nx - offset; i++)
	{
		for (int j = offset; j < data.GetHeightSF().nz - offset; j++)
		{
			SlopeField.at(i, j) = Heightfield.Slope(i, j);

			//double sumSlopes = SlopeField.at(i + 1, j) + 
			//				   SlopeField.at(i - 1, j) + 
			//	               SlopeField.at(i, j + 1) + 
			//	               SlopeField.at(i, j - 1);
			//
			//if (sumSlopes != 0.0)
			//{
			//	Drainage.at(i + 1, j) = SlopeField.at(i + 1, j) / sumSlopes;
			//	Drainage.at(i - 1, j) = SlopeField.at(i - 1, j) / sumSlopes;
			//	Drainage.at(i, j + 1) = SlopeField.at(i, j + 1) / sumSlopes;
			//	Drainage.at(i, j - 1) = SlopeField.at(i, j - 1) / sumSlopes;
			//}



			// STEEPEST
			std::array<const double, 8> neighborsSlopes = { SlopeField.at(i + 1, j), SlopeField.at(i + 1, j),  SlopeField.at(i, j + 1) ,  SlopeField.at(i, j - 1), SlopeField.at(i + 1, j + 1), SlopeField.at(i + 1, j - 1), SlopeField.at(i - 1, j + 1), SlopeField.at(i - 1, j - 1) };
			std::array<double*, 8> neighborsDrainage = { &Drainage.at(i + 1, j), &Drainage.at(i + 1, j),  &Drainage.at(i, j + 1) ,  &Drainage.at(i, j - 1), &Drainage.at(i + 1, j + 1), &Drainage.at(i + 1, j - 1), &Drainage.at(i - 1, j + 1), &Drainage.at(i - 1, j - 1) };

			int d = std::distance(neighborsSlopes.begin(), std::max_element(neighborsSlopes.begin(), neighborsSlopes.end()));

			*neighborsDrainage[d] = 1.0;
		}
	}
}

double ScalarField::Normalize(double value) const
{
	double val = value / m;

	return value / m;
}

void ScalarField::UpdateMinMaxValues(double value)
{
	minValue = std::min(minValue, value);
	maxValue = std::max(maxValue, value);

	m = std::max(abs(minValue), abs(maxValue));
}

void ScalarField::UpdateMinMaxValues()
{
	minValue = *std::min_element(values.begin(), values.end());
	maxValue = *std::max_element(values.begin(), values.end());

	m = std::max(abs(minValue), abs(maxValue));
	//m = maxValue;
}

void ScalarField::UpdateLaplacians()
{
	for (int i = 0; i < nx; i++)
	{
		for (int j = 0; j < nz; j++)
		{
			at(i, j) = Laplacian(i, j);
		}
	}

	UpdateMinMaxValues();
}

void ScalarField::UpdateSlopes()
{
	for (int i = 0; i < nx; i++)
	{
		for (int j = 0; j < nz; j++)
		{
			at(i, j) = Slope(i, j);
		}
	}

	UpdateMinMaxValues();
}

void ScalarField::ExportTexture(const std::string& filename) const
{
	Image outTexture(nx, nz);

	for (int i = 0; i < nx; i++)
	{
		for (int j = 0; j < nz; j++)
		{
			double value = Normalize(this->at(i, j));
			outTexture(i, j) = Color(value);
		}
	}

	std::stringstream ss;
	ss << "TerrainOutput/Texture" << filename.c_str() << nx << "x" << nz << ".png";
	write_image(outTexture, ss.str().c_str());
}