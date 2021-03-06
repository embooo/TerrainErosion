
#include <cstdio>
#include <ctype.h>
#include <climits>

#include <map>
#include <algorithm>

// TODO remove:
#include <regex>

#include "wavefront.h"



/*! renvoie le chemin d'acces a un fichier. le chemin est toujours termine par /
    pathname("path\to\file") == "path/to/"
    pathname("path\to/file") == "path/to/"
    pathname("path/to/file") == "path/to/"
    pathname("file") == "./"
 */
static
std::string pathname( const std::string& filename )
{
    std::string path= filename;
#ifndef WIN32
    std::replace(path.begin(), path.end(), '\\', '/');   // linux, macos : remplace les \ par /.
    size_t slash = path.find_last_of( '/' );
    if(slash != std::string::npos)
        return path.substr(0, slash +1); // inclus le slash
    else
        return "./";
#else
    std::replace(path.begin(), path.end(), '/', '\\');   // windows : remplace les / par \.
    size_t slash = path.find_last_of( '\\' );
    if(slash != std::string::npos)
        return path.substr(0, slash +1); // inclus le slash
    else
        return ".\\";
#endif
}

static 
std::string normalize_filename( const std::string& filename )
{
    std::string path= filename;
#ifndef WIN32
    std::replace(path.begin(), path.end(), '\\', '/');   // linux, macos : remplace les \ par /.
#else
    std::replace(path.begin(), path.end(), '/', '\\');   // windows : remplace les / par \.
#endif

    return path;
}

Mesh read_mesh( const char *filename )
{
    FILE *in= fopen(filename, "rb");
    if(in == NULL)
    {
        printf("[error] loading mesh '%s'...\n", filename);
        return Mesh::error();
    }
    
    Mesh data(GL_TRIANGLES);
    
    printf("loading mesh '%s'...\n", filename);
    
    std::vector<vec3> positions;
    std::vector<vec2> texcoords;
    std::vector<vec3> normals;
    int material_id= -1;
    
    std::vector<int> idp;
    std::vector<int> idt;
    std::vector<int> idn;
    
    char tmp[1024];
    char line_buffer[1024];
    bool error= true;
    for(;;)
    {
        // charge une ligne du fichier
        if(fgets(line_buffer, sizeof(line_buffer), in) == NULL)
        {
            error= false;       // fin du fichier, pas d'erreur detectee
            break;
        }
        
        // force la fin de la ligne, au cas ou
        line_buffer[sizeof(line_buffer) -1]= 0;
        
        // saute les espaces en debut de ligne
        char *line= line_buffer;
        while(*line && isspace(*line))
            line++;
        
        if(line[0] == 'v')
        {
            float x, y, z;
            if(line[1] == ' ')          // position x y z
            {
                if(sscanf(line, "v %f %f %f", &x, &y, &z) != 3)
                    break;
                positions.push_back( vec3(x, y, z) );
            }
            else if(line[1] == 'n')     // normal x y z
            {
                if(sscanf(line, "vn %f %f %f", &x, &y, &z) != 3)
                    break;
                normals.push_back( vec3(x, y, z) );
            }
            else if(line[1] == 't')     // texcoord x y
            {
                if(sscanf(line, "vt %f %f", &x, &y) != 2)
                    break;
                texcoords.push_back( vec2(x, y) );
            }
        }
        
        else if(line[0] == 'f')         // triangle a b c, les sommets sont numerotes a partir de 1 ou de la fin du tableau (< 0)
        {
            idp.clear();
            idt.clear();
            idn.clear();
            
            int next;
            for(line= line +1; ; line= line + next)
            {
                idp.push_back(0); 
                idt.push_back(0); 
                idn.push_back(0);         // 0: invalid index
                
                // analyse les attributs du sommet : p/t/n ou p//n ou p/t ou p...
                next= 0;
                if(sscanf(line, " %d/%d/%d %n", &idp.back(), &idt.back(), &idn.back(), &next) == 3) 
                    continue;
                else if(sscanf(line, " %d/%d %n", &idp.back(), &idt.back(), &next) == 2)
                    continue;
                else if(sscanf(line, " %d//%d %n", &idp.back(), &idn.back(), &next) == 2)
                    continue;
                else if(sscanf(line, " %d %n", &idp.back(), &next) == 1)
                    continue;
                else if(next == 0)      // fin de ligne
                    break;
            }
            
            // force une matiere par defaut, si necessaire
            if(material_id == -1)
            {
                material_id= data.materials().default_material_index();
                printf("usemtl default\n");
            }
            
            data.material(material_id);
            
            // triangule la face
            for(int v= 2; v +1 < (int) idp.size(); v++)
            {
                int idv[3]= { 0, v -1, v };
                for(int i= 0; i < 3; i++)
                {
                    int k= idv[i];
                    int p= (idp[k] < 0) ? (int) positions.size() + idp[k] : idp[k] -1;
                    int t= (idt[k] < 0) ? (int) texcoords.size() + idt[k] : idt[k] -1;
                    int n= (idn[k] < 0) ? (int) normals.size()   + idn[k] : idn[k] -1;
                    
                    if(p < 0) break; // error
                    
                    // attribut du ieme sommet
                    if(t >= 0) data.texcoord(texcoords[t]);
                    if(n >= 0) data.normal(normals[n]);
                    data.vertex(positions[p]);
                }
            }
        }
        
        else if(line[0] == 'm')
        {
           if(sscanf(line, "mtllib %[^\r\n]", tmp) == 1)
           {
               Materials materials= read_materials( normalize_filename(pathname(filename) + tmp).c_str() );
               // enregistre les matieres dans le mesh
               data.materials(materials);
           }
        }
        
        else if(line[0] == 'u')
        {
           if(sscanf(line, "usemtl %[^\r\n]", tmp) == 1)
               material_id= data.materials().find(tmp);
        }
    }
    
    fclose(in);
    
    if(error)
        printf("[error] loading mesh '%s'...\n%s\n\n", filename, line_buffer);
    else
        printf("mesh '%s': %d positions %s %s\n", filename, int(data.positions().size()), data.has_texcoord() ? "texcoord" : "", data.has_normal() ? "normal" : "");
    
    return data;
}


//! representation de l'indexation complete d'un sommet
struct vertex
{
    int material;
    int position;
    int texcoord;
    int normal;
    
    vertex( ) : material(-1), position(-1), texcoord(-1), normal(-1) {}
    vertex( const int m, const int p, const int t, const int n ) : material(m), position(p), texcoord(t), normal(n) {}
    
    // comparaison lexicographique de 2 sommets / des indices de leurs attributs
    bool operator< ( const vertex& b ) const
    {
        if(material != b.material) return material < b.material;
        if(position != b.position) return position < b.position;
        if(texcoord != b.texcoord) return texcoord < b.texcoord;
        if(normal != b.normal) return normal < b.normal;
        return false;
    }
};

Mesh read_indexed_mesh(const char* filename)
{
    FILE* in = fopen(filename, "rb");
    if (in == NULL)
    {
        printf("[error] loading indexed mesh '%s'...\n", filename);
        return Mesh::error();
    }

    Mesh data(GL_TRIANGLES);

    printf("loading indexed mesh '%s'...\n", filename);

    std::vector<vec3> positions;
    std::vector<vec2> texcoords;
    std::vector<vec3> normals;
    int material_id = -1;

    std::vector<int> idp;
    std::vector<int> idt;
    std::vector<int> idn;

    std::map<vertex, int> remap;

    char tmp[1024];
    char line_buffer[1024];
    bool error = true;

    for (;;)
    {
        // charge une ligne du fichier
        if (fgets(line_buffer, sizeof(line_buffer), in) == NULL)
        {
            error = false;       // fin du fichier, pas d'erreur detectee
            break;
        }

        // force la fin de la ligne, au cas ou
        line_buffer[sizeof(line_buffer) - 1] = 0;

        // saute les espaces en debut de ligne
        char* line = line_buffer;
        while (*line && isspace(*line))
            line++;

        if (line[0] == 'v')
        {
            float x, y, z;
            if (line[1] == ' ')          // position x y z
            {
                if (sscanf(line, "v %f %f %f", &x, &y, &z) != 3)
                    break;
                positions.push_back(vec3(x, y, z));
            }
            else if (line[1] == 'n')     // normal x y z
            {
                if (sscanf(line, "vn %f %f %f", &x, &y, &z) != 3)
                    break;
                normals.push_back(vec3(x, y, z));
            }
            else if (line[1] == 't')     // texcoord x y
            {
                if (sscanf(line, "vt %f %f", &x, &y) != 2)
                    break;
                texcoords.push_back(vec2(x, y));
            }
        }

        else if (line[0] == 'f')         // triangle a b c, les sommets sont numerotes a partir de 1 ou de la fin du tableau (< 0)
        {
            idp.clear();
            idt.clear();
            idn.clear();

            int next;
            for (line = line + 1; ; line = line + next)
            {
                idp.push_back(0);
                idt.push_back(0);
                idn.push_back(0);         // 0: invalid index

                // analyse les attributs du sommet : p/t/n ou p//n ou p/t ou p...
                next = 0;
                if (sscanf(line, " %d/%d/%d %n", &idp.back(), &idt.back(), &idn.back(), &next) == 3)
                {
                    continue;
                }
                else if (sscanf(line, " %d/%d %n", &idp.back(), &idt.back(), &next) == 2)
                {
                    continue;
                }
                else if (sscanf(line, " %d//%d %n", &idp.back(), &idn.back(), &next) == 2)
                {
                    continue;
                }
                else if (sscanf(line, " %d %n", &idp.back(), &next) == 1)
                {
                    continue;
                }
                else if (next == 0)      // fin de ligne
                    break;
            }

            // force une matiere par defaut, si necessaire
            if (material_id == -1)
            {
                material_id = data.materials().default_material_index();
                printf("usemtl default\n");
            }

            data.material(material_id);

            // triangule la face
            for (int v = 2; v + 1 < (int)idp.size(); v++)
            {
                int idv[3] = { 0, v - 1, v };
                for (int i = 0; i < 3; i++)
                {
                    int k = idv[i];
                    // indices des attributs du sommet
                    int p = (idp[k] < 0) ? (int)positions.size() + idp[k] : idp[k] - 1;
                    int t = (idt[k] < 0) ? (int)texcoords.size() + idt[k] : idt[k] - 1;
                    int n = (idn[k] < 0) ? (int)normals.size() + idn[k] : idn[k] - 1;

                    if (p < 0) break; // error

                    // recherche / insere le sommet 
                    auto found = remap.insert(std::make_pair(vertex(material_id, p, t, n), remap.size()));
                    if (found.second)
                    {
                        // pas trouve, copie les nouveaux attributs
                        if (t != -1) data.texcoord(texcoords[t]);
                        if (n != -1) data.normal(normals[n]);
                        data.vertex(positions[p]);
                    }

                    // construit l'index buffer
                    data.index(found.first->second);
                }
            }
        }

        else if (line[0] == 'm')
        {
            if (sscanf(line, "mtllib %[^\r\n]", tmp) == 1)
            {
                Materials materials = read_materials(normalize_filename(pathname(filename) + tmp).c_str());
                // enregistre les matieres dans le mesh
                data.materials(materials);
            }
        }

        else if (line[0] == 'u')
        {
            if (sscanf(line, "usemtl %[^\r\n]", tmp) == 1)
                material_id = data.materials().find(tmp);
        }
    }

    fclose(in);

    if (error)
        printf("[error] loading indexed mesh '%s'...\n%s\n\n", filename, line_buffer);
    else
        printf("  %d indices, %d positions %d texcoords %d normals\n",
            int(data.indices().size()), int(data.positions().size()), int(data.texcoords().size()), int(data.normals().size()));

    return data;
}

Mesh read_indexed_mesh( const char *filename, std::vector<vec4>& posLights, std::vector<vec4>& posLightBulbs)
{
    FILE *in= fopen(filename, "rb");
    if(in == NULL)
    {
        printf("[error] loading indexed mesh '%s'...\n", filename);
        return Mesh::error();
    }
    
    Mesh data(GL_TRIANGLES);
    
    printf("loading indexed mesh '%s'...\n", filename);
    
    std::vector<vec3> positions;
    std::vector<vec2> texcoords;
    std::vector<vec3> normals;
    int material_id= -1;
    
    std::vector<int> idp;
    std::vector<int> idt;
    std::vector<int> idn;
    
    std::map<vertex, int> remap;
    
    char tmp[1024];
    char line_buffer[1024];
    bool error= true;


    // TODO remove:
    bool foundALight = false;
    vec3 lightCenter = vec3(0,0,0);
    int nbVerticesLight = 0;

    // Light bulbs
    vec3 lightBulbCenter = vec3(0, 0, 0);
    bool foundALightBulb = false;
    int nbVerticesLightBulb = 0;
    
    for(;;)
    {
        // charge une ligne du fichier
        if(fgets(line_buffer, sizeof(line_buffer), in) == NULL)
        {
            error= false;       // fin du fichier, pas d'erreur detectee
            break;
        }
        
        // force la fin de la ligne, au cas ou
        line_buffer[sizeof(line_buffer) -1]= 0;
        
        // saute les espaces en debut de ligne
        char *line= line_buffer;
        while(*line && isspace(*line))
            line++;

        if (line[0] == 'o')
        {
            if (foundALight)
            {
                lightCenter.x /= nbVerticesLight;
                lightCenter.y /= nbVerticesLight;
                lightCenter.z /= nbVerticesLight;

                posLights.push_back(vec4(lightCenter, 1));
                //printf("Light located at : %f, %f, %f \n", lightCenter.x, lightCenter.y, lightCenter.z);

                lightCenter = vec3(0, 0, 0);
                nbVerticesLight = 0;
                foundALight = false;
            }
            if (strstr(line, "Glass") != NULL && !foundALight) {
                foundALight = true;
            }

        }

        if(line[0] == 'v')
        {
            float x, y, z;
            if(line[1] == ' ')          // position x y z
            {
                if(sscanf(line, "v %f %f %f", &x, &y, &z) != 3)
                    break;
                positions.push_back( vec3(x, y, z) );
                // TODO Remove:
                if (foundALight)
                {
                    nbVerticesLight++;
                    lightCenter.x += x;
                    lightCenter.y += y;
                    lightCenter.z += z;
                }
                
            }
            else if(line[1] == 'n')     // normal x y z
            {
                if(sscanf(line, "vn %f %f %f", &x, &y, &z) != 3)
                    break;
                normals.push_back( vec3(x, y, z) );
            }
            else if(line[1] == 't')     // texcoord x y
            {
                if(sscanf(line, "vt %f %f", &x, &y) != 2)
                    break;
                texcoords.push_back( vec2(x, y) );
            }
        }
        
        else if(line[0] == 'f')         // triangle a b c, les sommets sont numerotes a partir de 1 ou de la fin du tableau (< 0)
        {
            idp.clear();
            idt.clear();
            idn.clear();
            
            int next;
            for(line= line +1; ; line= line + next)
            {
                idp.push_back(0); 
                idt.push_back(0); 
                idn.push_back(0);         // 0: invalid index
                
                // analyse les attributs du sommet : p/t/n ou p//n ou p/t ou p...
                next= 0;
                if (sscanf(line, " %d/%d/%d %n", &idp.back(), &idt.back(), &idn.back(), &next) == 3)
                {
                    // TODO Remove:
                    if (foundALightBulb)
                    {
                        nbVerticesLightBulb+=1;

						lightBulbCenter.x += positions.at(idp.back() - 1).x;
						lightBulbCenter.y += positions.at(idp.back() - 1).y;
						lightBulbCenter.z += positions.at(idp.back() - 1).z;
                    }
                    continue;
                }
                else if(sscanf(line, " %d/%d %n", &idp.back(), &idt.back(), &next) == 2)
                {
                    // TODO Remove:
                    if (foundALightBulb)
                    {
                        nbVerticesLightBulb += 1;

                        lightBulbCenter.x += positions.at(idp.back() - 1).x;
                        lightBulbCenter.y += positions.at(idp.back() - 1).y;
                        lightBulbCenter.z += positions.at(idp.back() - 1).z;
                    }
                    continue;
                }
                else if(sscanf(line, " %d//%d %n", &idp.back(), &idn.back(), &next) == 2)
                {
                    // TODO Remove:
                    if (foundALightBulb)
                    {
                        nbVerticesLightBulb += 1;

                        lightBulbCenter.x += positions.at(idp.back() - 1).x;
                        lightBulbCenter.y += positions.at(idp.back() - 1).y;
                        lightBulbCenter.z += positions.at(idp.back() - 1).z;
                    }
                    continue;
                }
                else if(sscanf(line, " %d %n", &idp.back(), &next) == 1)
                {
                    // TODO Remove:
                    if (foundALightBulb)
                    {
                        nbVerticesLightBulb += 1;

                        lightBulbCenter.x += positions.at(idp.back() - 1).x;
                        lightBulbCenter.y += positions.at(idp.back() - 1).y;
                        lightBulbCenter.z += positions.at(idp.back() - 1).z;
                    }
                    continue;
                }
                else if(next == 0)      // fin de ligne
                    break;
            }
            
            // force une matiere par defaut, si necessaire
            if(material_id == -1)
            {
                material_id= data.materials().default_material_index();
                printf("usemtl default\n");
            }
            
            data.material(material_id);
            
            // triangule la face
            for(int v= 2; v +1 < (int) idp.size(); v++)
            {
                int idv[3]= { 0, v -1, v };
                for(int i= 0; i < 3; i++)
                {
                    int k= idv[i];
                    // indices des attributs du sommet
                    int p= (idp[k] < 0) ? (int) positions.size() + idp[k] : idp[k] -1;
                    int t= (idt[k] < 0) ? (int) texcoords.size() + idt[k] : idt[k] -1;
                    int n= (idn[k] < 0) ? (int) normals.size()   + idn[k] : idn[k] -1;
                    
                    if(p < 0) break; // error
                    
                    // recherche / insere le sommet 
                    auto found= remap.insert( std::make_pair(vertex(material_id, p, t, n), remap.size()) );
                    if(found.second)
                    {
                        // pas trouve, copie les nouveaux attributs
                        if(t != -1) data.texcoord(texcoords[t]);
                        if(n != -1) data.normal(normals[n]);
                        data.vertex(positions[p]);
                    }
                    
                    // construit l'index buffer
                    data.index(found.first->second);
                }
            }
        }
        
        else if(line[0] == 'm')
        {
           if(sscanf(line, "mtllib %[^\r\n]", tmp) == 1)
           {
               Materials materials= read_materials( normalize_filename(pathname(filename) + tmp).c_str() );
               // enregistre les matieres dans le mesh
               data.materials(materials);
           }
        }
        
        else if(line[0] == 'u')
        {
           if(sscanf(line, "usemtl %[^\r\n]", tmp) == 1)
               material_id= data.materials().find(tmp);

           // TODO Remove:
           if (foundALightBulb)
           {
               lightBulbCenter.x /= nbVerticesLightBulb;
               lightBulbCenter.y /= nbVerticesLightBulb;
               lightBulbCenter.z /= nbVerticesLightBulb;

               posLightBulbs.push_back(vec4(lightBulbCenter, 1));
               //printf("Light bulb located at : %f, %f, %f \n", lightBulbCenter.x, lightBulbCenter.y, lightBulbCenter.z);

               lightBulbCenter = vec3(0, 0, 0);
               nbVerticesLightBulb = 0;
               foundALightBulb = false;
           }

           if (strstr(line, "Emissive") != NULL && !foundALightBulb) {
               foundALightBulb = true;
           }
 
        }
    }
    
    fclose(in);
    
    if(error)
        printf("[error] loading indexed mesh '%s'...\n%s\n\n", filename, line_buffer);
    else
        printf("  %d indices, %d positions %d texcoords %d normals\n", 
            int(data.indices().size()), int(data.positions().size()), int(data.texcoords().size()), int(data.normals().size()));
    
    return data;
}



int write_mesh( const Mesh& mesh, const char *filename )
{
    if(mesh == Mesh::error())
        return -1;

    if(mesh.primitives() != GL_TRIANGLES)
        return -1;
    if(mesh.positions().size() == 0)
        return -1;
    if(filename == NULL)
        return -1;
    
    FILE *out= fopen(filename, "wt");
    if(out == NULL)
        return -1;
    
    printf("writing mesh '%s'...\n", filename);
    
    const std::vector<vec3>& positions= mesh.positions();
    for(unsigned i= 0; i < positions.size(); i++)
        fprintf(out, "v %f %f %f\n", positions[i].x, positions[i].y, positions[i].z);
    fprintf(out, "\n");
    
    const std::vector<vec2>& texcoords= mesh.texcoords();
    bool has_texcoords= (texcoords.size() == positions.size());
    for(unsigned i= 0; i < texcoords.size(); i++)
        fprintf(out, "vt %f %f\n", texcoords[i].x, texcoords[i].y);
    fprintf(out, "\n");
    
    const std::vector<vec3>& normals= mesh.normals();
    bool has_normals= (normals.size() == positions.size());
    for(unsigned i= 0; i < normals.size(); i++)
        fprintf(out, "vn %f %f %f\n", normals[i].x, normals[i].y, normals[i].z);
    fprintf(out, "\n");
    
    int material_id= -1;
    const std::vector<unsigned>& materials= mesh.material_indices();
    
    const std::vector<unsigned>& indices= mesh.indices();
    bool has_indices= (indices.size() > 0);
    unsigned n= has_indices ? indices.size() : positions.size();
    for(unsigned i= 0; i +2 < n; i+= 3)
    {
        if(material_id != int(materials[i/3]))
        {
            material_id= int(materials[i/3]);
            if(material_id != -1)
                fprintf(out, "usemtl %s\n", mesh.materials().name(material_id));
        }
        
        fprintf(out, "f");
        for(unsigned k= 0; k < 3; k++)
        {
            unsigned id= has_indices ? indices[i+k] +1 : i+k +1;
            fprintf(out, " %u", id);
            if(has_texcoords && has_normals)
                fprintf(out, "/%u/%u", id, id);
            else if(has_texcoords)
                fprintf(out, "/%u", id);
            else if(has_normals)
                fprintf(out, "//%u", id);
        }
        fprintf(out, "\n");
    }
    
    fclose(out);
    return 0;
}


Materials read_materials( const char *filename )
{
    Materials materials;
    
    FILE *in= fopen(filename, "rt");
    if(in == NULL)
    {
        printf("[error] loading materials '%s'...\n", filename);
        return materials;
    }
    
    printf("loading materials '%s'...\n", filename);
    
    Material *material= NULL;
    char tmp[1024];
    char line_buffer[1024];
    bool error= true;
    for(;;)
    {
        // charge une ligne du fichier
        if(fgets(line_buffer, sizeof(line_buffer), in) == NULL)
        {
            error= false;       // fin du fichier, pas d'erreur detectee
            break;
        }
        
        // force la fin de la ligne, au cas ou
        line_buffer[sizeof(line_buffer) -1]= 0;
        
        // saute les espaces en debut de ligne
        char *line= line_buffer;
        while(*line && isspace(*line))
            line++;
        
        if(line[0] == 'n')
        {
            if(sscanf(line, "newmtl %[^\r\n]", tmp) == 1)
            {
                int id= materials.insert(Material(Black()), tmp);
                material= &materials.material(id);
            }
        }
        
        if(material == nullptr)
            continue;
        
        if(line[0] == 'K')
        {
            float r, g, b;
            if(sscanf(line, "Kd %f %f %f", &r, &g, &b) == 3)
                material->diffuse= Color(r, g, b);
            else if(sscanf(line, "Ks %f %f %f", &r, &g, &b) == 3)
                material->specular= Color(r, g, b);
            else if(sscanf(line, "Ke %f %f %f", &r, &g, &b) == 3)
                material->emission= Color(r, g, b);
        }
        
        else if(line[0] == 'N')
        {
            float n;
            if(sscanf(line, "Ns %f", &n) == 1)          // Ns, puissance / concentration du reflet, modele blinn phong
                material->ns= n;
        }
        
        else if(line[0] == 'm')
        {
            if(sscanf(line, "map_Kd %[^\r\n]", tmp) == 1)
                material->diffuse_texture= materials.insert_texture( normalize_filename(pathname(filename) + tmp).c_str() );
                
            else if(sscanf(line, "map_Ks %[^\r\n]", tmp) == 1)
                material->specular_texture= materials.insert_texture( normalize_filename(pathname(filename) + tmp).c_str() );
                
            else if(sscanf(line, "map_Ke %[^\r\n]", tmp) == 1)
                material->emission_texture= materials.insert_texture( normalize_filename(pathname(filename) + tmp).c_str() );
        }
        
    }
    
    fclose(in);
    if(error)
        printf("[error] parsing line :\n%s\n", line_buffer);
    
    return materials;
}
