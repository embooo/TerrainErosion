
#include "mat.h"
#include "mesh.h"
#include "wavefront.h"

#include "orbiter.h"
#include "program.h"
#include "uniforms.h"
#include "draw.h"

#include "app_camera.h"        // classe Application a deriver


Mesh make_cell( )
{
    glLineWidth(2);    
    Mesh cell= Mesh(GL_LINES);
    cell.default_color(Color(0.8, 0.4, 0));
    
    // face avant
    cell.vertex(-1, -1, -1);
    cell.vertex(-1, 1, -1);
    cell.vertex(-1, 1, -1);
    cell.vertex(1, 1, -1);
    cell.vertex(1, 1, -1);
    cell.vertex(1, -1, -1);
    cell.vertex(1, -1, -1);
    cell.vertex(-1, -1, -1);
    
    // face arriere
    cell.vertex(-1, -1, 1);
    cell.vertex(-1, 1, 1);
    cell.vertex(-1, 1, 1);
    cell.vertex(1, 1, 1);
    cell.vertex(1, 1, 1);
    cell.vertex(1, -1, 1);
    cell.vertex(1, -1, 1);
    cell.vertex(-1, -1, 1);
    
    // aretes
    cell.vertex(-1, -1, -1);
    cell.vertex(-1, -1, 1);
    cell.vertex(-1, 1, -1);
    cell.vertex(-1, 1, 1);
    cell.vertex(1, 1, -1);
    cell.vertex(1, 1, 1);
    cell.vertex(1, -1, -1);
    cell.vertex(1, -1, 1);
    
    return cell;
}

struct BBox
{
    Point pmin;
    Point pmax;
    
    BBox& insert( const Point& p ) { pmin= min(pmin, p); pmax= max(pmax, p); return *this; }
    BBox& insert( const BBox& box ) { pmin= min(pmin, box.pmin); pmax= max(pmax, box.pmax); return *this; }
};

BBox EmptyBox( ) { return { Point(FLT_MAX, FLT_MAX, FLT_MAX), Point(-FLT_MAX, -FLT_MAX, -FLT_MAX) }; }



class TP : public AppCamera
{
public:
    // constructeur : donner les dimensions de l'image, et eventuellement la version d'openGL.
    TP( ) : AppCamera(1024, 640) {}
    
    int init( )
    {
        //~ m_objet= read_indexed_mesh("data/robot.obj");
        m_objet= read_indexed_mesh("data/bistro-small/exterior.obj");
        if(m_objet.triangle_count() == 0)
            return -1;
        
        m_cube= make_cell();
        
        /* construit une grille qui englobe les objets / la scene
            et reparti les triangles dans les cellules de la grille
         */
        m_objet.bounds(m_scene_pmin, m_scene_pmax);
        Vector d= Vector(m_scene_pmin, m_scene_pmax);
        
        printf("scene bounds %f %f %f\n", d.x, d.y, d.z);
        printf("  min %f %f %f\n", m_scene_pmin.x, m_scene_pmin.y, m_scene_pmin.z);
        printf("  max %f %f %f\n", m_scene_pmax.x, m_scene_pmax.y, m_scene_pmax.z);
        
        // construit un cube a partir de l'englobant de la scene
        float length= std::max(d.x, std::max(d.y, d.z));        // axe le plus etire de l'englobant de la scene
        m_grid_pmin= m_scene_pmin;
        m_grid_pmax= m_grid_pmin + Vector(length, length, length);
        
        // affecte une cellule a chaque triangle
        unsigned grid_size= 16;
        std::vector<unsigned> cells;
        for(int i= 0; i < m_objet.triangle_count(); i++)
        {
            // centre du triangle
            const TriangleData& triangle= m_objet.triangle(i);
            Point c= Point(triangle.a) + Vector(triangle.a, triangle.b) / 3 + Vector(triangle.a, triangle.c) / 3;
            
            // coordonnees de la cellule contenant le centre du triangle
            unsigned idx= (c.x - m_grid_pmin.x) / length * grid_size;
            unsigned idy= (c.y - m_grid_pmin.y) / length * grid_size;
            unsigned idz= (c.z - m_grid_pmin.z) / length * grid_size;
            
            // identifiant 1d de la cellule
            unsigned id= idx + idy * grid_size + idz * grid_size * grid_size;
            cells.push_back(id);
        }
        
        // trie les triangles par cellule
        m_groups= m_objet.groups(cells);
        printf("%d regions.\n", int(m_groups.size()));
        
        // construit l'englobant des triangles de chaque groupe / associes a chaque cellule
        for(unsigned i= 0; i < m_groups.size(); i++)
        {
            // group.first et group.n  representent des indices ou des sommets, pas des triangles. par contre 1 triangle = 3 sommets ou 3 indices...
            unsigned group_size= m_groups[i].n / 3;
            
            // parcours les triangles du groupe
            BBox bounds= EmptyBox();
            for(unsigned k= 0; k < group_size; k++)
            {
                const TriangleData& triangle= m_objet.triangle(m_groups[i].first/3 + k);
                
                bounds.insert(triangle.a);
                bounds.insert(triangle.b);
                bounds.insert(triangle.c);
            }
            
            m_bounds.push_back(bounds);
        }
        
        // placer la camera
        Point pmin, pmax;
        m_objet.bounds(pmin, pmax);
        camera().lookat(pmin, pmax);
        
        // place le frustum de test
        m_visibility_model= Identity();
        m_visibility_view= Lookat(Point(0, 50, 0), Point(0, 0, 0), Vector(-1, 0, 0));
        m_visibility_projection= Perspective(45, 1, 4, 50);
        
        // creer le shader program, uniquement necessaire pour l'option 2, cf render()
        m_program= read_program("tutos/tuto_regions.glsl");
        program_print_errors(m_program);
        
        // etat openGL par defaut
        glClearColor(0.2f, 0.2f, 0.2f, 1.f);        // couleur par defaut de la fenetre
        
        glClearDepth(1.f);                          // profondeur par defaut
        glDepthFunc(GL_LESS);                       // ztest, conserver l'intersection la plus proche de la camera
        glEnable(GL_DEPTH_TEST);                    // activer le ztest
        
        return 0;   // ras, pas d'erreur
    }
    
    // destruction des objets de l'application
    int quit( )
    {
        // etape 3 : detruire le shader program
        release_program(m_program);
        m_objet.release();
        return 0;
    }
    
    // dessiner une nouvelle image
    int render( )
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        // transformations standards
        Transform model;
        Transform view= camera().view();
        Transform projection= camera().projection();
        
        // modifie la position du frustum de test en fonction des fleches de direction
        if(key_state(SDLK_UP))
            m_visibility_model= m_visibility_model * Translation(0, 0, 0.25);     // en avant
        if(key_state(SDLK_DOWN))
            m_visibility_model= m_visibility_model * Translation(0, 0, -0.25);      // en arriere
            
        if(key_state(SDLK_LEFT))
            m_visibility_model= m_visibility_model * Translation(0.25, 0, 0);     // a droite
        if(key_state(SDLK_RIGHT))
            m_visibility_model= m_visibility_model * Translation(-0.25, 0, 0);     // a gauche
        
        // transformations du frustum de test
        m_visibility_view= Lookat(Point(0, 50, 0), Point(0, 0, 0), Vector(-1, 0, 0));
        Transform visibility_mvp= m_visibility_projection * m_visibility_view * m_visibility_model;
        Transform visibility_mvpInv= Inverse(visibility_mvp);
        
        // dessiner le frustum 
        {
            m_cube.default_color(Red());
            draw(m_cube, visibility_mvpInv, camera());            
        }
        
        // dessiner les regions...
        {
            // configurer le pipeline 
            glUseProgram(m_program);
            
            // configurer le shader program
            Transform mv= view * model;
            Transform mvp= projection * mv;
            
            program_uniform(m_program, "mvpMatrix", mvp);
            program_uniform(m_program, "mvMatrix", mv);
            
            // afficher chaque groupe / region
            for(unsigned i= 0; i < m_groups.size(); i++)
            {
                Color color= Color(1 - float(m_groups[i].index % 10) / 9, float(m_groups[i].index % 100)/ 99, 1 - float(m_groups[i].index % 1000) / 999);
                program_uniform(m_program, "color", color);
                //~ program_uniform(m_program, "color", Color(0.7));
                
                // 1 draw par groupe de triangles...
                m_objet.draw(m_groups[i].first, m_groups[i].n, m_program, /* use position */ true, /* use texcoord */ false, /* use normal */ true, /* use color */ false, /* use material index*/ false);
            }
        }
        
        return 1;
    }

protected:
    Transform m_model;
    Mesh m_objet;
    Mesh m_cube;
    GLuint m_texture;
    GLuint m_program;
    std::vector<TriangleGroup> m_groups;
    std::vector<BBox> m_bounds;

    Transform m_visibility_model;
    Transform m_visibility_view;
    Transform m_visibility_projection;

    Point m_scene_pmin;
    Point m_scene_pmax;
    Point m_grid_pmin;
    Point m_grid_pmax;

};


int main( int argc, char **argv )
{
    TP tp;
    tp.run();
    
    return 0;
}
