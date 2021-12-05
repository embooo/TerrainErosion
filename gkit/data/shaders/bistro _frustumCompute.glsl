#version 330

#ifdef VERTEX_SHADER

layout(location= 0) in vec3 position;
layout(location= 1) in vec2 texcoord;
layout(location= 2) in vec3 normal;
layout(location= 4) in int  materialID;

uniform mat4 mvpMatrix;
uniform mat4 model;

out vec3 p;
out vec2 t;
out vec3 n;
out int matID;
out mat4 modelMat;

void main( )
{
    p = vec3(vec4(position, 1.0));
    n = vec3(normalize(model * vec4(normal, 0.0)));
    t = texcoord;
    matID = materialID;
    gl_Position= mvpMatrix * vec4(position, 1);

}
#endif


#ifdef FRAGMENT_SHADER

in vec3 p;
in vec2 t;
in vec3 n;
flat in int matID;
in mat4 modelMat;
out vec4 fragment_color;

// Textures
uniform sampler2DArray texture_array;

uniform int diffuse_layer;
uniform int emission_layer;
uniform int specular_layer;
uniform int ns_layer;

// Colors
uniform vec3 diffuse_color;
uniform vec3 emissive_color;
uniform vec3 specular_color;
uniform float ns;


// Position
uniform vec3 camPos;

// Constants
const float M_PI = 3.14;
uniform int numLights;

#define MAX_TEXTURES 256
#define MAX_MATERIALS 256
#define MAX_LIGHTS 256

struct Material
{
    int diffuse_texture;        //!< indice de la texture de la couleur de base, ou -1.
    int specular_texture;        //!< indice de la texture, ou -1.
    int emission_texture;        //!< indice de la texture, ou -1.
    int ns_texture;             //!< indice de la texture de reflet, ou -1.

    vec4 diffuse;              //!< couleur diffuse / de base.
    vec4 emission;             //!< pour une source de lumiere.
    vec4 specular;             //!< couleur du reflet.
//    float ns;                   //!< concentration des reflets, exposant pour les reflets blinn-phong.

};

struct PointLight
{
    vec4 pos;
    vec4 color;
};

layout (std140, binding = 0) uniform Materials
{
    Material material[MAX_MATERIALS];
} materialsData;

layout (std140, binding = 1) uniform Lights
{
    PointLight lights[MAX_LIGHTS];
} lightsData;

float rand(vec2 co){
    return fract(sin(dot(co, vec2(12.9898, 78.233))) * 43758.5453);
}

void main( )
{
    // Positions
    vec3 lightDir = normalize(vec3(0, -1, 0));

    // Directions
    vec3 nn = normalize(n);
    vec3 directionalLight  = normalize(-lightDir);
    vec3 directionalLightColor = (1.0/M_PI) * vec3(0.039, 0.06, 0.09) * dot(nn, directionalLight) ;
    
    vec3 c = normalize(modelMat * vec4(camPos, 1)).xyz;

    // Colors
    vec4 diffuse  = texture(texture_array, vec3(t, materialsData.material[matID].diffuse_texture));
    vec4 emission = texture(texture_array, vec3(t, materialsData.material[matID].emission_texture));
    vec4 specular = texture(texture_array, vec3(t, materialsData.material[matID].specular_texture));
    vec4 ns_      = texture(texture_array, vec3(t, ns_layer));


    vec4 diffuseColor       = materialsData.material[matID].diffuse;
    vec4 emissiveColor      = materialsData.material[matID].emission;
    vec4 specularColor      = materialsData.material[matID].specular;

    // Lambert
    vec3 color = vec3(0);
    for(int i = 0; i < numLights; i++)
    {
        vec3 lightPos   = vec3(lightsData.lights[i].pos) ;
        vec3 lightColor = vec3(lightsData.lights[i].color);

        vec3 l = normalize(lightPos - p);
        vec3 o = normalize(c - p);
        vec3 h = normalize(o+l);

        float dist = length(lightPos - p);
        float attenuation = 1.0 / (1.0 + 0.7 * dist + 1.8 * (dist * dist));  

        float cosTheta   = max(0, dot(nn, l));
        float cosTheta_h = max(0, dot(nn, h));

        float alpha = (2 / pow(specularColor.y, 2) - 2);
        float fr= (alpha+8) / (8*M_PI) * pow(cosTheta_h, alpha);

        color+= (lightColor * attenuation * 50.0) * fr * cosTheta ;  
    }

    // Setup lights
    fragment_color= vec4(directionalLightColor, 1) + vec4(color,1) * diffuse + emissiveColor;

}
#endif
