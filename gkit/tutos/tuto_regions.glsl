//! \file tuto_region.glsl

#version 330

#ifdef VERTEX_SHADER
layout(location= 0) in vec3 position;
layout(location= 2) in vec3 normal;

uniform mat4 mvpMatrix;
uniform mat4 mvMatrix;

out vec3 vertex_position;
out vec3 vertex_normal;

void main( )
{
    gl_Position= mvpMatrix * vec4(position, 1);
    vertex_position= vec3(mvMatrix * vec4(position, 1));
    vertex_normal= mat3(mvMatrix) * normal;
}

#endif


#ifdef FRAGMENT_SHADER
in vec3 vertex_position;
in vec3 vertex_normal;

out vec4 fragment_color;

uniform vec4 color;

void main( )
{
    vec3 l= normalize(-vertex_position);        // la camera est la source de lumiere.
    vec3 n= normalize(vertex_normal);
    float cos_theta= abs(dot(n, l));
    fragment_color= color * cos_theta;
}

#endif
