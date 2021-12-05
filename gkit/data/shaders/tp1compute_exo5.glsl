#version 430

#ifdef COMPUTE_SHADER

uniform int val;

// readonly, writeonly, ou rien pour les deux 
layout(std430, binding = 0) readonly buffer readBuffer
{
	int entree[];
};

layout(std430, binding = 1) writeonly buffer writeBuffer
{
	int sortie[];
};

layout(local_size_x = 256) in;
void main()
{
	const uint ID = gl_GlobalInvocationID.x;

	if(ID < sortie.length())
	{
		sortie[ID] = entree[ID] + val;
	}
}

#endif