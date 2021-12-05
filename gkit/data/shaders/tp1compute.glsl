#version 430

#ifdef COMPUTE_SHADER

uniform int val;

// readonly, writeonly, ou rien pour les deux 
layout(std430, binding = 0) buffer myData
{
	int entree[];
};

layout(local_size_x = 256) in;
void main()
{
	const uint ID = gl_GlobalInvocationID.x;

	if(ID < entree.length())
	{
		entree[ID] = entree[ID] + val;
	}
}

#endif