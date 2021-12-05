#version 430

#ifdef COMPUTE_SHADER

uniform int val;

shared uint lastPos;

// readonly, writeonly, ou rien pour les deux 
layout(std430, binding = 0) readonly buffer readBuffer
{
	int entree[];
};

layout(std430, binding = 1) writeonly buffer writeBuffer
{
	int sortie[];
};

layout( std430, binding= 2 ) buffer countData
{
	uint count;
};

uniform int v= 5;

shared uint group_count;
shared uint group_offset;

layout(local_size_x = 256) in;
void main()
{
	uint ID =	gl_GlobalInvocationID.x;
	if(gl_LocalInvocationID.x == 0)
	{
		group_count = 0; // Initialise le compteur local au groupe de threads
	}
	barrier(); // attendre que l'initialisation soit fait avant de commencer à écrire


	// evaluer la condition
	if(entree[ID] < v)
	{
		offset = atomicAdd(group_count, 1);
	}

	// 
	if(gl_LocalInvocationID.x == 0)
	{
		
	}

//	const uint ID      = gl_GlobalInvocationID.x;
//	const uint localID = gl_LocalInvocationID.x;
//
//	if(localID == 0)
//	{
//		lastPos = 0;
//	}
//
//	barrier();
//
//	if(test(ID)) // for (i=0; i < sortie.length(); i++)
//	{
//
//
//	}



}



//layout ( std430 , binding = 0 ) writeonly buffer outputData
//{
//	int output [];
//};
//
//layout ( std430 , binding = 1 ) buffer counterData
//{
//	int count;
//};
//
//layout ( local_size_x = 256) in;
//void main ()
//{
//	uint ID = gl_GlobalInvocationID.x;
//	if ( test ( ID ) )
//	{
//		// * doit * etre visible par tous les threads de tous les groupes
//		int index = atomicAdd ( count , 1) ;
//		output [ index ]= f( ID ) ;
//	}
//}


#endif