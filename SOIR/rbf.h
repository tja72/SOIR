#pragma once
#include <iostream>

#include "Eigen.h"
#include "ImplicitSurface.h"
#include "Volume.h"
#include "MarchingCubes.h"
#include "PointCloud.h"
#include "SimpleMesh2.h"


int applyRBF_MarchingCubes(PointCloud2 m_pointcloud) {

	ImplicitSurface* surface;
	surface = new RBF(m_pointcloud);

	// fill volume with signed distance values
	unsigned int mc_res = 50; // resolution of the grid, for debugging you can reduce the resolution (-> faster)
	Volume vol(Vector3d(-0.1, -0.1, -0.1), Vector3d(1.1, 1.1, 1.1), mc_res, mc_res, mc_res, 1);
	for (unsigned int x = 0; x < vol.getDimX(); x++)
	{
		for (unsigned int y = 0; y < vol.getDimY(); y++)
		{
			for (unsigned int z = 0; z < vol.getDimZ(); z++)
			{
				Eigen::Vector3d p = vol.pos(x, y, z);
				double val = surface->Eval(p);
				vol.set(x, y, z, val);
			}
		}
	}

	// extract the zero iso-surface using marching cubes
	SimpleMesh2 mesh;
	for (unsigned int x = 0; x < vol.getDimX() - 1; x++)
	{
		std::cerr << "Marching Cubes on slice " << x << " of " << vol.getDimX() << std::endl;

		for (unsigned int y = 0; y < vol.getDimY() - 1; y++)
		{
			for (unsigned int z = 0; z < vol.getDimZ() - 1; z++)
			{
				ProcessVolumeCell(&vol, x, y, z, 0.00f, &mesh);
			}
		}
	}

	// write mesh to file
	//last step, so we should write out the mesh here :)
	//if (!mesh.WriteMesh(filenameOut))
	//{
	//	std::cout << "ERROR: unable to write output file!" << std::endl;
	//	return -1;
	//}

	delete surface;

	return 0;
}

