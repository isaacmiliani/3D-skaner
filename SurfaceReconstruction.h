//------------------------------------------------------------------------------
// <copyright file="SurfaceReconstruction.cpp" company="PCL">
//     Surface Reconstruction implementation for Point Clouds
// http://docs.pointclouds.org/trunk/group__surface.html
// </copyright>
//------------------------------------------------------------------------------
#pragma once
#include "ImageRenderer.h"
#include <pcl/visualization/pcl_visualizer.h>

/// <summary>
/// Surface Reconstruction class.
/// </summary>
class CSurfaceReconstruction{

public:
	/// <summary>
	/// Constructor
	/// </summary>
	CSurfaceReconstruction();

	/// <summary>
	/// Destructor
	/// </summary>
	~CSurfaceReconstruction();

	/// <summary>
	/// Poisson Surface Reconstruction
	/// </summary>
	HRESULT Poisson(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PolygonMesh *mesh);

	/// <summary>
	/// Greedy Triangulation Reconstruction
	/// </summary>
	HRESULT GreedyProjectionTriangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PolygonMesh *triangles);

	/// <summary>
	/// Point Cloud Normal Estimation
	/// </summary>
	HRESULT NormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, pcl::PointCloud<pcl::Normal>::Ptr normals);
};
