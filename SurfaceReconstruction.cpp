// System includes
#define NOMINMAX
#include "SurfaceReconstruction.h"
#pragma once
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h> 
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>

CSurfaceReconstruction::CSurfaceReconstruction(){

}
CSurfaceReconstruction::~CSurfaceReconstruction(){

}

HRESULT CSurfaceReconstruction::Poisson(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PolygonMesh *mesh){
	HRESULT hr = S_OK;

	/*for (size_t i = 0; i < normals->size(); ++i)
	{
		normals->points[i].normal_x *= -1;
		normals->points[i].normal_y *= -1;
		normals->points[i].normal_z *= -1;
	}*/

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointNormal>());
	concatenateFields(*pointcloud, *normals, *cloud_smoothed_normals);

	pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setDepth(9);
	poisson.setInputCloud(cloud_smoothed_normals);
	poisson.reconstruct(*mesh);

	return hr;
}


/// <summary>
/// Surface Reconstruction by Greedy Triangulation
/// </summary>
HRESULT CSurfaceReconstruction::GreedyProjectionTriangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PolygonMesh *triangles){
	
	HRESULT hr = S_OK;
	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*pointcloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	//// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	//// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	

	//// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);

	// Maximum acceptable distance for a point to be considered,
	// relative to the distance of the nearest point.
	gp3.setMu(2.5);
	// How many neighbors are searched for.
	gp3.setMaximumNearestNeighbors(100);
	
	// Points will not be connected to the current point
	// if their normals deviate more than the specified angle.
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	
	// Minimum and maximum angle there can be in a triangle.
	// The first is not guaranteed, the second is.
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	
	// If false, the direction of normals will not be taken into account
	// when computing the angle between them.
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(*triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	//pcl::io::saveVTKFile("mesh.vtk", triangles);
	return hr;

}

/// <summary>
/// Point Cloud Normal Estimation
/// </summary>
HRESULT CSurfaceReconstruction::NormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, pcl::PointCloud<pcl::Normal>::Ptr normals){
	HRESULT hr = S_OK;
	// Object for normal estimation.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;

	normalEstimation.setInputCloud(pointcloud);
	// For every point, use all neighbors in a radius of 3cm.
	normalEstimation.setRadiusSearch(0.03);
	// A kd-tree is a data structure that makes searches efficient. More about it later.

	// The normal estimation object will use it to find nearest neighbors.
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);

	// Calculate the normals.
	normalEstimation.compute(*normals);
	return hr;

}