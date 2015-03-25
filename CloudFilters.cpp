// System includes
#define NOMINMAX
#include "CloudFilters.h"
#pragma once
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h> // VoxelGrid
#include <pcl/filters/passthrough.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h> // StatisticalOutlierRemoval
#include <pcl/filters/bilateral.h>
#include <pcl/filters/fast_bilateral.h>

CCloudFilters::CCloudFilters(){

}
CCloudFilters::~CCloudFilters(){

}
HRESULT CCloudFilters::VoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	HRESULT hr = S_OK;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;

	sor.setInputCloud(cloud);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*cloud_filtered);

	*cloud = *cloud_filtered;
	return hr;
}

HRESULT CCloudFilters::StatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	
	HRESULT hr = S_OK;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.filter(*cloud_filtered);

	//sor.setNegative(true);
	sor.filter(*cloud_filtered);

	*cloud = *cloud_filtered;

	return hr;
}

HRESULT CCloudFilters::PassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	HRESULT hr = S_OK;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
	// Filter object.
	pcl::PassThrough<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	// Filter out all points with Z values not in the [0-2] range.
	filter.setFilterFieldName("y");
	filter.setFilterLimits(0.0, 5.0);

	filter.filter(*cloud_filtered);

	*cloud = *cloud_filtered;
	return hr;
}

HRESULT CCloudFilters::Bilateral(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	HRESULT hr = S_OK;
	
	float sigma_s = 4.5;
	float sigma_r = 0.03;
	int pnumber = (int)cloud->size();
	// Set up KDTree

	
	// Output Cloud = Input Cloud
	pcl::PointCloud<pcl::PointXYZ> filtered_cloud = *cloud;

	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	tree->setInputCloud(cloud);

	// Neighbors containers
	std::vector<int> k_indices;
	std::vector<float> k_distances;

	// Main Loop
	for (int point_id = 0; point_id < pnumber; ++point_id)
	{
		float BF = 0;
		float W = 0;

		tree->radiusSearch(point_id, 2 * sigma_s, k_indices, k_distances);

		// For each neighbor
		for (size_t n_id = 0; n_id < k_indices.size(); ++n_id)
		{
			float id = k_indices.at(n_id);
			float dist = sqrt(k_distances.at(n_id));
			float intensity_dist = abs(cloud->points[point_id].z - cloud->points[id].z);

			float w_a = exp(-(dist*dist) / (2 * sigma_s*sigma_s)); //G(dist, sigma_s);
			float w_b = exp(-(intensity_dist*intensity_dist) / (2 * sigma_r*sigma_r)); // G(intensity_dist, sigma_r);
			float weight = w_a * w_b;

			BF += weight * cloud->points[id].z;
			W += weight;
		}

		filtered_cloud.points[point_id].z = BF / W;
	}

	*cloud =  filtered_cloud;
	return hr;
}
/// <summary>
/// A Fast bilateral filter implementation for point cloud data
/// <summary>
HRESULT CCloudFilters::FastBilateral(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	HRESULT hr = S_OK;
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());

	//pcl::BilateralFilter<pcl::PointXYZ> bFilter;
	pcl::FastBilateralFilter<pcl::PointXYZ> bFastBilateral;

	bFastBilateral.setInputCloud(cloud);
	bFastBilateral.setSigmaR(0.015f);
	bFastBilateral.setSigmaS(0.0003f);
	bFastBilateral.applyFilter(*filtered_cloud);

	*cloud = *filtered_cloud;

	return hr;
}
/// <summary>
/// Moving Lest Square Surface Reconstruction from Point Cloud
/// </summary>
/// <param name="cloud">Pointer to input pointCloud</param>
/// <param name="cloud_smoothed">Pointer to output PointCloud.</param>
/// <returns>indicates success or failure</returns>
HRESULT CCloudFilters::MovingLeastSquares(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	HRESULT hr = S_OK;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZ>());
	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;

	// We can tell the algorithm to also compute smoothed normals (optional).
	mls.setComputeNormals(false);

	// Set parameters
	mls.setInputCloud(cloud);

	// If true, the surface and normal are approximated using a polynomial estimation
	// (if false, only a tangent one).
	mls.setPolynomialFit(true);

	// kd-tree object for performing searches.
	mls.setSearchMethod(tree);

	// Use all neighbors in a radius of 3cm.
	mls.setSearchRadius(0.03);

	// Reconstruct
	mls.process(*cloud_smoothed);

	*cloud = *cloud_smoothed;

	return hr;
}

/// <summary>
/// CropBox is a filter that allows the user to filter all the data inside of a given box
/// </summary>
/// <param name="cloud">Pointer to input pointCloud</param>
/// <returns>indicates success or failure</returns>
HRESULT CCloudFilters::CropBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	HRESULT hr = S_OK;
	Eigen::Vector4f minPoint;
	pcl::PointXYZ min, max;

	pcl::getMinMax3D(*cloud, min, max);
	minPoint[0] = min.x;  // define minimum point x
	minPoint[1] = min.y;  // define minimum point y
	minPoint[2] = min.z;  // define minimum point z

	Eigen::Vector4f maxPoint;
	maxPoint[0] = max.x;  // define max point x
	maxPoint[1] = max.y;  // define max point y
	maxPoint[2] = max.z;  // define max point z 

	Eigen::Vector3f boxTranslatation;
	boxTranslatation[0] = 0;
	boxTranslatation[1] = 0;
	boxTranslatation[2] = 0;

	Eigen::Vector3f boxRotation;
	boxRotation[0] = 0;  // rotation around x-axis
	boxRotation[1] = 0;  // rotation around y-axis
	boxRotation[2] = 0;  //in radians rotation around z-axis. this rotates your cube 45deg around z-axis. 

	Eigen::Affine3f boxTransform;

	pcl::CropBox<pcl::PointXYZ> cropFilter;
	cropFilter.setInputCloud(cloud);
	cropFilter.setMin(minPoint);
	cropFilter.setMax(maxPoint);
	cropFilter.setTranslation(boxTranslatation);
	cropFilter.setRotation(boxRotation);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

	cropFilter.filter(*cloud_filtered);

	*cloud = *cloud_filtered;

	return hr;
	
}