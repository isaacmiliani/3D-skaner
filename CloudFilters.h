//------------------------------------------------------------------------------
// <copyright file="CloudFilter.cpp" company="PCL">
//     Filter implementation for Point Clouds
//  http://docs.pointclouds.org/1.7.0/group__filters.html
// </copyright>
//------------------------------------------------------------------------------
#pragma once
#include "ImageRenderer.h"
#include <pcl/visualization/pcl_visualizer.h>

/// <summary>
/// CloudFilter class.
/// </summary>

class CCloudFilters{

public:
	/// <summary>
	/// Constructor
	/// </summary>
	CCloudFilters();

	/// <summary>
	/// Destructor
	/// </summary>
	~CCloudFilters();

	/// <summary>
	/// Downsampling a PointCloud
	/// </summary>
	HRESULT VoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	/// <summary>
	/// Removing outliers using a StatisticalOutlierRemoval filter
	/// </summary>
	HRESULT StatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	/// <summary>
	/// Filtering a PointCloud using a PassThrough filter
	/// </summary>
	HRESULT PassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	/// <summary>
	/// A bilateral filter implementation for point cloud data
	/// <summary>
	HRESULT Bilateral(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	/// <summary>
	/// A Fast bilateral filter implementation for point cloud data
	/// <summary>
	HRESULT FastBilateral(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	/// <summary>
	/// Moving Least Squares 
	/// </summary>
	HRESULT MovingLeastSquares(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	/// <summary>
	/// Crop Box is a filter that allows the user to filter all the data inside of a given box
	/// </summary>
	HRESULT CropBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};