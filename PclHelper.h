//------------------------------------------------------------------------------
// <copyright file="PclHelper.cpp" company="PCL">
//     Surface Reconstruction implementation for Point Clouds
// http://docs.pointclouds.org/trunk/group__surface.html
// </copyright>
//------------------------------------------------------------------------------
#pragma once
#include "ImageRenderer.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

/// <summary>
/// PCL visualization helper class.
/// </summary>
class CPclHelper{
public:
	/// <summary>
	/// Constructor
	/// </summary>
	CPclHelper();

	/// <summary>
	/// Destructor
	/// </summary>
	~CPclHelper();

	/// <summary>
	/// Show PointCloud 
	/// </summary>
	boost::shared_ptr<pcl::visualization::PCLVisualizer> ShowCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	/// <summary>
	/// Show PointCloud with Normals
	/// </summary>
	HRESULT ShowCloudWithNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewerPtr);

	/// <summary>
	/// Load PCD file into PointCloud
	/// </summary>
	HRESULT ReadPcdFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	/// <summary>
	/// Load PCD file into PointCloud
	/// </summary>
	HRESULT ReadPlyFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	/// <summary>
	/// Load and display Mesh from file
	/// </summary>
	boost::shared_ptr<pcl::visualization::PCLVisualizer> DisplayMesh(pcl::PolygonMesh *mesh);

	/// <summary>
	/// Load and display Mesh from file
	/// </summary>
	HRESULT DisplayMeshAndNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_rgb, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerPtr);

	/// <summary>
	/// Save a point cloud in PCD format
	/// </summary>
	HRESULT SavePointCloudAsPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr, const std::string &fileName);

	/// <summary>
	/// Load and display Mesh from file
	/// </summary>
	HRESULT SaveMeshAsPLY(const std::string &fileName, pcl::PolygonMesh mesh);
	
};

