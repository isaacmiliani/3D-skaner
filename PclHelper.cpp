// System includes
#define NOMINMAX
#include "PclHelper.h"

#pragma once
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

CPclHelper::CPclHelper(){

}
CPclHelper::~CPclHelper(){

}
/// <summary>
/// Shows PointCloud with PCL
/// </summary>
/// <param name="mesh">The mesh to save.</param>
/// <returns>indicates success or failure</returns>
boost::shared_ptr<pcl::visualization::PCLVisualizer> CPclHelper::ShowCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud(cloud);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "3D Point Cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();


	return(viewer);
	
}

/// <summary>
/// Create and shows PointCloud with Normals
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CPclHelper::ShowCloudWithNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewerPtr){
	HRESULT hr = S_OK;

	// Visualize them.
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));

	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	// Display one normal out of 20, as a line of length 3cm.
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 20, 0.03, "normals");

	viewerPtr = viewer;
	return hr;

}

/// <summary>
/// Read a PCD file into PointCloud
/// </summary>
HRESULT CPclHelper::ReadPcdFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ){
	HRESULT hr;
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read PCD file \n");
		return hr = E_UNEXPECTED;
	}

	return hr = S_OK;
}

/// <summary>
/// Read a PLY file into PointCloud
/// </summary>
HRESULT CPclHelper::ReadPlyFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	HRESULT hr;
	if (pcl::io::loadPLYFile(fileName, *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read PLY file \n");
		return hr = E_UNEXPECTED;
	}

	return hr = S_OK;
}


/// <summary>
/// Display Mesh 
/// </summary>
/// <returns>mesh viewer</returns>
boost::shared_ptr<pcl::visualization::PCLVisualizer> CPclHelper::DisplayMesh(pcl::PolygonMesh *mesh){
	HRESULT hr = S_OK;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPolygonMesh(*mesh);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "3D Mesh");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();


	return(viewer);
}
/// <summary>
/// Display Mesh 
/// </summary>
/// <returns>mesh viewer</returns>
HRESULT CPclHelper::SavePointCloudAsPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string &fileName){
	HRESULT hr = S_OK;
	pcl::io::savePCDFileASCII(fileName, *cloud);
	return hr;
}


/// <summary>
/// Load and display Mesh from RGB point cloud
/// </summary>
HRESULT CPclHelper::DisplayMeshAndNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_rgb, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerPtr)
{
	HRESULT hr = S_OK;
	// --------------------------------------------------------
	// -----Open 3D viewer and add point cloud and normals-----
	// --------------------------------------------------------
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(pointcloud_rgb);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.05);
	ne.compute(*cloud_normals1);

	// ---------------------------------------------------------------
	// -----Calculate surface normals with a search radius of 0.1-----
	// ---------------------------------------------------------------
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.1);
	ne.compute(*cloud_normals2);

	// Set Viewer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("RGB Point Cloud Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointcloud_rgb);
	viewer->addPointCloud<pcl::PointXYZRGB>(pointcloud_rgb, rgb, "pointcloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pointcloud");
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(pointcloud_rgb, cloud_normals2, 10, 0.05, "normals");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	viewerPtr = viewer;

	return hr;
}

/// <summary>
/// Save Mesh as PLY
/// </summary>
/// <returns>mesh viewer</returns>
HRESULT CPclHelper::SaveMeshAsPLY(const std::string &fileName, pcl::PolygonMesh mesh){
	HRESULT hr = S_OK;
	pcl::io::savePLYFile(fileName, mesh);
	return hr;
}

