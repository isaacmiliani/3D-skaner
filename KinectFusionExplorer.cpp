//------------------------------------------------------------------------------
// <copyright file="KinectFusionExplorer.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

// System includes
#define NOMINMAX
#include "stdafx.h"
#include "Windowsx.h"
#include "shobjidl.h" 

// Project includes
#include "resource.h"
#include <Kinect.h>
#include "KinectFusionExplorer.h"
#include "KinectFusionProcessorFrame.h"
#include "KinectFusionHelper.h"
#include <boost/thread/thread.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <pcl/io/vtk_io.h>

#define MIN_DEPTH_DISTANCE_MM 500   // Must be greater than 0
#define MAX_DEPTH_DISTANCE_MM 8000
#define MIN_INTEGRATION_WEIGHT 1    // Must be greater than 0
#define MAX_INTEGRATION_WEIGHT 1000
#define WM_FRAMEREADY           (WM_USER + 0)
#define WM_UPDATESENSORSTATUS   (WM_USER + 1)

#define MIN_SMOOTHING_DISTANCE_THRESHOLD 40   //4cm, could use up to around 0.1f
#define MAX_SMOOTHING_DISTANCE_THRESHOLD 100


// Object to storage Point Cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
// Object to storage Point Cloud with color information
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());

// PCL Visualizer 
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

// Object for storing the normals.
pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
pcl::PolygonMesh polymesh;

/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE, _In_ LPWSTR, _In_ int nCmdShow)
{
	CKinectFusionExplorer application;
	application.Run(hInstance, nCmdShow);
}

/// <summary>
/// Constructor
/// </summary>
CKinectFusionExplorer::CKinectFusionExplorer() :
m_hWnd(nullptr),
m_pD2DFactory(nullptr),
m_pDrawReconstruction(nullptr),
m_pDrawTrackingResiduals(nullptr),
m_pDrawDepth(nullptr),
m_bSavingMesh(false),
m_saveMeshFormat(Stl),
m_bInitializeError(false),
m_bColorCaptured(false),
m_bUIUpdated(false),
m_bDisplayCloud(false),
m_bDisplayCloudWithNormals(false),
m_bDisplayMesh(true),
m_bDisplayProcessedMesh(false)
{
	
}

/// <summary>
/// Destructor
/// </summary>
CKinectFusionExplorer::~CKinectFusionExplorer()
{
	// clean up Direct2D renderer
	SAFE_DELETE(m_pDrawReconstruction);

	// clean up Direct2D renderer
	SAFE_DELETE(m_pDrawTrackingResiduals);

	// clean up Direct2D renderer
	SAFE_DELETE(m_pDrawDepth);

	// clean up Direct2D
	SafeRelease(m_pD2DFactory);
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CKinectFusionExplorer::Run(HINSTANCE hInstance, int nCmdShow)
{
	MSG       msg = { 0 };
	WNDCLASS  wc = { 0 };

	// Dialog custom window class
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.cbWndExtra = DLGWINDOWEXTRA;
	wc.hInstance = hInstance;
	wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
	wc.hIcon = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
	wc.lpfnWndProc = DefDlgProcW;
	wc.lpszClassName = L"KinectFusionExplorerAppDlgWndClass";

	if (!RegisterClassW(&wc))
	{
		return 0;
	}

	// Create main application window
	HWND hWndApp = CreateDialogParamW(
		hInstance,
		MAKEINTRESOURCE(IDD_APP),
		nullptr,
		(DLGPROC)CKinectFusionExplorer::MessageRouter,
		reinterpret_cast<LPARAM>(this));

	// Show window
	ShowWindow(hWndApp, nCmdShow);

	// Main message loop
	while (WM_QUIT != msg.message)
	{
		if (GetMessage(&msg, nullptr, 0, 0))
		{
			// If a dialog message will be taken care of by the dialog proc
			if ((hWndApp != nullptr) && IsDialogMessageW(hWndApp, &msg))
			{
				continue;
			}

			TranslateMessage(&msg);
			DispatchMessageW(&msg);
		}
	}

	return static_cast<int>(msg.wParam);
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CKinectFusionExplorer::MessageRouter(
	HWND hWnd,
	UINT uMsg,
	WPARAM wParam,
	LPARAM lParam)
{
	CKinectFusionExplorer* pThis = nullptr;

	if (WM_INITDIALOG == uMsg)
	{
		pThis = reinterpret_cast<CKinectFusionExplorer*>(lParam);
		SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
	}
	else
	{
		pThis = reinterpret_cast<CKinectFusionExplorer*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
	}

	if (pThis)
	{
		return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
	}

	return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CKinectFusionExplorer::DlgProc(
	HWND hWnd,
	UINT message,
	WPARAM wParam,
	LPARAM lParam)
{

	switch (message)
	{
	case WM_INITDIALOG:
	{
		// Bind application window handle
		m_hWnd = hWnd;

		InitializeUIControls();
	}
	break;

	// If the title bar X is clicked, destroy app
	case WM_CLOSE:
		DestroyWindow(hWnd);
		break;

	case WM_DESTROY:
		// Quit the main message pump
		m_processor.StopProcessing();
		PostQuitMessage(0);
		break;

		// Handle button press
	case WM_COMMAND:
		ProcessUI(wParam, lParam);
		break;

		// Handle sliders
	case  WM_HSCROLL:
		UpdateHSliders();
		break;

	case WM_FRAMEREADY:
		HandleCompletedFrame();
		break;

	}

	return FALSE;
}

/// <summary>
/// Handle a completed frame from the Kinect Fusion processor.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
void CKinectFusionExplorer::HandleCompletedFrame()
{
	KinectFusionProcessorFrame const* pFrame = nullptr;

	// Flush any extra WM_FRAMEREADY messages from the queue
	MSG msg;
	while (PeekMessage(&msg, m_hWnd, WM_FRAMEREADY, WM_FRAMEREADY, PM_REMOVE)) {}

	m_processor.LockFrame(&pFrame);

	if (!m_bSavingMesh) // don't render while a mesh is being saved
	{
		if (m_processor.IsVolumeInitialized())
		{
			m_pDrawDepth->Draw(pFrame->m_pDepthRGBX, pFrame->m_cbImageSize);
			m_pDrawReconstruction->Draw(pFrame->m_pReconstructionRGBX, pFrame->m_cbImageSize);
			m_pDrawTrackingResiduals->Draw(pFrame->m_pTrackingDataRGBX, pFrame->m_cbImageSize);
		}

		SetStatusMessage(pFrame->m_statusMessage);
		SetFramesPerSecond(pFrame->m_fFramesPerSecond);
	}

	if (!m_bUIUpdated && m_processor.IsVolumeInitialized())
	{
		const int Mebi = 1024 * 1024;

		// We now create both a color and depth volume, doubling the required memory, so we restrict
		// which resolution settings the user can choose when the graphics card is limited in memory.
		if (pFrame->m_deviceMemory <= 1 * Mebi)  // 1GB
		{
			// Disable 640 voxel resolution in all axes - cards with only 1GB cannot handle this
			HWND hButton = GetDlgItem(m_hWnd, IDC_VOXELS_X_640);
			EnableWindow(hButton, FALSE);
			hButton = GetDlgItem(m_hWnd, IDC_VOXELS_Y_640);
			EnableWindow(hButton, FALSE);
			hButton = GetDlgItem(m_hWnd, IDC_VOXELS_Z_640);
			EnableWindow(hButton, FALSE);

			if (Is64BitApp() == FALSE)
			{
				// Also disable 512 voxel resolution in one arbitrary axis on 32bit machines
				hButton = GetDlgItem(m_hWnd, IDC_VOXELS_Y_512);
				EnableWindow(hButton, FALSE);
			}
		}
		else if (pFrame->m_deviceMemory <= 2 * Mebi)  // 2GB
		{
			if (Is64BitApp() == FALSE)
			{
				// Disable 640 voxel resolution in one arbitrary axis on 32bit machines
				HWND hButton = GetDlgItem(m_hWnd, IDC_VOXELS_Y_640);
				EnableWindow(hButton, FALSE);
			}
			// True 64 bit apps seem to be more able to cope with large volume sizes.
		}

		m_bUIUpdated = true;
	}

	m_bColorCaptured = pFrame->m_bColorCaptured;

	m_processor.UnlockFrame();
}

/// <summary>
/// Save Mesh to disk.
/// </summary>
/// <param name="mesh">The mesh to save.</param>
/// <returns>indicates success or failure</returns>
HRESULT CKinectFusionExplorer::SaveMeshFile(INuiFusionColorMesh *pMesh, KinectFusionMeshTypes saveMeshType)
{
	HRESULT hr = S_OK;

	if (nullptr == pMesh)
	{
		return E_INVALIDARG;
	}


	CComPtr<IFileSaveDialog> pSaveDlg;

	// Create the file save dialog object.
	hr = pSaveDlg.CoCreateInstance(__uuidof(FileSaveDialog));

	if (FAILED(hr))
	{
		return hr;
	}

	// Set the dialog title
	hr = pSaveDlg->SetTitle(L"Save Mesh");
	if (SUCCEEDED(hr))
	{
		// Set the button text
		hr = pSaveDlg->SetOkButtonLabel(L"Save");
		if (SUCCEEDED(hr))
		{
			// Set a default filename
			if (Stl == saveMeshType)
			{
				hr = pSaveDlg->SetFileName(L"KinectFusionReconstruction.stl");
			}
			else if (Obj == saveMeshType)
			{
				hr = pSaveDlg->SetFileName(L"KinectFusionReconstruction.obj");
			}
			else if (Ply == saveMeshType)
			{
				hr = pSaveDlg->SetFileName(L"KinectFusionReconstruction.ply");
			}

			if (SUCCEEDED(hr))
			{
				// Set the file type extension
				if (Stl == saveMeshType)
				{
					hr = pSaveDlg->SetDefaultExtension(L"stl");
				}
				else if (Obj == saveMeshType)
				{
					hr = pSaveDlg->SetDefaultExtension(L"obj");
				}
				else if (Ply == saveMeshType)
				{
					hr = pSaveDlg->SetDefaultExtension(L"ply");
				}

				if (SUCCEEDED(hr))
				{
					// Set the file type filters
					if (Stl == saveMeshType)
					{
						COMDLG_FILTERSPEC allPossibleFileTypes[] = {
							{ L"Stl mesh files", L"*.stl" },
							{ L"All files", L"*.*" }
						};

						hr = pSaveDlg->SetFileTypes(
							ARRAYSIZE(allPossibleFileTypes),
							allPossibleFileTypes);
					}
					else if (Obj == saveMeshType)
					{
						COMDLG_FILTERSPEC allPossibleFileTypes[] = {
							{ L"Obj mesh files", L"*.obj" },
							{ L"All files", L"*.*" }
						};

						hr = pSaveDlg->SetFileTypes(
							ARRAYSIZE(allPossibleFileTypes),
							allPossibleFileTypes);
					}
					else if (Ply == saveMeshType)
					{
						COMDLG_FILTERSPEC allPossibleFileTypes[] = {
							{ L"Ply mesh files", L"*.ply" },
							{ L"All files", L"*.*" }
						};

						hr = pSaveDlg->SetFileTypes(
							ARRAYSIZE(allPossibleFileTypes),
							allPossibleFileTypes);
					}

					if (SUCCEEDED(hr))
					{
						// Show the file selection box
						hr = pSaveDlg->Show(m_hWnd);

						// Save the mesh to the chosen file.
						if (SUCCEEDED(hr))
						{
							CComPtr<IShellItem> pItem;
							hr = pSaveDlg->GetResult(&pItem);

							if (SUCCEEDED(hr))
							{
								LPOLESTR pwsz = nullptr;
								hr = pItem->GetDisplayName(SIGDN_FILESYSPATH, &pwsz);

								if (SUCCEEDED(hr))
								{
									SetStatusMessage(L"Saving mesh file, please wait...");
									SetCursor(LoadCursor(nullptr, MAKEINTRESOURCE(IDC_WAIT)));

									if (Stl == saveMeshType)
									{
										hr = WriteBinarySTLMeshFile(pMesh, pwsz);
									}
									else if (Obj == saveMeshType)
									{
										hr = WriteAsciiObjMeshFile(pMesh, pwsz);
									}
									else if (Ply == saveMeshType)
									{
										hr = WriteAsciiPlyMeshFile(pMesh, pwsz, true, m_bColorCaptured);
									}

									CoTaskMemFree(pwsz);
								}
							}
						}
					}
				}
			}
		}
	}

	return hr;
}

/// <summary>
/// Open Mesh file
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CKinectFusionExplorer::OpenMeshFile()
{

	HRESULT hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED |
		COINIT_DISABLE_OLE1DDE);
	if (SUCCEEDED(hr))
	{
		IFileOpenDialog *pFileOpen;

		// Create the FileOpenDialog object.
		hr = CoCreateInstance(CLSID_FileOpenDialog, NULL, CLSCTX_ALL,
			IID_IFileOpenDialog, reinterpret_cast<void**>(&pFileOpen));

		if (SUCCEEDED(hr))
		{
			// Show the Open dialog box.
			hr = pFileOpen->Show(NULL);

			// Get the file name from the dialog box.
			if (SUCCEEDED(hr))
			{
				IShellItem *pItem;
				hr = pFileOpen->GetResult(&pItem);
				if (SUCCEEDED(hr))
				{
					PWSTR pszFilePath;
					CPclHelper pcl;
					std::string filePath;
					std::string extension;

					hr = pItem->GetDisplayName(SIGDN_FILESYSPATH, &pszFilePath);
					if (FAILED(hr))
						return hr;

					SetStatusMessage(L"Loading Point Cloud...");

					filePath = CT2A(pszFilePath);
					extension = filePath.substr(filePath.find_last_of(".") + 1);
						
					if (extension == "pcd")
						hr = pcl.ReadPcdFile(filePath, pointcloud);
					else if (extension == "ply")
						hr = pcl.ReadPlyFile(filePath, pointcloud);
					else
						SetStatusMessage(L"Unrecognized file extension...");

					if (FAILED(hr))
						SetStatusMessage(L"Load Failed...");
					else
						SetStatusMessage(L"Done!...");
				}
			}
		}
	}
	return hr;
}

/// <summary>
///  Reconstruct Mesh using selected parameters
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CKinectFusionExplorer::ReconstructMesh()
{
	HRESULT hr = S_OK;
	CPclHelper pcl;
	CCloudFilters filter;
	CSurfaceReconstruction reconstruction;
	// Object to storage filtered Point Cloud 


	if (m_params.m_bVoxelGridFilter) {
		SetStatusMessage(L"Applying Voxel Grid filter...");
		hr = filter.VoxelGrid(pointcloud);
		
		if (FAILED(hr)){
			SetStatusMessage(L"Voxel Grid filter failed...");
			return hr;
		}
	}

	if (m_params.m_bOutliersRemoval){
		SetStatusMessage(L"Applying Statistical Outlier Removal...");
		hr = filter.StatisticalOutlierRemoval(pointcloud);

		if (FAILED(hr)){
			SetStatusMessage(L"Statistical Outlier Removal failed...");
			return hr;
		}
	}
	

	if (m_params.m_bPassthroughFilter){
		SetStatusMessage(L"Applying Passthrough filter...");
		hr = filter.PassThrough(pointcloud);

		if (FAILED(hr)){
			SetStatusMessage(L"Passthrough filter failed...");
			return hr;
		}
	}

	if (m_params.m_bBilateralFilter){
		SetStatusMessage(L"Applying Bilateral filter...");
		hr = filter.Bilateral(pointcloud);

		if (FAILED(hr)){
			SetStatusMessage(L"Bilateral filter failed...");
			return hr;
		}
	}

	if (m_params.m_bFastBilateralFilter){
		SetStatusMessage(L"Applying Fast Bilateral filter...");
		hr = reconstruction.NormalEstimation(pointcloud, normals);
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields(*pointcloud, *normals, *cloud_with_normals);
		hr = filter.FastBilateral(cloud_with_normals);

		if (FAILED(hr)){
			SetStatusMessage(L"Fast Bilateral filter failed...");
			return hr;
		}
	}
		
	if (m_params.m_bCropBoxFilter){
		SetStatusMessage(L"Applying Cropbox filter...");
		hr = filter.CropBox(pointcloud);

		if (FAILED(hr)){
			SetStatusMessage(L"Cropbox filter failed...");
			return hr;
		}
	}

	if (m_params.m_bMovingLeastSquares){
		SetStatusMessage(L"Applying Moving Least Squares...");
		hr = filter.MovingLeastSquares(pointcloud);

		if (FAILED(hr)){
			SetStatusMessage(L"Moving Least Squares failed...");
			return hr;
		}
	}
		
	SetStatusMessage(L"Mesh Reconstruction...");

	switch (m_params.m_ReconstructionAlgorithm)
	{
	case 0:
		hr = reconstruction.NormalEstimation(pointcloud, normals);
		hr = reconstruction.Poisson(pointcloud, normals, &polymesh);
		break;
	case 1:
		//reconstruction.MarchingCubes(processed_pointcloud, cloud_smoothed);
		break;
	case 2:
		hr = reconstruction.NormalEstimation(pointcloud, normals);
		hr = reconstruction.GreedyProjectionTriangulation(pointcloud, normals, &polymesh);
		break;
	default:
		SetStatusMessage(L"There is no reconstruction algorithm selected...");
		break;
	}
	if (FAILED(hr)){
		SetStatusMessage(L"Mesh Reconstruction failed...");
		return hr;
	}

	SetStatusMessage(L"Done!...");

	if (m_bDisplayCloud)
		viewer = pcl.ShowCloud(pointcloud);

	if (m_bDisplayCloudWithNormals)
		pcl.DisplayMeshAndNormals(pointcloud_rgb, viewer);

	if (m_bDisplayMesh || m_bDisplayProcessedMesh)
		viewer = pcl.DisplayMesh(&polymesh);
	
	while (!viewer->wasStopped()){
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	viewer->close();

	return hr;
}

/// <summary>
///  Save Mesh 
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CKinectFusionExplorer::SaveMesh(pcl::PolygonMesh pMesh, KinectFusionMeshTypes saveMeshType)
{
	HRESULT hr = S_OK;
	CComPtr<IFileSaveDialog> pSaveDlg;

	// Create the file save dialog object.
	hr = pSaveDlg.CoCreateInstance(__uuidof(FileSaveDialog));

	if (FAILED(hr))
	{
		return hr;
	}

	// Set the dialog title
	hr = pSaveDlg->SetTitle(L"Save Mesh");
	if (SUCCEEDED(hr))
	{
		// Set the button text
		hr = pSaveDlg->SetOkButtonLabel(L"Save");
		if (SUCCEEDED(hr))
		{
			// Set a default filename
			if (Stl == saveMeshType)
			{
				hr = pSaveDlg->SetFileName(L"MeshReconstruction.stl");
			}
			else if (Obj == saveMeshType)
			{
				hr = pSaveDlg->SetFileName(L"MeshReconstruction.obj");
			}
			else if (Ply == saveMeshType)
			{
				hr = pSaveDlg->SetFileName(L"MeshReconstruction.ply");
			}

			if (SUCCEEDED(hr))
			{
				// Set the file type extension
				if (Stl == saveMeshType)
				{
					hr = pSaveDlg->SetDefaultExtension(L"stl");
				}
				else if (Obj == saveMeshType)
				{
					hr = pSaveDlg->SetDefaultExtension(L"obj");
				}
				else if (Ply == saveMeshType)
				{
					hr = pSaveDlg->SetDefaultExtension(L"ply");
				}

				if (SUCCEEDED(hr))
				{
					// Set the file type filters
					if (Stl == saveMeshType)
					{
						COMDLG_FILTERSPEC allPossibleFileTypes[] = {
							{ L"Stl mesh files", L"*.stl" },
							{ L"All files", L"*.*" }
						};

						hr = pSaveDlg->SetFileTypes(
							ARRAYSIZE(allPossibleFileTypes),
							allPossibleFileTypes);
					}
					else if (Obj == saveMeshType)
					{
						COMDLG_FILTERSPEC allPossibleFileTypes[] = {
							{ L"Obj mesh files", L"*.obj" },
							{ L"All files", L"*.*" }
						};

						hr = pSaveDlg->SetFileTypes(
							ARRAYSIZE(allPossibleFileTypes),
							allPossibleFileTypes);
					}
					else if (Ply == saveMeshType)
					{
						COMDLG_FILTERSPEC allPossibleFileTypes[] = {
							{ L"Ply mesh files", L"*.ply" },
							{ L"All files", L"*.*" }
						};

						hr = pSaveDlg->SetFileTypes(
							ARRAYSIZE(allPossibleFileTypes),
							allPossibleFileTypes);
					}

					if (SUCCEEDED(hr))
					{
						// Show the file selection box
						hr = pSaveDlg->Show(m_hWnd);

						// Save the mesh to the chosen file.
						if (SUCCEEDED(hr))
						{
							CComPtr<IShellItem> pItem;
							hr = pSaveDlg->GetResult(&pItem);

							if (SUCCEEDED(hr))
							{
								LPOLESTR pwsz = nullptr;
								hr = pItem->GetDisplayName(SIGDN_FILESYSPATH, &pwsz);

								if (SUCCEEDED(hr))
								{
									SetStatusMessage(L"Saving mesh file, please wait...");
									SetCursor(LoadCursor(nullptr, MAKEINTRESOURCE(IDC_WAIT)));

									PWSTR pszFilePath;
									CPclHelper pcl;
									const std::string filePath = CT2A(pwsz);

									if (Stl == saveMeshType)
									{
										//hr = WriteBinarySTLMeshFile(pMesh, pwsz);
									}
									else if (Obj == saveMeshType)
									{
										//hr = pcl::io::saveOBJFile()
									}
									else if (Ply == saveMeshType)
									{
										hr = pcl.SaveMeshAsPLY(filePath, pMesh);
									}
									CoTaskMemFree(pwsz);
								
									SetStatusMessage(L" Done!...");
								}
							}
						}
					}
				}
			}
		}
	}

	return hr;
}
/// <summary>
/// Gets a Pointcloud from KinectFusion Mesh
/// </summary>
HRESULT CKinectFusionExplorer::CreatePointCloud(INuiFusionColorMesh* mesh, bool flipYZ, bool outputColor){
	HRESULT hr = S_OK;

	int depthWidth = m_params.m_cDepthWidth;
	int depthHeight = m_params.m_cDepthHeight;
	// To Reserve Depth Frame Buffer
	std::vector<UINT16> depthBuffer(depthWidth * depthHeight);

	unsigned int numVertices = mesh->VertexCount();
	unsigned int numTriangleIndices = mesh->TriangleVertexIndexCount();
	unsigned int numTriangles = numVertices / 3;
	unsigned int numColors = mesh->ColorCount();

	const Vector3 *vertices = NULL;
	hr = mesh->GetVertices(&vertices);

	if (FAILED(hr))
	{
		return hr;
	}
	const int *colors = NULL;
	if (outputColor)
	{
		hr = mesh->GetColors(&colors);
		if (FAILED(hr))
		{
			return hr;
		}

		pointcloud_rgb->width = static_cast<uint32_t>(depthWidth);
		pointcloud_rgb->height = static_cast<uint32_t>(depthHeight);
		pointcloud_rgb->is_dense = false;

		for (int vertexIndex = 0; vertexIndex < numVertices; vertexIndex++){
			pcl::PointXYZRGB point;
			unsigned int color = colors[vertexIndex];

			point.r = ((color >> 16) & 255);
			point.g = ((color >> 8) & 255);
			point.b = (color & 255);

			point.x = vertices[vertexIndex].x;
			point.y = -vertices[vertexIndex].y;
			point.z = -vertices[vertexIndex].z;

			pointcloud_rgb->push_back(point);

		}
	}
	else
	{
		pointcloud->width = static_cast<uint32_t>(depthWidth);
		pointcloud->height = static_cast<uint32_t>(depthHeight);
		pointcloud->is_dense = false;
		for (int vertexIndex = 0; vertexIndex < numVertices; vertexIndex++){
			pcl::PointXYZ point;

			point.x = vertices[vertexIndex].x;
			point.y = -vertices[vertexIndex].y;
			point.z = -vertices[vertexIndex].z;

			pointcloud->push_back(point);
		}
	
	}

	return hr;
}

/// <summary>
/// Initialize the UI
/// </summary>
void CKinectFusionExplorer::InitializeUIControls()
{
	// Create NuiSensorChooser UI control
	RECT rc;
	GetClientRect(m_hWnd, &rc);

	POINT ptCenterTop;
	ptCenterTop.x = (rc.right - rc.left) / 2;
	ptCenterTop.y = 0;

	// Set slider ranges
	SendDlgItemMessage(
		m_hWnd,
		IDC_SLIDER_DEPTH_MIN,
		TBM_SETRANGE,
		TRUE,
		MAKELPARAM(MIN_DEPTH_DISTANCE_MM, MAX_DEPTH_DISTANCE_MM));

	SendDlgItemMessage(m_hWnd,
		IDC_SLIDER_DEPTH_MAX,
		TBM_SETRANGE,
		TRUE,
		MAKELPARAM(MIN_DEPTH_DISTANCE_MM, MAX_DEPTH_DISTANCE_MM));

	SendDlgItemMessage(
		m_hWnd,
		IDC_INTEGRATION_WEIGHT_SLIDER,
		TBM_SETRANGE,
		TRUE,
		MAKELPARAM(MIN_INTEGRATION_WEIGHT, MAX_INTEGRATION_WEIGHT));

	SendDlgItemMessage(
		m_hWnd,
		IDC_SMOOTHING_DISTANCE_THRESHOLD,
		TBM_SETRANGE,
		TRUE,
		MAKELPARAM(MIN_SMOOTHING_DISTANCE_THRESHOLD, MAX_SMOOTHING_DISTANCE_THRESHOLD));

	// Set slider positions
	SendDlgItemMessage(
		m_hWnd,
		IDC_SLIDER_DEPTH_MAX,
		TBM_SETPOS,
		TRUE,
		(UINT)m_params.m_fMaxDepthThreshold * 1000);

	SendDlgItemMessage(
		m_hWnd,
		IDC_SLIDER_DEPTH_MIN,
		TBM_SETPOS,
		TRUE,
		(UINT)m_params.m_fMinDepthThreshold * 1000);

	SendDlgItemMessage(
		m_hWnd,
		IDC_INTEGRATION_WEIGHT_SLIDER,
		TBM_SETPOS,
		TRUE,
		(UINT)m_params.m_cMaxIntegrationWeight);

	// Set slider positions
	SendDlgItemMessage(
		m_hWnd,
		IDC_SMOOTHING_DISTANCE_THRESHOLD,
		TBM_SETPOS,
		TRUE,
		(UINT)m_params.m_fSmoothingDistanceThreshold * 1000);

	// Set intermediate slider tics at meter intervals
	for (int i = 1; i < (MAX_DEPTH_DISTANCE_MM / 1000); i++)
	{
		SendDlgItemMessage(m_hWnd, IDC_SLIDER_DEPTH_MAX, TBM_SETTIC, 0, i * 1000);
		SendDlgItemMessage(m_hWnd, IDC_SLIDER_DEPTH_MIN, TBM_SETTIC, 0, i * 1000);
	}

	// Update slider text
	WCHAR str[MAX_PATH];
	swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", m_params.m_fMinDepthThreshold);
	SetDlgItemText(m_hWnd, IDC_MIN_DIST_TEXT, str);
	swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", m_params.m_fMaxDepthThreshold);
	SetDlgItemText(m_hWnd, IDC_MAX_DIST_TEXT, str);

	swprintf_s(str, ARRAYSIZE(str), L"%u", m_params.m_cMaxIntegrationWeight);
	SetDlgItemText(m_hWnd, IDC_INTEGRATION_WEIGHT_TEXT, str);

	// Set the radio buttons for Volume Parameters
	switch ((int)m_params.m_reconstructionParams.voxelsPerMeter)
	{
	case 768:
		CheckDlgButton(m_hWnd, IDC_VPM_768, BST_CHECKED);
		break;
	case 640:
		CheckDlgButton(m_hWnd, IDC_VPM_640, BST_CHECKED);
		break;
	case 512:
		CheckDlgButton(m_hWnd, IDC_VPM_512, BST_CHECKED);
		break;
	case 384:
		CheckDlgButton(m_hWnd, IDC_VPM_384, BST_CHECKED);
		break;
	case 256:
		CheckDlgButton(m_hWnd, IDC_VPM_256, BST_CHECKED);
		break;
	case 128:
		CheckDlgButton(m_hWnd, IDC_VPM_128, BST_CHECKED);
		break;
	default:
		m_params.m_reconstructionParams.voxelsPerMeter = 256.0f;	// set to medium default
		CheckDlgButton(m_hWnd, IDC_VPM_256, BST_CHECKED);
		break;
	}

	switch ((int)m_params.m_reconstructionParams.voxelCountX)
	{
	case 640:
		CheckDlgButton(m_hWnd, IDC_VOXELS_X_640, BST_CHECKED);
		break;
	case 512:
		CheckDlgButton(m_hWnd, IDC_VOXELS_X_512, BST_CHECKED);
		break;
	case 384:
		CheckDlgButton(m_hWnd, IDC_VOXELS_X_384, BST_CHECKED);
		break;
	case 256:
		CheckDlgButton(m_hWnd, IDC_VOXELS_X_256, BST_CHECKED);
		break;
	case 128:
		CheckDlgButton(m_hWnd, IDC_VOXELS_X_128, BST_CHECKED);
		break;
	default:
		m_params.m_reconstructionParams.voxelCountX = 384;	// set to medium default
		CheckDlgButton(m_hWnd, IDC_VOXELS_X_384, BST_CHECKED);
		break;
	}

	// Set the radio buttons for Smoothing Kernel Width
	switch ((int)m_params.m_cSmoothingKernelWidth)
	{
	case 0:
		CheckDlgButton(m_hWnd, IDC_KERNEL_WIDTH_NONE, BST_CHECKED);
		break;
	case 1:
		CheckDlgButton(m_hWnd, IDC_KERNEL_WIDTH_3X3, BST_CHECKED);
		break;
	case 2:
		CheckDlgButton(m_hWnd, IDC_KERNEL_WIDTH_5X5, BST_CHECKED);
		break;
	case 3:
		CheckDlgButton(m_hWnd, IDC_KERNEL_WIDTH_7X7, BST_CHECKED);
		break;
	default:
		m_params.m_cSmoothingKernelWidth = 1;	// 1=3x3,
		CheckDlgButton(m_hWnd, IDC_KERNEL_WIDTH_3X3, BST_CHECKED);
		break;
	}
	// Set the radio buttons for Downsampling factor
	switch ((int)m_params.m_cAlignPointCloudsImageDownsampleFactor)
	{
	case 1:
		CheckDlgButton(m_hWnd, IDC_DOWNSAMPLE_FACTOR_1, BST_CHECKED);
		break;
	case 2:
		CheckDlgButton(m_hWnd, IDC_DOWNSAMPLE_FACTOR_2, BST_CHECKED);
		break;
	case 4:
		CheckDlgButton(m_hWnd, IDC_DOWNSAMPLE_FACTOR_4, BST_CHECKED);
		break;
	default:
		m_params.m_cAlignPointCloudsImageDownsampleFactor = 2;	// 1=3x3,
		CheckDlgButton(m_hWnd, IDC_DOWNSAMPLE_FACTOR_2, BST_CHECKED);
		break;
	}


	switch ((int)m_params.m_reconstructionParams.voxelCountY)
	{
	case 640:
		CheckDlgButton(m_hWnd, IDC_VOXELS_Y_640, BST_CHECKED);
		break;
	case 512:
		CheckDlgButton(m_hWnd, IDC_VOXELS_Y_512, BST_CHECKED);
		break;
	case 384:
		CheckDlgButton(m_hWnd, IDC_VOXELS_Y_384, BST_CHECKED);
		break;
	case 256:
		CheckDlgButton(m_hWnd, IDC_VOXELS_Y_256, BST_CHECKED);
		break;
	case 128:
		CheckDlgButton(m_hWnd, IDC_VOXELS_Y_128, BST_CHECKED);
		break;
	default:
		m_params.m_reconstructionParams.voxelCountY = 512;	// set to medium default
		CheckDlgButton(m_hWnd, IDC_VOXELS_Y_512, BST_CHECKED);
		break;
	}

	switch ((int)m_params.m_reconstructionParams.voxelCountZ)
	{
	case 640:
		CheckDlgButton(m_hWnd, IDC_VOXELS_Z_640, BST_CHECKED);
		break;
	case 512:
		CheckDlgButton(m_hWnd, IDC_VOXELS_Z_512, BST_CHECKED);
		break;
	case 384:
		CheckDlgButton(m_hWnd, IDC_VOXELS_Z_384, BST_CHECKED);
		break;
	case 256:
		CheckDlgButton(m_hWnd, IDC_VOXELS_Z_256, BST_CHECKED);
		break;
	case 128:
		CheckDlgButton(m_hWnd, IDC_VOXELS_Z_128, BST_CHECKED);
		break;
	default:
		m_params.m_reconstructionParams.voxelCountZ = 384;	// set to medium default
		CheckDlgButton(m_hWnd, IDC_VOXELS_Z_384, BST_CHECKED);
		break;
	}

	switch ((int)m_params.m_ReconstructionAlgorithm)
	{
	case 0:
		CheckDlgButton(m_hWnd, IDC_RECONSTRUCTION_POISSON, BST_CHECKED);
		break;
	case 1:
		CheckDlgButton(m_hWnd, IDC_RECONSTRUCTION_GREEDY_TRIANGULATION, BST_CHECKED);
		break;
	case 2:
		CheckDlgButton(m_hWnd, IDC_RECONSTRUCTION_MARCHING_CUBES, BST_CHECKED);
		break;
	default:
		m_params.m_ReconstructionAlgorithm = Poisson;	// set to medium default
		CheckDlgButton(m_hWnd, IDC_RECONSTRUCTION_POISSON, BST_CHECKED);
		break;
	}

	if (Stl == m_saveMeshFormat)
	{
		CheckDlgButton(m_hWnd, IDC_MESH_FORMAT_STL_RADIO, BST_CHECKED);
	}
	else if (Obj == m_saveMeshFormat)
	{
		CheckDlgButton(m_hWnd, IDC_MESH_FORMAT_OBJ_RADIO, BST_CHECKED);
	}
	else if (Ply == m_saveMeshFormat)
	{
		CheckDlgButton(m_hWnd, IDC_MESH_FORMAT_PLY_RADIO, BST_CHECKED);
	}

	if (m_params.m_bCaptureColor)
	{
		CheckDlgButton(m_hWnd, IDC_CHECK_CAPTURE_COLOR, BST_CHECKED);
	}

	if (m_params.m_bAutoFindCameraPoseWhenLost)
	{
		CheckDlgButton(m_hWnd, IDC_CHECK_CAMERA_POSE_FINDER, BST_CHECKED);
	}
	if (m_params.m_bDisplaySurfaceNormals)
	{
		CheckDlgButton(m_hWnd, IDC_CHECK_SHOW_NORMALS, BST_CHECKED);
	}
	if (m_params.m_bVoxelGridFilter)
	{
		CheckDlgButton(m_hWnd, IDC_CHECK_VOXEL_GRID, BST_CHECKED);
	}
	if (m_params.m_bOutliersRemoval)
	{
		CheckDlgButton(m_hWnd, IDC_CHECK_OUTLIERS_REMOVAL, BST_CHECKED);
	}
	if (m_params.m_bPassthroughFilter)
	{
		CheckDlgButton(m_hWnd, IDC_CHECK_PASS_THROUGH, BST_CHECKED);
	}
	if (m_params.m_bBilateralFilter)
	{
		CheckDlgButton(m_hWnd, IDC_CHECK_BILATERAL, BST_CHECKED);
	}
	if (m_params.m_bCropBoxFilter)
	{
		CheckDlgButton(m_hWnd, IDC_CHECK_CROPBOX, BST_CHECKED);
	}

	// Shows MESH
	CheckDlgButton(m_hWnd, IDC_CHECK_SHOW_MESH, BST_CHECKED);

	m_saveMeshFormat = m_params.m_saveMeshType;


}

/// <summary>
/// Process the UI inputs
/// </summary>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
void CKinectFusionExplorer::ProcessUI(WPARAM wParam, LPARAM)
{
	// If it was for the display surface normals toggle this variable
	if (IDC_CHECK_CAPTURE_COLOR == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle capture color
		m_params.m_bCaptureColor = !m_params.m_bCaptureColor;
	}
	// If it was for the display surface normals toggle this variable
	if (IDC_CHECK_MIRROR_DEPTH == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle depth mirroring
		m_params.m_bMirrorDepthFrame = !m_params.m_bMirrorDepthFrame;

		// Un-check pause
		CheckDlgButton(m_hWnd, IDC_CHECK_PAUSE_INTEGRATION, BST_UNCHECKED);
		m_processor.ResetReconstruction();
	}
	if (IDC_CHECK_SHOW_NORMALS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_bDisplaySurfaceNormals = !m_params.m_bDisplaySurfaceNormals;
	}
	// If it was for the display surface normals toggle this variable
	if (IDC_CHECK_CAPTURE_COLOR == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle capture color
		m_params.m_bCaptureColor = !m_params.m_bCaptureColor;
	}
	// If it was the reset button clicked, clear the volume
	if (IDC_BUTTON_RESET_RECONSTRUCTION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Un-check pause
		CheckDlgButton(m_hWnd, IDC_CHECK_PAUSE_INTEGRATION, BST_UNCHECKED);
		m_processor.ResetReconstruction();
	}
	// If it was the start button clicked, start the scanning
	if (IDC_BUTTON_START == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		//StartScanner();
		HRESULT hr=S_OK;
		hr = StartScanner();
		//hr=m_processor.StartProcessing();
		HWND hButton = GetDlgItem(m_hWnd, IDC_BUTTON_STOP);
		EnableWindow(hButton, TRUE);
		hButton = GetDlgItem(m_hWnd, IDC_BUTTON_START);
		EnableWindow(hButton, FALSE);
		// Un-check pause
		/*bool wasPaused = m_params.m_bPauseIntegration;
		HWND hButton = GetDlgItem(m_hWnd, IDC_BUTTON_START);
		if (wasPaused){
			EnableWindow(hButton, TRUE);
			hButton = GetDlgItem(m_hWnd, IDC_BUTTON_STOP);
			EnableWindow(hButton, FALSE);
			m_params.m_bPauseIntegration = false;
		}
		else{
			EnableWindow(hButton, FALSE);
			hButton = GetDlgItem(m_hWnd, IDC_BUTTON_STOP);
			EnableWindow(hButton, TRUE);
			StartScanner();
			}*/
	}
	// If it was the stop button clicked, stop the scanning
	if (IDC_BUTTON_STOP == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		HRESULT hr = S_OK;
		hr = m_processor.StopProcessing();
		
		HWND hButton = GetDlgItem(m_hWnd, IDC_BUTTON_STOP);
		EnableWindow(hButton, FALSE);
		hButton = GetDlgItem(m_hWnd, IDC_BUTTON_START);
		EnableWindow(hButton, TRUE);
		
		// Toggle the pause state of the reconstruction
		bool wasPaused = m_params.m_bPauseIntegration;
		/*if (!wasPaused){
			m_processor.StopProcessing();
			HWND hButton = GetDlgItem(m_hWnd, IDC_BUTTON_STOP);
			m_processor.StopProcessing();
			EnableWindow(hButton, FALSE);
			hButton = GetDlgItem(m_hWnd, IDC_BUTTON_START);
			EnableWindow(hButton, TRUE);
			m_params.m_bPauseIntegration = true;
		}*/


	}
	// If it was the mesh button clicked, mesh the volume and save
	if (IDC_BUTTON_MESH_RECONSTRUCTION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		SetStatusMessage(L"Creating and saving mesh of reconstruction, please wait...");
		m_bSavingMesh = true;

		// Pause integration while we're saving
		bool wasPaused = m_params.m_bPauseIntegration;
		m_params.m_bPauseIntegration = true;
		m_processor.SetParams(m_params);

		INuiFusionColorMesh *mesh = nullptr;
		HRESULT hr = m_processor.CalculateMesh(&mesh);

		if (SUCCEEDED(hr))
		{
			// Save mesh
			hr = SaveMeshFile(mesh, m_saveMeshFormat);

			if (SUCCEEDED(hr))
			{
			SetStatusMessage(L"Saved Kinect Fusion mesh.");
			}
			else if (HRESULT_FROM_WIN32(ERROR_CANCELLED) == hr)
			{
			SetStatusMessage(L"Mesh save canceled.");
			}
			else
			{
			SetStatusMessage(L"Error saving Kinect Fusion mesh!");
			}

			// Release the mesh
			SafeRelease(mesh);
		}
		else
		{
			SetStatusMessage(L"Failed to create mesh of reconstruction.");
		}

		// Restore pause state of integration
		m_params.m_bPauseIntegration = wasPaused;
		m_processor.SetParams(m_params);

		m_bSavingMesh = false;
	}

	// If it was the mesh button clicked, mesh the volume and save
	if (IDC_BUTTON_OPEN_FILE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		OpenMeshFile();

		//CPclHelper pcl;
		//std::string filePath;
		//std::string extension;

		//SetStatusMessage(L" Loading Point Cloud...");

		//filePath = "KinectFusionReconstruction.ply"; //CT2A(pszFilePath);
		//extension = filePath.substr(filePath.find_last_of(".") + 1);

		//if (extension == "pcd"){
		//	pcl.ReadPcdFile(filePath, pointcloud);
		//	SetStatusMessage(L" Done!...");
		//}
		//else if (extension == "ply"){
		//	pcl.ReadPlyFile(filePath, pointcloud);
		//	SetStatusMessage(L" Done!...");
		//}
		//else 
		//	SetStatusMessage(L" Wrong Format...");


		
	}
	if (IDC_BUTTON_RECONSTRUCT == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		ReconstructMesh();
	}
	if (IDC_BUTTON_SAVE_MESH == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		SaveMesh(polymesh, m_params.m_saveMeshType);
		/*CPclHelper pcl;
		const std::string filePath = "last_ply_saved.ply";
		pcl.SaveMeshAsPLY(filePath, polymesh);*/
	}
	if (IDC_CHECK_PAUSE_INTEGRATION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle the pause state of the reconstruction
		m_params.m_bPauseIntegration = !m_params.m_bPauseIntegration;
	}
	if (IDC_VPM_768 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelsPerMeter = 768.0f;
	}
	if (IDC_VPM_640 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelsPerMeter = 640.0f;
	}
	if (IDC_VPM_512 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelsPerMeter = 512.0f;
	}
	if (IDC_VPM_384 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelsPerMeter = 384.0f;
	}
	if (IDC_VPM_256 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelsPerMeter = 256.0f;
	}
	if (IDC_VPM_128 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelsPerMeter = 128.0f;
	}
	if (IDC_VOXELS_X_640 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountX = 640;
	}
	if (IDC_VOXELS_X_512 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountX = 512;
	}
	if (IDC_VOXELS_X_384 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountX = 384;
	}
	if (IDC_VOXELS_X_256 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountX = 256;
	}
	if (IDC_VOXELS_X_128 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountX = 128;
	}
	if (IDC_VOXELS_Y_640 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountY = 640;
	}
	if (IDC_VOXELS_Y_512 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountY = 512;
	}
	if (IDC_VOXELS_Y_384 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountY = 384;
	}
	if (IDC_VOXELS_Y_256 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountY = 256;
	}
	if (IDC_VOXELS_Y_128 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountY = 128;
	}
	if (IDC_VOXELS_Z_640 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountZ = 640;
	}
	if (IDC_VOXELS_Z_512 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountZ = 512;
	}
	if (IDC_VOXELS_Z_384 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountZ = 384;
	}
	if (IDC_VOXELS_Z_256 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountZ = 256;
	}
	if (IDC_VOXELS_Z_128 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountZ = 128;
	}
	if (IDC_MESH_FORMAT_STL_RADIO == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_saveMeshFormat = Stl;
	}
	if (IDC_MESH_FORMAT_OBJ_RADIO == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_saveMeshFormat = Obj;
	}
	if (IDC_MESH_FORMAT_PLY_RADIO == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_saveMeshFormat = Ply;
	}
	if (IDC_KERNEL_WIDTH_NONE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_cSmoothingKernelWidth = 0;
	}
	if (IDC_KERNEL_WIDTH_3X3 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_cSmoothingKernelWidth = 1;
	}
	if (IDC_KERNEL_WIDTH_5X5 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_cSmoothingKernelWidth = 2;
	}
	if (IDC_KERNEL_WIDTH_7X7 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_cSmoothingKernelWidth = 3;
	}
	if (IDC_DOWNSAMPLE_FACTOR_1 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_cAlignPointCloudsImageDownsampleFactor = 1;
	}
	if (IDC_DOWNSAMPLE_FACTOR_2 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_cAlignPointCloudsImageDownsampleFactor = 2;
	}
	if (IDC_DOWNSAMPLE_FACTOR_4 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_cAlignPointCloudsImageDownsampleFactor = 4;
	}
	if (IDC_CHECK_VOXEL_GRID == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_bVoxelGridFilter = !m_params.m_bVoxelGridFilter;
	}
	if (IDC_CHECK_OUTLIERS_REMOVAL == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_bOutliersRemoval = !m_params.m_bOutliersRemoval;
	}
	if (IDC_CHECK_PASS_THROUGH == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_bPassthroughFilter = !m_params.m_bPassthroughFilter;
	}
	if (IDC_CHECK_BILATERAL == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_bBilateralFilter = !m_params.m_bBilateralFilter;
	}
	if (IDC_CHECK_FAST_BILATERAL == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_bFastBilateralFilter = !m_params.m_bFastBilateralFilter;
	}
	if (IDC_CHECK_CROPBOX == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_bCropBoxFilter = !m_params.m_bCropBoxFilter;
	}
	if (IDC_RECONSTRUCTION_POISSON == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_ReconstructionAlgorithm = Poisson;
	}
	if (IDC_RECONSTRUCTION_GREEDY_TRIANGULATION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_ReconstructionAlgorithm = GreedyTriangulation;
	}
	if (IDC_RECONSTRUCTION_MARCHING_CUBES == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_ReconstructionAlgorithm = MarchingCubes;
	}
	if (IDC_CHECK_SHOW_CLOUD == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_bDisplayCloud = !m_bDisplayCloud;
	}
	if (IDC_CHECK_CLOUD_WITH_NORMALS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_bDisplayCloudWithNormals = !m_bDisplayCloudWithNormals;
	}
	if (IDC_CHECK_SHOW_MESH == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_bDisplayMesh = !m_bDisplayMesh;
	}
	if (IDC_CHECK_PROCESSED_MESH == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_bDisplayProcessedMesh = !m_bDisplayProcessedMesh;
	}
	m_processor.SetParams(m_params);
}

/// <summary>
/// Update the internal variable values from the UI Horizontal sliders.
/// </summary>
void CKinectFusionExplorer::UpdateHSliders()
{
	int mmMinPos = (int)SendDlgItemMessage(m_hWnd, IDC_SLIDER_DEPTH_MIN, TBM_GETPOS, 0, 0);

	if (mmMinPos >= MIN_DEPTH_DISTANCE_MM && mmMinPos <= MAX_DEPTH_DISTANCE_MM)
	{
		m_params.m_fMinDepthThreshold = (float)mmMinPos * 0.001f;
	}

	int mmMaxPos = (int)SendDlgItemMessage(m_hWnd, IDC_SLIDER_DEPTH_MAX, TBM_GETPOS, 0, 0);

	if (mmMaxPos >= MIN_DEPTH_DISTANCE_MM && mmMaxPos <= MAX_DEPTH_DISTANCE_MM)
	{
		m_params.m_fMaxDepthThreshold = (float)mmMaxPos * 0.001f;
	}

	int maxWeight = (int)SendDlgItemMessage(m_hWnd, IDC_INTEGRATION_WEIGHT_SLIDER, TBM_GETPOS, 0, 0);
	float smoothingDistance = (float)SendDlgItemMessage(m_hWnd, IDC_SMOOTHING_DISTANCE_THRESHOLD, TBM_GETPOS, 0, 0);
	
	m_params.m_cMaxIntegrationWeight = maxWeight % (MAX_INTEGRATION_WEIGHT + 1);
	m_params.m_fSmoothingDistanceThreshold = smoothingDistance * 0.001f;

	// update text
	WCHAR str[MAX_PATH];
	swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", m_params.m_fMinDepthThreshold);
	SetDlgItemText(m_hWnd, IDC_MIN_DIST_TEXT, str);
	swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", m_params.m_fMaxDepthThreshold);
	SetDlgItemText(m_hWnd, IDC_MAX_DIST_TEXT, str);

	swprintf_s(str, ARRAYSIZE(str), L"%u", m_params.m_cMaxIntegrationWeight);
	SetDlgItemText(m_hWnd, IDC_INTEGRATION_WEIGHT_TEXT, str);

	m_processor.SetParams(m_params);
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
HRESULT CKinectFusionExplorer::StartScanner()
{
	HRESULT hr = S_OK;

	// Init Direct2D
	D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

	int width = m_params.m_cDepthWidth;
	int height = m_params.m_cDepthHeight;
	// Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
	// We'll use this to draw the data we receive from the Kinect to the screen
	if (m_pDrawDepth != nullptr)
		m_pDrawDepth = nullptr;
	m_pDrawDepth = new ImageRenderer();
	hr = m_pDrawDepth->Initialize(
		GetDlgItem(m_hWnd, IDC_DEPTH_VIEW),
		m_pD2DFactory,
		width,
		height,
		width * sizeof(long));

	if (FAILED(hr))
	{
		SetStatusMessage(L"Failed to initialize the Direct2D draw device.");
		m_bInitializeError = true;
	}

	m_pDrawReconstruction = new ImageRenderer();
	hr = m_pDrawReconstruction->Initialize(
		GetDlgItem(m_hWnd, IDC_RECONSTRUCTION_VIEW),
		m_pD2DFactory,
		width,
		height,
		width * sizeof(long));

	if (FAILED(hr))
	{
		SetStatusMessage(L"Failed to initialize the Direct2D draw device.");
		m_bInitializeError = true;
	}

	m_pDrawTrackingResiduals = new ImageRenderer();
	hr = m_pDrawTrackingResiduals->Initialize(
		GetDlgItem(m_hWnd, IDC_TRACKING_RESIDUALS_VIEW),
		m_pD2DFactory,
		width,
		height,
		width * sizeof(long));

	if (FAILED(hr))
	{
		SetStatusMessage(L"Failed to initialize the Direct2D draw device.");
		m_bInitializeError = true;
	}

	if (FAILED(m_processor.SetWindow(m_hWnd, WM_FRAMEREADY, WM_UPDATESENSORSTATUS)) ||
		FAILED(m_processor.SetParams(m_params)) || m_processor.StartProcessing())
	{
		m_bInitializeError = true;
	}

	return hr;
}
/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
void CKinectFusionExplorer::SetStatusMessage(const WCHAR * szMessage)
{
	size_t length = 0;
	if (FAILED(StringCchLength(
		szMessage,
		KinectFusionProcessorFrame::StatusMessageMaxLen,
		&length)))
	{
		length = 0;
	}

	if (length > 0)
	{
		SendDlgItemMessageW(m_hWnd, IDC_STATUS, WM_SETTEXT, 0, (LPARAM)szMessage);
		m_tickLastStatus = GetTickCount64();
	}
	else
	{
		// Clear the status message after a timeout (as long as frames are flowing)
		if (GetTickCount64() - m_tickLastStatus > cStatusTimeoutInMilliseconds &&
			m_fFramesPerSecond > 0)
		{
			SendDlgItemMessageW(m_hWnd, IDC_STATUS, WM_SETTEXT, 0, 0);
			m_tickLastStatus = GetTickCount64();
		}
	}
}

/// <summary>
/// Set the frames-per-second message
/// </summary>
/// <param name="fFramesPerSecond">current frame rate</param>
void CKinectFusionExplorer::SetFramesPerSecond(float fFramesPerSecond)
{
	if (fFramesPerSecond != m_fFramesPerSecond)
	{
		m_fFramesPerSecond = fFramesPerSecond;
		WCHAR str[MAX_PATH] = { 0 };
		if (fFramesPerSecond > 0)
		{
			swprintf_s(str, ARRAYSIZE(str), L"%5.2f FPS", fFramesPerSecond);
		}

		SendDlgItemMessageW(m_hWnd, IDC_FRAMES_PER_SECOND, WM_SETTEXT, 0, (LPARAM)str);
	}
}
