/******************************************
GreedyProjectionTriangulation from Anqi Zhu
2019.05.24 ChenyangCheung
*******************************************/
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h> 
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>       



#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Mutex: //
boost::mutex cloud_mutex;

PointT last_point;
PointT temp_point;

struct callback_args {
	// structure used to pass arguments to the callback function
	PointCloudT::Ptr clicked_points_3d;
	pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};


void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
	struct callback_args* data = (struct callback_args *)args;
	if (event.getPointIndex() == -1)
		return;

	PointT current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z);
	data->clicked_points_3d->points.push_back(current_point);
	// Draw clicked points in red:
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
	data->viewerPtr->removePointCloud("clicked_points");
	//data->viewerPtr->removeShape("ab");
	data->viewerPtr->removeAllShapes();
	//data->viewerPtr->removePointCloud("abArrow");
	data->viewerPtr->spinOnce();//刷新窗口，正确

	PointCloudT::Ptr line(new PointCloudT);
	line->points.push_back(last_point);
	line->points.push_back(current_point);
	data->viewerPtr->addPointCloud(line, red, "clicked_points");
	data->viewerPtr->addArrow(last_point, current_point, 255, 0, 0, 0, 0, 0, "ab");
	data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");//设置点大小
	std::cout << "A: " << last_point.x << " " << last_point.y << " " << last_point.z << std::endl;
	std::cout << "B: " << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
	float distance;
	distance = sqrt((last_point.x - current_point.x)*(last_point.x - current_point.x) + (last_point.y - current_point.y)*(last_point.y - current_point.y) + (last_point.z - current_point.z)*(last_point.z - current_point.z));
	std::cout << "AB Distance: " << distance << std::endl;
	last_point = current_point;


}

int main()
{
	/*点云向下采样，平滑化*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile("Q0121_35.pcd", *cloud0);



	pcl::VoxelGrid<pcl::PointXYZ> grid; //VoxelGrid 把一个给定的点云，聚集在一个局部的3D网格上,并下采样和滤波点云数据

	grid.setLeafSize(0.025, 0.025, 0.025); //设置体元网格的叶子大小
										   //下采样 源点云
	grid.setInputCloud(cloud0); //设置输入点云
	grid.filter(*cloud0); //下采样和滤波

						  // 创建 KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree0(new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal> mls_points;

	// 定义最小二乘实现的对象mls
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	mls.setComputeNormals(true);  //设置在最小二乘计算中需要进行法线估计

								  // Set parameters
	mls.setInputCloud(cloud0);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree0);
	mls.setSearchRadius(0.2); // 采样距离

							  // Reconstruct
	mls.process(mls_points);

	// Save output
	pcl::io::savePCDFile("VG_filtered_cow001_cloud_10-mls.pcd", mls_points);
	//pcl::io::savePCDFile("cloud_cluster_166_zCut-mls.pcd", mls_points);
	//pcl::io::savePCDFile("cloud_240_xzCut_filtered-mls.pcd", mls_points);






	/*点云载入模块*/
	// 点云模型读入,此处读入为PCD格式点云文件.数据类型为PointXYZ.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("VG_filtered_cow001_cloud_10-mls.pcd", *cloud) == -1)
		//if (pcl::io::loadPCDFile<pcl::PointXYZ>("cloud_cluster_166_zCut-mls.pcd", *cloud) == -1)
		//if (pcl::io::loadPCDFile<pcl::PointXYZ>("cloud_240_xzCut_filtered-mls.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file mypointcloud.pcd\n");  //若读取失败将提示
		return -1;
	}
	std::cerr << "点云读入   完成" << std::endl;


	/*法向估计模块*/
	// Normal estimation（法向量估计）
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//创建法向估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//创建法向数据指针
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//创建kdtree用于法向计算时近邻搜索
	tree->setInputCloud(cloud);//为kdtree输入点云
	n.setInputCloud(cloud);//为法向估计对象输入点云
	n.setSearchMethod(tree);//设置法向估计时采用的搜索方式为kdtree
							//n.setRadiusSearch(0.1);//设置法向估计时,搜索半径
	n.setKSearch(20);//设置法向估计时,k近邻搜索的点数
	n.compute(*normals);  //进行法向估计

	std::cerr << "法线计算   完成" << std::endl;

	/*点云数据与法向数据拼接*/
	// 创建同时包含点和法向的数据结构的指针
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	//将已获得的点数据和法向数据拼接
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);


	// 创建另一个kdtree用于重建
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	//为kdtree输入点云数据,该点云数据类型为点和法向
	tree2->setInputCloud(cloud_with_normals);

	/*曲面重建模块*/
	// 创建贪婪三角形投影重建对象
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//创建多边形网格对象,用来存储重建结果
	pcl::PolygonMesh triangles;

	//设置参数
	gp3.setSearchRadius(250);  // 设置连接点之间的最大距离（最大边长）用于确定k近邻的球半径（默认为0）数值越小贴片的三角越小
	gp3.setMu(3);  // 设置最近邻距离的乘子，已得到每个点的最终搜索半径（默认为0）数值越小贴片的三角越小
	gp3.setMaximumNearestNeighbors(100);  //设置搜索的最近邻点的最大数量
	gp3.setMaximumSurfaceAngle(M_PI / 2); // 45 degrees 最大平面角
	gp3.setMinimumAngle(M_PI / 36); // 5 degrees 每个三角的最大角度
	gp3.setMaximumAngle(5 * M_PI / 6); // 150 degrees
	gp3.setNormalConsistency(false);  //若法向量一致，设为true
									  // 设置点云数据和搜索方式
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	//开始重建
	gp3.reconstruct(triangles);
	std::cerr << "重建   完成" << std::endl;


	//将重建结果保存到硬盘文件中,重建结果以VTK格式存储
	pcl::io::saveVTKFile("mymesh.vtk", triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();
	fstream fs;
	fs.open("partsID.txt", ios::out);
	if (!fs)
	{
		return -2;
	}
	fs << "点云数量为：" << parts.size() << "\n";
	for (int i = 0; i < parts.size(); i++)
	{
		if (parts[i] != 0)
		{
			fs << parts[i] << "\n";   //这的fs对吗？
		}
	}


	//图形显示模块
	//创建显示对象指针
	std::cerr << "开始显示 ........" << std::endl;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0.6);  //设置窗口颜色
	viewer->addPolygonMesh(triangles, "my");  //设置所要显示的网格对象
											  //设置网格模型显示模式
	viewer->setRepresentationToSurfaceForAllActors(); //网格模型以面片形式显示  
													  //viewer->setRepresentationToPointsForAllActors(); //网格模型以点形式显示  
													  //viewer->setRepresentationToWireframeForAllActors();  //网格模型以线框图模式显示

	cloud_mutex.lock();    // for not overwriting the point cloud


	viewer->addCoordinateSystem(0.01);  //设置坐标系,参数为坐标显示尺寸
	viewer->initCameraParameters();



	last_point.x = 0;
	last_point.y = 0;
	last_point.z = 0;
	// Add point picking callback to viewer:
	struct callback_args cb_args;
	PointCloudT::Ptr clicked_points_3d(new PointCloudT);
	cb_args.clicked_points_3d = clicked_points_3d;
	cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
	viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
	std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

	viewer->spin();
	std::cout << "done." << std::endl;

	cloud_mutex.unlock();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


	// Finish
	return 0;
}