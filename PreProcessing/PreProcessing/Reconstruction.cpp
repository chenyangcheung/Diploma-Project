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
	data->viewerPtr->spinOnce();//ˢ�´��ڣ���ȷ

	PointCloudT::Ptr line(new PointCloudT);
	line->points.push_back(last_point);
	line->points.push_back(current_point);
	data->viewerPtr->addPointCloud(line, red, "clicked_points");
	data->viewerPtr->addArrow(last_point, current_point, 255, 0, 0, 0, 0, 0, "ab");
	data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");//���õ��С
	std::cout << "A: " << last_point.x << " " << last_point.y << " " << last_point.z << std::endl;
	std::cout << "B: " << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
	float distance;
	distance = sqrt((last_point.x - current_point.x)*(last_point.x - current_point.x) + (last_point.y - current_point.y)*(last_point.y - current_point.y) + (last_point.z - current_point.z)*(last_point.z - current_point.z));
	std::cout << "AB Distance: " << distance << std::endl;
	last_point = current_point;


}

int main()
{
	/*�������²�����ƽ����*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile("Q0121_35.pcd", *cloud0);



	pcl::VoxelGrid<pcl::PointXYZ> grid; //VoxelGrid ��һ�������ĵ��ƣ��ۼ���һ���ֲ���3D������,���²������˲���������

	grid.setLeafSize(0.025, 0.025, 0.025); //������Ԫ�����Ҷ�Ӵ�С
										   //�²��� Դ����
	grid.setInputCloud(cloud0); //�����������
	grid.filter(*cloud0); //�²������˲�

						  // ���� KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree0(new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal> mls_points;

	// ������С����ʵ�ֵĶ���mls
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	mls.setComputeNormals(true);  //��������С���˼�������Ҫ���з��߹���

								  // Set parameters
	mls.setInputCloud(cloud0);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree0);
	mls.setSearchRadius(0.2); // ��������

							  // Reconstruct
	mls.process(mls_points);

	// Save output
	pcl::io::savePCDFile("VG_filtered_cow001_cloud_10-mls.pcd", mls_points);
	//pcl::io::savePCDFile("cloud_cluster_166_zCut-mls.pcd", mls_points);
	//pcl::io::savePCDFile("cloud_240_xzCut_filtered-mls.pcd", mls_points);






	/*��������ģ��*/
	// ����ģ�Ͷ���,�˴�����ΪPCD��ʽ�����ļ�.��������ΪPointXYZ.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("VG_filtered_cow001_cloud_10-mls.pcd", *cloud) == -1)
		//if (pcl::io::loadPCDFile<pcl::PointXYZ>("cloud_cluster_166_zCut-mls.pcd", *cloud) == -1)
		//if (pcl::io::loadPCDFile<pcl::PointXYZ>("cloud_240_xzCut_filtered-mls.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file mypointcloud.pcd\n");  //����ȡʧ�ܽ���ʾ
		return -1;
	}
	std::cerr << "���ƶ���   ���" << std::endl;


	/*�������ģ��*/
	// Normal estimation�����������ƣ�
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//����������ƶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//������������ָ��
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//����kdtree���ڷ������ʱ��������
	tree->setInputCloud(cloud);//Ϊkdtree�������
	n.setInputCloud(cloud);//Ϊ������ƶ����������
	n.setSearchMethod(tree);//���÷������ʱ���õ�������ʽΪkdtree
							//n.setRadiusSearch(0.1);//���÷������ʱ,�����뾶
	n.setKSearch(20);//���÷������ʱ,k���������ĵ���
	n.compute(*normals);  //���з������

	std::cerr << "���߼���   ���" << std::endl;

	/*���������뷨������ƴ��*/
	// ����ͬʱ������ͷ�������ݽṹ��ָ��
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	//���ѻ�õĵ����ݺͷ�������ƴ��
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);


	// ������һ��kdtree�����ؽ�
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	//Ϊkdtree�����������,�õ�����������Ϊ��ͷ���
	tree2->setInputCloud(cloud_with_normals);

	/*�����ؽ�ģ��*/
	// ����̰��������ͶӰ�ؽ�����
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//����������������,�����洢�ؽ����
	pcl::PolygonMesh triangles;

	//���ò���
	gp3.setSearchRadius(250);  // �������ӵ�֮��������루���߳�������ȷ��k���ڵ���뾶��Ĭ��Ϊ0����ֵԽС��Ƭ������ԽС
	gp3.setMu(3);  // ��������ھ���ĳ��ӣ��ѵõ�ÿ��������������뾶��Ĭ��Ϊ0����ֵԽС��Ƭ������ԽС
	gp3.setMaximumNearestNeighbors(100);  //��������������ڵ���������
	gp3.setMaximumSurfaceAngle(M_PI / 2); // 45 degrees ���ƽ���
	gp3.setMinimumAngle(M_PI / 36); // 5 degrees ÿ�����ǵ����Ƕ�
	gp3.setMaximumAngle(5 * M_PI / 6); // 150 degrees
	gp3.setNormalConsistency(false);  //��������һ�£���Ϊtrue
									  // ���õ������ݺ�������ʽ
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	//��ʼ�ؽ�
	gp3.reconstruct(triangles);
	std::cerr << "�ؽ�   ���" << std::endl;


	//���ؽ�������浽Ӳ���ļ���,�ؽ������VTK��ʽ�洢
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
	fs << "��������Ϊ��" << parts.size() << "\n";
	for (int i = 0; i < parts.size(); i++)
	{
		if (parts[i] != 0)
		{
			fs << parts[i] << "\n";   //���fs����
		}
	}


	//ͼ����ʾģ��
	//������ʾ����ָ��
	std::cerr << "��ʼ��ʾ ........" << std::endl;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0.6);  //���ô�����ɫ
	viewer->addPolygonMesh(triangles, "my");  //������Ҫ��ʾ���������
											  //��������ģ����ʾģʽ
	viewer->setRepresentationToSurfaceForAllActors(); //����ģ������Ƭ��ʽ��ʾ  
													  //viewer->setRepresentationToPointsForAllActors(); //����ģ���Ե���ʽ��ʾ  
													  //viewer->setRepresentationToWireframeForAllActors();  //����ģ�����߿�ͼģʽ��ʾ

	cloud_mutex.lock();    // for not overwriting the point cloud


	viewer->addCoordinateSystem(0.01);  //��������ϵ,����Ϊ������ʾ�ߴ�
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