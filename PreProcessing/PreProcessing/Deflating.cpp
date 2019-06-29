/******************************************
点云分割去平面去分离聚类
2019.05.22 ChenyangCheung
*******************************************/
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <stdio.h>
#include <io.h>
#include <Windows.h>
#include <direct.h> 


int cluster_extraction(int counter, char filename[]);
void savePCD(int counter, char filename[]);

int main(void)
{
	mkdir("Cattle/Q0206");
	mkdir("Cattle/after");
	int cloud_counter = 0;
	int cluster_counter = 0;

	dir = "Cattle/Q0206/*.pcd";
	if ((lfDir = _findfirst(dir, &fileDir)) == -1l)
		printf("No file is found\n");
	else {
		do {
			cluster_counter = cluster_extraction(cluster_counter, fileDir.name);
		} while (_findnext(lfDir, &fileDir) == 0);
	}
	_findclose(lfDir);


	dir = "Cattle/after/*.pcd";
	if ((lfDir = _findfirst(dir, &fileDir)) == -1l)
		printf("No file is found\n");
	else {
		do {
			printf("%s\n", fileDir.name);
			//cloud_counter++;

			char dir[200];
			char* dir1 = "Cattle/after/";
			sprintf(dir, "%s%s", dir1, fileDir.name);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

			if (pcl::io::loadPCDFile(dir, *cloud))
			{
				std::cout << "error";
			}

			pcl::visualization::PCLVisualizer viewer(dir);

			viewer.addPointCloud(cloud, "cattle");
			viewer.setSize(300, 200);


			while (!viewer.wasStopped())
			{
				viewer.spinOnce();
			}

		} while (_findnext(lfDir, &fileDir) == 0);
	}
	_findclose(lfDir);




	return (0);

}


int cluster_extraction(int counter, char filename[])
{
	char dir[200];
	char* dir1 = "Cattle/Q0206/";
	char* dir2 = filename;
	sprintf(dir, "%s%s", dir1, dir2);

	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read(dir, *cloud);
	std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl;

	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.03);

	int i = 0, nr_points = (int)cloud_filtered->points.size();
	while (cloud_filtered->points.size() > 0.8 * nr_points)
	{

		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);


		extract.filter(*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloud_filtered = cloud_f;
	}



	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.05);
	ec.setMinClusterSize(30);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);


	int j = counter;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Ddefalt: " << cloud_cluster->points.size() << " data points." << std::endl;
		std::stringstream ss;
		ss << "Cattle/after/cloud_after_" << j << ".pcd";
		writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false);
		std::cout << "save " << ss.str() << std::endl;
		j++;
	}

	return (j);
}
