/******************************************
统计、体素、条件。条件滤波修改参数
2019.05.21 ChenyangCheung
*******************************************/
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

int show_cloud(char filename1[], char filename2[]);
void Conditional_filter(char dir[], char con_dir[]);
void VG_filter(char dir[], char vg_dir[]);
void SOR_filter(char dir[], char SOR_dir[]);

int main(void)
{
	char dir[] = "Q0206_10.pcd";

	char conditional_filtered_dir[200];
	sprintf(conditional_filtered_dir, "%s%s", "Conditional_filtered_", dir);

	char SOR_filtered_dir[200];
	sprintf(SOR_filtered_dir, "%s%s", "SOR_filtered_", dir);

	char VG_filtered_dir[200];
	sprintf(VG_filtered_dir, "%s%s", "VG_filtered_", dir);
	Conditional_filter(dir, conditional_filtered_dir);
	SOR_filter(conditional_filtered_dir, SOR_filtered_dir);
	VG_filter(SOR_filtered_dir, VG_filtered_dir);

	show_cloud(dir, VG_filtered_dir);

	return 0;
}

void Conditional_filter(char dir[], char con_dir[])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCDReader reader;
	reader.read(dir, *cloud);

	std::cerr << "PointCloud before Conditional Removal filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());

	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
		pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 2.629)));   //Z 大于
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
		pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 3.128)));  //Z 小于

	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
		pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -0.761)));//Y 大于
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
		pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 1.085)));  //Y 小于

	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
		pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, -1.227)));  //X 大于
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
		pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 1.531)));  //X 小于

	pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
	condrem.setCondition(range_cond);
	condrem.setInputCloud(cloud);
	condrem.setKeepOrganized(true);
	condrem.filter(*cloud_filtered);


	std::cerr << "PointCloud after Conditional Removal filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;
	pcl::io::savePCDFileASCII(con_dir, *cloud_filtered);
}

void SOR_filter(char dir[], char sor_dir[])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCDReader reader;
	reader.read(dir, *cloud);
	std::cerr << "PointCloud before Statistical Outlier Removal filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(140);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);
	std::cerr << "PointCloud after Statistical Outlier Removal filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;
	sor.filter(*cloud_filtered);
	pcl::io::savePCDFileASCII(sor_dir, *cloud_filtered);
}

void VG_filter(char dir[], char vg_dir[])
{
	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr VG_cloud_filtered_blob(new pcl::PCLPointCloud2());
	pcl::PointCloud<pcl::PointXYZ>::Ptr VG_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	reader.read(dir, *cloud_blob);
	std::cerr << "PointCloud before Voxel Grid Removal filtering: " << cloud_blob->width * cloud_blob->height
		<< " data points (" << pcl::getFieldsList(*cloud_blob) << ")." << std::endl;

	pcl::VoxelGrid<pcl::PCLPointCloud2> VG_sor;
	VG_sor.setInputCloud(cloud_blob);
	VG_sor.setLeafSize(0.01f, 0.01f, 0.01f);
	VG_sor.filter(*VG_cloud_filtered_blob);
	std::cerr << "PointCloud after filtering: " << VG_cloud_filtered_blob->width * VG_cloud_filtered_blob->height
		<< " data points (" << pcl::getFieldsList(*VG_cloud_filtered_blob) << ")." << std::endl;

	pcl::fromPCLPointCloud2(*VG_cloud_filtered_blob, *VG_cloud_filtered);
	pcl::io::savePCDFileASCII(vg_dir, *VG_cloud_filtered);
}


int show_cloud(char dir[], char dir_filter[])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	FILE *stream1, *stream2;
	stream1 = fopen(dir, "r");
	stream2 = fopen(dir_filter, "r");
	if (stream2 == NULL || stream2 == NULL)
		printf("The file was not opened\n");
	else
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		if (pcl::io::loadPCDFile(dir, *cloud))
		{
			std::cout << "error";
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

		if (pcl::io::loadPCDFile(dir_filter, *cloud_filtered))
		{
			std::cout << "error";
		}

		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer->initCameraParameters();
		int v1(0);
		viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer->setBackgroundColor(0.3, 0.3, 0.3, v1);
		viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
		viewer->addPointCloud<pcl::PointXYZ>(cloud, "befor filter", v1);

		int v2(0);
		viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer->setBackgroundColor(0, 0, 0, v2);
		viewer->addText("Radius: 0.1", 20, 20, "v2 text", v2);
		viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "after filter", v2);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "befor filter");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "after filter");

		while (!viewer->wasStopped())
		{
			viewer->spinOnce();
		}
	}
	return 0;
}