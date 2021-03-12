/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}
// 내가 해본거어어어ㅓ....................................
std::unordered_set<int> Ransac3d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	// Time Segmentation process
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult; // 가장 높은 inlier수가 여기에 있다. // unordered set 순서가 지정되지 않은 세트
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers // 반복문 사용
	while(maxIterations--)
	{
		// 무작위로 점 두개 선택
		std::unordered_set<int> inliers; // unordered inlier set는 int이다.
		while(inliers.size()<3)
            inliers.insert(rand()%(cloud->points.size()));
        float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		auto itr = inliers.begin();
        x1 = cloud->points[*itr].x; 
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
       itr++;     
       x2 = cloud->points[*itr].x; 
       y2 = cloud->points[*itr].y;
       z2 = cloud->points[*itr].z;
       itr++; 
       x3 = cloud->points[*itr].x; 
       y3 = cloud->points[*itr].y;
       z3 = cloud->points[*itr].z;

       float A = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
       float B = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
       float C = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
       float D = -A*x1 -B*y1 -C*z1;

		for(int index=0; index<cloud->points.size(); index++)
		{
			if(inliers.count(index)>0) 
			    continue;
			pcl::PointXYZ point = cloud->points[index];	
			float x3 = point.x;
			float y3 = point.y;
			float z3 = point.z;
			
			float d = fabs(A*x3 + B*y3 + C*z3 + D) / sqrt(A*A + B*B + C*C); 
			if(d <= distanceTol) 
			    inliers.insert(index); 
	
		}
		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers; 
		} 
	}
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "Ransac took" << elapsedTime.count() << "milliseconds" << std::endl;
	
	return inliersResult;

}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	// Time Segmentation process
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult; // 가장 높은 inlier수가 여기에 있다. // unordered set 순서가 지정되지 않은 세트
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers // 반복문 사용
	while(maxIterations--)
	{
		// 무작위로 점 두개 선택
		std::unordered_set<int> inliers; // unordered inlier set는 int이다.
		while(inliers.size() < 2)
		    inliers.insert(rand()%(cloud->points.size())); // inlier의 수가 2미만이 될때까지 무작위로 point 삽입 // cloud->points.size()는 클라우드가 가진 포인트의 수

		float x1, y1, x2, y2;

		auto itr = inliers.begin(); // 첫번째 값이 무엇인지 보기 위해 // begin()은 포인터, inlier의 맨 처음을 가리킨다.
		x1 = cloud->points[*itr].x; // 포인터를 역참조하여 값 확인 // point에 대한 .x 값 // [*itr]은 인덱스... 클라우드의 해당 인덱스를 가지고 오는것
		y1 = cloud->points[*itr].y; 
		itr++; // 반복을 위해
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y; // 무작위로 선택한 inlier의 x1 y1 x2 y2 값

		float a = (y1-y2);
		float b = (x2- x1);
		float c = (x1*y2-x2*y1);

		for(int index=0; index<cloud->points.size(); index++) // index는 0으로 시작해서 포인트 크기 될때까지 반복
		{
			if(inliers.count(index)>0) // 선택한 두 점(직선으로 만든 점) 이면 
			    continue;
			pcl::PointXYZ point = cloud->points[index];	// 아니면 클라우드로 인덱싱하여 지점 알아내기 (index를 이용해서 pxl PointXYZ 포인트 얻어서, x y 값 알아낸다)
			float x3 = point.x;
			float y3 = point.y;
			
			float d = fabs(a*x3+b*y3+c) / sqrt(a*a + b*b); // 직선의 방정식 이용해서 거리계산 fabs
			if(d <= distanceTol) // 해당 임계값 내에 있는지 확인 후
			    inliers.insert(index); //삽입
	
		}
		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers; // inlierResult 갱신
		} 
	}
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "Ransac took" << elapsedTime.count() << "milliseconds" << std::endl;
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	//pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 25, 1.0); //원래 이거임 ㅜㅜ

	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	std::unordered_set<int> inliers = Ransac3d(cloud, 100, 0.3);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
