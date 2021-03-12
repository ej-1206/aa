/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "quiz/cluster/kdtree.h"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer); 
    
    Lidar* lidar = new Lidar(cars,0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan(); 

    renderPointCloud(viewer,inputCloud,"ej");
}

void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    viewer->initCameraParameters();

    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}





void clusterHelper(int indice, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
	processed[indice] = true;  
	cluster.push_back(indice);

	std::vector<int> nearest = tree->search(points[indice], distanceTol); 
	
	for(int id : nearest)
	{
		if(!processed[id])
		    clusterHelper(id, points, cluster, processed, tree, distanceTol);
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>> points, KdTree* tree, float distanceTol)
{



	std::vector<std::vector<int>> clusters;

	std::vector<bool> processed(points.size(),false); 

	int i = 0;
	while(i < points.size())
	{
		if(processed[i])             
		{
			i++;
			continue;
		}
		std::vector<int> cluster;    
		clusterHelper(i, points, cluster, processed, tree, distanceTol); 
        if (cluster.size() > 10 && cluster.size() < 300)
        {
            clusters.push_back(cluster); 
		    i++;
        }
		
	}
 
	return clusters;

}

pcl::PointCloud<pcl::PointXYZI>::Ptr CreateData(std::vector<std::vector<float>> points)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZI point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = points[i][2];

  		cloud->points.push_back(point); 

  	}
  	cloud->width = cloud->points.size(); 
  	cloud->height = 1;

  	return cloud;

}

std::unordered_set<int> Ransac3d(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult; 
	srand(time(NULL));
	
	while(maxIterations--)
	{
		std::unordered_set<int> inliers;
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
			pcl::PointXYZI point = cloud->points[index];	
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

/*void Project(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    auto startTime = std::chrono::steady_clock::now();
    ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    ProcessPointClouds<pcl::PointXYZ> pointProcessor1;
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessor.FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-11,-5,-3,1), Eigen::Vector4f(25,7,1,1));

    std::unordered_set<int> inliers = Ransac3d(filterCloud, 100, 0.3);

    pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());
    std::vector<std::vector<float>> points;
	for(int index = 0; index < filterCloud->points.size(); index++)
	{
		pcl::PointXYZI point = filterCloud->points[index];
		if(inliers.count(index))
        {
            cloudInliers->points.push_back(point);
        }
			
		else
        {
            cloudOutliers->points.push_back(point);
            std::vector<float> please;
            please.push_back(point.x);
            please.push_back(point.y);
            please.push_back(point.z);

            points.push_back(std::vector<float>(please));
        }

	}

    KdTree* tree = new KdTree;  
    for (int i=0; i<points.size(); i++)          
    	tree->insert(points[i],i);  

    std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 0.53);

    auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

    int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> CLUSTER;  
    for(std::vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int indice: cluster)
            clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],points[indice][2]));
  		CLUSTER.push_back(clusterCloud);   
  		++clusterId;
  	}

    int CLUSTERID=0;
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : CLUSTER)                       
    {
        std::cout << "cluster size";
        //pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "차량 장애물"+std::to_string(CLUSTERID),colors[CLUSTERID%colors.size()]); 
        Box box = pointProcessor1.BoundingBox(cluster);
        renderBox(viewer,box,CLUSTERID);   
        ++CLUSTERID;
    }

    renderPointCloud(viewer, cloudInliers, "planeCloud", Color(1,1,1));   


    if(clusters.size()==0)
  		renderPointCloud(viewer,filterCloud,"data");
        
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}  
      

}
*/

void Project_Streaming(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcessor, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
    ProcessPointClouds<pcl::PointXYZ> pointProcessor_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessor.FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-10,-5,-2,1), Eigen::Vector4f (30,8,1,1)); 

    //////////////////////////// Use Ransac3d (learned at class) ////////////////////////////
    std::unordered_set<int> inliers = Ransac3d(filterCloud, 100, 0.3);

    pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());
    std::vector<std::vector<float>> points;
	for(int index = 0; index < filterCloud->points.size(); index++)
	{
		pcl::PointXYZI point = filterCloud->points[index];
		if(inliers.count(index))
        {
            cloudInliers->points.push_back(point);
        }
			
		else
        {
            cloudOutliers->points.push_back(point);
            std::vector<float> please;
            please.push_back(point.x);
            please.push_back(point.y);
            please.push_back(point.z);

            points.push_back(std::vector<float>(please));
        }

	}

    //////////////////////////// Use KD-Tree and EuclideanCluster learned at class ////////////////////////////

    KdTree* tree = new KdTree;  
    for (int i=0; i<points.size(); i++)          
    	tree->insert(points[i],i);  

    std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 0.53);

    int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> CLUSTER;  
    for(std::vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int indice: cluster)
            clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],points[indice][2]));
  		CLUSTER.push_back(clusterCloud);   
  		++clusterId;
  	}

    int CLUSTERID=0;
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : CLUSTER)                       
    {
        renderPointCloud(viewer, cluster, "차량 장애물"+std::to_string(CLUSTERID),colors[CLUSTERID%colors.size()]); 
        Box box = pointProcessor_.BoundingBox(cluster);
        renderBox(viewer,box,CLUSTERID);   
        ++CLUSTERID;
    }


    renderPointCloud(viewer, cloudInliers, "planeCloud", Color(1,1,1)); 
}

void cityBlock_Streaming(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcessor, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
    inputCloud = pointProcessor.FilterCloud(inputCloud, 0.3, Eigen::Vector4f (-10,-5,-2,1), Eigen::Vector4f (30,8,1,1)); 
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 25, 0.3); 
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudCluster = pointProcessor.Clustering(segmentCloud.first, 0.53, 10, 500);  
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)}; // 색상 지정 빨 파 초 분
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudCluster)                       
    {
        std::cout << "cluster size";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "차량 장애물"+std::to_string(clusterId),colors[clusterId%colors.size()]); 
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);   
        ++clusterId;
    }
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0)); //G
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    //Project(viewer);


    ProcessPointClouds<pcl::PointXYZI> pointProcessorI; 
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src//sensors/data/pcd/data_2"); 
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;  

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds(); 
        viewer->removeAllShapes();

        inputCloudI = pointProcessorI.loadPcd((*streamIterator).string()); 
        Project_Streaming(viewer, pointProcessorI, inputCloudI);
        //cityBlock_Streaming(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
        {
            streamIterator = stream.begin();
        }
        viewer->spinOnce();
    }
    
    

}

