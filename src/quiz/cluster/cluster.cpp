/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);

  	viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");
  	return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZ point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = 0;

  		cloud->points.push_back(point);  // point 삽입

  	}
  	cloud->width = cloud->points.size(); // 클라우드의 크기 설정
  	cloud->height = 1;

  	return cloud;

}


void render2DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0) // 트리 시각화. 매개변수: 노드 뷰어 윈도우 iteration depth
{

	if(node!=NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;
		// split on x axis
		if(depth%2==0) // 짝수, x분할
		{
			viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, 0),pcl::PointXYZ(node->point[0], window.y_max, 0),0,0,1,"line"+std::to_string(iteration)); // 파란선긋기
			lowerWindow.x_max = node->point[0];
			upperWindow.x_min = node->point[0];
		}
		// split on y axis
		else           // 홀수, y분할
		{ 
			viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], 0),pcl::PointXYZ(window.x_max, node->point[1], 0),1,0,0,"line"+std::to_string(iteration));  // 빨간선긋기
			lowerWindow.y_max = node->point[1];
			upperWindow.y_min = node->point[1];
		}
		iteration++;

		render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
		render2DTree(node->right,viewer, upperWindow, iteration, depth+1);


	}

}

// 와 진짜 존나 고소하고싶다 와와와와  std::vector<bool>& processed 이거였음. &안붙여서 결과안나온거임. 코드 다 보여줘야지 장난해? ㅜㅜㅜㅜㅜㅜㅜ
void clusterHelper(int indice, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
	processed[indice] = true;  // 해당 지점 처리중으로 표시
	cluster.push_back(indice); // cluster로 삽입

	std::vector<int> nearest = tree->search(points[indice], distanceTol); // 근처에 있는 포인트 찾기 포인트 인덱스 줘서 11개의 포인트 참조
	
	for(int id : nearest)
	{
		if(!processed[id])
		    clusterHelper(id, points, cluster, processed, tree, distanceTol);
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>> points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;

	std::vector<bool> processed(points.size(),false); // 프로세서 처리 여부 확인 위해 포인트 크기만큼 벡터 생성. // 일단 False로 설정.

	int i = 0;
	while(i < points.size())
	{
		if(processed[i])             // 포인트가 처리되었다면, 아마 i++해서  다음..으로 넘어가라는듯
		{
			i++;
			continue;
		}
		std::vector<int> cluster;    // 포인트가 처리되지 않은 경우 새로운 cluster 생성 (정수 벡터)
		clusterHelper(i, points, cluster, processed, tree, distanceTol); // Proximity 접근성 // 포인트id, 포인트, 클러스터, 포인트 처리 여부, kdtree(주변 포인트 탐색 위해), distanceTol
		clusters.push_back(cluster); // 클러스터 삽입
		i++;
	}
 
	return clusters;

}

/*int main ()
{

	// Create viewer
	Box window;
  	window.x_min = -10;
  	window.x_max =  10;
  	window.y_min = -10;
  	window.y_max =  10;
  	window.z_min =   0;
  	window.z_max =   0;
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25); // zoom은 화면크비(비율)

	// Create data
	std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };
	//std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3} };
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points); // 포인트 삽입

	KdTree* tree = new KdTree;                          // KD Tree 생성 // KD Tree 구조 보면 노드도 생성된다.
  
    for (int i=0; i<points.size(); i++)          
    	tree->insert(points[i],i);                      // insert 함수 이용해서 트리에 포인트 삽입

  	int it = 0;
  	render2DTree(tree->root,viewer,window, it);         // 렌더링 된 2D Tree 호출 (it = iteration) 
  
  	//std::cout << "Test Search" << std::endl;
  	//std::vector<int> nearby = tree->search({-6,7},3.0); // 트리 호출 후 검색 // distanceTol이 3.0인  점 -6.7 
  	//for(int index : nearby)
    //  std::cout << index << ",";                        // 인덱스 확인
  	//std::cout << std::endl;

	std::cout << "Test Search" << std::endl;
  	std::vector<int> testSearch = tree->search({-6.0, 6.5}, 3.0); 
  	for(int nearbyID : testSearch)
      std::cout << nearbyID << ",";                       
  	std::cout << std::endl;  

  	// Time segmentation process
  	auto startTime = std::chrono::steady_clock::now();
  	//
  	std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 3.0);
  	//
  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  	// Render clusters
  	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	for(std::vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int indice: cluster)
  			clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],0));
  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
  		++clusterId;
  	}
  	if(clusters.size()==0)
  		renderPointCloud(viewer,cloud,"data");
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
*/


////////////////////////////////////////////////// 3D ///////////////////////////////////////////////////////////


void render3DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0) // 트리 시각화. 매개변수: 노드 뷰어 윈도우 iteration depth
{

	if(node!=NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;
		// split on x axis
		if(depth%3==0) // x분할
		{
			viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, 0),pcl::PointXYZ(node->point[0], window.y_max, 0),0,0,1,"line"+std::to_string(iteration)); // 파란선긋기
			lowerWindow.x_max = node->point[0];
			upperWindow.x_min = node->point[0];
		}
		// split on y axis
		else if(depth%3 == 1) // y분할
		{ 
			viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], 0),pcl::PointXYZ(window.x_max, node->point[1], 0),1,0,0,"line"+std::to_string(iteration));  // 빨간선긋기
			lowerWindow.y_max = node->point[1];
			upperWindow.y_min = node->point[1];
		}
		else
		{
			viewer->addLine(pcl::PointXYZ(window.z_min, node->point[2], 0),pcl::PointXYZ(window.z_max, node->point[2], 0),0,1,0,"line"+std::to_string(iteration));  // 초록선긋기
			lowerWindow.z_max = node->point[2];
			upperWindow.z_min = node->point[2];
		}

		iteration++;

		render3DTree(node->left,viewer, lowerWindow, iteration, depth+1);
		render3DTree(node->right,viewer, upperWindow, iteration, depth+1);


	}

}


int main ()
{

	// Create viewer
	Box window;
  	window.x_min = -100;
  	window.x_max =  100;
  	window.y_min = -100;
  	window.y_max =  100;
  	window.z_min = -100;
  	window.z_max =  100;
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 250); // zoom은 화면크비(비율)

	// Create data
	std::vector<std::vector<float>> points = { {-6.2,7,-10}, {-6.3,8.4,-10}, {-5.2,7.1,20}, {-5.7,6.3,20}, {7.2,6.1,1}, {8.0,5.3,1}, {7.2,7.1,1}, {0.2,-7.1,-2}, {1.7,-6.9,-50}, {-1.2,-7.2,-2}, {2.2,-8.9,-50} };
	//std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3} };
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points); // 포인트 삽입

	KdTree* tree = new KdTree;                          // KD Tree 생성 // KD Tree 구조 보면 노드도 생성된다.
  
    for (int i=0; i<points.size(); i++)          
    	tree->insert(points[i],i);                      // insert 함수 이용해서 트리에 포인트 삽입

  	//int it = 0;
  	//render3DTree(tree->root,viewer,window, it);         // 렌더링 된 2D Tree 호출 (it = iteration) 

	std::cout << "Test Search" << std::endl;
  	std::vector<int> testSearch = tree->search({-6.0, 6.5,-3}, 3.0); 
  	for(int nearbyID : testSearch)
      std::cout << nearbyID << ",";                       
  	std::cout << std::endl;  

  	// Time segmentation process
  	auto startTime = std::chrono::steady_clock::now();
  	//
  	std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 3.0);
  	//
  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  	// Render clusters
  	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	for(std::vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int indice: cluster)
  			clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],points[indice][2]));
  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
  		++clusterId;
  	}
  	if(clusters.size()==0)
  		renderPointCloud(viewer,cloud,"data");

	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}