// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

//필터링
template<typename PointT>                                                                                                                   // 공간 영역 정의 벡터. minpoint, maxpoint의 xyz                                              
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    auto startTime = std::chrono::steady_clock::now();

    // Voxel Grid Downsampling 복셀그리드의 모든 셀에 단일 포인트만 있는지 확인
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>); // cloudFiltered 생성
    vg.setInputCloud(cloud);                         // cloud 제공 
    vg.setLeafSize(filterRes, filterRes, filterRes); // 해당 셀 크기 정의
    vg.filter(*cloudFiltered);                       // 결과를 cloudFiltered에 저장

    // ROI 설정?
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);   // cloudRegion 생성 
    pcl::CropBox<PointT> region(true);   // pcl cropbox 사용하여 ROI 설정 cropbox 내부 포인트를 다룰거라 true로 설정.
    region.setMin(minPoint);  
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered); // Voxel Grid 필터를 적용한 cloudFiltered
    region.filter(*cloudRegion);         // 결과를 cloudRegion에 저장. ROI 내부의 포인트만 남는다.

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);     // 어떤 부분을 제거하고싶다면..? roof : 고정된 값. ***
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -100, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, 0, 1));
    roof.setInputCloud(cloudRegion);     // ROI 이외의 부분 제거 후
    roof.filter(indices);                // 결과를 indices에 저장

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices}; // inliers 생성
    for(int point : indices)
        inliers->indices.push_back(point);                  // indices를 통해 inliers로 삽입


    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);   // 입력 클라우드는 cloudRegion(얘를 입력클라우드로 두고.)
    extract.setIndices(inliers);          // roof 포인트의 인덱스
    extract.setNegative(true);            // true로 설정해서 해당 점 삭제
    extract.filter(*cloudRegion);         // 결과를 cloudRegion에 저장.

    /* *** : 특정 지점 제거하고싶을때
    roof 라는 고정된 값 이용. 
    제거하고싶은 지점의 min max 설정
    입력클라우드는 ROI 처리 후의 cloudRegion
    제거하고싶은 지점의 인덱스를 filter를 이용해서 indices에 저장
    
    inliers 생성 후, 제거하고싶은 지점의 인덱스를 inliers에 넣어준다.
    
    setInputCloud : 입력클라우드                                       // cloudRegion을 입력클라우드로 두고
    SetIndices : 인덱스 번호                                           // 제거하고 싶은 지점의 인덱스
    SetNegative : 해당 인덱스를 삭제할지, 인덱스 외의 것들을 삭제할지     // 해당 인덱스 제거
    filter : 어쨌든 제거하고 싶은부분 제거 후, () 안 변수에 저장.         // 남은 부분은 *cloudRegion에 저장
    */
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;  

    return cloudRegion;

}


template<typename PointT> 
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ()); // 장애물 포인트 클라우드 생성
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT> ()); // 평면 포인트 클라우드 생성

    for(int index : inliers->indices) // inlier의 인덱스를 가져와서 
        planeCloud->points.push_back(cloud->points[index]); // 아마.. 반복문을 통해 inlier를 planeCloud에 추가한다.

    pcl::ExtractIndices<PointT> extract; // 인덱스 추출? 음.. 내생각엔 inlier를 planeCloud에 추가 후, 남은것들을 추출해서 obstCloud에 추가하는듯
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud); //inlier가 아닌 모든 포인트가 유지된다?ㅜㅜ // 내부 포인트는 해당 참조 클라우드에서 제거된다?

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud); // 두개의 포인트 클라우드 반환 (segResult 변수에 저장 후 리턴)

    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.

    // 여기부터는 문서 복붙 ㅜㅜ
    pcl::SACSegmentation<pcl::PointXYZI> seg; // Seg 객체 생성 *************PointXYZ 또는 PointXYZI 중 해당되는거로 하기!***************
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices}; //inlier 정의// 원래 이거 pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients}; //coefficients(계수)정의 // 원래 이거pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

    //seg 객체에 대한 파라미터 설정 (객체, 모델, 메소드 유형, 거리임계값 등 매개변수 설정)
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE); // 모델 타입 SACMODEL_PLANE으로 설정 (coefficient가 이 평면이 무엇인지 정의)
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations); // iteration 설정
    seg.setDistanceThreshold (distanceThreshold); // 거리 임계값 설정

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud); // 입력 클라우드 제공 // 라이다 스캔할때 생성하는 마지막 PCD파일
    seg.segment (*inliers, *coefficients); // inlier 생성 얘네가 참조로 전달될거라 인덱스도 같이 전달된대 (이 정보를 활용해서 포인트 클라우드를 plane과 obstacle로 분리)

    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud); // 위에 있는 SeparateClouds 함수 호출 후 segResult 변수에 저장 
    

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;


    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>); // KdTree 구조를 사용하여 검색기능을 수행하기 위해 pcl::KdTree 클래스를 상속하는 클래스
    tree->setInputCloud(cloud); // 클라우드 입력

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec; // pcl::EuclideanClusterExtraction 함수를 사용해서 유클리드 클러스터 추출 // 포인트유형이 PointT인 EuclideanClusterExtraction 객체 생성 (ec?)
    ec.setClusterTolerance(clusterTolerance);   //
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices); // extract 메소드를 호출해서 클래스터 인덱스 생성하라고?

    // 보니까 ec 객체 생성 후 ex 매개변수들 설정하는것. 원래 일케 하는듯..

    for(pcl::PointIndices getIndices: clusterIndices) // pcl::PointIndices 유형 clusterInices만큼 반복
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);  // 새 cloud cluster 를 만든대요..

        for(int index: getIndices.indices)
          cloudCluster->points.push_back (cloud->points[index]); // index 반복.. 시발 // 방금 만든 cloud cluster에  cloud를 넣어준대.. 

        cloudCluster->width = cloudCluster->points.size();  
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;   
        clusters.push_back(cloudCluster);  // 내가 만든 clusters에 cloud cluster를 저장 // 매개변수로 받은 cloud를 cloud Cluster에 저장 후 clusters에 저장?
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

//////////////////////////////////////////////////// 프로젝트 ////////////////////////////////////////////////////

template<typename PointT> //맞음
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Ransac3d(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult; // 가장 높은 inlier수가 여기에 있다. // unordered set 순서가 지정되지 않은 세트
	srand(time(NULL));

    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ()); // 장애물 포인트 클라우드 생성
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT> ()); // 평면 포인트 클라우드 생성

	while(maxIterations--) // Return indicies of inliers from fitted line with most inliers // 반복문 사용
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
			//pcl::PointXYZ point = cloud->points[index];	
            PointT point = cloud->points[index];	
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

    for(int index = 0; index < cloud->points.size(); index++)
    {
        //pcl::PointXYZ point = cloud->points[index];
        PointT point = cloud->points[index];
        if(inliersResult.count(index))
			planeCloud->points.push_back(point);
		else
			obstCloud->points.push_back(point);

    }


	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "Ransac took" << elapsedTime.count() << "milliseconds" << std::endl;
	
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud); // 두개의 포인트 클라우드 반환 (segResult 변수에 저장 후 리턴)

    return segResult;
    

}

