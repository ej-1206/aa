/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;  // point
	int id;                    // id
	Node* left;                // 노드의 왼쪽, 오른쪽 자식
	Node* right;  

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}         // 새로운 노드를 생성하는 기능
};

/*struct KdTree  //구조체
{
	Node* root;// Node 구조체 이용해서 루트노드 생성

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id) //Node** node (이중포인터) 노드포인터는 실제로 루트가 시작되는것. 따라서 node는 노드의 메모리주소?
	{
		if(*node==NULL)                  // 노드가 NULL 인지 확인. 역참조중 
		    *node = new Node(point,id);  // 새로운 노드 할당
		else
		{
			uint cd = depth%2;           // uint 양수인 정수 depth 홀수 짝수 여부. cd는 0또는1. point[0]은 x, [1]은 y

			if(point[cd] < ((*node)->point[cd])) 
			   insertHelper(&((*node)->left), depth+1, point, id); // 현재 노드와 새로운 포인트의 x,y값 비교 작으면 왼쪽 크면 오른쪽
			else
			   insertHelper(&((*node)->right), depth+1, point, id);   

		}	
	}

	void insert(std::vector<float> point, int id)  // float 형 벡터이기 때문에 x와 y로 구성되어있다? // id 는 point에 대한 고유 식별자.. 인덱스래..
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root,0,point,id); // NULL인 노드에 재할당  // &root 주소 전달 // 매개변수 0은 depth (depth가 0인것은 루트라는 뜻)

	}

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>&ids)
	{
		if(node!=NULL)
		{   // 해당 노드가 대상 상자 내에 있는지 여부 확인, x값을 얻는다
		    // 상자 범위 = 타겟 +- distanceTol // [0]은 x, [1]은 y 
			if((node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) && (node->point[1] >= target[1] - distanceTol) && (node->point[1] <= target[1] + distanceTol))
			{   // 제곱근 거리 공식 이용해서 상자 내에 있는지 확인
				float distance = sqrt((node->point[0] - target[0])*(node->point[0] - target[0]) + (node->point[1] - target[1])*(node->point[1] - target[1]));
				if(distance <= distanceTol)
				    ids.push_back(node->id); // 노드 id를 ids로 푸시
			}
			// 오른쪽 혹은 왼쪽으로 흐르도록
			if((target[depth%2] - distanceTol) < node->point[depth%2])       // target의 박스가 노드의 x,y 값(depth%2가 0또는1로 나오니까) 보다 작은지 확인 
			    searchHelper(target, node->left, depth+1, distanceTol, ids); // 왼쪽으로 흐르고, depth 1 증가
			if((target[depth%2] + distanceTol) > node->point[depth%2])       // 노드의 x,y 값보다 큰지 확인 
			    searchHelper(target, node->right, depth+1, distanceTol, ids);// 오른쪽으로 흐르고, depth 1 증가
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids); // target 값 전달(2D니까 x,y)  root는 어디서 온거냐(struct 구조 안에 있더라) depth(0)
		return ids;
	}
	

};
*/

//////////////////////////////////////////////////// 프로젝트 ////////////////////////////////////////////////////

struct KdTree  //구조체
{
	Node* root;// Node 구조체 이용해서 루트노드 생성

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id) //Node** node (이중포인터) 노드포인터는 실제로 루트가 시작되는것. 따라서 node는 노드의 메모리주소?
	{
		if(*node==NULL)                  // 노드가 NULL 인지 확인. 역참조중 
		    *node = new Node(point,id);  // 새로운 노드 할당
		else
		{
			uint cd = depth%3;           // uint 양수인 정수 depth 홀수 짝수 여부. cd는 0또는1. point[0]은 x, [1]은 y

			if(point[cd] < ((*node)->point[cd])) 
			   insertHelper(&((*node)->left), depth+1, point, id); // 현재 노드와 새로운 포인트의 x,y값 비교 작으면 왼쪽 크면 오른쪽
			else
			   insertHelper(&((*node)->right), depth+1, point, id);   

		}	
	}

	void insert(std::vector<float> point, int id)  // float 형 벡터이기 때문에 x와 y로 구성되어있다? // id 는 point에 대한 고유 식별자.. 인덱스래..
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root,0,point,id); // NULL인 노드에 재할당  // &root 주소 전달 // 매개변수 0은 depth (depth가 0인것은 루트라는 뜻)

	}

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>&ids)
	{
		if(node!=NULL)
		{   // 해당 노드가 대상 상자 내에 있는지 여부 확인, x값을 얻는다
		    // 상자 범위 = 타겟 +- distanceTol // [0]은 x, [1]은 y 
			if((node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) && (node->point[1] >= target[1] - distanceTol) && (node->point[1] <= target[1] + distanceTol) && (node->point[2] >= target[2] - distanceTol) && (node->point[2] <= target[2] + distanceTol))
			{   // 제곱근 거리 공식 이용해서 상자 내에 있는지 확인
				float distance = sqrt((node->point[0] - target[0])*(node->point[0] - target[0]) + (node->point[1] - target[1])*(node->point[1] - target[1]) + (node->point[2] - target[2])*(node->point[2] - target[2]));
				if(distance <= distanceTol)
				    ids.push_back(node->id); // 노드 id를 ids로 푸시
			}
			// 오른쪽 혹은 왼쪽으로 흐르도록
			if((target[depth%3] - distanceTol) < node->point[depth%3])       // target의 박스가 노드의 x,y 값(depth%2가 0또는1로 나오니까) 보다 작은지 확인 
			    searchHelper(target, node->left, depth+1, distanceTol, ids); // 왼쪽으로 흐르고, depth 1 증가
			if((target[depth%3] + distanceTol) > node->point[depth%3])       // 노드의 x,y 값보다 큰지 확인 
			    searchHelper(target, node->right, depth+1, distanceTol, ids);// 오른쪽으로 흐르고, depth 1 증가
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids); // target 값 전달(2D니까 x,y)  root는 어디서 온거냐(struct 구조 안에 있더라) depth(0)
		return ids;
	}
	

};




