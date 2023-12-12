/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void inserthelper(Node* &root, std::vector<float> point, int id, int depth)
	{
		if (root == nullptr)
		{
			root = new Node(point, id);
		}
		else
		{
			//bool insertLeft = ((depth % 2 == 0) && (point[0] < root->point[0])) || ((depth % 2 != 0) && (point[1] < root->point[1]));
			int currentDepth = depth % 2;
			bool insertLeft = (point[currentDepth] < root->point[currentDepth]);
			if(insertLeft)
				inserthelper(root->left, point, id, depth+1);
			else
				inserthelper(root->right, point, id, depth+1);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		inserthelper(root, point, id, 0);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




