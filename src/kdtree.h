/* \author Aaron Brown */
// Quiz on implementing kd tree
#ifndef KDTREE_H_
#define KDTREE_H_

#include "render/render.h"


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

	void insertHelper(Node* &node, std::vector<float> point, int id, int depth)
	{
		if (node == nullptr)
		{
			node = new Node(point, id);
		}
		else
		{
			//bool insertLeft = ((depth % 2 == 0) && (point[0] < node->point[0])) || ((depth % 2 != 0) && (point[1] < node->point[1]));
			int currentDepth = depth % 2;
			bool insertLeft = (point[currentDepth] < node->point[currentDepth]);
			if(insertLeft)
				insertHelper(node->left, point, id, depth+1);
			else
				insertHelper(node->right, point, id, depth+1);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(root, point, id, 0);
	}

	bool pointNearTarget(float distanceTol, std::vector<float> point, std::vector<float> target)
	{
		float x_min = target[0] - distanceTol;
		float y_min = target[1] - distanceTol;
		float x_max = target[0] + distanceTol;
		float y_max = target[1] + distanceTol;
		return ((point[0] >= x_min) && (point[0] <= x_max) && (point[1] >= y_min) && (point[1] <= y_max)); 
	}

	float distance(std::vector<float> point, std::vector<float> target)
	{
		return std::sqrt(std::pow((target[1]-point[1]), 2) + std::pow((target[0]-point[0]), 2));
	}

	void searchHelper(Node* &node, std::vector<float> target, std::vector<int> &ids, float distanceTol, int depth)
	{
		if(node != nullptr)
		{	
			std::vector<float> point = node->point;
			if(pointNearTarget(distanceTol, point, target) && (distance(point, target) <= distanceTol))
			{
				ids.push_back(node->id);
			}
			
			int currentDepth = depth % 2;
			bool searchLeft = ((target[currentDepth] - distanceTol) < point[currentDepth]);
			bool searchRight = ((target[currentDepth] + distanceTol) > point[currentDepth]);

			if(searchLeft)
				searchHelper(node->left, target, ids, distanceTol, depth+1);
			if(searchRight)
				searchHelper(node->right, target, ids, distanceTol, depth+1);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, target, ids, distanceTol, 0);
		return ids;
	}

};

#endif