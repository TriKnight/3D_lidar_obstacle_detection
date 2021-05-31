/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node *root;

	KdTree()
		: root(NULL)
	{
	}

	~KdTree()
	{
		delete root;
	}
	void insertKdTree(Node **node, uint depth, std::vector<float> point, int id)
	{
		//Tree is empty
		if (*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
		{
			//Calculate the current depth  
			uint cd = (depth % 3); //k-dimensional space to partition (2 to 2D, 3 to 3D)
			if (point[cd] < ((*node)->point[cd]))
				insertKdTree(&((*node)->left), depth + 1, point, id);
			else
				insertKdTree(&((*node)->right), depth + 1, point, id);
		}
	}
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertKdTree(&root, 0, point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	void searchPoint(std::vector<float> target, Node *node, int depth, float distanceTol, std::vector<int> &ids)
	{
		if (node != NULL)
		{	//x= point[0], y= point[1], z =point[2]
			float x = node->point[0];
			float y = node->point[1];
			float z = node->point[2];
			float target_x = target[0];
			float target_y = target[1];
			float target_z = target[2];

			if (x >= target_x - distanceTol && 
				x <= target_x + distanceTol &&
				y >= target_y - distanceTol && 
				y <= target_y + distanceTol &&
				z >= target_z - distanceTol && 
				z <= target_z + distanceTol)
			{	//calculate the distance from the point (sphere radius)
				float sphere_radius = sqrt((x-target_x)*(x-target_x)+(y-target_y)*(y-target_y)+(z-target_z)*(z-target_z));
				if (sphere_radius <= distanceTol)
					ids.push_back(node->id);
			}

			// check across boundary
			if ((target[depth % 3] - distanceTol) < node->point[depth % 3])
			{
				searchPoint(target, node->left, depth + 1, distanceTol, ids);
			}
			if ((target[depth % 3] + distanceTol) > node->point[depth % 3])
			{
				searchPoint(target, node->right, depth + 1, distanceTol, ids);
			}
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchPoint(target, root, 0, distanceTol, ids);
		return ids;
	}
};
