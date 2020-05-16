/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <cmath>


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
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
        insertHelper(root, 0, point, id);

	}
    void insertHelper(Node *&node, int depth, std::vector<float> point, int id) {
	    int dim = point.size();
	    int cur_cdim = depth % dim;
        if(node == nullptr) {
            node = new Node(point, id);  //should not be local variable
        } else if(point[cur_cdim] < node->point[cur_cdim]) {
            insertHelper(node->left, depth+1, point, id);
        } else {
            insertHelper(node->right, depth+1, point, id);
        }
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol) {
		std::vector<int> ids;
        searchHelper(root, 0, target, distanceTol, ids);
		return ids;
	}

    void searchHelper(Node* const& node, int depth, std::vector<float> target, float distanceTol, std::vector<int>& ids) {
        if (node == nullptr) {
            return;
        }
        //comparison with the node and decide which side to explore
        float leftbound = target[0] - distanceTol, rightbound = target[0] + distanceTol;
        float lowerbound = target[1] - distanceTol, upperbound = target[1] + distanceTol;
        float x = node->point[0], y = node->point[1];

        if ( x >= leftbound && x <= rightbound && y <=upperbound && y>=lowerbound ) {
            float distance = sqrt(pow(target[0] - x, 2) + pow(target[1] - y, 2));
            if (distance <= distanceTol) {
                ids.push_back(node->id);
            }
        }
        //check the cross boundary
        int dim = target.size();
        int cur_cdim = depth % dim;
        if (node->point[cur_cdim] > target[cur_cdim] - distanceTol) {
            searchHelper(node->left, depth+1, target, distanceTol, ids);
        }
        if (node->point[cur_cdim] < target[cur_cdim] + distanceTol) {
            searchHelper(node->right, depth+1, target, distanceTol, ids);
        }
    }

};




