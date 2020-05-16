//
// Created by cckai on 5/15/20.
//

#ifndef KDTREE_H
#define KDTREE_H

#include <cmath>


// Structure to represent node of kd tree
struct Node
{
    Eigen::Vector3f point;
    int id;
    Node* left;
    Node* right;

    Node(Eigen::Vector3f arr, int setId)
            :	point(arr), id(setId), left(NULL), right(NULL)
    {}
};

struct KdTree
{
    Node* root;

    KdTree()
            : root(NULL)
    {}

    void insert(Eigen::Vector3f point, int id)
    {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root
        insertHelper(root, 0, point, id);

    }
    void insertHelper(Node *&node, int depth, Eigen::Vector3f point, int id) {
        int cur_cdim = depth % 3;
        if(node == nullptr) {
            node = new Node(point, id);  //should not be local variable
        } else if(point[cur_cdim] < node->point[cur_cdim]) {
            insertHelper(node->left, depth+1, point, id);
        } else {
            insertHelper(node->right, depth+1, point, id);
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(Eigen::Vector3f target, float distanceTol) {
        std::vector<int> ids;
        searchHelper(root, 0, target, distanceTol, ids);
        return ids;
    }

    void searchHelper(Node* const& node, int depth, Eigen::Vector3f target, float distanceTol, std::vector<int>& ids) {
        if (node == nullptr) {
            return;
        }
        //comparison with the node and decide which side to explore
        float leftbound = target[0] - distanceTol, rightbound = target[0] + distanceTol;
        float lowerbound = target[1] - distanceTol, upperbound = target[1] + distanceTol;
        float zlbound = target[2] - distanceTol, zubound = target[2] + distanceTol;

        float x = node->point[0], y = node->point[1], z = node->point[2];

        if ( x >= leftbound && x <= rightbound && y <=upperbound && y>=lowerbound && z >= zlbound && z <= zubound) {
            float distance = sqrt(pow(target[0] - x, 2) + pow(target[1] - y, 2) + pow(target[2] - z, 2));
            if (distance <= distanceTol) {
                ids.push_back(node->id);
            }
        }
        //check the cross boundary
        int cur_cdim = depth % 3;
        if (node->point[cur_cdim] > target[cur_cdim] - distanceTol) {
            searchHelper(node->left, depth+1, target, distanceTol, ids);
        }
        if (node->point[cur_cdim] < target[cur_cdim] + distanceTol) {
            searchHelper(node->right, depth+1, target, distanceTol, ids);
        }
    }

};

#endif //KDTREE_H
