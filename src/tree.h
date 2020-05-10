/* \author Aaron Brown */
// Quiz on implementing kd tree
#include <iostream>
#include <vector>
template<class PointT>
struct Node
{
	PointT point;
	int id;
	Node<PointT>* left;
	Node<PointT>* right;

	Node(PointT p, int setId)
	:	point(p), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
class Tree
{
private:
	Node<PointT>* root;

public:
	Tree() : root(NULL){}

	void insertHelper(Node<PointT> **root, PointT point, int id, int depth){
        if(*root == NULL){
            *root = new Node<PointT>(point, id);
        }else if((depth % 2) == 0){
            if(point.x > (*root)->point.x){
                insertHelper(&(*root)->right, point, id, ++depth);
            }else{
                insertHelper(&(*root)->left, point, id, ++depth);
            }
        } else if ((depth % 2) != 0){
            if(point.y > (*root)->point.y){
                insertHelper(&(*root)->right, point, id, ++depth);
            }else{
                insertHelper(&(*root)->left, point, id, ++depth);
            }
        }
	}
	void insert(PointT point, int id){
        insertHelper(&root, point, id, 0);
	}
	void searchHelper(Node<PointT> **node, PointT &target, std::vector<int> &ids, float &distanceTol, int &depth){
        if(*node != NULL) {
            if((((*node)->point.x <= target.x +distanceTol)) &&
               ((*node)->point.x >= (target.x-distanceTol)) &&
               ((*node)->point.y <= target.y+distanceTol) &&
               ((*node)->point.y <= target.y+distanceTol) &&
               ((*node)->point.z <= target.z+distanceTol) &&
               ((*node)->point.z >= target.z-distanceTol)){
                double distance {sqrt(pow((*node)->point.x-target.x, 2.0)+pow((*node)->point.y-target.y, 2.0))};
                if(distance <= distanceTol)
                    ids.push_back((*node)->id);
            }

            std::vector<float> target_point {target.x, target.y, target.z};
            std::vector<float> scan_point {target.x, target.y, target.z};

            if(scan_point[depth%3] > target_point[depth%3]-distanceTol) {
                searchHelper(&(*node)->left, target, ids, distanceTol, ++depth);
            }

            if(scan_point[depth%3] < target_point[depth%3]+distanceTol){
                searchHelper(&(*node)->right, target,  ids, distanceTol, ++depth);
            }
        }
        return;
	}
	std::vector<int> search(PointT target, float distanceTol){
        std::vector<int> ids;
        int depth {0};
        searchHelper(&root, target, ids, distanceTol, depth);
        return ids;
	}
};




