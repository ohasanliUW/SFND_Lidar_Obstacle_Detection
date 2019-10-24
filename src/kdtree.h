/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"
#include "debug.h"
#include <pcl/common/common.h>

#ifdef VERBOSE
#define KD_TREE_LOG(format, ...) \
    LOG("KD-Tree LOG: " format, ## __VA_ARGS__)
#define KD_TREE_ERROR(format, ...) \
    ERROR("KD-Tree Error: " format, ## __VA_ARGS__)
#else
#define KD_TREE_LOG(format, ...)
#define KD_TREE_ERROR(format, ...)
#endif
#define KD_TREE_DIR(n) (n % 3)

// Structure to represent node of kd tree
template <typename PointT>
struct Node
{
	PointT point;
	int id;
    Node<PointT>* left;
    Node<PointT>* right;

	Node(PointT point, int setId)
	:	point(point), id(setId), left(nullptr), right(nullptr)
	{}

    ~Node() {
        KD_TREE_LOG("~Node(): %d", id);
        if (left) delete left;
        if (right) delete right;
    }
};

template <typename PointT>
struct KdTree
{
    Node<PointT>* root;

	KdTree()
	: root(nullptr)
	{}

    ~KdTree() {
        KD_TREE_LOG("~KdTree()");
        delete root;
    }

    // Insert point into KD-Tree with specified id
	void
    insert(const PointT& point, int id)
	{
        insertHelper(&root, point, id, 0);
	}

    // Insert all of the points in Point Cloud into the KD-Tree
    void
    insert(typename pcl::PointCloud<PointT>::Ptr cloud)
    {
        insertHelper(&root, cloud->begin(), cloud->begin(), cloud->end(), 0);
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int>
    search(const PointT& target, float distanceTol)
	{
		std::vector<int> ids;

        searchHelper(target, distanceTol, root, 0, ids);
		return ids;
	}

    private:

    void
    insertHelper(Node<PointT> **node,
                 typename pcl::PointCloud<PointT>::iterator head,
                 typename pcl::PointCloud<PointT>::iterator start,
                 typename pcl::PointCloud<PointT>::iterator end,
                 int depth)
    {
        // if start and end is the same, then create the node and return
        if (start >= end) {
            return;
        }

        // sort from start to end over X, Y or Z axis
        // the axis is determined by depth % 3
        auto dir = depth % 3;
        //KD_TREE_LOG("DEPTH %d\tDIR %d\n", depth, dir);
        std::sort(start, end, [dir](PointT p1, PointT p2) {
            return p1.data[dir] < p2.data[dir];
        });

        //KD_TREE_LOG("ALL POINTS HAVE BEEN SORTED");
        
        // Now, the middle point is std::distance(start, end) / 2
        // find the median and recursively insert each half with depth + 1
        auto mid = std::distance(start, end) / 2;

        // Create a node at mid point
        if (nullptr == *node) {
            try {
                *node = new Node<PointT>(*(start+mid), std::distance(head, start+mid));
            } catch(std::bad_alloc& ba) {
                return;
            }
        }

        // insert left subtree
        insertHelper(&(*node)->left, head, start, start + mid, depth + 1);
        // insert right subtree
        insertHelper(&(*node)->right, head, start + mid + 1, end, depth + 1);
    }

    void
    insertHelper(Node<PointT> **node, const PointT& pointArg, int idArg, int depth)
    {
        if (nullptr == *node) {
            try {
                *node = new Node<PointT>(pointArg, idArg);
            } catch (std::bad_alloc& ba) {
                KD_TREE_ERROR("Out of Memory");
                return;
            }
        } else {
            // check the depth
            // if depth is odd, compare by Y axis
            // else compare by X axis
            auto dir = KD_TREE_DIR(depth);
            if (pointArg.data[dir] < (*node)->point.data[dir]) {
                insertHelper(&(*node)->left, pointArg, idArg, depth+1);
            } else {
                insertHelper(&(*node)->right, pointArg, idArg, depth+1);
            }
        }
    }

    void
    searchHelper(const PointT& target,
                 const float distanceTol,
                 const Node<PointT>* node,
                 const int level,
                 std::vector<int>& nbrs)
    {
        if (nullptr == node) {
          //  KD_TREE_LOG("LEVEL %d is empty, return back", level);
            return;
        }

        // lambda function to check if target t is within cubic box with its
        // center at t where t is at tol distance from every surface of the box
        auto inBox = [](const PointT& t,
                        const PointT& p,
                        const float tol) {
            return ((p.data[0] <= (t.data[0] + tol)) && (p.data[0] >= (t.data[0] - tol))) &&
                   ((p.data[1] <= (t.data[1] + tol)) && (p.data[1] >= (t.data[1] - tol))) &&
                   ((p.data[2] <= (t.data[2] + tol)) && (p.data[2] >= (t.data[2] - tol)));
        };

        // Determine the direction of the split (0 - X, 1 - Y, 2 - Z)
        auto dir = KD_TREE_DIR(level);

        //KD_TREE_LOG("LEVEL %d | DIR %d", level, dir);

        // if target falls into the box, calculate the distance between target
        // and the node and add it to the nearest list if tolerable
        if (inBox(target, node->point, distanceTol)) {
            auto dist = sqrt(pow(target.data[0] - node->point.data[0], 2) +
                             pow(target.data[1] - node->point.data[1], 2) +
                             pow(target.data[2] - node->point.data[2], 2));

            //KD_TREE_LOG("(%f, %f, %f) is inside (%f, %f, %f) +- %f",
             //           node->point.data[0], node->point.data[1], node->point.data[2],
              //          target.data[0], target.data[1], target.data[2], distanceTol);

            if (dist < distanceTol) {
                nbrs.push_back(node->id);
            }
        }

        // search left subtree
        if (target.data[dir] - distanceTol < node->point.data[dir]) {
            //KD_TREE_LOG("Searching left of %c axis", dir + 'X');
            searchHelper(target, distanceTol, node->left, level + 1, nbrs);
        }

        // search right subtree
        if (target.data[dir] + distanceTol > node->point.data[dir]) {
            //KD_TREE_LOG("Searching right of %c axis", dir + 'X');
            searchHelper(target, distanceTol, node->right, level + 1, nbrs);
        }
    }
	

};




