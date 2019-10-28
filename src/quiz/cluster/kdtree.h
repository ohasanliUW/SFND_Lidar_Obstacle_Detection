/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include "../../debug.h"
#include <pcl/common/common.h>

#define KD_TREE_LOG(format, ...) \
    LOG("KD-Tree LOG: " format, ## __VA_ARGS__)
#define KD_TREE_ERROR(format, ...) \
    ERROR("KD-Tree Error: " format, ## __VA_ARGS__)

#define KD_TREE_DIR(n) (n % 3)

// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> arr, int setId)
    :   point(arr), id(setId), left(nullptr), right(nullptr)
    {}

    ~Node() {
        KD_TREE_LOG("~Node(): %d", id);
        if (left) delete left;
        if (right) delete right;
    }
};

struct KdTree
{
    Node* root;

    KdTree()
    : root(nullptr)
    {}

    ~KdTree() {
        KD_TREE_LOG("~KdTree()");
        delete root;
    }

    void
    insert(std::vector<float> point, int id)
    {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root 
        insertHelper(&root, point, id, 0);
    }

    void
    insert(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        insertHelper(&root, cloud->begin(), cloud->end(), 0, 0);

    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;

        searchHelper(target, distanceTol, root, 0, ids);
        return ids;
    }

    private:

    void
    insertHelper(Node **node,
                 pcl::PointCloud<pcl::PointXYZ>::iterator start,
                 pcl::PointCloud<pcl::PointXYZ>::iterator end,
                 int idArg, int depth)
    {
        auto dir = depth % 3;
        std::sort(start, end, [dir](pcl::PointXYZ p1, pcl::PointXYZ p2){
            return p1.data[dir] < p2.data[dir];        
        });
    }

    void
    insertHelper(Node **node, std::vector<float> pointArg, int idArg, int depth)
    {
        if (nullptr == *node) {
            try {
                *node = new Node(pointArg, idArg);
            } catch (std::bad_alloc& ba) {
                KD_TREE_ERROR("Out of Memory");
                return;
            }
        } else {
            // check the depth
            // if depth is odd, compare by Y axis
            // else compare by X axis
            auto dir = KD_TREE_DIR(depth);
            if (pointArg[dir] < (*node)->point[dir]) {
                insertHelper(&(*node)->left, pointArg, idArg, depth+1);
            } else {
                insertHelper(&(*node)->right, pointArg, idArg, depth+1);
            }
        }
    }

    void
    searchHelper(const std::vector<float>& target,
                 const float distanceTol,
                 const Node* node,
                 const int level,
                 std::vector<int>& nbrs)
    {
        if (nullptr == node) {
            KD_TREE_LOG("LEVEL %d is empty, return back", level);
            return;
        }

        auto inBox = [](const std::vector<float>& t,
                        const std::vector<float>& p,
                        const float tol) {
            return ((p[0] <= (t[0] + tol)) && (p[0] >= (t[0] - tol))) &&
                   ((p[1] <= (t[1] + tol)) && (p[1] >= (t[1] - tol))) &&
                   ((p[2] <= (t[2] + tol)) && (p[2] >= (t[2] - tol)));
        };

        // Split is over Y axis if 1, X axis otherwise
        auto dir = KD_TREE_DIR(level);

        KD_TREE_LOG("LEVEL %d | DIR %d", level, dir);

        if (inBox(target, node->point, distanceTol)) {
            auto dist = sqrt(pow(target[0] - node->point[0], 2) +
                             pow(target[1] - node->point[1], 2) +
                             pow(target[2] - node->point[2], 2));

            KD_TREE_LOG("(%f, %f, %f) is inside (%f, %f, %f) +- %f",
                        node->point[0], node->point[1], node->point[2],
                        target[0], target[1], target[2], distanceTol);

            if (dist < distanceTol) {
                nbrs.push_back(node->id);
            }
        }

        if (target[dir] - distanceTol < node->point[dir]) {
            KD_TREE_LOG("Searching left of %c axis", dir + 'X');
            searchHelper(target, distanceTol, node->left, level + 1, nbrs);
        }
        if (target[dir] + distanceTol > node->point[dir]) {
            KD_TREE_LOG("Searching right of %c axis", dir + 'X');
            searchHelper(target, distanceTol, node->right, level + 1, nbrs);
        }
    }
    

};




