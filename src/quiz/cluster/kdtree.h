/* \author Aaron Brown */
// Quiz on implementing kd tree

#include <memory>

template<typename PointT>
struct KdTree {
    struct Node {
        PointT point;
        int id;
        size_t depth;
        std::unique_ptr<Node> left;
        std::unique_ptr<Node> right;

        Node(PointT arr, int setId, size_t depth)
        :	point(arr), id(setId), depth(depth), left(nullptr), right(nullptr)
        {}
    };
    
	std::unique_ptr<Node> root;

	KdTree()
	: root(nullptr)
	{}
    
    void insert_impl(std::unique_ptr<Node>& node, PointT point, int id, size_t depth) {
        if (not node) {
            node = std::make_unique<Node>(point, id, depth);
            return;
        }
        
        size_t dim = depth % 3;
        if (point.data[dim] < node->point.data[dim]) {
            insert_impl(node->left, point, id, depth + 1);
        } else {
            insert_impl(node->right, point, id, depth + 1);
        }
    }

	void insert(PointT point, int id)
	{
        insert_impl(root, point, id, 0);
	}
    
    void search_impl(const std::unique_ptr<Node>& node, PointT target, float distanceTol, std::vector<int>& ids) {
        if (not node) {
            return;
        }
        
        auto dx = std::fabs(node->point.data[0] - target.data[0]);
        auto dy = std::fabs(node->point.data[1] - target.data[1]);
        auto dz = std::fabs(node->point.data[2] - target.data[2]);
        if (dx <= distanceTol and dy <= distanceTol and dz <= distanceTol) {
            // point in the box - check distance and recurse both subtrees
            auto dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (dist <= distanceTol) {
                ids.push_back(node->id);
            }
        }
        size_t dim = node->depth % 3;
        if (target.data[dim] - distanceTol < node->point.data[dim]) {
            search_impl(node->left, target, distanceTol, ids);
        }
        if (target.data[dim] + distanceTol > node->point.data[dim]) {
            search_impl(node->right, target, distanceTol, ids);
        }
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
        search_impl(root, target, distanceTol, ids);
		return ids;
	}
};





#if 0
// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
    size_t depth;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId, size_t depth)
	:	point(arr), id(setId), depth(depth), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
    
    ~KdTree() {
        clear_impl(root);
    }
    
    void clear_impl(Node* node) {
        clear_impl(node->left);
        clear_impl(node->right);
        if(node != NULL) {
            delete node;
        }
    }
    
    void insert_impl(Node** node, std::vector<float> point, int id, size_t depth) {
        if (*node == NULL) {
            *node = new Node(point, id, depth);
            return;
        }
        
        size_t dim = depth % 2;
        if (point[dim] < (*node)->point[dim]) {
            insert_impl(&(*node)->left, point, id, depth + 1);
        } else {
            insert_impl(&(*node)->right, point, id, depth + 1);
        }
    }

	void insert(std::vector<float> point, int id)
	{
        insert_impl(&root, point, id, 0);
	}
    
    void search_impl(Node* node, std::vector<float> target, float distanceTol, std::vector<int>& ids) {
        if (node == NULL) {
            return;
        }
        auto dx = std::fabs(node->point[0] - target[0]);
        auto dy = std::fabs(node->point[1] - target[1]);
        if (dx <= distanceTol and dy <= distanceTol) {
            // point in the box - check distance and recurse both subtrees
            auto dist = std::sqrt(dx * dx + dy * dy);
            if (dist <= distanceTol) {
                ids.push_back(node->id);
            }
        }
        size_t dim = node->depth % 2;
        if (target[dim] - distanceTol < node->point[dim]) {
            search_impl(node->left, target, distanceTol, ids);
        }
        if (target[dim] + distanceTol > node->point[dim]) {
            search_impl(node->right, target, distanceTol, ids);
        }
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        search_impl(root, target, distanceTol, ids);
		return ids;
	}
};
#endif



