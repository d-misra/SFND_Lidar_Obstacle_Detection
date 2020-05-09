/* \author Aaron Brown */
// Quiz on implementing kd tree

#include <utility>
#include <memory>
#include <queue>

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node {
    std::vector<float> point;
    int id;
    std::unique_ptr<Node> left;
    std::unique_ptr<Node> right;

    Node(std::vector<float> arr, int setId)
            : point(std::move(arr)), id(setId), left(nullptr), right(nullptr) {}
};

struct KdTree {
    std::unique_ptr<Node> root;

    KdTree()
            : root(nullptr) {}

    void insert(const std::vector<float> &point, int id) {
        if (root == nullptr) {
            root = std::make_unique<Node>(point, id);
            return;
        }

        auto *node = &root;
        auto dimension = -1;
        const auto numDims = 2; // TODO: property of the tree

        while (true) {
            dimension = dimension + 1 < numDims ? dimension + 1 : 0; // avoid modulo, use conditional assignment
            const auto& n = (*node); // avoid some lookups

            const auto shouldGoLeft = point[dimension] <= n->point[dimension];
            if (shouldGoLeft) {
                if (n->left != nullptr) {
                    node = &n->left;
                    continue;
                }

                n->left = std::move(std::make_unique<Node>(point, id));
                break;
            }

            // At this point, we need to go right.
            if (n->right != nullptr) {
                node = &n->right;
                continue;
            }

            n->right = std::move(std::make_unique<Node>(point, id));
            break;
        }
    }


    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(const std::vector<float> &target, float distanceTol) {
        const auto numDims = 2; // TODO: property of the tree
        const auto distanceTolSq = distanceTol * distanceTol;

        std::vector<int> ids;
        if (target.size() < numDims || root == nullptr) {
            return ids;
        }

        std::queue<std::unique_ptr<Node>*> boundary{};
        boundary.push(&root);

        auto dimension = -1;
        while (!boundary.empty()) {
            dimension = dimension + 1 < numDims ? dimension + 1 : 0; // avoid modulo, use conditional assignment

            const auto& n = (*boundary.front());
            boundary.pop();
            assert(n != nullptr);

#ifdef KD_BOX_BROADPHASE
            // We're using a box check as a broadphase check.
            const auto leftBoundary = n->point[0] >= (target[0] - distanceTol);
            const auto rightBoundary = n->point[0] <= (target[0] + distanceTol);
            const auto upperBoundary = n->point[1] >= (target[1] - distanceTol);
            const auto lowerBoundary = n->point[1] <= (target[1] + distanceTol);
            const auto withinBox = leftBoundary && rightBoundary && upperBoundary && lowerBoundary;
            if (!withinBox) return;
#endif

            // The box broadphase above meant to avoid more costly distance checks when a
            // trivial box check can already rule them out.
            // One of the reasons for this idea is that in order to determine the Euclidean distance,
            // we'd need to take a square root.
            // However, by utilizing the squared Euclidean distance we not only avoid the square root,
            // but we also get to skip four conditional assignments.
            const auto dx = (n->point[0] - target[0]);
            const auto dy = (n->point[1] - target[1]);
            const auto distanceSq = dx * dx + dy * dy;
            const auto withinCircle = distanceSq <= distanceTolSq;  // avoids taking the square root
            if (withinCircle) {
                ids.push_back(n->id);
            }

            // Finally, we check whether the target extends the split of this node in either direction.
            const auto hasLeftChild = n->left != nullptr;
            const auto hasRightChild = n->right != nullptr;
            const auto boxExtendsLeft = (target[dimension] - distanceTol) < n->point[dimension];
            const auto boxExtendsRight = (target[dimension] + distanceTol) > n->point[dimension];

            if (hasLeftChild && boxExtendsLeft) {
                boundary.push(&n->left);
            }

            if (hasRightChild && boxExtendsRight) {
                boundary.push(&n->right);
            }
        }

        return ids;
    }

};




