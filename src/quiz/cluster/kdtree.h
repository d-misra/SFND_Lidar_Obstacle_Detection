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
    std::unique_ptr<Node> smaller;
    std::unique_ptr<Node> greater;

    Node(std::vector<float> arr, int setId)
            : point(std::move(arr)), id(setId), smaller(nullptr), greater(nullptr) {}
};

// Helper structure used during radius search.
struct NodeBoundary {
    const std::unique_ptr<Node>* nextNode;
    const std::size_t nextDimension;

    NodeBoundary(const std::unique_ptr<Node>* nextNode, const std::size_t nextDimension)
            : nextNode(nextNode), nextDimension(nextDimension) {}

    NodeBoundary(const NodeBoundary& other)
            : nextNode(other.nextNode), nextDimension(other.nextDimension) {}

    NodeBoundary(NodeBoundary&& other)
            : nextNode(std::move(other.nextNode)), nextDimension(other.nextDimension) {}
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
                if (n->smaller != nullptr) {
                    node = &n->smaller;
                    continue;
                }

                n->smaller = std::move(std::make_unique<Node>(point, id));
                break;
            }

            // At this point, we need to go right.
            if (n->greater != nullptr) {
                node = &n->greater;
                continue;
            }

            n->greater = std::move(std::make_unique<Node>(point, id));
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

        std::queue<NodeBoundary> boundary{};
        boundary.push(NodeBoundary{&root, 0});

        auto dimension = -1;
        while (!boundary.empty()) {
            const auto entry = boundary.front();
            boundary.pop();

            const auto& n = *entry.nextNode;
            assert(n != nullptr);

            const auto dimension = entry.nextDimension;
            const auto nextDimension = (dimension + 1) < numDims ? (dimension + 1) : 0;

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
            const auto hasSmallerChild = n->smaller != nullptr;
            const auto hasGreaterChild = n->greater != nullptr;
            const auto shouldScanSmaller = (target[dimension] - distanceTol) <= n->point[dimension];
            const auto shouldScanGreater = (target[dimension] + distanceTol) >= n->point[dimension];

            if (hasSmallerChild && shouldScanSmaller) {
                boundary.push(NodeBoundary{&n->smaller, nextDimension});
            }

            if (hasGreaterChild && shouldScanGreater) {
                boundary.push(NodeBoundary{&n->greater, nextDimension});
            }
        }

        return ids;
    }

};




