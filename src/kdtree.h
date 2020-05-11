/* \author Aaron Brown */
// Quiz on implementing kd tree

#ifndef KDTREE_H
#define KDTREE_H

#include <utility>
#include <memory>
#include <queue>

#include "render/render.h"


// Structure to represent node of kd tree
struct Node {
    const float *const point;
    std::size_t id;
    std::unique_ptr<Node> smaller;
    std::unique_ptr<Node> greater;

    Node(const float *const arr, std::size_t setId)
            : point(arr), id(setId), smaller(nullptr), greater(nullptr) {}

    ~Node() = default;
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
    std::size_t numDimensions;

    KdTree()
            : root(nullptr), numDimensions(3) {}

    ~KdTree() = default;

    void insert(const float *const point, std::size_t id) {
        if (root == nullptr) {
            root = std::make_unique<Node>(point, id);
            return;
        }

        auto *node = &root;
        auto dimension = -1;

        while (true) {
            dimension = (dimension + 1) < numDimensions ? (dimension + 1) : 0; // avoid modulo, use conditional assignment
            const auto& n = (*node); // avoid some lookups

            const auto isSmaller = point[dimension] <= n->point[dimension];
            if (isSmaller) {
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

    std::vector<int> search(const float* target, float distanceTol) {
        const auto distanceTolSq = distanceTol * distanceTol;

        std::vector<int> ids;
        if (root == nullptr) { // TODO: right now we're assuming the target has the right size ...
            return ids;
        }

        std::queue<NodeBoundary> boundary{};
        boundary.push(NodeBoundary{&root, 0});

        while (!boundary.empty()) {
            const auto entry = boundary.front();
            boundary.pop();

            const auto& n = *entry.nextNode;
            assert(n != nullptr);

            const auto dimension = entry.nextDimension;
            const auto nextDimension = (dimension + 1) < numDimensions ? (dimension + 1) : 0;

            // We're using the squared Euclidean distance in order to avoid taking the square root
            // while being able to skip a box check.
            auto distanceSsq = 0.;
            for (auto d = 0; d < numDimensions; ++d) {
                const auto delta = (n->point[d] - target[d]);
                distanceSsq += delta * delta;
            }
            const auto withinSphere = distanceSsq <= distanceTolSq;
            if (withinSphere) {
                ids.push_back(n->id);
            }

            // Finally, we check whether the target and tolerance exceed the split of this node in either direction.
            const auto hasLeftChild = n->smaller != nullptr;
            const auto hasRightChild = n->greater != nullptr;
            const auto boxExtendsLeft = (target[dimension] - distanceTol) <= n->point[dimension];
            const auto boxExtendsRight = (target[dimension] + distanceTol) >= n->point[dimension];

            if (hasLeftChild && boxExtendsLeft) {
                boundary.push(NodeBoundary{&n->smaller, nextDimension});
            }

            if (hasRightChild && boxExtendsRight) {
                boundary.push(NodeBoundary{&n->greater, nextDimension});
            }
        }

        return ids;
    }

};

#endif
