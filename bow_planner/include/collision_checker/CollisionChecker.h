//
// Created by airlab on 6/9/25.
//

#ifndef COLLISIONCHECKER_COLLISIONCHECKER_H
#define COLLISIONCHECKER_COLLISIONCHECKER_H
#include <vector>
#include <array>
#include <memory>
#include "kdtree.h"
#include <Eigen/Dense>

namespace bow{

    class CollisionChecker : public std::enable_shared_from_this<CollisionChecker> {
    public:
        using CCPtr = std::shared_ptr<CollisionChecker>;
        CollisionChecker(const std::vector<double>& X, const std::vector<double>& Y, double robotRadius):_robotRadius(robotRadius)
        {
            if(X.size() != Y.size() || X.empty()) {
                return;
            }
            
            for (int i = 0; i < X.size(); ++i) {
              std::vector<double> point(2);
              point[0] = X[i];
              point[1] = Y[i];
              nodes.push_back(Kdtree::KdNode(point));
            }
            _kdtree = std::make_unique<Kdtree::KdTree>(&nodes);
            _initialized = true;

            // std::cout << "CollisionChecker initialized with " << nodes.size() << " points." << std::endl;

        }

        CCPtr getSharedPtr()
        {
            return shared_from_this();
        }

        bool isCollision(const std::vector<Eigen::Matrix<double, 5, 1>>&trajectory)
        {
            if (!_initialized) {
                std::cerr << "CollisionChecker not initialized." << std::endl;
                return false; // Not initialized, no collision check
            }

            for (int j = trajectory.size(); j-- > 0;) {
                auto state = trajectory[j];
                std::vector<double> test_point(2);
                test_point[0] = state(0);
                test_point[1] = state(1);
                Kdtree::KdNodeVector result;
                _kdtree->k_nearest_neighbors(test_point, _robotRadius, &result);
                if(!result.empty())
                {
                    // Check if the nearest point is within the robot's radius
                    double dx = result[0].point[0] - test_point[0];
                    double dy = result[0].point[1] - test_point[1];
                    double distance_squared = dx * dx + dy * dy;
                    if (distance_squared < _robotRadius * _robotRadius)
                    {
                        std::cout << "Collision detected at point: (" << test_point[0] << ", " << test_point[1] << ")" << std::endl;
                        return true; // Collision detected
                    }
                }         
            }
            return false;
        }

    private:
        Kdtree::KdNodeVector nodes;
        bool _initialized = false;
        double _robotRadius;
        std::unique_ptr<Kdtree::KdTree> _kdtree;

    };
    using CCPtr = std::shared_ptr<CollisionChecker>;

}
#endif //COLLISIONCHECKER_COLLISIONCHECKER_H
