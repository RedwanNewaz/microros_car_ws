//
// Created by airlab on 6/9/25.
//

#ifndef COLLISIONCHECKER_COLLISIONCHECKER_H
#define COLLISIONCHECKER_COLLISIONCHECKER_H
#include <vector>
#include <array>
#include <memory>
#include <Eigen/Dense>
#include <list>
#include "olcUTIL_QuadTree.h"

namespace bow{

    class CollisionChecker : public std::enable_shared_from_this<CollisionChecker> {

    public:
        using CCPtr = std::shared_ptr<CollisionChecker>;
        CollisionChecker(const std::vector<double>& X, const std::vector<double>& Y, double robotRadius, std::vector<double> boundary)
        {
            _x_min = boundary[0];
            _x_max = boundary[1];
            _y_min = boundary[2];
            _y_max = boundary[3];

            _robotRadius = robotRadius;
            double radii = _robotRadius;

            for(int i = 0; i < X.size(); ++i)
            {
                obstacle obs;
                obs.id = i + 1;
                obs.type = 2;
                obs.x = X[i];
                obs.y = Y[i];
                obs.width = radii;
                obs.height = radii;
                olc::utils::geom2d::rect<float> rect{{obs.x, obs.y}, {obs.width, obs.height}};
                obstacles_.insert(obs, rect);
            }

        }

        CCPtr getSharedPtr()
        {
            return shared_from_this();
        }

        bool isCollision(const std::vector<Eigen::Matrix<double, 5, 1>>&trajectory)
        {

            for (int j = trajectory.size(); j-- > 0;) {
                auto state = trajectory[j];
                double wx = state(0);
                double wy = state(1);
                if(isWorkspaceCollision(wx, wy, _robotRadius))
                    return true;
            }
            return false;
        }

        bool isWorkspaceCollision(double wx, double wy, double length)
        {
            if(wx < _x_min || wx > _x_max || wy < _y_min || wy > _y_max)
                return true;
            obstacle robot;
            robot.id = 0;
            robot.type = 2;
            robot.x = wx;
            robot.y = wy;
            robot.width = length ;
            robot.height = length;
            olc::utils::geom2d::rect<float> rect{{robot.x, robot.y}, {robot.width, robot.height}};

            auto potential_collisions = obstacles_.search(rect);
            if(!potential_collisions.empty())
            {
                for(auto& it: potential_collisions)
                {
                    return true; // collision detected
                }
            }
            return false;
        }

        bool outside_safe_boundary(double wx, double wy)
        {
            return (wx < _x_min || wx > _x_max || wy < _y_min || wy > _y_max);
        }


    private:
        double _robotRadius;
        double _x_min, _x_max, _y_min, _y_max;
        struct obstacle{
            int id;
            int type;
            float x;
            float y;
            float width;
            float height;
        };
        olc::utils::QuadTreeContainer<obstacle> obstacles_;
    };
    using CCPtr = std::shared_ptr<CollisionChecker>;

}
#endif //COLLISIONCHECKER_COLLISIONCHECKER_H