//
// Created by airlab on 6/9/25.
//

#ifndef COLLISIONCHECKER_COLLISIONCHECKER_H
#define COLLISIONCHECKER_COLLISIONCHECKER_H
#include <vector>
#include <array>
#include <memory>
#include "OccupancyMap.h"
#include <Eigen/Dense>

namespace bow{

    class CollisionChecker : public std::enable_shared_from_this<CollisionChecker> {
    public:
        using CCPtr = std::shared_ptr<CollisionChecker>;
        CollisionChecker(const std::vector<double>& X, const std::vector<double>& Y, double robotRadius, double mapResolution, std::vector<double> boundary)
        {
            _x_min = boundary[0];
            _x_max = boundary[1];
            _y_min = boundary[2];
            _y_max = boundary[3];

            _robotRadius = robotRadius;
            _mapResolution = mapResolution;

            for(int i = 0; i < X.size(); ++i)
            {
                int x = toMapCoordX(X[i]);
                int y = toMapCoordY(Y[i]);
                occupancyMap_.markOccupied(x, y);
            }

        }

        CCPtr getSharedPtr()
        {
            return shared_from_this();
        }

        bool outside_safe_boundary(double x, double y) const
        {
            return (x < safe_boundary_[0] || x > safe_boundary_[1] || y < safe_boundary_[2] || y > safe_boundary_[3]);
        }

        bool isCollision(const std::vector<Eigen::Matrix<double, 5, 1>>&trajectory)
        {

            for (int j = trajectory.size(); j-- > 0;) {
                auto state = trajectory[j];
                double wx = state(0);
                double wy = state(1);

                // if(wx < _x_min || wx > _x_max || wy < _y_min || wy > _y_max) {
                //     return true; // Skip if the state is out of bounds
                // }
                // check if the state is within the safe boundary
                if (outside_safe_boundary(wx, wy)) {
                    return true; // Skip if the state is out of bounds
                }

                int x = toMapCoordX(wx);
                int y = toMapCoordY(wy);
                if (isCollision(x, y)) {
                    return true;
                }
            }
            return false;
        }

        bool isOccupied(int x, int y)
        {
            return occupancyMap_.isOccupied(x, y);
        }
        bool isCollision(int x, int y)
        {
            return occupancyMap_.isCollision(x, y, _robotRadius / _mapResolution);
        }
        int getRobotRadius() const
        {
            return static_cast<int>(_robotRadius / _mapResolution);
        }

        double toWorldCoordX(int xx) const
        {
            double x = (xx - ox) / static_cast<double>(MapWidth());
            return _x_min + x * (_x_max - _x_min);
        }

        double toWorldCoordY(int yy) const
        {
            double y = (yy - oy) / static_cast<double>(MapHeight());
            return _y_min + y * (_y_max - _y_min);
        }

        int toMapCoordX(double xx) const
        {
            double x = (xx - _x_min) / (_x_max - _x_min);
            return ox + x * MapWidth();
        }

        int toMapCoordY(double yy) const
        {
            double y = (yy - _y_min) / (_y_max - _y_min);
            return oy +  y * MapHeight();
        }
        int MapWidth() const
        {
            int scale = (_x_max - _x_min) / _mapResolution;
            return   scale;
        }
        int MapHeight() const
        {
            int scale = (_y_max - _y_min) / _mapResolution;
            return  scale;
        }
    private:
        double _robotRadius;
        double _mapResolution;

        double _x_min, _x_max, _y_min, _y_max;
        int ox = 0;
        int oy = 0;
        OccupancyMap occupancyMap_;
        std::vector<double> safe_boundary_{-3.0, 3.0, -2.8, 2.8}; // x_min, x_max, y_min, y_max
    };
    using CCPtr = std::shared_ptr<CollisionChecker>;

}
#endif //COLLISIONCHECKER_COLLISIONCHECKER_H