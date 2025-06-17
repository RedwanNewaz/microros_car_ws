#pragma once
#include "bow_param.h"
#include "utils/RosParamManager.h"
#include "collision_checker/CollisionChecker.h"
#include <vector>
#include <array>
#include <Eigen/Core>
#include <random>
#include <chrono>


namespace bow {
    using Traj = std::vector<Eigen::Matrix<double, 5, 1>>;
    using Obstacle = std::vector<Eigen::Vector2d>;
    using State = Eigen::Matrix<double, 5, 1>;
    using Point = Eigen::Vector2d;
    using Control = Eigen::Vector2d;

    class BOPlanner {
    public:
        // Bayesian Optimization parameters
        BO_PARAM(size_t, dim_in, 2);
        BO_PARAM(size_t, dim_out, 1);
        BO_PARAM(size_t, nb_constraints, 1);

        // Constructor
        BOPlanner(const State& x, const Point& goal, const CCPtr& cc, const ParamPtr& pm_);

        // compute optimal control for a finite planning horizon
        Eigen::Vector2d computeControl();

        //  Public interface
        std::pair<bool, Traj> solve(double time, bool verbose=true);

        // Operator for Bayesian optimization
        Eigen::VectorXd operator()(const Eigen::VectorXd& u) const;

        Traj calcTrajectory(State x, double v, double y, const Point& goal) const;

    private:
        // Member variables
        Point goal_;
        State x_;
        CCPtr cc_;
        ParamPtr pm_;

        // yaml config parameters
        double max_speed_;
        double min_speed_;
        double max_yawrate_;
        double dt_;
        double goal_radius_;
        double robot_radius_;
        double predict_time_;
        int pref_speed_index_;


        // Helper functions
        Eigen::Vector2d scaledU(const Eigen::VectorXd& u) const;
        // ODE-based motion model
        State motionODE(const State& x, const Control& u, double dt) const;


        
        std::pair<int, double> calcToGoalCost(const Traj& traj, const Point& goal) const;
        // Custom Runge-Kutta 4 integration method
        State rungeKutta4Integration(
                const std::function<void(const State&, State&, double)>& derivatives,
                const State& initial_state,
                double t0,
                double dt
        )const;
    };

} // namespace bow