#include "bow/BOW.h"
#include "bow/bow_param.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <boost/fusion/include/vector.hpp>
#include <limbo/tools/macros.hpp>
#include <limbo/tools/parallel.hpp>
#include <limbo/experimental/bayes_opt/cboptimizer.hpp>
int limbo::Params::num_samples = 15;
namespace bow {

    BOPlanner::BOPlanner(const State& x, const Point& goal, const CCPtr & cc, const ParamPtr & pm)
            : goal_(goal)
            , x_(x)
            , cc_(cc)
            , pm_(pm)
    {

        max_speed_      = pm_->get_param<double>("max_speed");
        min_speed_      = pm_->get_param<double>("min_speed");
        max_yawrate_    = pm_->get_param<double>("max_yawrate");
        dt_             = pm_->get_param<double>("dt");
        goal_radius_    = pm_->get_param<double>("goal_radius");
        robot_radius_   = pm_->get_param<double>("robot_radius");
        predict_time_   = pm_->get_param<double>("predict_time");
        pref_speed_index_ = pm_->get_param<double>("pref_speed_index");
        limbo::Params::num_samples = pm_->get_param<double>("num_samples");
    }


    Eigen::VectorXd BOPlanner::operator()(const Eigen::VectorXd& u) const {
        Eigen::VectorXd res(2);

        // limbo sample u in [0, 1] range which needs to map/scale to the robot's vel domains
        Eigen::Vector2d uu = scaledU(u);

        // feed forward trajectory for a sampled control for a given predict_time
        Traj traj = calcTrajectory(x_, uu(0), uu(1),  goal_);
        auto[index, goalDist] = calcToGoalCost(traj, goal_);
        // erase the part of trajectory that does not help to reach goal location
        traj.erase(traj.begin() + index + 1, traj.end());
        // add safety constraint in terms of distance between robot and nearset obstacles
        res(1) = 1.0 - static_cast<float>(cc_->isCollision(traj));
        // convert the cost function to positive reward
        res(0) = (res(1) > 0.0 ) ? -goalDist : -1.0e3;
        return res;
    }

    Eigen::Vector2d BOPlanner::scaledU(const Eigen::VectorXd& u) const {
        Eigen::Vector2d uu;
        uu(0) = min_speed_ + (max_speed_ - min_speed_) * u(0);
        // yaw angle is symmetric (there is no min_yaw rate)
        uu(1) = -max_yawrate_ + 2 * max_yawrate_ * u(1);
        return uu;
    }

    State BOPlanner::rungeKutta4Integration(
            const std::function<void(const State&, State&, double)>& derivatives,
            const State& initial_state,
            double t0,
            double dt
    )const {
        State k1, k2, k3, k4;
        State state = initial_state;

        // k1
        derivatives(state, k1, t0);
        k1 *= dt;

        // k2
        State temp_state = state + 0.5 * k1;
        derivatives(temp_state, k2, t0 + 0.5 * dt);
        k2 *= dt;

        // k3
        temp_state = state + 0.5 * k2;
        derivatives(temp_state, k3, t0 + 0.5 * dt);
        k3 *= dt;

        // k4
        temp_state = state + k3;
        derivatives(temp_state, k4, t0 + dt);
        k4 *= dt;

        // Final state update
        state += (k1 + 2*k2 + 2*k3 + k4) / 6.0;

        return state;
    }


    // ODE-based motion model using differential equations
    State BOPlanner::motionODE(const State& x, const Control& u, double dt) const {
        // Define the state derivative function
        auto stateDeriv = [&](const State& state, State& dxdt, double) {
            // Kinematic bicycle model differential equations
            dxdt(0) = state(3) * std::cos(state(2)); // dx/dt = v * cos(theta)
            dxdt(1) = state(3) * std::sin(state(2)); // dy/dt = v * sin(theta)
            dxdt(2) = state(4); // dtheta/dt = omega (yaw rate)
            dxdt(3) = 0; // dv/dt = 0 (constant velocity control)
            dxdt(4) = 0; // domega/dt = 0 (constant yaw rate control)
        };

        // Use Eigen's RungeKutta4 integrator for numerical integration
        State next_state = rungeKutta4Integration(stateDeriv, x, 0, dt);

        // Normalize theta to [-pi, pi]
        next_state(2) = std::fmod(next_state(2) + M_PI, 2 * M_PI) - M_PI;

        // Directly set the control inputs
        next_state(3) = u(0);
        next_state(4) = u(1);

        return next_state;
    }


    Traj BOPlanner::calcTrajectory(State x, double v, double y, const Point& goal) const {
        // Pre-allocate maximum possible trajectory size to avoid reallocations
        Traj traj;
        traj.reserve(static_cast<size_t>(predict_time_ / dt_) + 1);

        // Use Eigen's aligned vector for better performance
        Eigen::Matrix<double, 2, 1, Eigen::DontAlign> goal_vec = goal;

        // Use Eigen's vector operations for distance calculation
        traj.push_back(x);

        // Use Eigen's vectorized operations
        const Control u = Eigen::Vector2d(v, y);

        // Preallocate variables to avoid repeated memory allocations
        double time = 0.0;
        Eigen::Matrix<double, 2, 1, Eigen::DontAlign> pos_diff;

        // Use Eigen::NumTraits for better numerical stability
        double distance = Eigen::NumTraits<double>::highest();

        // Use vectorized operations and early termination
        while (time <= predict_time_ && distance > goal_radius_) {
            // Optimize motion calculation
            x = motionODE(x, u, dt_);

            // Vectorized distance calculation
            pos_diff = goal_vec - x.head<2>();
            distance = pos_diff.norm();

            traj.push_back(x);
            time += dt_;
        }

        // Optional: Shrink to fit to release extra memory
        traj.shrink_to_fit();

        return traj;
    }

    std::pair<int, double> BOPlanner::calcToGoalCost(const Traj& traj, const Point& goal) const {
        // Early exit for empty trajectory
        if (traj.empty()) {
            return {-1, std::numeric_limits<double>::max()};
        }

        // Use Eigen's vectorized operations
        Eigen::Map<const Eigen::MatrixXd> trajectory_matrix(
                traj[0].data(),
                5,
                static_cast<Eigen::Index>(traj.size())
        );

        // Extract x and y coordinates in a vectorized manner
        Eigen::VectorXd x_coords = trajectory_matrix.row(0);
        Eigen::VectorXd y_coords = trajectory_matrix.row(1);

        // Vectorized distance calculation
        Eigen::VectorXd distances =
                ((x_coords.array() - goal[0]).square() +
                 (y_coords.array() - goal[1]).square()).sqrt();

        // Find minimum distance and its index
        double minDist;
        Eigen::Index bestIndex;
        distances.minCoeff(&bestIndex);
        minDist = distances(bestIndex);

        return {static_cast<int>(bestIndex), minDist};
    }



    Eigen::Vector2d BOPlanner::computeControl() {
        using namespace limbo;
        using Stop_t = boost::fusion::vector<stop::MaxIterations<Params>>;
        using Stat_t = boost::fusion::vector<
                stat::Samples<Params>,
        stat::BestObservations<Params>,
                stat::AggregatedObservations<Params>
                >;
//        using Mean_t = mean::Data<Params>;
        using Kernel_t = kernel::SquaredExpARD<Params>;
        using Mean_t = mean::Constant<Params>;
//        using Kernel_t = kernel::Exp<Params>;
        using GP_t = model::GP<Params, Kernel_t, Mean_t>;
        using Constrained_GP_t = model::GP<Params, Kernel_t, Mean_t>;
        using Acqui_t = experimental::acqui::ECI<Params, GP_t, Constrained_GP_t>;
        using Init_t = init::RandomSampling<Params>;

        tools::par::init();

        experimental::bayes_opt::CBOptimizer<
                Params,
        modelfun<GP_t>,
                acquifun<Acqui_t>,
                statsfun<Stat_t>,
                initfun<Init_t>,
                stopcrit<Stop_t>,
                experimental::constraint_modelfun<Constrained_GP_t>
                > opt;
        opt.optimize(*this);
        auto uu = opt.best_sample();
        Eigen::Vector2d  uu_scaled = scaledU(uu);
        return  uu_scaled;
    }

    std::pair<bool, Traj> BOPlanner::solve(double time, bool verbose) {

        auto terminate = [&](const State& x)
        {
            auto dx = x(0) - goal_(0);
            auto dy = x(1) - goal_(1);
            auto dist = sqrt(dx * dx + dy * dy);
            return dist < goal_radius_;
        };


        // Excute planner and Record end time
        auto start_time = std::chrono::high_resolution_clock::now();
        Traj final_result;
        bool solution_found = false;
        do{
            BOPlanner mpc(x_, goal_, cc_->getSharedPtr(), pm_->getSharedPtr());
            auto u = mpc.computeControl();
            auto traj = calcTrajectory(x_, u(0), u(1),  goal_);
         
            if(!cc_->isCollision(traj) && !traj.empty())
            {
//                 instead of one step, we can use preferred speed
                int N = std::min((int) traj.size() - 1, pref_speed_index_);
                for(int i = 0; i < x_.size(); i++)
                    x_[i] = traj[N](i);
                std::copy(traj.begin(), traj.begin() + N, std::back_inserter(final_result));
            }
            solution_found = terminate(x_);
            auto end_time = std::chrono::high_resolution_clock::now();
            auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
            if (elapsed_time > time) {
                    std::cout << "[BOW]: Time limit reached, stopping optimization." << std::endl;
                break;
            }
        }  while (!solution_found);

        // update current state using state transition function
        auto end_time = std::chrono::high_resolution_clock::now();
        // Calculate duration
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        if(verbose)
            std::cout << "[BOW]: Solution found in time: " << elapsed_time << " ms" << std::endl;

        return std::make_pair(solution_found, final_result);
    }

} // namespace bow