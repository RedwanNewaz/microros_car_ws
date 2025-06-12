#pragma once 
#include <memory>
#include <string> 
#include <iostream>
struct param_manager : public std::enable_shared_from_this<param_manager> 
{
    double dt;
    double predict_time;
    double robot_radius;
    double goal_radius;
    int pref_speed_index;
    int num_samples;
    double max_speed;
    double min_speed;
    double max_yawrate;
    double map_resolution;

    std::shared_ptr<param_manager> getSharedPtr() {
        return shared_from_this();
    }

    template<class T>
    T get_param(const std::string& field) {
        if(field == "dt") {
            return dt;
        } else if(field == "predict_time") {
            return predict_time;
        } else if(field == "robot_radius") {
            return robot_radius;
        } else if(field == "goal_radius") {
            return goal_radius;
        } else if(field == "pref_speed_index") {
            return pref_speed_index;
        } else if(field == "num_samples") {
            return num_samples;
        } else if(field == "max_speed") {
            return max_speed;
        } else if(field == "min_speed") {
            return min_speed;
        } else if(field == "max_yawrate") {
            return max_yawrate;
        }
        else if(field == "map_resolution") {
            return map_resolution;
        } else {
            throw std::runtime_error("Parameter not found: " + field);
        }
    }
    
};

using ParamPtr = std::shared_ptr<param_manager>;