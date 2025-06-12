#ifndef PARAM_MANAGER_H
#define PARAM_MANAGER_H

#include <yaml-cpp/yaml.h>
#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>

template<class T>
struct Matrix2D {
    std::vector<std::vector<T>> data;

    bool operator==(const Matrix2D<T>& other) const {
        return other.data.size() == data.size() &&
               (data.empty() || other.data[0].size() == data[0].size());
    }

    friend std::ostream& operator<<(std::ostream& os, const Matrix2D<T>& d) {
        for (const auto& row : d.data) {
            for (const T& value : row) {
                os << value << " ";
            }
            os << "\n";
        }
        return os;
    }
};

namespace YAML {
    template<class T>
    struct convert<Matrix2D<T>> {
    static Node encode(const Matrix2D<T>& rhs) {
        Node node;
        for (const auto& d : rhs.data) {
            node.push_back(d);
        }
        return node;
    }

    static bool decode(const Node& node, Matrix2D<T>& rhs) {
        if (!node.IsSequence()) return false;
        rhs.data.clear();
        for (const auto& temp : node) {
            if (!temp.IsSequence()) return false;
            std::vector<T> cols;
            for (const auto& item : temp) {
                cols.push_back(item.as<T>());
            }
            rhs.data.push_back(cols);
        }
        return true;
    }
};
}

class param_manager : public std::enable_shared_from_this<param_manager> {
public:
    explicit param_manager(const std::string& file) : config_(YAML::LoadFile(file)) {}

    template<class T>
    T get_param(const std::string& field) {
        return config_[field].template as<T>();
    }

    template<class T>
    T get_param(const std::string& field1, const std::string& field2) {
        return config_[field1][field2].template as<T>();
    }

    template<class T>
    T get_param(const std::string& field1, const std::string& field2, const std::string& field3) {
        return config_[field1][field2][field3].template as<T>();
    }

    template<class T>
    void get_obstacles(T& result) {
        Matrix2D<float> obstacles = config_["obstacles"].as<Matrix2D<float>>();
        std::copy(obstacles.data.begin(), obstacles.data.end(), std::back_inserter(result));
    }

    template<class T>
    std::vector<std::vector<T>> get_ndarray(const std::string& field) {
        auto result = config_[field].as<Matrix2D<T>>();
        return result.data;
    }

    std::shared_ptr<param_manager> getSharedPtr() {
        return shared_from_this();
    }

private:
    YAML::Node config_;
};

using ParamPtr = std::shared_ptr<param_manager>;

#endif // PARAM_MANAGER_H
