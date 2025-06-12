#pragma once 
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <vector>
struct Pixel {
    uint16_t x;
    uint16_t y;

    Pixel(uint16_t x, uint16_t y) : x(x), y(y) {}

    bool operator==(const Pixel& other) const {
        return x == other.x && y == other.y;
    }
};
namespace std {
    template <>
    struct hash<Pixel> {
        size_t operator()(const Pixel& p) const {
            return (static_cast<uint32_t>(p.x) << 16) | p.y;
        }
    };
}


class OccupancyMap {
private:
    std::unordered_set<Pixel> _map;
    std::unordered_map<Pixel, uint32_t> _map_count;
    uint16_t _resolution = 1;
    uint32_t _maxPixelCount = 0;
    double _x_min = 0.0;
    double _x_max = 0.0;
    double _y_min = 0.0;
    double _y_max = 0.0;
    uint16_t ox = 0; // Origin x-coordinate in pixel space
    uint16_t oy = 0; // Origin y-coordinate in pixel space
    double _mapResolution;



public:
    using CLOUD_DATA = std::vector<std::array<double, 3>>;
    OccupancyMap(const std::vector<double>& targetArea, double mapResolution):_mapResolution(mapResolution)
    {
        // Initialize the occupancy map based on the target area
        // targetArea is a vector of 4 doubles: [x_min, x_max, y_min, y_max]
        if (targetArea.size() != 4) {
            throw std::invalid_argument("targetArea must contain exactly 4 elements: [x_min, x_max, y_min, y_max]");
        }
        _x_min = targetArea[0];
        _x_max = targetArea[1];
        _y_min = targetArea[2];
        _y_max = targetArea[3];
        if (_x_min >= _x_max || _y_min >= _y_max) {
            throw std::invalid_argument("Invalid target area: x_min must be less than x_max and y_min must be less than y_max");
        }
    }

    CLOUD_DATA getMap(double probThres) const
    {
        CLOUD_DATA cloud;
        for (const auto& pixel : _map) {
            double x = toWorldCoordX(pixel.x);
            double y = toWorldCoordY(pixel.y);
            double z = _map_count.at(pixel) / static_cast<double>(_maxPixelCount); // Normalize count to [0, 1]
            if(z >= probThres) // Only include points above the probability threshold
                cloud.push_back({x, y, z}); 
        }
        return cloud;
    }

    void insert(double x, double y)
    {
        uint16_t xx = toMapCoordX(x);
        uint16_t yy = toMapCoordY(y);
        markOccupied(xx, yy);
    }

    void reset()
    {
        _map.clear();
        _map_count.clear();
        _maxPixelCount = 0;
    }



    double toWorldCoordX(uint16_t xx) const
    {
        double x = _resolution * (xx - ox) / static_cast<double>(mapWidth());
        return _x_min + x * (_x_max - _x_min);
    }

    double toWorldCoordY(uint16_t yy) const
    {
        double y = _resolution * (yy - oy) / static_cast<double>(mapHeight());
        return _y_min + y * (_y_max - _y_min);
    }

    uint16_t mapWidth() const
    {
        uint16_t scale = (_x_max - _x_min) / _mapResolution;
        return _resolution * scale;
    }
    uint16_t mapHeight() const
    {
        uint16_t scale = (_y_max - _y_min) / _mapResolution;
        return _resolution * scale;
    }

    uint16_t toMapCoordX(double xx) const
    {
        double x = (xx - _x_min) / (_x_max - _x_min);
        return ox + x * mapWidth();
    }

    uint16_t toMapCoordY(double yy) const
    {
        double y = (yy - _y_min) / (_y_max - _y_min);
        return oy +  y * mapHeight();
    }

    // Marks a cell (x,y) as occupied
    void markOccupied(uint16_t x, uint16_t y) {
        auto xx = ceil(x / _resolution);
        auto yy = ceil(y / _resolution);
        Pixel pixel(xx, yy);
        if (_map_count.count(pixel) == 0) {
            _map_count[pixel] = 0;
        }
        _map_count[pixel]++;
        _maxPixelCount = std::max(_maxPixelCount, _map_count[pixel]);
        _map.insert(pixel);
    }


    void set_resolution(uint16_t res)
    {
        this->_resolution = res;
    }


    // Checks if (x,y) is occupied
    bool isOccupied(uint16_t x, uint16_t y) {
        auto xx = ceil(x / _resolution);
        auto yy = ceil(y / _resolution);
        return _map.count(Pixel(xx, yy)) > 0;
    }


};