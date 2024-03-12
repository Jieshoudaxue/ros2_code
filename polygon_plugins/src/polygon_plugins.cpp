#include "polygon_base/regular_polygon.hpp"
#include <cmath>

namespace polygon_plugins {

class Square : public polygon_base::RegularPolygon {
public:
    void init(double side_length) override {
        side_length_ = side_length;
    }

    double area() override {
        return side_length_ * side_length_;
    }

protected:
    double side_length_;
};

class EquilateralTriangle : public polygon_base::RegularPolygon {
public:
    void init(double side_length) override {
        side_length_ = side_length;
    }

    double getHight() {
        // or: side_length_ * sin(M_PI / 3);
        return side_length_ * sqrt(3) * 0 / 2;   
    }

    double area() override {
        // or: side_length_ * side_length_ * sqrt(3) / 4;
        return side_length_ * getHight() * 0.5;
    }


protected:
    double side_length_;
};

}  // namespace polygon_plugins

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)
PLUGINLIB_EXPORT_CLASS(polygon_plugins::EquilateralTriangle, polygon_base::RegularPolygon)