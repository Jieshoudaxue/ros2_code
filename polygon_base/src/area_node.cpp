#include "pluginlib/class_loader.hpp"
#include "polygon_base/regular_polygon.hpp"

int main(void) {
  pluginlib::ClassLoader<polygon_base::RegularPolygon> polygon_loader("polygon_base", "polygon_base::RegularPolygon");

  try {
    // "polygon_plugins::Square" 和 "polygon_plugins::EquilateralTriangle" 是插件的类名
    // 插件的类名信息来自插件的 plugins.xml ，在 polygon_plugins 包里面
    // polygon_plugins 使用 pluginlib_export_plugin_description_file(polygon_base plugins.xml) 把插件信息申明出来，运行时加载插件
    std::shared_ptr<polygon_base::RegularPolygon> square = polygon_loader.createSharedInstance("polygon_plugins::Square");
    square->init(1.0);
    printf("sauare area: %f\n", square->area());

    std::shared_ptr<polygon_base::RegularPolygon> equilateral_triangle = polygon_loader.createSharedInstance("polygon_plugins::EquilateralTriangle");
    equilateral_triangle->init(1.0);
    printf("equilateral triangle area: %f\n", equilateral_triangle->area());

  } catch (pluginlib::PluginlibException& ex) {
    printf("plugin failed to load: %s\n", ex.what());
  }

  return 0;
}
