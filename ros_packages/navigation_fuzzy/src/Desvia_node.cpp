#include <iostream>
#include "navigation_fuzzy/Desvia.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_fuzzy");

  Desvia desvia("navigation_fuzzy");
  desvia.spin();

  return 0;
}
