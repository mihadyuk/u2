#ifndef NAVIGATOR_HPP_
#define NAVIGATOR_HPP_

#include "nav_sphere.hpp"
#include "navigator_types.hpp"

namespace control {

/**
 *
 */
class Navigator {
public:
  Navigator(void);
  NavOut<float> update(const NavIn<float> &in);
  void stop(void);
  void loadLine(const NavLine<float> &line);
private:
  bool ready = false;
  NavSphere<float> sphere;
};

}/* namespace */

#endif /* NAVIGATOR_HPP_ */
