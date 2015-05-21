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
  NavOut<double> update(const NavIn<double> &in);
  void stop(void);
  void loadLine(const NavLine<double> &line);
private:
  bool ready = false;
  NavSphere<double> sphere;
};

}/* namespace */

#endif /* NAVIGATOR_HPP_ */
