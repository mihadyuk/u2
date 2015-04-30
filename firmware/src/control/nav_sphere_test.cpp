#include "main.h"

#include "nav_sphere.hpp"

float nav_sphere_test(void){
  NavSphere<float> sphere;

  float latA = deg2rad(53.8735);
  float lonA = deg2rad(27.5022);
  float latB = deg2rad(53.9069);
  float lonB = deg2rad(27.5571);

  /* overshot */
  float latD1 = deg2rad(53.9072);
  float lonD1 = deg2rad(27.5877);
  float heading_over = 0;
  float distance_over = 10000;
  float xtd_over = 0;
  float atd_over = 0;

  /* undershot */
  float latD2 = deg2rad(53.8896);
  float lonD2 = deg2rad(27.5502);
  float heading_under = 0;
  float distance_under = 10000;
  float xtd_under = 0;
  float atd_under = 0;

  bool overshot;

  sphere.updatePoints(latA, lonA, latB, lonB);

  sphere.course(latD1, lonD1, &heading_over, &distance_over);
  sphere.crosstrack(latD1, lonD1, &xtd_over, &atd_over);

  sphere.course(latD2, lonD2, &heading_under, &distance_under);
  sphere.crosstrack(latD2, lonD2, &xtd_under, &atd_under);

  overshot = sphere.isOvershot(latD1, lonD1);
  overshot = sphere.isOvershot(latD2, lonD2);

  /* warning suppressors */
  if (true == overshot)
    return xtd_under * xtd_over;
  else
    return atd_under * atd_over;
}
