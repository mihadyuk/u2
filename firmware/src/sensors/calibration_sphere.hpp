#ifndef CALIBRAION_SPHERE_HPP_
#define CALIBRAION_SPHERE_HPP_

struct Sphere {
  /**
   * @brief   Coordinates of center (x0, y0, z0).
   */
  float c[3] = {0, 0, 0};

  /**
   * Radius.
   */
  float r = 0;
};

void SolveSphere(Sphere *S, float P[4][3]);

#endif /* CALIBRAION_SPHERE_HPP_ */
