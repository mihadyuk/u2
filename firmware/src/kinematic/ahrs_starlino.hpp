#ifndef AHRS_STARLINO_H_
#define AHRS_STARLINO_H_


/******************************************************************
* DCM helpers */
/*
 *       | Rxx Rxy Rxz |
 * DCM = | Ryx Ryy Ryz |
 *       | Rzx Rzy Rzz |
 */

/**
 *
 */
class AHRSStarlino {
public:
  AHRSStarlino(void);
  void start(void);
  void stop(void);
  void update(float *euler, const float *acc, const float *gyro, const float *mag, float dT);
  float Rxx(void) const {return dcmEst[0][0];}
  float Rxy(void) const {return dcmEst[1][0];}
  float Rxz(void) const {return dcmEst[2][0];}
  float Ryx(void) const {return dcmEst[0][1];}
  float Ryy(void) const {return dcmEst[1][1];}
  float Ryz(void) const {return dcmEst[2][1];}
  float Rzx(void) const {return dcmEst[0][2];}
  float Rzy(void) const {return dcmEst[1][2];}
  float Rzz(void) const {return dcmEst[2][2];}
private:
  float dcmEst[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
  const float   *magweight = nullptr;
  const float   *accweight = nullptr;
  size_t step = 0;  /* increments on each call to imu_update */
  bool ready = false;
};

#endif /* AHRS_STARLINO_H_ */
