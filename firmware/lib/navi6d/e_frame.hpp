#ifndef E_FRAME_HPP
#define E_FRAME_HPP
//#include <math.h>

/*
 *  DECLARATIONS
 */
#define A_WGS84    6378137.0
#define B_WGS84    6356752.31424518
#define E2_WGS84   0.006694380004260827
#define EP2_WGS84  0.006739496756587 /* (E2_WGS84/(1 - E2_WGS84)) */
#define F_WGS84    0.003352810671831 /* (E2_WGS84/(1 + sqrt(1 - E2_WGS84))) */

template <typename T>
bool geo2ecef(T r_ecef[3][1], T r_geo[3][1]);

template <typename T>
bool ecef2geo(T r_geo[3][1], T r_ecef[3][1]);

template <typename T>
bool geo2lv(T r_lv[3][1], T r_geo[3][1], T r0_geo[3][1]);

template <typename T>
bool lv2geo(T r_geo[3][1], T r_lv[3][1], T r0_geo[3][1]);

template <typename T>
bool lv2ecef(T r_ecef[3][1], T r_lv[3][1], T r0_geo[3][1]);

template <typename T>
bool el_curv(T &Mh, T &Nh, T r_geo[3][1]);

/*
 *  DEFINITIONS
 */
template <typename T>
bool geo2ecef(T r_ecef[3][1], T r_geo[3][1])
{
  T phi,lambda,h,sinphi,N,Nhcp;
  phi = r_geo[0][0];
  lambda = r_geo[1][0];
  h = r_geo[2][0];
  sinphi = sin(phi);
  N  = A_WGS84/sqrt(1.0 - E2_WGS84*sinphi*sinphi);
  Nhcp = (N+h)*cos(phi);
  
  r_ecef[0] = Nhcp*cos(lambda);
  r_ecef[1] = Nhcp*sin(lambda);
  r_ecef[2] = (N*(1.0-E2_WGS84) + h)*sinphi;
  return 1;
//   if (
//       isinf(r_ecef[0]) || isnan(r_ecef[0]) ||
//       isinf(r_ecef[1]) || isnan(r_ecef[1]) ||
//       isinf(r_ecef[2]) || isnan(r_ecef[2])
//      )
//     return CH_FAILED;
//   else
//     return CH_SUCCESS;
}

template <typename T>
bool ecef2geo(T r_geo[3][1], const T r_ecef[3][1]){

  T x, y, z;
  T lambda, rho, beta, phi, sb, cb, sb3, cb3;
  int count = 0;
  const int c_iter = 7;
  x = r_ecef[0][0];
  y = r_ecef[1][0];
  z = r_ecef[2][0];
  lambda = atan2(y,x);
  rho = sqrt(x*x+y*y);
  beta = atan2(z, (1.0 - F_WGS84) * rho);
  while (c_iter < 7){
    sb = sin(beta);
    cb = cos(beta);
    sb3 = sb*sb*sb;
    cb3 = cb*cb*cb;
    phi = atan2(z + B_WGS84 * EP2_WGS84 * sb3,
                rho - A_WGS84 * E2_WGS84  * cb3);
    beta = atan2((1.0 - F_WGS84)*sin(phi), cos(phi));
    count = count + 1;
  }

  T sinphi;
  sinphi = sin(phi);
  T N;
  N = A_WGS84/sqrt(1.0 - E2_WGS84 * sinphi * sinphi);
  T h;
  h = rho * cos(phi) + (z + E2_WGS84 * N * sinphi) * sinphi - N;

  r_geo[0][0] = phi;
  r_geo[1][0] = lambda;
  r_geo[2][0] = h;
  return 1;
//   if (
//       isinf(r1_wgs[0]) || isnan(r1_wgs[0]) ||
//       isinf(r1_wgs[1]) || isnan(r1_wgs[1]) ||
//       isinf(r1_wgs[2]) || isnan(r1_wgs[2])
//      )
//     return CH_FAILED;
//   else
//     return CH_SUCCESS;
}




template <typename T>
bool geo2lv(T r_lv[3][1], T r_geo[3][1], T r0_geo[3][1])
{
  T r0_ecef[3][1],r1_ecef[3][1];
 
  T phi0,lambda0,sp,cp,sl,cl;
  T dr[3];

  phi0 = r0_geo[0][0];
  lambda0 = r0_geo[1][0];
  sp = sin(phi0);
  cp = cos(phi0);
  sl = sin(lambda0);
  cl = cos(lambda0);

  geo2ecef<T>(r0_ecef,r0_geo);
  geo2ecef<T>(r1_ecef,r_geo);
  dr[0] = r1_ecef[0][0]-r0_ecef[0][0];
  dr[1] = r1_ecef[1][0]-r0_ecef[1][0];
  dr[2] = r1_ecef[2][0]-r0_ecef[2][0];

  r_lv[0][0] =  (cp*dr[2] - cl*dr[0]*sp - dr[1]*sl*sp);
  r_lv[1][0] =  (cl*dr[1] - dr[0]*sl);
  r_lv[2][0] = -(dr[2]*sp + cl*cp*dr[0] + cp*dr[1]*sl);
  return true;
// 
//   if (
//       isinf(r[0]) || isnan(r[0]) ||
//       isinf(r[1]) || isnan(r[1]) ||
//       isinf(r[2]) || isnan(r[2])
//      )
//     return CH_FAILED;
//   else
//     return CH_SUCCESS;
}


template <typename T>
bool lv2ecef(T r_ecef[3][1], T r_lv[3][1], T r0_geo[3][1])
{
  T r0_ecef[3][1];
  T phi0,lambda0,sp,cp,sl,cl;
  T r_lv_enu[3];
  phi0 = r0_geo[0][0];
  lambda0 = r0_geo[1][0];
  sp = sin(phi0);
  cp = cos(phi0);
  sl = sin(lambda0);
  cl = cos(lambda0);
  geo2ecef<T>(r0_ecef, r0_geo);
  
  //NED2ENU
  r_lv_enu[0] =  r_lv[1][0];
  r_lv_enu[1] =  r_lv[0][0];
  r_lv_enu[2] = -r_lv[2][0];
  r_ecef[0][0]   = -sl*r_lv_enu[0] - cl*sp*r_lv_enu[1] + cl*cp*r_lv_enu[2] + r0_ecef[0][0];
  r_ecef[1][0]   =  cl*r_lv_enu[0] - sl*sp*r_lv_enu[1] + cp*sl*r_lv_enu[2] + r0_ecef[1][0];
  r_ecef[2][0]   =                   cp*r_lv_enu[1]    + sp*r_lv_enu[2]    + r0_ecef[2][0];
  return 1;
//   if (
//       isinf(r_ecef[0]) || isnan(r_ecef[0]) ||
//       isinf(r_ecef[1]) || isnan(r_ecef[1]) ||
//       isinf(r_ecef[2]) || isnan(r_ecef[2])
//      )
//     return CH_FAILED;
//   else
//     return CH_SUCCESS;
}

template <typename T>
bool lv2geodetic(T r_geo[3][1], T r_lv[3][1], T r0_geo[3][1])
{
  T r_ecef[3][1];
  bool lv2ecef_res, ecef2geo_res;
  lv2ecef_res = lv2ecef<T>(r_ecef, r_lv, r0_geo);
  ecef2geo_res = ecef2geo<T>(r_geo, r_ecef);
  if (
      (lv2ecef_res == 0) ||
      (ecef2geo_res == 0)
     )
    return 0;
  else
    return 1;
}


template <typename T>
bool el_curv(T &Mh, T &Nh, T r_geo[3][1])
{
  T tmp, cp, sp, acp, bsp, ab, sqrt_tmp, phi, h;
  phi = r_geo[0][0];
  h = r_geo[2][0];
  ab = A_WGS84*B_WGS84;
  cp = cos(phi);
  sp = sin(phi);
  acp = A_WGS84*cp;
  bsp = B_WGS84*sp;
  tmp = acp*acp + bsp*bsp;
  sqrt_tmp = sqrt(tmp);
  Mh = ab*ab/(sqrt_tmp*sqrt_tmp*sqrt_tmp) + h;
  Nh = A_WGS84*A_WGS84/sqrt_tmp + h;
  return 1;
//   if (
//       isinf(*M) || isnan(*M) ||
//       isinf(*N) || isnan(*N)
//      )
//     return CH_FAILED;
//   else
//     return CH_SUCCESS;
}

#endif //E_FRAME_HPP
