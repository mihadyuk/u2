#ifndef SINS_ERROR_HPP
#define SINS_ERROR_HPP
#include "matrix_math.hpp"
#include "att.hpp"
#include "qmath.hpp"
#include "wgs84.hpp"

struct KalmanFlags{
  bool sns_r_en;  //0
  bool sns_h_en;  //1
  bool sns_v_n_en;  //2
  bool sns_v_e_en;  //3
  bool sns_v_d_en;  //4
  bool odo_en;    //5
  bool nonhol_y_en; //6
  bool nonhol_z_en; //7
  bool alt_b_en;  //8
  bool baro_fix_en; //9
  bool mag_en;    //10
  bool roll_en;   //11
  bool pitch_en;  //12
  bool course_en; //13
  bool zihr_en;   //14

  KalmanFlags():
    sns_r_en(false), sns_h_en(false), sns_v_n_en(false), sns_v_e_en(false), sns_v_d_en(false),
    odo_en(false), nonhol_y_en(false), nonhol_z_en(false),
    alt_b_en(false), baro_fix_en(false), mag_en(false),
    roll_en(false), pitch_en(false),
    course_en(false), zihr_en(false)
  {
  }
  
};

template <typename T, int nX, int nQm>  
void set_FG_psi(T F[nX][nX], T G[nX][nQm], T r[3][1],
                T v[3][1], T qnb[4][1], T fb[3][1], T wb[3][1], T dT){
  T Cnb[3][3], Wcec[3][1], Wcie[3][1], Wcic[3][1], Wcec_skew[3][3], Wcie_skew[3][3], Wcic_skew[3][3];
  T I[3][3], Z[3][3];
  T I6[6][6], Z36[3][6];         
  T fn[3][1];
  T fn_skew[3][3];
  T Mh, Nh, Re, h;
  m_set<T,nX,nX>(F,0.0);
  
  h = r[2][0];
  const T g0 = 9.7803267714;
  el_curv(Mh, Nh, r);
  Re = sqrt((Mh-h)*(Nh-h));
  m_eye<T,3>(I);
  m_zeros<T,3,3>(Z);
  m_eye<T,6>(I6);
  m_zeros<T,3,6>(Z36);
  
  quat2dcm(Cnb,qnb);
  
  m_mul<T,3,3,1>(fn,Cnb,fb);
    
  get_WcecWcie<T>(Wcec, Wcie, r, v);
  m_plus<T,3,1>(Wcic,Wcec,Wcie);
  
  cpm<T>(Wcec_skew, Wcec);
  cpm<T>(Wcie_skew, Wcie);
  cpm<T>(Wcic_skew, Wcic);
  cpm<T>(fn_skew, fn);
   
  //// Fr
  //Frr
  set_sub<T,nX,nX,3,3>(F, Wcec_skew, 0, 0, -1.0);
  //Frv
  set_sub<T,nX,nX,3,3>(F, I, 0, 3, 1.0);
  //Fre
  set_sub<T,nX,nX,3,3>(F, Z, 0, 6, 1.0);
  if (nX > 9){
  //Frba
  set_sub<T,nX,nX,3,3>(F, Z, 0, 9, 1.0);
  //Frbw
  set_sub<T,nX,nX,3,3>(F, Z, 0, 12, 1.0);
  }
  
  if (nX > 15){
  //Frsa
  set_sub<T,nX,nX,3,3>(F, Z, 0, 15, 1.0);
  //Frsw
  set_sub<T,nX,nX,3,3>(F, Z, 0, 18, 1.0);
  }
  
  if (nX > 21){
  //Frga
  set_sub<T,nX,nX,3,6>(F, Z36, 0, 21, 1.0);
  //Frgw
  set_sub<T,nX,nX,3,6>(F, Z36, 0, 27, 1.0);
  }
  
  //// Fv
  //Fvr
  T Fvr[3][3];
  Fvr[0][0] = -g0/Mh; Fvr[0][1] =  0.0;   Fvr[0][2] = 0.0;
  Fvr[1][0] =  0.0;   Fvr[1][1] = -g0/Mh; Fvr[1][2] = 0.0;
  Fvr[2][0] =  0.0;   Fvr[2][1] =  0.0;   Fvr[2][2] = 2.0*g0/(Re+h);
  set_sub<T,nX,nX,3,3>(F, Fvr, 3, 0, 1.0); //(0-2)
  //Fvv
  T Fvv[3][3];
  m_plus<T,3,3>(Fvv,Wcie_skew,Wcic_skew);
  set_sub<T,nX,nX,3,3>(F, Fvv, 3, 3, -1.0); //(3-5)
  //Fve
  set_sub<T,nX,nX,3,3>(F, fn_skew, 3, 6, 1.0);//(6-8)
  if (nX >9){
  //Fvba
  set_sub<T,nX,nX,3,3>(F, Cnb, 3, 9, 1.0);//(9-11)
  //Fvbw --Z
  }
  
  if (nX > 15){
  //Fvsa
  T diag_fb[3][3];
  T Fvsa[3][3];
  m_diag<T,3>(diag_fb, fb);
  m_mul<T,3,3,3>(Fvsa,Cnb,diag_fb);
  set_sub<T,nX,nX,3,3>(F, Fvsa, 3, 15, 1.0);//(15-17)
  //Fvsw --Z
  }
  
  if (nX > 21){
  //Fvga
  T Ga[3][6], Fvga[3][6];
  m_non_ort(Ga, fb);
  m_mul<T,3,3,6>(Fvga,Cnb,Ga);
  set_sub<T,nX,nX,3,6>(F, Fvga, 3, 21, 1.0);
  //Fvgw --Z36
  }

  //// Fe
  //Fer --Z
  //Fev --Z
  //Fee
  if (nX > 9){
    set_sub<T,nX,nX,3,3>(F, Wcic_skew, 6, 6, -1.0);
    //Feba --Z
    //Febw
    set_sub<T,nX,nX,3,3>(F, Cnb, 6, 12, -1.0);
  }
  if (nX > 15){
  //Fesa --Z
  //Fesw
  T diag_wb[3][3];
  T Fesw[3][3];
  m_diag<T,3>(diag_wb, wb);
  m_mul<T,3,3,3>(Fesw,Cnb,diag_wb);
  set_sub<T,nX,nX,3,3>(F, Fesw, 6, 18, -1.0);
  }
  if (nX > 21){
  //Fega --Z36
  //Fegw
  T Gw[3][6], Fegw[3][6];
  m_non_ort(Gw, wb);
  m_mul<T,3,3,6>(Fegw,Cnb,Gw);
  set_sub<T,nX,nX,3,6>(F, Fegw, 6, 27, -1.0);
  }
  
  m_mul_s<T,nX,nX>(F, F, dT);
  m_plus_eye<T,nX>(F);
  
  //G
  m_set<T,nX,nQm>(G,0.0);
  
  set_sub<T,nX,nQm,3,3>(G, Cnb, 3, 0, 1.0);
  set_sub<T,nX,nQm,3,3>(G, Cnb, 6, 3, -1.0);
  if (nX > 9){
    set_sub<T,nX,nQm,3,3>(G, I, 9, 6, 1.0);
    set_sub<T,nX,nQm,3,3>(G, I, 12, 9, 1.0);
  }
  if (nX > 15){
    set_sub<T,nX,nQm,3,3>(G, I, 15, 12, 1.0);
    set_sub<T,nX,nQm,3,3>(G, I, 18, 15, 1.0);
  }
  if (nX > 21){
    set_sub<T,nX,nQm,6,6>(G, I6, 21, 18, 1.0);
    set_sub<T,nX,nQm,6,6>(G, I6, 27, 24, 1.0);
  }

}

template <typename T>
void set_Hang(T Hang[3][3], T Cbv[3][3], T Cnb[3][3], T phi[3][1]){
  T c11 = Cnb[0][0], c12 = Cnb[0][1], c13 = Cnb[0][2],
    c21 = Cnb[1][0], c22 = Cnb[1][1], c23 = Cnb[1][2],
    c31 = Cnb[2][0], c32 = Cnb[2][1], c33 = Cnb[2][2];

  T b11 = Cbv[0][0], b12 = Cbv[0][1], b13 = Cbv[0][2],
    b21 = Cbv[1][0], b22 = Cbv[1][1], b23 = Cbv[1][2],
    b31 = Cbv[2][0], b32 = Cbv[2][1], b33 = Cbv[2][2];

  T phix = phi[0][0];
  T phiy = phi[1][0];
  T phiz = phi[2][0];

  T t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,
    t11,t12,t13,t14,t15,t16,t17,t18,t19,t20,t21;

  t1  = b12*c21 + b22*c22 + b32*c23;
  t2  = b11*c21 + b21*c22 + b31*c23;
  t3  = c31 + c11*phiy - c21*phix;
  t4  = c32 + c12*phiy - c22*phix;
  t5  = c33 + c13*phiy - c23*phix;
  t6  = c21 - c11*phiz + c31*phix;
  t7  = c22 - c12*phiz + c32*phix;
  t8  = c23 - c13*phiz + c33*phix;
  t9  = c11 + c21*phiz - c31*phiy;
  t10 = c12 + c22*phiz - c32*phiy;
  t11 = c13 + c23*phiz - c33*phiy;

  t12 = b13*t3 + b23*t4 + b33*t5;
  t13 = b13*c21 + b23*c22 + b33*c23;
  t14 = b11*c31 + b21*c32 + b31*c33;
  t15 = b12*t3 + b22*t4 + b32*t5;
  t16 = b11*t3 + b21*t4 + b31*t5;
  t17 = b11*t6 + b21*t7 + b31*t8;
  t18 = b11*t9 + b21*t10 + b31*t11;
  t19 = b12*c11 + b22*c12 + b32*c13;
  t20 = b11*c11 + b21*c12 + b31*c13;
  t21 = b13*c11 + b23*c12 + b33*c13;

  Hang[0][0] = -(t1/t12 - (t13*t15)/t12*t12)/(t15*t15/t12*t12 + 1.0);
  Hang[0][1] =  (t19/t12 - (t15*t21)/t12*t12)/(t15*t15/t12*t12 + 1.0);
  Hang[0][2] = 0.0;

  Hang[1][0] =  (t2/t12 - (t13*t16)/t12*t12)/sqrt((1.0 - t16*t16/t12*t12));
  Hang[1][1] = -(t20/t12 - (t16*t21)/t12*t12)/sqrt((1.0 - t16*t16/t12*t12));
  Hang[1][2] = 0.0;

  Hang[2][0] = t14/(t18*(t17*t17/t18*t18 + 1.0));
  Hang[2][1] = (t14*t17)/(t18*t18*(t17*t17/t18*t18 + 1.0));
  Hang[2][2] = -(t20/t18 + (t2*t17)/t18*t18)/(t17*t17/t18*t18 + 1.0);
}

template <typename T, int nY, int nX>  
void set_H(T H[nY][nX], KalmanFlags flg, T qnb[4][1], T qbv[4][1], T r[3][1], T v[3][1], T mn[3][1], T wb[3][1],
           T phi[3][1], T l_sns[3][1], T l_wheel[3][1], T dT){
  T Cnb[3][3];
  T Cbn[3][3];
  T Cbv[3][3];
  T Cvb[3][3];
  
  T qbn[4][1];
  T qvb[4][1];
  T qnv[4][1];
  T I[3][3]; 
  T Z1_nX[1][nX];   
  m_zeros<T,1,nX>(Z1_nX); 
  
  m_eye<T,3>(I);
  m_set<T,nY,nX>(H,0.0);
  qcon<T>(qbn, qnb);
  qcon<T>(qvb, qbv);
  quat2dcm<T>(Cbn,qbn);
  quat2dcm<T>(Cnb,qnb);
  quat2dcm<T>(Cbv,qbv);
  quat2dcm<T>(Cvb,qvb);
  
  qmult<T>(qnv,qnb,qbv);
  
  T l_sns_skew[3][3];
  T Cnb_l_skew[3][3];
  cpm<T>(l_sns_skew, l_sns);
  m_mul<T,3,3,3>(Cnb_l_skew, Cnb, l_sns_skew);
  
  //// 0-5
  set_sub<T,nY,nX,3,3>(H, I, 0, 0, 1.0);
  set_sub<T,nY,nX,3,3>(H, Cnb_l_skew, 0, 6, 1.0);
  if (flg.sns_r_en == false){
    set_sub<T,nY,nX,1,nX>(H, Z1_nX, 0, 0, 1.0);
    set_sub<T,nY,nX,1,nX>(H, Z1_nX, 1, 0, 1.0);
  }
  
  //disable sns altitude measurement
  if (flg.sns_h_en == false){
    set_sub<T,nY,nX,1,nX>(H, Z1_nX, 2, 0, 1.0);
  }
  
  
  set_sub<T,nY,nX,3,3>(H, I, 3, 3, 1.0);
    
  T Mh, Nh;
  T Wcec[3][1];
  T Wcie[3][1];
  T Wcic[3][1];
  T tmp1[3][3];
  T tmp2[3][3];
  T tmp3[3][3];
  T tmp4[3][1];
  T Z_1_nX[1][nX];
  m_zeros<T,1,nX>(Z_1_nX);
  el_curv(Mh, Nh, r);  
  get_WcecWcie<T>(Wcec, Wcie, r, v);
  m_plus<T,3,1>(Wcic,Wcec,Wcie);
  cpm<T>(tmp1,Wcic); //tmp1 = Wcic_skew
  m_mul<T,3,3,3>(tmp2, tmp1, Cnb_l_skew); //tmp2 = Wcic_skew*(Cnb*l_sns_skew)
  m_mul<T,3,3,1>(tmp4, Cnb_l_skew, wb); //tmp1 = Cnb_l_skew*wb
  cpm<T>(tmp3,tmp4); //tmp3 = tmp4_skew
  m_plus<T,3,3>(tmp1,tmp2,tmp3);
  set_sub<T,nY,nX,3,3>(H, tmp1, 3, 6, -1.0);
    
  //
  if (nX > 12){
    set_sub<T,nY,nX,3,3>(H, Cnb_l_skew, 3, 12, -1.0);
  }
    
  //
  if (nX > 12){
    set_sub<T,nY,nX,3,3>(H, Cnb_l_skew, 3, 12, -1.0);
  }
  if (flg.sns_v_n_en == false){ 
    set_sub<T,nY,nX,1,nX>(H, Z_1_nX, 3, 0, 1.0);
  }
  if (flg.sns_v_e_en == false){ 
    set_sub<T,nY,nX,1,nX>(H, Z_1_nX, 4, 0, 1.0);
  }
  if (flg.sns_v_d_en == false){ 
    set_sub<T,nY,nX,1,nX>(H, Z_1_nX, 5, 0, 1.0);
  }
  
    //
    
  
  
  if (nY > 6){
    //// 6-8
    // Odometer
    T Hve[3][3];
    T Hvbw[3][3];
    T v_skew[3][3];
    T l_wheel_skew[3][3];
    T Cvn[3][3];
    cpm<T>(v_skew, v);   
    m_mul<T,3,3,3>(Cvn,Cvb,Cbn);  
    m_mul<T,3,3,3>(Hve,Cvn,v_skew);
  
    cpm<T>(l_wheel_skew, l_wheel);
    //Hvv
    set_sub<T,nY,nX,3,3>(H, Cvn, 6, 3, 1.0);
    //Hve
    set_sub<T,nY,nX,3,3>(H, Hve, 6, 6, -1.0);
    
    if (nX > 12){
      //Hvbw
      m_mul<T,3,3,3>(Hvbw,Cvb,l_wheel_skew);
      set_sub<T,nY,nX,3,3>(H, Hvbw, 6, 12, -1.0);
    }
  
    if (flg.odo_en == false){
      set_sub<T,nY,nX,1,nX>(H, Z1_nX, 6, 0, 1.0);
    }
  
    //non-holonomic
    if (flg.nonhol_y_en == false){    
      set_sub<T,nY,nX,1,nX>(H, Z1_nX, 7, 0, 1.0);
    }
  
    if (flg.nonhol_z_en == false){    
      set_sub<T,nY,nX,1,nX>(H, Z1_nX, 8, 0, 1.0);
    }
  
    //// 9
    if (flg.alt_b_en == true){
      H[9][2] = 1.0;
    } 
  
    //10-12
    if (flg.mag_en == true){
      T mn_skew[3][3];  
      T Hme[3][3];
      cpm<T>(mn_skew, mn);
      m_mul<T,3,3,3>(Hme,Cbn,mn_skew);
      //Hme
      set_sub<T,nY,nX,3,3>(H, Hme, 10, 6, -1.0);
    }
  
    //
    //13-15
    T Hang[3][3];
    T Z13[1][3];
    m_zeros<T,1,3>(Z13);
    set_Hang(Hang, Cbv, Cnb, phi);
    set_sub<T,nY,nX,3,3>(H, Hang, 13, 6, 1.0);
  
    if (flg.roll_en == false){
      set_sub<T,nY,nX,1,3>(H, Z13, 13, 6, 1.0);
    }
    if (flg.pitch_en == false){
      set_sub<T,nY,nX,1,3>(H, Z13, 14, 6, 1.0);
    }
    if (flg.course_en == false){
      set_sub<T,nY,nX,1,3>(H, Z13, 15, 6, 1.0);
    }
  
    //16
    //ZIHR
    T eu[3][1];
    quat2euler<T>(eu, qnv);
    if (flg.zihr_en == true){
      T Hzihre[1][3];
      T sec_th;
      T cp, sp;    
      T tmp[1][3];
      if (eu[1][0] != 0.0){
        cp = cos(eu[0][0]);
        sp = sin(eu[0][0]);
        sec_th = 1.0/cos(eu[1][0]);
          
        tmp[0][0] = 0.0;
        tmp[0][1] = sec_th*sp*dT;
        tmp[0][2] = sec_th*cp*dT;
        if (nX > 12){
          m_mul<T,1,3,3>(Hzihre,tmp,Cvb);
          set_sub<T,nY,nX,1,3>(H, Hzihre, 16, 12, 1.0);
        }
      }
    }
  }
 
}

template <typename T, int nY>  
void set_Y(T Y[nY][1], T r[3][1], T v[3][1], T r_sns[3][1], T v_sns[3][1], T alt_b[1][1], T v_odo[3][1], T mb[3][1],
           T mn[3][1], T ang_in[1][1], T head_prev[1][1], T qnb[4][1], T qbv[4][1], T eu[3][1], T l_sns[3][1], T wbib[3][1]){
  
  m_set<T,nY,1>(Y,0.0);
  T D[3][3];
  T Di[3][3];
  T l_sns_skew[3][3];
  T tmp[3][1];
  T tmp2[3][1];
  T r_sns_hat[3][1];
  T v_sns_hat[3][1];
  T dr[3][1], dv[3][1], dv_odo[3][1], dmb[3][1];
  
  T Cbn[3][3];
  T Cnb[3][3];
  T Cvb[3][3];
  T Cbv[3][3];
  T qbn[4][1];
  T qvb[4][1];
  T v_wheel[3][1];
  qcon<T>(qbn, qnb);
  quat2dcm<T>(Cbn,qbn);  
  quat2dcm<T>(Cnb,qnb); 
  
  qcon<T>(qvb, qbv);
  quat2dcm<T>(Cvb,qvb);
  quat2dcm<T>(Cbv,qbv);
  
  //
  get_D(Di,r);
  m_mul<T,3,3,1>(tmp,Cnb,l_sns);
  m_mul<T,3,3,1>(tmp2,Di,tmp);
  m_plus<T,3,1>(r_sns_hat,r,tmp2);
  //
  m_mul<T,3,3,1>(tmp,D,dr);
  get_D(D,r);
  m_minus<T,3,1>(dr,r_sns_hat,r_sns);
  m_mul<T,3,3,1>(tmp,D,dr);
  set_sub<T,3,1,3,1>(dr, tmp, 0, 0, 1.0);
  
  //
  cpm<T>(l_sns_skew, l_sns);
  T Wcec[3][1];
  T Wcie[3][1];
  T Wcic_skew[3][3];
  T tmp3[3][1];
  get_WcecWcie(Wcec, Wcie, r, v);
  m_plus<T,3,1>(tmp,Wcec,Wcie);
  cpm<T>(Wcic_skew, tmp);
  m_mul<T,3,3,1>(tmp,Cnb,l_sns);
  m_mul<T,3,3,1>(tmp2,Wcic_skew,tmp);
  m_mul<T,3,3,1>(tmp,l_sns_skew,wbib);
  m_mul<T,3,3,1>(tmp3,Cnb,tmp);
  m_plus<T,3,1>(tmp,tmp2,tmp3);
  m_minus<T,3,1>(v_sns_hat,v,tmp);
  m_minus<T,3,1>(dv,v_sns_hat,v_sns);
  //


  /*qcon<T>(qbn, qnb);
  quat2dcm<T>(Cbn,qbn);  
  quat2dcm<T>(Cnb,qnb); 
  
  qcon<T>(qvb, qbv);
  quat2dcm<T>(Cvb,qvb);
  quat2dcm<T>(Cbv,qbv);*/
  
  T Cnv[3][3];
  T qnv[4][1];
  m_mul<T,3,3,3>(Cnv,Cnb,Cbv);
  qmult<T>(qnv,qnb,qbv);
  
  //odometer measurement to wheel frame 
  m_mul<T,3,3,1>(v_wheel,Cbn,v);  
  m_mul<T,3,3,1>(tmp,Cvb,v_wheel);
  m_copy<T,3,1>(v_wheel,tmp);
  
  m_minus<T,3,1>(dv_odo,v_wheel,v_odo);
  
  T mb_hat[3][1];
  m_norm<T,3>(mn);
  m_norm<T,3>(mb);
  m_mul<T,3,3,1>(mb_hat,Cbn,mn);
  m_minus<T,3,1>(dmb,mb_hat,mb);
  
    
  T dang[3][1];
  m_minus<T,3,1>(dang,eu,ang_in);

  for (int i = 0; i<3; i++){
    if (dang[i][0] < -PI_att){
      dang[i][0] += c2pi_att;
    }
    else if(dang[i][0]>PI_att){
      dang[i][0] -= c2pi_att;
    }
  }
  
  T dhead_zihr[1][1];
  dhead_zihr[0][0] = eu[2][0]-head_prev[0][0];

  T dalt_b[1][1];
  dalt_b[0][0] = r[2][0]-alt_b[0][0];

  //set
  set_sub<T,nY,1,3,1>(Y, dr, 0, 0, 1.0);
  set_sub<T,nY,1,3,1>(Y, dv, 3, 0, 1.0);
  if (nY > 6){
    set_sub<T,nY,1,3,1>(Y, dv_odo, 6, 0, 1.0);
    set_sub<T,nY,1,1,1>(Y, dalt_b, 9, 0, -1.0);
    set_sub<T,nY,1,3,1>(Y, dmb, 10, 0, 1.0);
    set_sub<T,nY,1,3,1>(Y, dang, 13, 0, 1.0);
    set_sub<T,nY,1,1,1>(Y, dhead_zihr, 16, 0, 1.0);
  }
}

template <typename T>  
void get_dth(T dth[3][1], T r[3][1], T dr[3][1]){
  
  T phi, Mh, Nh;
  phi = r[0][0];
  el_curv(Mh, Nh, r);
  dth[0][0] =  dr[1][0]/Nh;
  dth[1][0] = -dr[0][0]/Mh;
  dth[2][0] = -dr[1][0]*tan(phi)/Nh;
}


#endif //SINS_ERROR_HPP
