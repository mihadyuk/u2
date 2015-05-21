#ifndef MATLAB_INTERFACE_HPP
#define MATLAB_INTERFACE_HPP


template <typename T, int nX, int nY>
void set_matrix(T M_out[nX][nY], double *M_in){
  const int r0 = nX;
  const int c0 = nY;
  int r, c;
  r = 0;
  for (int i = 0; i<r0*c0; i++){
    if ((i>0) && (i%c0 == 0)){
      r++;
    }
    c = (i-c0*r);
    M_out[r][c] = static_cast<T>(M_in[c*r0+r]);
  }
}

template <typename T, int nX, int nY>
void get_matrix(double *P_out, T P_in[nX][nY]){
  const int r0 = nX;
  const int c0 = nY;
  int r, c;
  r = 0;
  for (int i = 0; i<r0*c0; i++){
    if ((i>0) && (i%c0 == 0)){
      r++;
    }
    c = (i-r*c0);
    P_out[r0*c+r] = static_cast<T>(P_in[r][c]);
  }
}



#endif //MATLAB_INTERFACE_HPP
