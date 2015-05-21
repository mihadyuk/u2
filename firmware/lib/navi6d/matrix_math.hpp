#ifndef MATRIX_MATH_HPP
#define MATRIX_MATH_HPP

template <typename T, int m, int n>
void m_copy(T Mres[m][n], T M[m][n]){
  for (int i = 0; i<m; i++){
    for (int j = 0; j<n; j++){
      Mres[i][j] = M[i][j];
    }
  }
}

template <typename T, int m>
void set_diag_cov(T M[m][m], int b, int e, T s){
  for (int i = b; i<=e; i++){
    M[i][i] = s*s;
  }  
}

template <typename T, int m, int n, int k, int l>
void set_sub(T M[m][n], T Msub[k][l], int row, int col, T s){
  for (int i = 0; i<k; i++){
    for (int j = 0; j<l; j++){
      M[row+i][col+j] = s*Msub[i][j];
    }
  }  
}

template <typename T, int k, int l, int m, int n>
void get_sub(T Msub[k][l], T M[m][n], int row, int col){
  for (int i = 0; i<k; i++){
    for (int j = 0; j<l; j++){
      Msub[i][j] = M[row+i][col+j];
    }
  }  
}


// template <typename T, int m, int n, int p>
// void m_mul(T Mres[m][p], T M1[m][n], T M2[n][p]){
//   T c;
//   for (int i = 0; i<m; i++){
//     for (int j = 0; j<p; j++){
//       c = 0.0;
//       for (int k = 0; k<n; k++){
//         c += M1[i][k]*M2[k][j];
//       }
//       Mres[i][j] = c;
//     }
//   }  
// }

//optimized
template <typename T, int m, int n, int p>
void m_mul(T Mres[m][p], T M1[m][n], T M2[n][p]){
  T a1, a2, a3, a4;
  T b1, b2, b3, b4;
  T c;
  
  int Nround = n-(n%4);
  for (int i = 0; i<m; i++){
    for (int j = 0; j<p; j++){      
      c = 0.0;
      for (int k = 0; k<Nround; k+=4){
        a1 = M1[i][k];
        a2 = M1[i][k+1];
        a3 = M1[i][k+2];
        a4 = M1[i][k+3];
        
        b1 = M2[k][j];
        b2 = M2[k+1][j];
        b3 = M2[k+2][j];
        b4 = M2[k+3][j];
        
        c += a1*b1;
        c += a2*b2;
        c += a3*b3;
        c += a4*b4;  
      }
      for (int k = Nround; k<n; k++){
        c += M1[i][k]*M2[k][j];
      }
      Mres[i][j] = c;
    }
  }  
}



//multiply on transpose Mres = M1*M2'
template <typename T, int m, int n, int p>
void m_mul_t(T Mres[m][p], T M1[m][n], T M2[p][n]){
  T c;
  for (int i = 0; i<m; i++){
    for (int j = 0; j<p; j++){
      c = 0.0;
      for (int k = 0; k<n; k++){
        c += M1[i][k]*M2[j][k];
      }
      Mres[i][j] = c;
    }
  }  
}

template <typename T, int m, int n>
void m_mul_s( T Mres[m][n], T M[m][n], T s){
  for (int i = 0; i<m; i++){
    for (int j = 0; j<n; j++){
      Mres[i][j] = M[i][j]*s;
    }
  }  
}

template <typename T, int m, int n>
void m_div_s(T Mres[m][n], T M[m][n], T s){
  for (int i = 0; i<m; i++){
    for (int j = 0; j<n; j++){
      Mres[i][j] = M[i][j]/s;
    }
  }  
}

template <typename T, int m, int n>
void m_get_row(T Mres[1][n], T M[m][n], int r){
  for (int i = 0; i<n; i++){
    Mres[0][i] = M[r][i];
  }  
}

template <typename T, int m, int n>
void m_tran(T Mres[n][m], T M[m][n]){
  for (int i = 0; i<m; i++){
    for (int j = 0; j<n; j++){
      Mres[j][i] = M[i][j];
    }
  }  
}


template <typename T, int m, int n>
void m_plus(T Mres[m][n], T M1[m][n], T M2[m][n]){
  for (int i = 0; i<m; i++){
    for (int j = 0; j<n; j++){
      Mres[i][j] = M1[i][j] + M2[i][j];
    }
  }  
}

template <typename T, int m, int n>
void m_plus_alpha_beta(T Mres[m][n], T M1[m][n], T M2[m][n], T alpha, T beta){
  for (int i = 0; i<m; i++){
    for (int j = 0; j<n; j++){
      Mres[i][j] = alpha*M1[i][j] + beta*M2[i][j];
    }
  }  
}

template <typename T, int m, int n>
void m_plus_s( T Mres[m][n], T M[m][n], T s){
  for (int i = 0; i<m; i++){
    for (int j = 0; j<n; j++){
      Mres[i][j] = M[i][j] + s;
    }
  }  
}

template <typename T, int m, int n>
void m_minus( T Mres[m][n], T M1[m][n], T M2[m][n]){
  for (int i = 0; i<m; i++){
    for (int j = 0; j<n; j++){
      Mres[i][j] = M1[i][j] - M2[i][j];
    }
  }  
}

template <typename T, int m>
void m_eye(T I[m][m]){
  for (int i = 0; i<m; i++){
    for (int j = 0; j<m; j++){
      I[i][j] = 0.0;
    }
    I[i][i] = 1.0;
  }  
}

template <typename T, int m, int n>
void m_zeros(T Z[m][n]){
  for (int i = 0; i<m; i++){
    for (int j = 0; j<n; j++){
      Z[i][j] = 0.0;
    }
  }  
}

template <typename T, int m>
void m_plus_diag(T M[m][m], T D[m][1]){
  for (int i = 0; i<m; i++){
    M[i][i] += D[i][0];
  }  
}

template <typename T, int m>
void m_plus_eye(T M[m][m]){
  for (int i = 0; i<m; i++){
    M[i][i] += 1;
  }  
}

template <typename T, int m>
void m_diag(T Mres[m][m], T M[m][1]){
  for (int i = 0; i<m; i++){
    for (int j = 0; j<m; j++){
      Mres[i][j] = 0.0;
    }
    Mres[i][i] = M[i][0];
  }  
}

template <typename T>
void m_non_ort(T Mres[3][6], T M[3][1]){
  T ax, ay, az;
  ax = M[0][0]; ay = M[1][0]; az = M[2][0];
  
  Mres[0][0] = ay;  Mres[0][1] = az;  Mres[0][2] = 0.0; Mres[0][3] = 0.0; Mres[0][4] = 0.0; Mres[0][5] = 0.0;
  Mres[1][0] = 0.0; Mres[1][1] = 0.0; Mres[1][2] = ax;  Mres[1][3] = az;  Mres[1][4] = 0.0; Mres[1][5] = 0.0;
  Mres[2][0] = 0.0; Mres[2][1] = 0.0; Mres[2][2] = 0.0; Mres[2][3] = 0.0; Mres[2][4] = ax;  Mres[2][5] = ay;
}

template <typename T, int m, int n>
void m_set(T Mres[m][n], T s){
  for (int i = 0; i<m; i++){
    for (int j = 0; j<n; j++){
      Mres[i][j] = s;
    }
  }  
}

template <typename T, int m, int n>
void m_sigma_mul(T Mres[m][m], T M[m][n], T Q[n][n]){
  T tmp[m][n];
  // Mres = M*Q*M.';
  m_mul<T,m,n,n>(tmp,M,Q);
  m_mul_t<T,m,n,m>(Mres,tmp,M);
}

template <typename T, int m, int n>
void m_sigma_mul_diag(T Q[m][m], T G[m][n], T Qm[n][1]){
  T c;
  for (int k = 0; k<m; k++){
    for (int i = k; i<m; i++){  
      c = 0.0;
      for (int j = 0; j<n; j++){
        c += G[k][j]*G[i][j]*Qm[j][0];
      }
      Q[k][i] = c;
      Q[i][k] = c;
    }
  }
}

template <typename T, int m>
void m_add_row(T M[m][m], int r1, int r2, T s){
  for (int i = 0; i<m;i++){
    M[r1][i] += s*M[r2][i]; 
  }
}

template <typename T, int m>
void m_norm(T M[m][1]){
  T n;
  n = 0.0;
  for (int i = 0; i<m;i++){
    n += M[i][0]*M[i][0]; 
  }
  n = sqrt(n);
  if (n != 0){
    for (int i = 0; i<m;i++){
      M[i][0] /= n; 
    }
  }  
}

template <typename T, int m>
T m_vec_norm(T vec[m][1]){
  T n;
  n = 0.0;
  for (int i = 0; i<m;i++){
    n += vec[i][0]*vec[i][0]; 
  }
  n = sqrt(n);
  return n;
}


template <typename T, int m>
void m_mul_row(T M[m][m], int r1, T s){
  for (int i = 0; i<m;i++){
    M[r1][i] *= s;  
  }
}

//inverse
template <typename T, int m>
void m_inv(T Minv[m][m], T M[m][m]){
  m_eye<T, m>(Minv);
  T c;
  for (int i = 0; i<m;i++){
    for (int j = 0; j<m;j++){
      if (i != j){
        c = -M[j][i]/M[i][i];
        m_add_row<T,m>(M, j, i, c);
        m_add_row<T,m>(Minv, j, i, c);
      }
    }
  }
  for (int i = 0; i<m;i++){
    for (int j = 0; j<m;j++){
      Minv[i][j] /= M[i][i];
    }
  }
}


#endif //MATRIX_MATH_HPP
