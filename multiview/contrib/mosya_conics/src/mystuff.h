
#ifndef MOSYA_CONICS__STDINC__HPP
#define MOSYA_CONICS__STDINC__HPP

#include <Eigen/Dense>
#include <iostream>

namespace mosya {

using namespace std;
using namespace Eigen;

//  define precision by commenting out one of the two lines:

typedef long double reals;
//typedef double reals;

typedef long long integers;
//typedef double reals;

//  next define vector/matrix types for "eigen3" library

typedef Matrix<reals, Dynamic, 1> Mnx1;
typedef Matrix<reals, Dynamic, 2> Mnx2;
typedef Matrix<reals, Dynamic, 6> Mnx6;
typedef Matrix<reals, Dynamic, Dynamic> Mnxm;
typedef Matrix<reals, 2, 1> M2x1; 
typedef Matrix<reals, 3, 1> M3x1; 
typedef Matrix<reals, 4, 1> M4x1; 
typedef Matrix<reals, 5, 1> M5x1;
typedef Matrix<reals, 6, 1> M6x1;
typedef Matrix<reals, 2, 2> M2x2; 
typedef Matrix<reals, 3, 3> M3x3;
typedef Matrix<reals, 4, 4> M4x4;
typedef Matrix<reals, 5, 5> M5x5;
typedef Matrix<reals, 6, 6> M6x6;

//   next define some frequently used constants:

const reals One=1.,Two=2.,Three=3.,Four=4.,Five=5.,Six=6.;
const reals Pi=3.141592653589793238462643383L;
const reals REAL_MAX=numeric_limits<reals>::max();
const reals REAL_MIN=numeric_limits<reals>::min();

//   next define some frequently used functions:

template<typename T>
inline T SQR(T t) { return t*t; };

reals RootOfCubicEquation(reals& a, reals& b, reals& c);
void eigen2x2(reals a, reals b, reals c,
              reals& d1, reals& d2, reals& Vx, reals& Vy);
void AtoG(M6x1 ParAin, M5x1 &ParG, int &code);
void ProjectPointsOntoEllipse(Mnx1 &X, Mnx1 &Y, M5x1 &ParG,
                              Mnx1 &Xproj, Mnx1 &Yproj);
void ProjectPointsOntoHyperbola(Mnx1 X, Mnx1 Y, M5x1 ParG,
                                Mnx1 &Xproj, Mnx1 &Yproj);
void ProjectPointsOntoParabola(Mnx1 X, Mnx1 Y, M5x1 ParG,
                               Mnx1 &Xproj, Mnx1 &Yproj);

void RandomNormalPair( reals& x, reals& y );
int IsCrossingWindow(M6x1 A, reals R);
void ProjectPointsOntoConicByWEPeq3NR (Mnx1& X, Mnx1& Y, M6x1& A,
                                       Mnx1& Xproj, Mnx1& Yproj);
void ProjectionQuality(Mnx1& X, Mnx1& Y, M6x1& A,
                       Mnx1& Xproj, Mnx1& Yproj, Mnx1& Q);
void DistanceToConicApprx(Mnx1 &X, Mnx1 &Y, M6x1 &A,
                          Mnx1 &Xproj, Mnx1 &Yproj, Mnx1 &Q);
void AdjustProjectedPointsOnConic(Mnx1& X, Mnx1& Y, M6x1& ParA,
                                  Mnx1& Xproj, Mnx1& Yproj,
                                  Mnx1& XprojA, Mnx1& YprojA, int kmax);
void ProjectPointsOntoConicByNewtonAK (Mnx1& X, Mnx1& Y, M6x1& ParA,
                                       Mnx1& Xproj, Mnx1& Yproj);
void ProjectPointsOntoConicByEberlyModified (Mnx1& X, Mnx1& Y, M6x1& ParA,
                                             Mnx1& Xproj, Mnx1& Yproj);

} // namespace mosya

#endif

