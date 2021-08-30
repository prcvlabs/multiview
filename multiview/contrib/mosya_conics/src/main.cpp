#include <iostream>
#include <cmath>
#include <ctime>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <iomanip>
#include <vector>

#include "mystuff.h"

using namespace mosya;

#ifdef ZAPPY_ZAPPY
#include "RandomNormalPair.cpp"
#include "AtoG.cpp"
//#include "GtoA.cpp"
//#include "ProjectPointsOntoEllipse.cpp"
//#include "ProjectPointsOntoHyperbola.cpp"
//#include "ProjectPointsOntoParabola.cpp"
//#include "ProjectPointsOntoConicByEberlyOrig.cpp"
#include "eigen2x2.cpp"
#include "ProjectPointsOntoConicByEberlyModified.cpp"
#include "AdjustProjectedPointsOnConic.cpp"
#include "DistanceToConicApprx.cpp"
#include "ProjectionQuality.cpp"
#include "IsCrossingWindow.cpp"
#include "RootOfCubicEquation.cpp"
//#include "ProjectPointsOntoConicByWEPqz.cpp"
//#include "ProjectPointsOntoConicByWEPeq3Schur.cpp"
#include "ProjectPointsOntoConicByWEPeq3NR.cpp"
//#include "ProjectPointsOntoConicByWEPinv.cpp"
//#include "ProjectPointsOntoConicByQuarticEquation.cpp"
#include "ProjectPointsOntoConicByNewtonAK.cpp"
#endif

//  Main program for testing projection methods
//  Specify the methods by selecting which functions the program calls
//  Choose double or long double precision in file "mystuff.h"

int main()
{
    const int n=2,histCols=18,histRows=6,timing=1;

    integers i,j,k,N=0,N0=0,N1=0,N2=0,N3=0,Nmiss=0;
    int code,hist1[histRows][histCols]={0}, hist2[histRows][histCols]={0};
	
    const reals Window=One,Emin=pow(10.,-histCols-1);  
	
    reals r1,r2;
    reals E1[histRows][n], E2[histRows][n];
    double Times[histRows]={0.};
	
    M6x1 A;  
    M5x1 ParG;
    Mnx1 X(n,1),Y(n,1),D(n,1),Dmin(n,1),Q(n,1);
    Mnx1 Xproj(n,1),Yproj(n,1),XprojA(n,1),YprojA(n,1),Xbest(n,1),Ybest(n,1);
    Mnxm XprojAll(n,histRows),YprojAll(n,histRows);
	
    vector<reals> Xv(n),Yv(n);
	
    clock_t iniTime, finTime;        // auxiliary variables for timing
    timespec ts1, ts2;
	
    srand ( (unsigned)time(NULL) );  // seed the random generator
    cout.precision(15);

    while(1)   //  main loop over random conics and points
	{
            //  generate a conic:
	
            RandomNormalPair(r1,r2);  A(0) = r1;  A(1) = r2;
            RandomNormalPair(r1,r2);  A(2) = r1;  A(3) = r2;
            RandomNormalPair(r1,r2);  A(4) = r1;  A(5) = r2;
	
            reals eps=1.e-13; RandomNormalPair(r1,r2);  A(2) = A(1)*A(1)/A(0) + eps*r1;  // make it a parabola (if desired)
	
            A.normalize();      //  normalization of the parameter vector (unnecessary)
	
            //  discard the conic if it is of the wrong type:
	
            if (IsCrossingWindow(A,Window)==0)  {  Nmiss++; continue;  }  //  discard conics that do not cross the data window
	
            AtoG(A,ParG,code);                       //   determine the conic type
            if (code>3) { N0++;  continue; }         //   discard degenerate conics
	
            if (code==1) N1++;   //  count ellipses
            if (code==2) N2++;   //  count hyperbolas
            if (code==3) N3++;   //  count parabolas
            N++;                 //  count all conics together
	
            //   generate n data points:
	
            for (i=0; i<n; i++)
                {
                    Xv[i] = Window*(Two*rand()/RAND_MAX - One);
                    Yv[i] = Window*(Two*rand()/RAND_MAX - One);
                    X(i) = Xv[i];
                    Y(i) = Yv[i];
                    Dmin(i) = REAL_MAX;
                }

            //   First projection method:
	
            if (timing) {clock_gettime(CLOCK_REALTIME, &ts1);}
            ProjectPointsOntoConicByWEPeq3NR(X,Y,A,Xproj,Yproj);  //  choose a projection method here
            if (timing) {clock_gettime(CLOCK_REALTIME, &ts2); Times[0] += (ts2.tv_sec-ts1.tv_sec) + 1.e-9*(ts2.tv_nsec - ts1.tv_nsec);}
            ProjectionQuality(X,Y,A,Xproj,Yproj,Q);
            DistanceToConicApprx(X,Y,A,Xproj,Yproj,D);
            for (i=0; i<n; i++)
                {
                    E2[0][i] = Q(i);
                    if (Dmin(i) > D(i))
                        {
                            Dmin(i) = D(i);
                            Xbest(i) = Xproj(i);
                            Ybest(i) = Yproj(i);
                        }
                }
            XprojAll.col(0) = Xproj;
            YprojAll.col(0) = Yproj;
	
            //   Second projection method:
	
            if (timing) {clock_gettime(CLOCK_REALTIME, &ts1);}
            AdjustProjectedPointsOnConic(X,Y,A,Xproj,Yproj,XprojA,YprojA,2);  //  choose a projection method here
            Xproj = XprojA;  Yproj = YprojA;
            if (timing) {clock_gettime(CLOCK_REALTIME, &ts2); Times[1] += (ts2.tv_sec-ts1.tv_sec) + 1.e-9*(ts2.tv_nsec - ts1.tv_nsec);}
            ProjectionQuality(X,Y,A,Xproj,Yproj,Q);
            DistanceToConicApprx(X,Y,A,Xproj,Yproj,D);
            for (i=0; i<n; i++)
                {
                    E2[1][i] = Q(i);
                    if (Dmin(i) > D(i))
                        {
                            Dmin(i) = D(i);
                            Xbest(i) = Xproj(i);
                            Ybest(i) = Yproj(i);
                        }
                }
            XprojAll.col(1) = Xproj;
            YprojAll.col(1) = Yproj;
	
            //   Third projection method:

            if (timing) {clock_gettime(CLOCK_REALTIME, &ts1);}
            ProjectPointsOntoConicByNewtonAK(X,Y,A,Xproj,Yproj);  //  choose a projection method here
            if (timing) {clock_gettime(CLOCK_REALTIME, &ts2); Times[2] += (ts2.tv_sec-ts1.tv_sec) + 1.e-9*(ts2.tv_nsec - ts1.tv_nsec);}
            ProjectionQuality(X,Y,A,Xproj,Yproj,Q);
            DistanceToConicApprx(X,Y,A,Xproj,Yproj,D);
            for (i=0; i<n; i++)
                {
                    E2[2][i] = Q(i);
                    if (Dmin(i) > D(i))
                        {
                            Dmin(i) = D(i);
                            Xbest(i) = Xproj(i);
                            Ybest(i) = Yproj(i);
                        }
                }
            XprojAll.col(2) = Xproj;
            YprojAll.col(2) = Yproj;
	
            //   Fourth projection method:
        
            if (timing) {clock_gettime(CLOCK_REALTIME, &ts1);}
            AdjustProjectedPointsOnConic(X,Y,A,Xproj,Yproj,XprojA,YprojA,2);  //  choose a projection method here
            Xproj = XprojA;  Yproj = YprojA;
            if (timing) {clock_gettime(CLOCK_REALTIME, &ts2); Times[3] += (ts2.tv_sec-ts1.tv_sec) + 1.e-9*(ts2.tv_nsec - ts1.tv_nsec);}
            ProjectionQuality(X,Y,A,Xproj,Yproj,Q);
            DistanceToConicApprx(X,Y,A,Xproj,Yproj,D);
            for (i=0; i<n; i++)
                {
                    E2[3][i] = Q(i);
                    if (Dmin(i) > D(i))
                        {
                            Dmin(i) = D(i);
                            Xbest(i) = Xproj(i);
                            Ybest(i) = Yproj(i);
                        }
                }
            XprojAll.col(3) = Xproj;
            YprojAll.col(3) = Yproj;
	
            //   Fifth projection method:
        
            if (timing) {clock_gettime(CLOCK_REALTIME, &ts1);}
            ProjectPointsOntoConicByEberlyModified(X,Y,A,Xproj,Yproj);  //  choose a projection method here
            if (timing) {clock_gettime(CLOCK_REALTIME, &ts2); Times[4] += (ts2.tv_sec-ts1.tv_sec) + 1.e-9*(ts2.tv_nsec - ts1.tv_nsec);}
            ProjectionQuality(X,Y,A,Xproj,Yproj,Q);
            DistanceToConicApprx(X,Y,A,Xproj,Yproj,D);
            for (i=0; i<n; i++)
                {
                    E2[4][i] = Q(i);
                    if (Dmin(i) > D(i))
                        {
                            Dmin(i) = D(i);
                            Xbest(i) = Xproj(i);
                            Ybest(i) = Yproj(i);
                        }
                }
            XprojAll.col(4) = Xproj;
            YprojAll.col(4) = Yproj;
	
            //   Sixth projection method:
        
            if (timing) {clock_gettime(CLOCK_REALTIME, &ts1);}
            AdjustProjectedPointsOnConic(X,Y,A,Xproj,Yproj,XprojA,YprojA,2);  //  choose a projection method here
            Xproj = XprojA;  Yproj = YprojA;
            if (timing) {clock_gettime(CLOCK_REALTIME, &ts2); Times[5] += (ts2.tv_sec-ts1.tv_sec) + 1.e-9*(ts2.tv_nsec - ts1.tv_nsec);}
            //if (timing) {cout << endl; cin.ignore();} 
            ProjectionQuality(X,Y,A,Xproj,Yproj,Q);
            DistanceToConicApprx(X,Y,A,Xproj,Yproj,D);
            for (i=0; i<n; i++)
                {
                    E2[5][i] = Q(i);
                    if (Dmin(i) > D(i))
                        {
                            Dmin(i) = D(i);
                            Xbest(i) = Xproj(i);
                            Ybest(i) = Yproj(i);
                        }
                }
            XprojAll.col(5) = Xproj;
            YprojAll.col(5) = Yproj;
	
            //   Record the results of all projection methods
        
            for (i=0; i<n; i++)
                {
                    for (j=0; j<histRows; j++)   {  E1[j][i] = sqrt(SQR(XprojAll(i,j)-Xbest(i)) + SQR(YprojAll(i,j)-Ybest(i)));  }
            
                    for (j=0; j<histRows; j++)   {  if (E1[j][i] < Emin) E1[j][i] = Emin;  }
                    for (j=0; j<histRows; j++)   {  if (E2[j][i] < Emin) E2[j][i] = Emin;  }
                }
        
            for (i=0; i<n; i++)
                {
                    for (j=0; j<histRows; j++) 
                        {
                            k = floor(-log10(E1[j][i]));
                            if (k<1) k=1;  
                            if (k<=histCols) hist1[j][k-1]++;
                
                            k = floor(-log10(E2[j][i]));
                            if (k<1) k=1;  
                            if (k<=histCols) hist2[j][k-1]++;
                        }
                }
        
            if (((N*n)%2000000)==0) 
                {
                    for (j=0; j<histRows; j++) 
                        {
                            for (i=0;i<histCols;i++)  cout << setw(6) << hist1[j][i];
                            cout << endl;
                        }
                    cout << endl;
	    
                    for (j=0; j<histRows; j++) 
                        {
                            for (i=0;i<histCols;i++)  cout << setw(6) << hist2[j][i];
                            cout << endl;
                        }
                    cout << endl;
	    
                    if (timing)
                        {
                            cout << " Runnig times per conic (in milisec): ";
                            for (j=0; j<histRows; j++) cout << "  " << setprecision(5) << Times[j]/N*1000000.;
                            cout << endl << endl;;
                        }
	    
                    cout << " conics used:  N = " << N << "  (Ell: " << N1 << "  Hyp: " << N2 << "  Par: " << N3 << ") " << endl;
                    cout << " conics removed:  " << N0 << " degenetare and " << Nmiss << " missing the window" << endl << endl;
                }
            //if ((N*n)>=1000000000) return 0;
        }
}

