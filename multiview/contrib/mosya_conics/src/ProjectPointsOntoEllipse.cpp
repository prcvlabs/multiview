
#include "mystuff.h"

namespace mosya {

void ProjectPointsOntoEllipse(Mnx1 &X, Mnx1 &Y, M5x1 &ParG, Mnx1 &Xproj, Mnx1 &Yproj)
/*                            <---------- Input --------->  <------ Output -------->

   Projecting a given set of points onto an ellipse

   This is a modified version of an iterative algorithm published by D. Eberly
     Internet publication: "Distance from a point to an ellipse in 2D" (2004)
                           Geometric Tools, LLC, www.geometrictools.com
     Book publication: "3D Game Engine Design", 2nd edition.
                      Morgan Kaufmann Publishers, San Francisco, CA, 2007.
                              (see Section 14.13.1)

   Input:  two n-vectors X and Y containing the coordinates of n points
           ParG is a 5x1 vector of the ellipse parameters:
           
           ParG(0)  is the x-coordinate of the ellipse's center
           ParG(1)  is the y-coordinate of the ellipse's center
           ParG(2)  is the major semi-axis
           ParG(3)  is the minor semi-axis
           ParG(4)  is the angle of tilt of the ellipse
           
           Note: the user needs to make sure that ParG(2) >= ParG(3) > 0.

   Output:  two n-vectors Xproj and Yproj containing the coordinates of n projections of the data points

   The algorithm is proven to converge and reaches an accuracy of 14-15 significant digit
   It takes 5-6 iterations per point, on average.
   
          Nikolai Chernov,  August 2012
*/
{
    int n=X.rows(),i,iter,iterMax;
    reals a,b,c,s,aa,bb,D,u,v,T,F,Tnew,disc,xprojx,yprojx,xprojy,yprojy,Fx,Fy;
    Mnx1 X0(n,1), Y0(n,1);       //  column vectors of length n (matrices n times 1)

    a = ParG(2); b = ParG(3);    //  a is the major semi-axis, b is the minor semi-axis
    aa = a*a;    bb = b*b;       //  squares of the axes
    D  = (a-b)*(a+b);            //  "distorsion measure"

    if ((a <= 0.)||(b <= 0.))    //  check validity of input
    {
        cout << "Axes of the ellipse must be positive" << endl;
        return;
    }
    if (a < b)
    {
        cout << "Major axis of the ellipse cannot be smaller than its minor axis" << endl;
        return;
    }

    s = sin(ParG(4)); c = cos(ParG(4));                    //  rotation parameters
    X0  = (X.array()-ParG(0))*c + (Y.array()-ParG(1))*s;   //  rotating x-coordinates
    Y0  = (Y.array()-ParG(1))*c - (X.array()-ParG(0))*s;   //  rotating y-coordinates

    iterMax = 100;   //  Maximum number of Newton's ietrations. Usually, 4-6 are enough

    for (i=0; i<n; i++)                    //  main loop over the points
    {
        u = abs(X0(i));  v = abs(Y0(i));   //  coordinates of the point
        T = a*u-D;    if (T<b*v)  T = b*v;        //  initial value of the T variable
        if ((T <= 0.)&&(D <= 0.))    //  circle (a=b) and point at its center
        {
            Xproj(i) = 0.;  Yproj(i) = b;   // project the center to the North pole
            continue;
        }
        if (T <= 0.)      //  true ellipse (a>b) and point on major axis near or at center
        {
            Xproj(i) = aa*X0(i)/D;        //  compute the projection by exact formulas
            disc = One - SQR(a*u/D);   if (disc < 0.) disc = 0.;
            Yproj(i) = b*sqrt(disc);
            continue;
        }


        //  the main, non-singular case
        //  start Newton's iterations.

        for (iter=0; iter<iterMax; iter++)   //  loop of Newton's iterations to solve F(T)=0
        {
            F  = SQR(a*u/(T+D)) + SQR(b*v/T) - One;     //  value of F; we need to find T such that F(T)=0
            if (F <= 0.)  break;                                      //  gone too far, emergency stop
            //  derivative of F with respect to T is -2.0*(SQR(a*u/(T+D))/(T+D) + SQR(b*v/T)/T)
            Tnew = T + F/Two/(SQR(a*u/(T+D))/(T+D) + SQR(b*v/T)/T);
            if ((!isfinite(Tnew))||(T == Tnew))  break;         //  no progress, terminate iterations
            T = Tnew;                         //  Newton's iteration
        }                      //  end of the loop of Newton's iterations

        //          compute the projection of the point onto the ellipse

        xprojx = aa*u/(T+D);            //  first candidate for projection
        disc = One-xprojx*xprojx/aa;
        if (disc<0.) disc = 0.;
        yprojx = b*sqrt(disc);
        yprojy = bb*v/T;                //  second candidate for projection
        disc = One-yprojy*yprojy/bb;
        if (disc<0.) disc = 0.;
        xprojy = a*sqrt(disc);

        Fx = SQR(xprojx-u) + SQR(yprojx-v);  // squared distance to first candidate
        Fy = SQR(xprojy-u) + SQR(yprojy-v);  // squared distance to second candidate

        if (Fx < Fy)        //    the first  candidate is better
        {
            Xproj(i) = xprojx;
            Yproj(i) = yprojx;
        }
        else                //    the second candidate is better
        {
            Xproj(i) = xprojy;
            Yproj(i) = yprojy;
        }                   //    end comparing the two candidates

        //         reflect the projection point into the proper quadrant

        if (X0(i)<0.) Xproj(i) = -Xproj(i);
        if (Y0(i)<0.) Yproj(i) = -Yproj(i);
        
    }     //  end the main loop over the points

    //       rotate back to the original system

    X0  = Xproj.array()*c - Yproj.array()*s;        //  rotating x-coordinates
    Y0  = Xproj.array()*s + Yproj.array()*c;        //  rotating y-coordinates
    Xproj = X0.array() + ParG(0);                   //  shifting x-coordinates
    Yproj = Y0.array() + ParG(1);                   //  shifting y-coordinates

    // RSS = (X - Xproj).squaredNorm() + (Y - Yproj).squaredNorm();  //  computing the Residual Sum of Squares
}
} // namespace mosya

