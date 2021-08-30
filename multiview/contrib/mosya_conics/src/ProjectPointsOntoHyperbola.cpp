
namespace mosya {

void ProjectPointsOntoHyperbola(Mnx1 X, Mnx1 Y, M5x1 ParG, Mnx1 &Xproj, Mnx1 &Yproj)
/*                              <-------- Input -------->  <------ Output -------->

   Projecting a given set of points onto a hyperbola

   This method is proposed by Nikolai Chernov and Hui Ma in their article
   "Least squares fitting of quadratic curves and surfaces"
              published in book 
   "Computer Vision", Editor S. R. Yoshida, Nova Science Publishers 2011; pp. 285-302
   
   It is an adaptation of Eberly's method of projecting points onto ellipses

   Input:  two n-vectors X and Y containing the coordinates of n points
           ParG is a 5x1 vector of the ellipse parameters:
           
           ParG(0)  is the x-coordinate of the hyperbola's center
           ParG(1)  is the y-coordinate of the hyperbola's center
           ParG(2)  is the distance from the center to each vertex (denoted by a)
           ParG(3)  is the vertical distance from each vertex to the asymptotes (denoted by b)
           ParG(4)  is the angle of tilt of the hyperbola
           
           Note: the user needs to make sure that ParG(2) > 0 and ParG(3) > 0.
           
           Note: the equation of the hyperbola in the canonical coordinates is
               x^2/a^2 - y^2/b^2 = 1
           we call a and b "semi-axes", by analogy with ellipses (though their meaning is different)

   Output:  two n-vectors Xproj and Yproj containing the coordinates of n projections of the points

   The algorithm is proven to converge and reaches an accuracy of 14-15 significant digit
   It takes 7-8 iterations per point, on average.
   
          Nikolai Chernov,  August 2012
*/
{
    int n=X.rows(),i,iter,iterMax;
    const reals One=1.,Two=2.,s2=sqrt(Two);
    reals a,b,c,s,aa,bb,u,v,au,bv,T,Tnew,F0,F,Finf,D,xprojx,yprojx,xprojy,yprojy,Fx,Fy;
    Mnx1 X0(n,1), Y0(n,1);     //  column vectors of length n (matrices n times 1)

    a = ParG(2); b = ParG(3);    //  a and b are the semi-axes
    aa = a*a;    bb = b*b;       //  squares of the semi-axes
    D = aa + bb;                 //  auxiliary parameter

    s = sin(ParG(4)); c = cos(ParG(4));                    //  rotation parameters
    X0  = (X.array()-ParG(0))*c + (Y.array()-ParG(1))*s;   //  rotating x-coordinates
    Y0  = (Y.array()-ParG(1))*c - (X.array()-ParG(0))*s;   //  rotating y-coordinates

    iterMax = 100;   //  Maximum number of Newton's ietrations. Usually, 5-7 are enough
    
    for (i=0; i<n; i++)                  //  main loop over the points
    {
        u = abs(X0(i));   v = abs(Y0(i));        //  coordinates of the point
        au = a*u;         bv = b*v;              //  products of coordinates and axes
    
        F0 = u*u/aa - v*v/bb - One;       //  the F-value at the point (u,v)

        if (F0==0.)                       //  point on the hyperbola already
        {
            Xproj(i) = u;  Yproj(i) = v;
            continue;
        }
        
        Finf = SQR(sqrt(au) + sqrt(bv))*(au - bv)/D/D - One;    //  the F-value at the inflection point
    
        if (F0 > 0.)
        {
            if (Finf > 0.)
            {
                T = ( (D*bv/(au+bv) > D-au) ? (D*bv/(au+bv)) : (D-au) );
                if (T <= 0.)   T = 0;
                else
                {
                    for (iter=0; iter<iterMax; iter++)    //  loop of Newton's iterations to solve F(T)=0
                    {
                        F  = SQR(au/(D-T)) - SQR(bv/T) - One;    //  value of F at the current T
                        if ((F >= 0.)||(T >= bb))  break;        //  gone too far, emergency stop
                        
                        //  derivative of F with respect to T is 2.0*(SQR(au/(D-T))/(D-T) + SQR(bv/T)/T)
                        
                        Tnew = T - F/Two/(SQR(au/(D-T))/(D-T) + SQR(bv/T)/T);

                        if ((!isfinite(Tnew))||(T == Tnew))  break;      //  no progress, terminate iterations
                        T = Tnew;                                        //  Newton's iteration
                    }                //  end of the loop of Newton's iterations
                }
            }
            else
            {
                T = bb;
                for (iter=0; iter<iterMax; iter++)    //  loop of Newton's iterations to solve F(T)=0
                {
                    F  = SQR(au/(D-T)) - SQR(bv/T) - One;    //  value of F at the current T
                    if ((F <= 0.)||(T <= 0.))  break;        //  gone too far, emergency stop
                    
                    //  derivative of F with respect to T is 2.0*(SQR(au/(D-T))/(D-T) + SQR(bv/T)/T)
                    
                    Tnew = T - F/Two/(SQR(au/(D-T))/(D-T) + SQR(bv/T)/T);
                    if ((!isfinite(Tnew))||(T == Tnew))  break;         //  no progress, terminate iterations
                    T = Tnew;                                           //  Newton's iteration
                }                    //  end of the loop of Newton's iterations
            }
            if (T <= 0.)         //  singularity: the point is on the horizontal axis
            {
                if  (u <= D/a)   //  the point is not too far from the origin
                {
                    xprojx = a;  //    projects onto the vertex of the hyperbola
                    yprojx = 0.;
                    xprojy = xprojx;
                    yprojy = yprojx;
                }
                else             //  the point is too far from the origin
                {
                    xprojx = u*aa/(aa+bb);  //  projection is above the horizontal axis
                    yprojx = SQR(xprojx/a)-One;  
                    if (yprojx<0.) yprojx=0.;
                    yprojx = b*sqrt(yprojx);
                    xprojy = xprojx;
                    yprojy = yprojx;
                }
            }
            else                 //  the generic case
            {
                xprojx = u*aa/(D-T);            //   first candidate for projection
                yprojx = SQR(xprojx/a)-One;  
                if (yprojx<0.) yprojx=0.;
                yprojx = b*sqrt(yprojx);
                yprojy = v*bb/T;                //   second candidate for projection
                xprojy = a*sqrt(One+SQR(yprojy/b));
            }
        }
        else
        {
            if (Finf < 0.)
            {
                if (au <= 0.)  T = 0.;
                else           T = ( (D*au/(au+s2*bv)) < (au/s2) ? (D*au/(au+s2*bv)) : (au/s2) );
                if (T <= 0.)   T = 0;
                else
                {
                    for (iter=0; iter<iterMax; iter++)    //  loop of Newton's iterations to solve F(T)=0
                    {
                        F  = SQR(au/T) - SQR(bv/(D-T)) - One;    //  value of F at the current T
                        if ((F <= 0.)||(T >= aa))  break;        //  gone too far, emergency stop
                        
                        //  derivative of F with respect to T is -2.0*(SQR(au/T)/T + SQR(bv/(D-T))/(D-T))
                        
                        Tnew = T + F/Two/(SQR(au/T)/T + SQR(bv/(D-T))/(D-T));
                        if ((!isfinite(Tnew))||(T == Tnew))  break;         //  no progress, terminate iterations
                        T = Tnew;                                           //  Newton's iteration
                    }                      //  end of the loop of Newton's iterations
                }
            }
            else
            {
                T = aa;
                for (iter=0; iter<iterMax; iter++)    //  loop of Newton's iterations to solve F(T)=0
                {
                    F  = SQR(au/T) - SQR(bv/(D-T)) - One;    //  value of F at the current T
                    if ((F >= 0.)||(T <= 0.))  break;        //  gone too far, emergency stop
                    
                    //  derivative of F with respect to T is -2.0*(SQR(au/T)/T + SQR(bv/(D-T))/(D-T))
                    
                    Tnew = T + F/Two/(SQR(au/T)/T + SQR(bv/(D-T))/(D-T));
                    if ((!isfinite(Tnew))||(T == Tnew))  break;         //  no progress, terminate iterations
                    T = Tnew;                                           //  Newton's iteration
                }                     //  end of the loop of Newton's iterations
            }
            if (T <= 0.)              //  singularity: the point is on the vertical axis
            {
                yprojx = v*bb/D;
                xprojx = a*sqrt(One+SQR(yprojx/b));
                xprojy = xprojx;
                yprojy = yprojx;
            }
            else                      //  the generic case
            {
                xprojx = u*aa/T;              //   first candidate for projection
                yprojx = SQR(xprojx/a)-One;  
                if (yprojx<0.) yprojx=0.;
                yprojx = b*sqrt(yprojx);
                yprojy = v*bb/(D-T);          //   second candidate for projection
                xprojy = a*sqrt(One+SQR(yprojy/b));
            }
        }
    
        //            compute the projection of the point onto the hyperbola
    
        Fx = SQR(xprojx-u) + SQR(yprojx-v);    //  "quality" of first  candidate
        Fy = SQR(xprojy-u) + SQR(yprojy-v);    //  "quality" of second candidate
    
        if (Fx < Fy)           //    the first  candidate is better
        {
            Xproj(i) = xprojx;
            Yproj(i) = yprojx;
        }
        else                   //    the second candidate is better
        {
            Xproj(i) = xprojy;
            Yproj(i) = yprojy;
        }                      //    end comparing the two candidates
            
    }         //   end the main loop over the points
    
    for (i=0; i<n; i++)             //    reflect the projection point into the proper quadrant
    {
        if (X0(i)<0.) Xproj(i) = -Xproj(i);
        if (Y0(i)<0.) Yproj(i) = -Yproj(i);
    }

//       rotate back to the original system

    X0  = Xproj.array()*c - Yproj.array()*s;        //  rotating x-coordinates
    Y0  = Xproj.array()*s + Yproj.array()*c;        //  rotating y-coordinates
    Xproj = X0.array() + ParG(0);                   //  shifting x-coordinates
    Yproj = Y0.array() + ParG(1);                   //  shifting y-coordinates
    
    //RSS = (X - Xproj).squaredNorm() + (Y - Yproj).squaredNorm();  //  computing the Residual Sum of Squares
}

} // namespace mosya


