
namespace mosya {

void ProjectPointsOntoParabola(Mnx1 X, Mnx1 Y, M5x1 ParG, Mnx1 &Xproj, Mnx1 &Yproj)
/*                             <-------- Input -------->  <------ Output -------->

   Projecting a given set of points onto a parabola

   This method is proposed by Nikolai Chernov and Hui Ma in their article
   "Least squares fitting of quadratic curves and surfaces"
              published in book 
   "Computer Vision", Editor S. R. Yoshida, Nova Science Publishers 2011; pp. 285-302
   
   It is an adaptation of Eberly's method of projecting points onto ellipses

   Input:  two n-vectors X and Y containing the coordinates of n points
           ParG is a 5x1 vector of the ellipse parameters:
           
           ParG(0)  is the x-coordinate of the parabola's vertex
           ParG(1)  is the y-coordinate of the parabola's vertex
           ParG(2)  is the distance from the focus to the directrix (often denoted by p)
           ParG(3)  is the angle of tilt of the ellipse
           
           Note: the user needs to make sure that ParG(2) > 0.
           
           Note: the equation of the hyperbola in the canonical coordinates is
                 y^2 = 2px

   Output:  two n-vectors Xproj and Yproj containing the coordinates of n projections of the points

   The algorithm is proven to converge and reaches an accuracy of 14-15 significant digit
   It takes 7-8 iterations per point, on average.

        Nikolai Chernov,  August 2012
        
*/
{
    int n=X.rows(),i,iter,iterMax;
    const reals s2=sqrt(Two),TwoThirds=Two/Three;
    reals p,pp,psqrt,c,s,u,v,uu,vv,T,Tnew,F,xprojx,yprojx,xprojy,yprojy,Fx,Fy;
    Mnx1 X0(n,1), Y0(n,1);     //  column vectors of length n (matrices n times 1)
    
    p = ParG(2);  pp = p*p; psqrt = sqrt(p);

    s = sin(ParG(3)); c = cos(ParG(3));                    //  rotation parameters
    X0  = (X.array()-ParG(0))*c + (Y.array()-ParG(1))*s;   //  rotating x-coordinates
    Y0  = (Y.array()-ParG(1))*c - (X.array()-ParG(0))*s;   //  rotating y-coordinates

    iterMax = 100;   //  Maximum number of Newton's ietrations. Usually, 5-7 are enough
    
    for (i=0; i<n; i++)                  //  main loop over the points
    {
        u = X0(i);   v = abs(Y0(i));        //  coordinates of the point
        uu = u*u;    vv = v*v;
        
        if (u <= p)  T = pow(v/p/s2,TwoThirds);
        else         T = ( (v/Two/sqrt(u-p)/psqrt) < pow(v/p/Two,TwoThirds) ? (v/Two/sqrt(u-p)/psqrt) : pow(v/p/Two,TwoThirds) );
        
        if (T <= 0.)  T = 0.;     //  safeguard against singular cases
        else
        {
            for (iter=0; iter<iterMax; iter++)      //  loop of Newton's iterations to solve F(T)=0
            {
                F  = SQR(v/T) - Two*p*(u-p+p*T);    //  value of F at the current T
                if ((F <= 0.)||(T <= 0.))  break;        //  gone too far, emergency stop
                    
                //  derivative of F with respect to T is -2.0*(SQR(v/T)/T + pp)
                    
                Tnew = T + F/Two/(SQR(v/T)/T + pp);
                if ((!isfinite(Tnew))||(T == Tnew))  break;         //  no progress, terminate iterations
                T = Tnew;                                           //  Newton's iteration
            }                    //  end of the loop of Newton's iterations
        }
        if (T <= 0.)             //  singularity: the point is on the horizontal axis
        {
            if (u <= p)          //  point is to the left of the curvature center
            {
                xprojx = 0.;     //  project to the vertex (0,0)
                yprojx = 0.;
            }
            else                 //  point is to the right of the curvature center
            {
                xprojx = u-p;    //  project above the axis
                if (xprojx <= 0.) xprojx = 0.;     //  safeguard against singular cases
                yprojx = sqrt(Two*p*xprojx);
            }
            xprojy = xprojx;
            yprojy = yprojx;
        }
        else                     //  the generic case
        {
            xprojx = u + p*(T-1);       //   first candidate for projection
            if (xprojx <= 0.)  xprojx = 0.;     //  safeguard against singular cases
            yprojx = sqrt(Two*p*xprojx);
            yprojy = v/T;               //   second candidate for projection
            xprojy = yprojy*yprojy/p/Two;
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
