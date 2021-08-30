
namespace mosya {

int CircleCase (reals& A3, reals& A4, reals& A5, reals& A6, reals& xp, reals& yp)
/*              <---------------- Input ----------------->  <----- Output ----->

   Special case:  the conic is a circle
                  since A1=A3, no need to supply A1
   
    note that we are projecting the point (0,0) onto the conic
          
       Nikolai Chernov,  July 2012

*/
{
    reals AA,Frac;

    AA = A4*A4 + A5*A5;
    if (AA == 0.)
    {
        xp = sqrt(-A6/A3);
        yp = 0.;
    }
    else
    {
        Frac = -A6/(One + sqrt(One-A3*A6/AA))/AA;
        xp = A4*Frac;
        yp = A5*Frac;
    }
    if ((isfinite(xp))&&(isfinite(yp)))  return 0;   //  normal stop, returns 0 and the projected point
    else return 1;                                   //  failure, returns the "red flag"

}        //  end of function CircleCase

int PointOnMajorAxis (reals& A1, reals& A3, reals& A4, reals& A6, reals& xp, reals& yp)
/*                    <---------------- Input ----------------->  <----- Output ----->

   Special case:  the point (0,0) is on the major axis of the conic
                  since A5=0, no need to supply A5
   
    note that we are projecting the point (0,0) onto the conic
          
       Nikolai Chernov,  July 2012

*/
{
    reals polynom;
    
    xp = A4/(A3-A1);
    polynom = (A1*xp+A4+A4)*xp+A6;
    if (polynom <= 0.)
    {
        yp = sqrt(-polynom/A3);
    }
    else
    {
        xp = -A6/(A4+sqrt(A4*A4-A1*A6));
        yp = 0.;
    }
    if ((isfinite(xp))&&(isfinite(yp)))  return 0;   //  normal stop, returns 0 and the projected point
    else return 1;                                   //  failure, returns the "red flag"

}        //  end of function PointOnMajorAxis

int PointOnMinorAxis (reals& A1, reals& A3, reals& A5, reals& A6, reals& xp, reals& yp)
/*                    <---------------- Input ----------------->  <----- Output ----->

   Special case:  the point (0,0) is on the minor axis of the conic
                  since A4=0, no need to supply A4
   
    note that we are projecting the point (0,0) onto the conic
          
       Nikolai Chernov,  July 2012

*/
{
    reals polynom;
    
    if (A1 >= 0.)
    {
        xp = 0.;
        yp = -A6/(A5+sqrt(A5*A5-A3*A6));
    }
    else
    {
        yp = A5/(A1-A3);
        polynom = (A3*yp+A5+A5)*yp+A6;
        if (polynom <= 0.)
        {
            xp = sqrt(-polynom/A1);
        }
        else
        {
            xp = 0.;
            yp = -A6/(A5+sqrt(A5*A5-A3*A6));
        }
    }
    if ((isfinite(xp))&&(isfinite(yp)))  return 0;   //  normal stop, returns 0 and the projected point
    else return 1;                                   //  failure, returns the "red flag"

}        //  end of function PointOnMinorAxis

void ProjectPointsOntoConicByEberlyModified (Mnx1& X, Mnx1& Y, M6x1& ParA, Mnx1& Xproj, Mnx1& Yproj)
/*                                            <---------- Input --------->  <------- Output ------->

   Projecting a given set of points onto a conic (ellipse or hyperbola or parabola)
   
   Method:  modified Eberly's algorithm 
            (applied to the conic in algebraic parameters)

   The conic is defined by quadratic equation 
      Ax^2 + 2Bxy + Cy^2 + 2Dx + 2Ey + F = 0

   Input:  A = (A,B,C,D,E,F)' is the column vector of the parameters of the conic
           X and Y are the column vectors of the coordinates of n data points

   Output:  Xproj and Yproj are the column vectors of the coordinates of computed projections
       
       Nikolai Chernov,  July 2012

*/
{
    int n=X.rows(),i,ii,iter,iterMax,proj_found;
    reals A1,A3,A4,A5,A6,A4i,A5i,A6i,u,v,sign_x,sign_y,xp,yp,signF,den1,den2,CE,AD,AT1,AT2,Aa1,Aa2,AA1,AA2,T,F,Finf,Tnew,A44,A55,A36,A16;
    reals Vx,Vy,d1,d2,xprojx,xprojy,yprojx,yprojy,qqqx,qqqy,discx,discy,Fx,Fy;
    Mnx1 U(n,1), V(n,1), Xpr(n,1), Ypr(n,1);    //  column vectors of length n (matrices n times 1)

    
    Xproj = X;  Yproj = Y;


//      eigendecomposition of the 2x2 matrix of main parameters:
//            A  B
//            B  C

    eigen2x2(ParA(0),ParA(2),ParA(1),d1,d2,Vx,Vy);
//             A       C       B     e-val e-vec
//     d1,d2 are eigenvalues (|d1| >= |d2|
//     Vx,Vy are the components of the first unit eigenvector
//               the second need not be computed, it is (Vy,-Vx)

//      next compute the conic parameters in the new (rotated) system
//           note that A2=0, so we only need A1, A3, A4, A5, A6

    A3 = abs(d1);     //   A3 is always non-negative
    if (d1 >= 0.)     //   other parameters depend on the sign of d1
    {
        A1  = d2; 
        A4i = Vy*ParA(3)-Vx*ParA(4); 
        A5i = Vx*ParA(3)+Vy*ParA(4); 
        A6i = ParA(5);
    }
    else
    {
        A1  = -d2; 
        A4i = -Vy*ParA(3)+Vx*ParA(4); 
        A5i = -Vx*ParA(3)-Vy*ParA(4); 
        A6i = -ParA(5);
    }

//     note:  A4i,A5i,A6i are the "initial" values, to be changed later for each point (u,v)
    
    U  = X*Vy - Y*Vx;   //  rotating x-coordinates
    V  = X*Vx + Y*Vy;   //  rotating y-coordinates
    
    iterMax = 100;   //  Maximum number of Newton's ietrations. Usually, 4-6 are enough

    for (i=0;i<n;i++)        //  Main loop over the points to be projected
    {
        u = U(i);   v = V(i);    //  the current point

//          Next we translate the coordinate system so that the current
//          point (u,v) moves to the origin, i.e., it becomes (0,0)
//          Accordingy we modify A4, A5, A6.
       
        A4 = A4i + A1*u;
        A5 = A5i + A3*v;
        A6 = A6i + (A1*u + A4i + A4i)*u + (A3*v + A5i + A5i)*v;
        
//          compute some auxiliary frequently used variables

        A44 = A4*A4;
        A55 = A5*A5;
        A16 = A1*A6;
        A36 = A3*A6;

//         Next we negate the x-coordinate if needed
//         to ensure that A4>=0  (for our convenience)
    
        sign_x = One;
        if (A4 < 0.)   {  A4 = -A4;  sign_x = -One;  }

//         Next we negate the y-coordinate if needed
//         to ensure that A5>=0 
    
        sign_y = One;
        if (A5 < 0.)   {  A5 = -A5;  sign_y = -One;  }
        
        if ((A3-A1)*(A3-A1)==0.)     //  Exceptional case: conic is a circle
        {
            if (CircleCase(A3,A4,A5,A6,xp,yp))  //  failure in CircleCase
            {
                Xpr(i) = u;
                Ypr(i) = v;
            }
            else                                //  success in CircleCase 
            {
                Xpr(i) = u + sign_x*xp;
                Ypr(i) = v + sign_y*yp;
            }
            continue;
        }
            
        if (A44 == 0.)     //  Degenerate case: point on the minor axis
        {
            if (PointOnMinorAxis(A1,A3,A5,A6,xp,yp))  //  failure in PointOnMinorAxis
            {
                Xpr(i) = u;
                Ypr(i) = v;
            }
            else                                      //  success in PointOnMinorAxis
            {
                Xpr(i) = u + sign_x*xp;
                Ypr(i) = v + sign_y*yp;
            }
            continue;
        }
            
        if (A55 == 0.)     //  Degenerate case: point on the major axis
        {
            if (PointOnMajorAxis(A1,A3,A4,A6,xp,yp))  //  failure in PointOnMajorAxis
            {
                Xpr(i) = u;
                Ypr(i) = v;
            }
            else                                      //  success in PointOnMajorAxis
            {
                Xpr(i) = u + sign_x*xp;
                Ypr(i) = v + sign_y*yp;
            }
            continue;
        }
    
//             Next we choose an initial value of T
//              (T is the main auxiliary variable)

        if (A1 < 0.)                 //  Hyperbola
        {
            CE = sqrt(A5*sqrt( A3));
            AD = sqrt(A4*sqrt(-A1));
            Finf = (AD-CE)/SQR(A3-A1)*(A44*(A3+A3-A1-A1*CE/AD)/AD+A55*(A3-A1-A1+A3*AD/CE)/CE)+A6;
        }

//             Finf is the value of F at the inflection point

        if ((A1 < 0.) && (Finf > 0.))
        {
            signF = -One;
            if (A6 <= 0.)  T = 0.;
            else           T = A6/(A44-A16+sqrt(A44-A16)*A4);
        }
        else
        {
            signF = One;
            if (A6 >= 0.)  T = 0.;
            else
            { 
                if (A1 < 0.)
                {
                    T = A6/(A55-A36+sqrt(A55-A36)*A5);
                }
                else
                {
                    den1 = A44*((One-A16/A44)+sqrt(One-A16/A44));
                    den2 = A55*((One-A36/A55)+sqrt(One-A36/A55));
                    T = (den1 > den2) ? (A6/den1) : (A6/den2);
                }
            }
        }

        if (A3*T+One <= 0.)     //    Degenerate case: point on the major axis
        {
            if (PointOnMajorAxis(A1,A3,A4,A6,xp,yp))  //  failure in PointOnMajorAxis
            {
                Xpr(i) = u;
                Ypr(i) = v;
            }
            else                                      //  success in PointOnMajorAxis
            {
                Xpr(i) = u + sign_x*xp;
                Ypr(i) = v + sign_y*yp;
            }
            continue;
        }

        if (A1*T+One <= 0.)     //    Degenerate case: point on the minor axis
        {
            if (PointOnMinorAxis(A1,A3,A5,A6,xp,yp))  //  failure in PointOnMinorAxis
            {
                Xpr(i) = u;
                Ypr(i) = v;
            }
            else                                      //  success in PointOnMinorAxis
            {
                Xpr(i) = u + sign_x*xp;
                Ypr(i) = v + sign_y*yp;
            }
            continue;
        }
 
//               the main, non-singular case
//               start Newton's iterations to solve equation F(T)=0

        proj_found = 0;     //   flag will be set to 1 if the projection is found within the next loop

        for (iter=0;iter<iterMax;iter++)      //   loop of Newton's iterations to solve F(T)=0
        {
            AT1 = A1*T+One;  AT2 = A3*T+One;
            if ((AT1<=0.)||(AT2<=0.)) break;
            Aa1 = A4/AT1;  AA1 = Aa1*Aa1;  
            Aa2 = A5/AT2;  AA2 = Aa2*Aa2;
        
            F = -T*(AA1*(AT1+One) + AA2*(AT2+One)) + A6;   // value of F; we need to find T such that F(T)=0
            
            if (signF*F < 0.)        //  gone too far: emergency stop
            {
                if (iter == 0)      //  started on the wrong side (due to a hidden singularity)
                {
                    if (A4 > A5)  ii = PointOnMajorAxis(A1,A3,A4,A6,xp,yp);
                    else          ii = PointOnMinorAxis(A1,A3,A5,A6,xp,yp);
                    if (ii)  //  failure in PointOnMinorAxis or PointOnMajorAxis above
                    {
                        Xpr(i) = u;
                        Ypr(i) = v;
                    }
                    else  //  success in PointOnMinorAxis or PointOnMajorAxis above
                    {
                        Xpr(i) = u + sign_x*xp;
                        Ypr(i) = v + sign_y*yp;
                    }
                    proj_found = 1;
                }
                break;
            }
            //  derivative of F with respect to T is -2.0*(AA1/AT1 + AA2/AT2)
            
            Tnew = T + F/Two/(AA1/AT1 + AA2/AT2);
            
            if ((!isfinite(Tnew))||(T == Tnew))  break;         //  no progress, terminate iterations
            
            T = Tnew;                         //  Newton's iteration
            
        }                              //  end of Newton's iterations

        if (proj_found) continue;
        
        //  Next use the computed valu of T to find the projection points
        //    two projection point candidates will be found first
        
        //    (xprojx,yprojx)  is the first candidate
    
        xprojx = -T*Aa1;
        qqqx = (A1*xprojx+A4+A4)*xprojx+A6;
        discx = A55-A3*qqqx;
        if (discx > 0.)
        {
            yprojx = -qqqx/(A5+sqrt(discx));
            Fx = xprojx*xprojx + yprojx*yprojx;        //    "quality" of the first candidate
        }
        else   Fx = REAL_MAX;
        
        //    (xprojy,yprojy)  is the second candidate
    
        yprojy = -T*Aa2;
        qqqy = (A3*yprojy+A5+A5)*yprojy+A6;
        discy = A44-A1*qqqy;
        if (discy > 0.)
        {
            xprojy = -qqqy/(A4+sqrt(discy));
            Fy = xprojy*xprojy + yprojy*yprojy;        //    "quality" of the second candidate
        }
        else   Fy = REAL_MAX;

        if ((Fx == REAL_MAX) && (Fy == REAL_MAX))      //    neither candidate exists, assume the conic is a circle
        {
            if (CircleCase(A3,A4,A5,A6,xp,yp))  //  failure in CircleCase
            {
                Xpr(i) = u;
                Ypr(i) = v;
            }
            else                                //  success in CircleCase 
            {
                Xpr(i) = u + sign_x*xp;
                Ypr(i) = v + sign_y*yp;
            }
            continue;
        }

        if (Fx < Fy)         //    the first  candidate is better  (or the second does not exist)
        {
            Xpr(i) = sign_x*xprojx + u;
            Ypr(i) = sign_y*yprojx + v;
        }
        else                 //    the second candidate is better  (or the first  does not exist)
        {
            Xpr(i) = sign_x*xprojy + u;
            Ypr(i) = sign_y*yprojy + v;
        }
    }

    Xproj  = Xpr*Vy + Ypr*Vx;   //  rotating x-coordinates
    Yproj  =-Xpr*Vx + Ypr*Vy;   //  rotating y-coordinates
    
}      //  end of function ProjectPointsOntoConicByEberlyAlg
 
} // namespace mosya

