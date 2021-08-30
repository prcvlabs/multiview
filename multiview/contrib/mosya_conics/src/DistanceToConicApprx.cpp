
namespace mosya {

void DistanceToConicApprx(Mnx1 &X, Mnx1 &Y, M6x1 &A, Mnx1 &Xproj, Mnx1 &Yproj, Mnx1 &Q)
/*                        <--------------------- Input --------------------->  Output

   Approximate distance to a conic from given points
   computed with the help of their projections onto the conic
   (used to evaluate the quality of the projections)
   
   The projected points must satisfy two criteria:
    (a) the distance from them to the original points should be the smallest possible
    (b) they should lie on the conic
   
   So the quality of the projection is the sum of two terms:
    (a) the distance from the projected point to the original point
    (b) the "gradient weighted algebraic" distance from the projected point to the conic
         (muptiplied by a "penalty factor")
         
   As it is impossible to directly compute the distance from the projected points to the conic,
   the "gradient weighted algebraic" distance is used instead. 
   
   The latter is multipliued by a large "penalty factor" because it maybe smaller than the actual geometric distance; 
   and in some instances it is much smaller (10 or 100 hunder times smaller...) than the geometric distance.

      Nikolai Chernov,    July 2012

*/
{
    int n=X.rows(),i;
    reals Penalty=1000.,di,Pi,Dx,Dy,A1=A(0),A2=A(1),A3=A(2),A4=A(3),A5=A(4),A6=A(5),Two=2.;

    for (i=0;i<n;i++)
    {
        di = SQR(X(i)-Xproj(i)) + SQR(Y(i)-Yproj(i));   //  distance to projection point
        Dx = A1*Xproj(i) + A2*Yproj(i) + A4;
        Dy = A2*Xproj(i) + A3*Yproj(i) + A5;
        Pi = (Dx + A4)*Xproj(i) + (Dy + A5)*Yproj(i) + A6;
        Q(i) = sqrt(di) + Penalty*abs(Pi)/sqrt(Dx*Dx+Dy*Dy)/Two;
        if (!isfinite(Q(i)))  Q(i) = numeric_limits<reals>::max();
    }

}  //  end of function ProjectionQuality

} // namespace mosya

