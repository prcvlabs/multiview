
namespace mosya {

int IsCrossingWindow(M6x1 A, reals R)
/*

*/
{
    reals RR,F0,x,y,det,Two=2.;

    RR = R*R;

    F0 = (A(0) + A(1) + A(1) + A(2))*RR + Two*(A(3) + A(4))*R + A(5);
    
    if (((A(0) - A(1) - A(1) + A(2))*RR + Two*(A(3) - A(4))*R + A(5))*F0 <= 0) return(1);
    if (((A(0) - A(1) - A(1) + A(2))*RR - Two*(A(3) - A(4))*R + A(5))*F0 <= 0) return(1);
    if (((A(0) + A(1) + A(1) + A(2))*RR - Two*(A(3) + A(4))*R + A(5))*F0 <= 0) return(1);

    y = -(A(1)*R+A(4))/A(2);
    if (abs(y) < R)  {  if ((A(0)*RR + Two*A(1)*R*y + A(2)*y*y + Two*(A(3)*R + A(4)*y) + A(5))*F0 <= 0) return(1);  }

    y = (A(1)*R-A(4))/A(2);
    if (abs(y) < R)  {  if ((A(0)*RR - Two*A(1)*R*y + A(2)*y*y - Two*(A(3)*R - A(4)*y) + A(5))*F0 <= 0) return(1);  }

    x = -(A(1)*R+A(3))/A(0);
    if (abs(x) < R)   {  if ((A(0)*x*x + Two*A(1)*R*x + A(2)*RR + Two*(A(3)*x + A(4)*R) + A(5))*F0 <= 0) return(1);  }

    x = (A(1)*R-A(3))/A(0);
    if (abs(x) < R)   {  if ((A(0)*x*x - Two*A(1)*R*x + A(2)*RR + Two*(A(3)*x - A(4)*R) + A(5))*F0 <= 0) return(1);  }
    
    det = A(0)*A(2) - A(1)*A(1);            //   determinant
    if (abs(det)<1.e-100) return(0);       //   if singular, skip this step
    x = (A(3)*A(2) - A(4)*A(1))/det;
    y = (A(0)*A(4) - A(1)*A(3))/det;
    if ((abs(x)<R) && (abs(y)<R))  {  if ((A(0)*x*x + Two*A(1)*x*y + A(2)*y*y + Two*(A(3)*x + A(4)*y) + A(5))*F0 <= 0) return(1);  }

    return(0);
}                //  end of function IsCrossingWindow

} // namespace mosya

