
namespace mosya {

void AdjustProjectedPointsOnConic(Mnx1& X, Mnx1& Y, M6x1& ParA, Mnx1& Xproj, Mnx1& Yproj, Mnx1& XprojA, Mnx1& YprojA, int kmax)
/*                                          <------------------------ Input --------------------->  <-------- Output -------->   Input

   Adjust ("polish") the projected points on the conic
     (this step ensures maximal accuracy)
   
   Applies Newton iterations that improve the quality of the projection
   
   Input:  X,Y  -  arrays (vectors) of the coordinates of the given data points
           Xproj,Yproj -   arrays (vectors) of the coordinates of the previously found projections
           A  -  the vector of algebraic parameters of the conic
           kmax  -  the maximum number of Newton iterations (usially just 1 or 2 are enough)
           
   Output:  XprojA,YprojA -   arrays (vectors) of the coordinates of the ajusted projection points

      Nikolai Chernov,    July 2012

*/
{
    int n=X.rows(),i,k;
    reals u,v,x,y,dx,dy,Q,Qbest,xbest,ybest,Two=2.;
    reals F,G,Fx,Fy,Gx,Gy,det,A1=ParA(0),A2=ParA(1),A3=ParA(2),A4=ParA(3),A5=ParA(4),A6=ParA(5);
    
    if (kmax <= 0)                        //  do nothing (no iterations)
    {  
        XprojA = Xproj;  YprojA = Yproj;  //  simply copy the given projections
        return;
    }
   
    for (i=0; i<n; i++)
    {
        u = X(i);  v = Y(i);        //  coordinates of the original point
        x = Xproj(i);               //  coordinates of its projection...
        y = Yproj(i);               //  found previously (to be adjusted here)
        Qbest = numeric_limits<reals>::max();
        for (k=0; k<kmax+1; k++)
        {
            Fx = A1*x + A2*y + A4;
            Fy = A2*x + A3*y + A5;
                 //   Fx and Fy are the derivatives of F (see below) with respect to x and y

            F = ((Fx + A4)*x + (Fy + A5)*y + A6)/Two;
                       //  ideally, F=0
                       //  (only true if the projected point lies ON the conic)
                 
            G = (x-u)*Fy - (y-v)*Fx;
                 //  ideally, G=0
                 //  (only true if the projection is orthogonal to the conic)
                 
            Q = Four*F*F + G*G;       //  Q is the "quality" of the adjustment
           
            if (Q>=Qbest) break;        //  no improvement, quit
            
            xbest = x;  ybest = y;   Qbest = Q;
            
            if (k==kmax)  break;        //  maxed out, quitting

            Gx = A2*(x-u) - A1*(y-v) + Fy;
            Gy = A3*(x-u) - A2*(y-v) - Fx;
                //  Gx and Gy are the derivatives of G
                //  with respect to x and y
                
                //  Next we compute the Newton step   (solve the system of 2 linear equations)
            det = Fx*Gy - Fy*Gx;            //   determinant
            dx = (F*Gy - G*Fy)/det;
            dy = (G*Fx - F*Gx)/det;
            if ((!isfinite(dx))||(!isfinite(dy)))  break;
            
            x = x - dx;  y = y - dy;  // adjust the projection
        }
        XprojA(i) = xbest;  YprojA(i) = ybest;  //  record the adjusted projection
    }
}        //  end of function AdjustProjectedPointsOnConic

} // namespace mosya

