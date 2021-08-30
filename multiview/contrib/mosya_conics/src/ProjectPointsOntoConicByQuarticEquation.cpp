
namespace mosya {

void ProjectPointsOntoConicByQuarticEquation (Mnx1& X, Mnx1& Y, M6x1& A, Mnx1& Xproj, Mnx1& Yproj)
/*                                            <-------- Input -------->  <------- Output ------->

   Projecting a given set of points onto a conic (ellipse or hyperbola or parabola)
   
   Method:  reduce the problem to a quartic equation
            and solve the latter by computing the eigenvalues
            of the companion matrix (using real Schur decomposition)

   The conic is defined by quadratic equation 
      Ax^2 + 2Bxy + Cy^2 + 2Dx + 2Ey + F = 0

   Input:  A = (A,B,C,D,E,F)' is the column vector of the parameters of the conic
           X and Y are the column vectors of the coordinates of n data points

   Output:  Xproj and Yproj are the column vectors of the coordinates of computed projections
       
       Nikolai Chernov,  August 2012

*/
{
    int n=X.rows(),i,j,k;
    reals A1=A(0),A2=A(1),A3=A(2),A4=A(3),A5=A(4),A6=A(5),u,v,x,y,d,dmin,den;
    M3x3 M33,L33;
    M3x1 L0,Lx,Ly;   //  auxiliary vectors of length 3
    M4x4 M44;
    M4x1 roots;

    L0 << One, -(A1+A3), A1*A3-A2*A2;
    L33 = A6*L0*L0.transpose();
   
    for (i=0; i<n; i++)   //  main loop over the data points
    {
        u = X(i);  v = Y(i);   //  coordinates of the point to be projected
     
        Lx << u, A2*v-A3*u+A4, A2*A5-A3*A4;
        Ly << v, A2*u-A1*v+A5, A2*A4-A1*A5;
     
        M33 = A1*Lx*Lx.transpose() + Two*A2*Lx*Ly.transpose() + A3*Ly*Ly.transpose() + Two*A4*Lx*L0.transpose() + Two*A5*Ly*L0.transpose() + L33;
        
        //  next form the companion matrix M44 of size 4x4

        M44.row(0) << -(M33(0,1)+M33(1,0))/M33(0,0), -(M33(0,2)+M33(1,1)+M33(2,0))/M33(0,0), -(M33(1,2)+M33(2,1))/M33(0,0), -M33(2,2)/M33(0,0);
        M44.block(1,0,3,3) = M3x3::Identity();
        M44.col(3).tail(3) << 0, 0, 0;

        RealSchur<M4x4> ss(M44);   //  real Schur decomposition of the matrix M44

        //  next extract all real eigenvalues (which are the roots of the quartic equation)

        k = 0;
        if (ss.matrixT()(1,0)==0.)  
        {
            roots(k++) = ss.matrixT()(0,0);
            if (ss.matrixT()(2,1)==0.)  roots(k++) = ss.matrixT()(1,1);
        }
        if (ss.matrixT()(3,2)==0.)
        {
            roots(k++) = ss.matrixT()(3,3);
            if (ss.matrixT()(2,1)==0.)  roots(k++) = ss.matrixT()(2,2);
        }
        
        if (k==0)  
        {
            cout << " No real roots of the quartic equation" << endl;  
            //    error message if there are no real roots (should never happen)
            Xproj(i) = u;
            Yproj(i) = v;
            continue;
        }   

        //  next compute all the projection points (up to four)
        //  and choose the one closest to the given point (u,v)
                
        dmin = REAL_MAX;
        for (j=0; j<k; j++)
        {
            den = roots(j)*(L0(0)*roots(j) + L0(1)) + L0(2);
            
            x = (roots(j)*(Lx(0)*roots(j) + Lx(1)) + Lx(2))/den;
            y = (roots(j)*(Ly(0)*roots(j) + Ly(1)) + Ly(2))/den;
            
            d = SQR(x-u) + SQR(y-v);   //  distance (squared) to the given point (u,v)
            if (dmin >= d)
            {
                dmin = d;
                Xproj(i) = x;
                Yproj(i) = y;
            }
        }
            
    }
}          //  end of function ProjectPointsOntoConicByQuarticEquation

} // namespace mosya

