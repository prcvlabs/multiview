

namespace mosya {

void AtoG(M6x1 ParAin, M5x1 &ParG, int &code)
/*        <- Input ->  <----- Output ------>

  Conversion of Algebraic parameters of a conic to its Geometric parameters

   Algebraic parameters are coefficients A,B,C,D,E,F in the algebraic
   equation     Ax^2 + 2Bxy + Cy^2 + 2Dx + 2Ey + F = 0

   Geometric parameters depend on the type of the conic (ellipse, etc.)

   Input:  ParA = (A,B,C,D,E,F)' is the vector of Algebraic parameters

   Output: code is the code of the conic type (see below)
           ParG is the vector of Geometric parameters (see below)

   code:   1 - ellipse  2 - hyperbola  3 - parabola
           4 - intersecting lines  5 - parallel lines
           6 - coincident lines    7 -single line
           8 - single point        9 - imaginary ellipse
          10 - imaginary parallel lines
          11 - "impossible" equation, 1=0 or -1=0 (no solutions)

           Geometric parameters are determined only for 
               the most common types of conics:

   1. Ellipses:  canonical equation  x^2/a^2 + y^2/b^2 = 1
                 a>b or a=b are the major and minor semi-axes

       ParG = [Xcenter, Ycenter, a, b, AngleOfTilt]'

   2. Hyperbolas:  canonical equation  x^2/a^2 - y^2/b^2 = 1
                 a  is the distance from the center to each vertex
                 b  is the vertical distance from each vertex to asymptotes

       ParG = [Xcenter, Ycenter, a, b, AngleOfTilt]'

   3. Parabolas:  canonical equation  y^2 = 2px
                 p  is the distance from the focus to the directix

       ParG = [Xcenter, Ycenter, p, AngleOfTilt]'

   4. Intersecting lines:  canonical equation  (cos(u)*x+sin(u)*y+d)*(cos(v)*x+sin(v)*y+e) = 0
                   u,v are directional angles of the lines
                   d,e are distances from the origin (0,0) to the lines

       ParG = [u,d,v,e]'

        Nikolai Chernov,  February 2012
*/
{
    reals pi=3.14159265358979323846,det22,det33,dettwo,a,b,p,Angle,H,ab,Dmax,Dmin,thetap,thetam,dp,dm,Two=2.,Ten=10.; 
    int i,Imax,Imin;
    M6x1 ParA; M3x1 D3,Qmin,Qmax,Qp,Qm; M2x1 V,U,D2,Uc,Center; M3x3 M33,Q33; M2x2 M22,Q22;
    ParA = ParAin;
    ParA.normalize();   //  normalize the given algebraic parameter vector

    if ((Ten+ParA(0)==Ten)&&(Ten+ParA(1)==Ten)&&(Ten+ParA(2)==Ten))  //  if no quadratic part...
    {
        if ((Ten+ParA(3)==Ten)&&(Ten+ParA(4)==Ten))  code = 11;     //  the "pole", extreme singularity
        else  code = 7;                           //  single line (no quadratic part, only linear part)
        return;
    }
    
    M33 << ParA(0), ParA(1), ParA(3), ParA(1), ParA(2), ParA(4), ParA(3), ParA(4), ParA(5); //  big, 3x3 matrix
    det33 = M33.determinant();                  //       3x3 determinant
    
    M22 << ParA(0), ParA(1), ParA(1), ParA(2);   //  small, 2x2 matrix  (quadratic part)
    det22 = M22.determinant();                   //         2x2 determinant

    if (Ten+det33 == Ten)         //  if the big matrix is singular...
    {
        if (Ten+det22 == Ten)     //  if the small matrix is singular...
        {
            dettwo = ParA(0)*ParA(5) - ParA(3)*ParA(3) + ParA(2)*ParA(5) - ParA(4)*ParA(4);
            
            if (dettwo > 0.)  code = 10;        //  imaginary parallel lines
            if (dettwo < 0.)  code =  5;        //  parallel lines
            if (Ten+dettwo == Ten)  code = 6;  //  coincident lines
            return;
        }
        if (det22 > 0.)
        {
            code = 8;    //  single point
            return;
        }
        else  
        {
            code = 4;                  //  intersecting lines
            
            SelfAdjointEigenSolver<M3x3> eigen_solver_3(M33);    //   eigendecomposition of the 3x3 matrix M33
            D3  = eigen_solver_3.eigenvalues();                  //   D3  is the 3-vector of eigenvalues
            Q33 = eigen_solver_3.eigenvectors();                 //   Q33 is the 3x3 matrix of eigenvectors
            
            Imax = Imin = 0;  Dmax = Dmin = D3(0);              //   Find the min/max eigenvalues
            for (i=1; i<3; i++)                                 //   and their positions (indices)
            {
                if (D3(i)>Dmax)
                {
                    Dmax = D3(i);
                    Imax = i;
                }
                if (D3(i)<Dmin)
                {
                    Dmin = D3(i);
                    Imin = i;
                }
            }
            Qmax = Q33.col(Imax);
            Qmax = Qmax.array()*sqrt(abs(Dmax));
            Qmin = Q33.col(Imin);
            Qmin = Qmin.array()*sqrt(abs(Dmin));
            Qp = Qmax + Qmin;
            Qm = Qmax - Qmin;
            
            thetap = atan2(Qp(1),Qp(0));              //  Direction of the first line
            dp = Qp(2)/sqrt(Qp(1)*Qp(1)+Qp(0)*Qp(0)); //  Distance from the origin to the first line
            thetam = atan2(Qm(1),Qm(0));              //  Direction of the second line
            dm = Qm(2)/sqrt(Qm(1)*Qm(1)+Qm(0)*Qm(0)); //  Distance from the origin to the second line
            ParG << thetap, dp, thetam, dm, 0;        //     Form geometric parameters
            return;
        }
    }
//     Next: non-degenrate types of conics

    V << ParA(3), ParA(4);     // vector of coefficients%  in the linear part
    
    SelfAdjointEigenSolver<M2x2> eigen_solver(M22);   //   eigendecomposition of the 2x2 matrix M22
    D2  = eigen_solver.eigenvalues();                 //   D is the 2-vector of eigenvalues
    Q22 = eigen_solver.eigenvectors();                //   Q is the 2x2 matrix of eigenvectors
    
    U  = Q22.transpose()*V;       //  orthogonal transformation of coordinates causes adjusting the vector V
    if ((Ten+D2(0)==Ten)||(Ten+D2(1)==Ten))
    {
        code = 3;                     //   parabola
        if (abs(D2(0))>abs(D2(1)))
        {
            Uc(0) = -U(0)/D2(0);
            Uc(1) = -(U(0)*Uc(0) + ParA(5))/2/U(1);
            Center = Q22*Uc;                    //   Center of the parabola
            p = -U(1)/D2(0);                    //   the distance from the focus to the directix
            Angle = atan2(Q22(1,1), Q22(0,1));  //   Angle of tilt
            if (p<0.)
            {
                p = -p;
                Angle = Angle + pi;
            }
            ParG << Center, p, Angle, 0.;        //   Form geometric parameters
        }
        else
        {
            Uc(1) = -U(1)/D2(1);
            Uc(0) = -(U(1)*Uc(1) + ParA(5))/2/U(0);
            Center = Q22*Uc;                    //   Center of the parabola
            p = -U(0)/D2(1);                    //   the distance from the focus to the directix
            Angle = atan2(Q22(1,0), Q22(0,0));  //   Angle of tilt
            if (p<0.)
            {
                p = -p;
                Angle = Angle + pi;
            }
            ParG << Center, p, Angle, 0.;        //   Form geometric parameters
        }
        return;
    }
//        now non-degenerate and non-parabolic types of conics

    Uc = -U.array()/D2.array();
    Center = Q22*Uc;                    //  Center of the conic
    H = -U.dot(Uc) - ParA(5);  //  Free term
    if (D2(0)*D2(1) < 0.)
    {
        code = 2;                   //   hyperbola
        if (D2(0)*H > 0.)
        {
            a = sqrt(H/D2(0));          //   Axis crossing the hyperbola
            b = sqrt(-H/D2(1));         //   Axis missing the hyperbola
            Angle = atan2(Q22(1,0),Q22(0,0));    //  Angle of tilt
            if (Angle < 0.)  Angle = Angle + pi;  // ensure the range [0,pi]
        
        }
        else
        {
           a = sqrt(H/D2(1));           //   Axis crossing the hyperbola
           b = sqrt(-H/D2(0));          //   Axis missing the hyperbola
           Angle = atan2(Q22(1,1),Q22(0,1));  //  Angle of tilt
           if (Angle < 0)  Angle = Angle + pi;  // ensure the range [0,pi]
        }
        ParG << Center, a, b, Angle;      // Form geometric parameters
        return;
    }
    else
    {
        if (H*D2(0) <= 0.)  code = 9;     //  imaginary ellipse
        else
        {
            code = 1;        //   ellipse
            a = sqrt(H/D2(0));                       //   Major semi-axis
            b = sqrt(H/D2(1));                       //   Minor semi-axis
            Angle = atan2(Q22(1,0),Q22(0,0));        //   Angle of tilt
            if (Angle < 0.)  Angle = Angle + pi;     // ensure the range [0,pi]
            if (a < b)         //  making sure that a is major, b is minor
            {
               ab = a; a = b; b = ab;    //   interchange a and b
               Angle = Angle - pi/Two;   //   adjust angle of tile
               if (Angle < 0.)  Angle = Angle + pi;  // ensure the range [0,pi]
            }
            ParG << Center, a, b, Angle;     // Form geometric parameters
        }
    }
}

} // namespace mosya

