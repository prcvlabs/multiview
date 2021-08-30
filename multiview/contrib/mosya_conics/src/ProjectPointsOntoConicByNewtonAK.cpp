
namespace mosya {
void ProjectPointsOntoConicByNewtonAK (Mnx1& X, Mnx1& Y, M6x1& ParA, Mnx1& Xproj, Mnx1& Yproj)
/*                                     <---------- Input --------->  <------- Output ------->

   Projecting a given set of points onto a conic (ellipse or hyperbola or parabola)
   
   Method: Newton's iterations starting at the data point 
           (with safeguard checkups by Alexey Korepanov)
   
   The conic is defined by quadratic equation 
      Ax^2 + 2Bxy + Cy^2 + 2Dx + 2Ey + F = 0

   Input:  A = (A,B,C,D,E,F)' is the column vector of the parameters of the conic
           X and Y are the column vectors of the coordinates of n data points

   Output:  Xproj and Yproj are the column vectors of the coordinates of computed projections
       
       Alexey Korepanov,  September 2012

*/
{
    const reals alpha=1.0/8192.0;
    const int n=X.rows();
    const reals &A=ParA(0),&B=ParA(1),&C=ParA(2),&D=ParA(3),&E=ParA(4),&F=ParA(5);
    reals x,y,xp,yp;

    Xproj = X;  Yproj = Y;
        
    // find orthogonal matrix V such that
    //    |A   B|
    // V* |     | V
    //    |B   C|
    // is diagonal

    // A-lambda, where lambda is eigenvalue of above matrix
    reals AmL = ( A-C + (A-C>0.?One:-One)*std::sqrt((A-C)*(A-C)+Four*B*B) ) / 2,
          AmL2pB2 = std::sqrt(AmL*AmL+B*B),
          v1 = -B/AmL2pB2,
          v2 = AmL/AmL2pB2;

    // now 
    //     |v1  -v2|
    // V = |       |
    //     |v2   v1|

    for (int i=0;i<n;i++)
    {
        x = xp = X(i);
        y = yp = Y(i);
        
        // get signs of elements of jacobian at (xp,yp) in "standard" coordinate system
        reals sx = A*xp+B*yp+D,
              sy = B*xp+C*yp+E;
        // jacobian; we know starting jacobian
        reals j11 =  -sy,  j12 =  sx,
              j21 = Two*sx, j22 = Two*sy;
        // J V
        reals jv11 = j11*v1 + j12*v2,  jv12 = -j11*v2 + j12*v1,
              jv21 = j21*v1 + j22*v2,  jv22 = -j21*v2 + j22*v1;
        if (jv11==0. || jv12==0. || jv21==0. || jv22==0.) {
            std::cout << "aborting, line " << __LINE__ << "\n";
            exit(EXIT_FAILURE);
        }
        bool sjv11=jv11>0., sjv12=jv12>0.,
             sjv21=jv21>0., sjv22=jv22>0.;

        // current values of R,Q
        reals R = 0.,
              Q = (sx+D)*xp + (sy+E)*yp + F,
              R2pQ2 = R*R + Q*Q;

        bool stop = false;
        // ready to start? iterate!
        do {

            // get det of jacobian
            reals detJ = j11*j22-j12*j21;
            // check for 0 Jacobian
            if (detJ==0.) {
                std::cout << "zero jacobian!!!!!111111111 line: " << __LINE__ << "\n\n";
                exit(EXIT_FAILURE);
            }
            // descent
            reals dx = (j12*Q-j22*R)/detJ,
                  dy = (j21*R-j11*Q)/detJ,
                  descent = -R2pQ2;

            // reduce step if it is too long
            while (true) {
                // try x+dx, y+dy:
                reals xdx = x+dx, ydy = y+dy;

                if (x==xdx && y==ydy) {
                    stop = true;
                    break;
                }

                // new jacobian (SIMPLIFY ME?)
                j11 = -(B*xdx+C*ydy+E) + B*(xp-xdx) - A*(yp-ydy);   j12 = (A*xdx+B*ydy+D) + C*(xp-xdx) - B*(yp-ydy);
                j21 = Two*(A*xdx+B*ydy+D);                          j22 = Two*(B*xdx+C*ydy+E);

                // jacobian in standard coordinate system
                jv11 = j11*v1 + j12*v2;  jv12 = -j11*v2 + j12*v1;
                jv21 = j21*v1 + j22*v2;  jv22 = -j21*v2 + j22*v1;
                
                if (    !   (
                            ( (sjv11 && jv11>0.) || (!sjv11 && jv11<0.) )
                            &&
                            ( (sjv12 && jv12>0.) || (!sjv12 && jv12<0.) )
                            &&
                            ( (sjv21 && jv21>0.) || (!sjv21 && jv21<0.) )
                            &&
                            ( (sjv22 && jv22>0.) || (!sjv22 && jv22<0.) )
                            ) ) {
                    dx /= Two;
                    dy /= Two;
                    descent /= Two;
                    continue;
                }

                // values of R,Q at x+dx, y+dy
                reals Rn = (xp-xdx)*(B*xdx+C*ydy+E) - (yp-ydy)*(A*xdx+B*ydy+D),
                      Qn = ((A*xdx+B*ydy+D)+D)*xdx + ((B*xdx+C*ydy+E)+E)*ydy + F,
                      Rn2pQn2 = Rn*Rn + Qn*Qn;
                if (Rn2pQn2 < R2pQ2 + alpha*descent) {
                    x = xdx;
                    y = ydy;
                    R = Rn;
                    Q = Qn;
                    R2pQ2 = Rn2pQn2;
                    break;
                } else {
                    reals lambda = - descent / (Rn2pQn2-R2pQ2 - descent)/2.;
                    if (lambda<0.125L)
                        lambda = 0.125L;
                    dx *= lambda;
                    dy *= lambda;
                    descent *= lambda;
                }
            }
        } while (!stop);

        Xproj(i) = x;
        Yproj(i) = y;

    }
}          //  end of function ProjectPointsOntoConicByQuarticEquation

} // namespace mosya

