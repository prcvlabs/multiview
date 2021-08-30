
namespace mosya {

void ProjectPointsOntoConicByWEPeq3NR (Mnx1& X, Mnx1& Y, M6x1& A, Mnx1& Xproj, Mnx1& Yproj)
/*                                     <-------- Input -------->  <------- Output ------->

   Projecting a given set of points onto a conic (ellipse or hyperbola or parabola)
   
   Method:  WEP in which a degenerate conic is found by reducing it to a cubic equation
            and solving the latter analytically 
            (by the algorithm taken from "Numerical Recepies", section 5.6)

   The conic is defined by quadratic equation 
      Ax^2 + 2Bxy + Cy^2 + 2Dx + 2Ey + F = 0

   Input:  A = (A,B,C,D,E,F)' is the column vector of the parameters of the conic
           X and Y are the column vectors of the coordinates of n data points

   Output:  Xproj and Yproj are the column vectors of the coordinates of computed projections
       
       Nikolai Chernov,  August 2012

*/
{
    int n=X.rows(),i,iter,iterMax,j,jlast,jmin,k;
    reals a,b,c,d,a1,a2,b1,b2,c1,c2,cc,dd,cd,dc,a0,b0,c0,d1,d2,dj,dmin;
    reals F,Fnew,x,xnew,Fder,Step,a3,A0=A(0),A1=A(1),A2=A(2),A3=A(3),A4=A(4),A5=A(5);
    reals u,v,det,detmax,den,rootsum,p,q,r,disc,V1x,V1y;
    const reals small=One/pow(REAL_MAX,0.25);
    M3x3 C33,D33,G33,C33orig;
    M3x1 L1,L2;
    M4x1 Xint,Yint;

    C33orig << A0, A1, A3, A1, A2, A4, A3, A4, A5; //  3x3 symmetric matrix of the conic parameters
    
    for (i=0; i<n; i++)
    {
        u = X(i);  v = Y(i);   //  coordinates of the point to be projected
        C33 = C33orig;
     
                  //  Next compute D, the auxiliary symmetric matrix 
                  //  it comes from the orthogonality condition
                  
        D33 << A1+A1, A2-A0, A0*v-A1*u+A4, A2-A0, -A1-A1, A1*v-A2*u-A3, A0*v-A1*u+A4, A1*v-A2*u-A3, Two*(A3*v-A4*u);
    
                  //  Next find a degenerate matrix, G33, in the matrix pencil (C33,D33)
                  
        cc = C33.determinant();
        dd = D33.determinant();
        
        //  swap the matrices C33 and D33 if the latter is "more singular" than the former

        if (abs(cc) > abs(dd))  { G33 = C33;  C33 = D33;  D33 = G33;  cd = cc;  cc = dd;  dd = cd; }
        cd = (C33+D33).determinant();
        dc = (C33-D33).determinant();
        a = -dd;   b = (cd+dc)/Two-cc;   c = (dc-cd)/Two+dd;   d = cc;
        a0 = b/a;  b0 = c/a;  c0 = d/a;
        
        //  next call the cubic equation solver

        x = RootOfCubicEquation(a0,b0,c0);
        
        //  next "polish" the roots by Newton's iterations
        
        F = (C33-x*D33).determinant();
        iterMax = 10;   a3 = a+a+a;   b2 = b+b;
        for (iter=0;iter<iterMax;iter++)
        {
            Fder = (a3*x+b2)*x+c;   //  derivative of F with respect to x
            Step = F/Fder;
            if (x == x - Step)  break;         //  no progress, terminate iterations
            xnew = x - Step;                      //  Newton's iteration
            if (!isfinite(xnew))  break;
            Fnew = (C33-xnew*D33).determinant();
            if (abs(Fnew) >= abs(F))  break;
            x = xnew;   F = Fnew;
        }                       //  end of Newton's iterations
        
        G33 = C33-x*D33;        //  G33 is the singular matrix defining the degenerate conic in the pencil
        
        //  Next big step: extract two lines from the degenerate conic
        
        //  Select the 2x2 principal minor of the singular matrix G33 with the largest determinant
        //    (there are three 2x2 principal minors depending on which rows and columns you use)
                
        detmax = abs(G33(0,0)*G33(1,1)-G33(0,1)*G33(1,0));
        jlast = 3;
        det = abs(G33(2,2)*G33(1,1)-G33(1,2)*G33(2,1));
        if (detmax < det)
        {
            detmax = det;
            jlast = 1;
        }
        det = abs(G33(0,0)*G33(2,2)-G33(0,2)*G33(2,0));
        if (detmax < det)  jlast = 2;
        
        switch (jlast)
        {
            case 1:
                a = G33(1,1);  b = G33(2,2);  c = G33(1,2);
                p = G33(1,0);  q = G33(2,0);  r = G33(0,0); break;
            case 2:
                a = G33(0,0);  b = G33(2,2);  c = G33(0,2);
                p = G33(0,1);  q = G33(2,1);  r = G33(1,1); break;
            case 3:
                a = G33(0,0);  b = G33(1,1);  c = G33(0,1);
                p = G33(0,2);  q = G33(1,2);  r = G33(2,2);
        }
        
        //  eigendecomposition of the 2x2 principal minor selected previously
        
        //  first find d1 and d2, the eigenvalues of the 2x2 principal minor selected previously

        disc = SQR(a-b) + Four*c*c;
        d1 = (a+b > 0.) ? (a + b + sqrt(disc))/Two : (a + b - sqrt(disc))/Two;
        d2 = (a*b - c*c)/d1;

        if (d1*d2 > 0.)  d2 = 0.;  //  they should NOT have the same sign
        
        //  next find V1x and V1y, the components of the first unit eigenvector 
        //  of the 2x2 principal minor selected previously

        if (abs(a-d1)>abs(b-d1))
        {
            den = sqrt(c*c + SQR(d1-a));
            
            if (den==0.)  { V1x = 1.;  V1y = 0.; }
            else 
            {
                V1x = c/den;
                V1y = (d1-a)/den;
            }
        }
        else
        {
            den = sqrt(c*c + SQR(d1-b));
            
            if (den==0.)  { V1x = 1.;  V1y = 0.; }
            else 
            {
               V1x = (d1-b)/den;
               V1y = c/den;
            }
        }
        
        //  the second unit eigenvector need not be computed, it is (-V1y,V1x)
        
        //  Next find the equations of the two lines forning the degenerate conic
     
        //  equation of the first  line is L1(0)*x + L1(1)*y + L1(2) = 0
        //  equation of the second line is L2(0)*x + L2(1)*y + L2(2) = 0

        reals sd1=sqrt(abs(d1)), sd2=sqrt(abs(d2));
        a1 = sd1*V1x - sd2*V1y;
        b1 = sd1*V1y + sd2*V1x;
        a2 = sd1*V1x + sd2*V1y;
        b2 = sd1*V1y - sd2*V1x;
        if (d1<0.)  {   a2 = -a2;  b2 = -b2;   }
        
        det = a1*b2 - a2*b1;
        c1 = Two*(a1*q - b1*p)/det;
        c2 = Two*(b2*p - a2*q)/det;
        
        switch (jlast)
        {
            case 1:
                L1 << c1,a1,b1;  L2 << c2,a2,b2; break;
            case 2:
                L1 << a1,c1,b1;  L2 << a2,c2,b2; break;
            case 3:
                L1 << a1,b1,c1;  L2 << a2,b2,c2;
        }

//         Next fing the points of intersection of the lines L1,L2 with our conic
//            by solving the relevant quadratic equations

//          a,b,c will be the coefficients of the quadratic equation; more pecisely
//                  ax^2 - 2bx + c = 0

//          disc = b*b - a*c   will be the discriminant

        k = 0;      //     counter of points of intersection (total up to four)
     
//                  The first line gives us up to two points

        if ((abs(L1(0)) >= abs(L1(1))) && abs(L1(0)) > small)
        {
            p = L1(1)/L1(0);  q = L1(2)/L1(0);
            
            a = (A0*p-Two*A1)*p + A2;
            b = (A1 - A0*p)*q + A3*p - A4;
            c = (A0*q-Two*A3)*q + A5;
            
            disc = b*b - a*c;
            
            if (a*a==0.)  //  a=0, so the equation is linear
            {
                Yint(k) = c/b/Two;
                Xint(k) = -p*Yint(k) - q;
                k++;
            }
            else if (disc >= 0.)
            {
                rootsum = (b > 0.) ? (b + sqrt(disc)) : (b - sqrt(disc));
                Yint(k) = rootsum/a;
                Xint(k) = -p*Yint(k) - q;
                k++;
                Yint(k) = c/rootsum;
                Xint(k) = -p*Yint(k) - q;
                k++;
            }
        }

        if ((abs(L1(1)) > abs(L1(0))) && abs(L1(1)) > small)
        {
            p = L1(0)/L1(1);  q = L1(2)/L1(1);
            
            a = (A2*p-Two*A1)*p + A0;
            b = (A1 - A2*p)*q + A4*p - A3;
            c = (A2*q-Two*A4)*q + A5;
             
            disc = b*b - a*c;
            
            if (a*a==0.)  //  a=0, so the equation is linear
            {
                Xint(k) = c/b/Two;
                Yint(k) = -p*Xint(k) - q;
                k++;
            }
            else if (disc >= 0.)
            {
                rootsum = (b > 0.) ? (b + sqrt(disc)) : (b - sqrt(disc));
                Xint(k) = rootsum/a;
                Yint(k) = -p*Xint(k) - q;
                k++;
                Xint(k) = c/rootsum;
                Yint(k) = -p*Xint(k) - q;
                k++;
            }
        }

//                  The second line gives us up to two points

        if ((abs(L2(0)) >= abs(L2(1))) && (abs(L2(0)) > small))
        {
            p = L2(1)/L2(0);  q = L2(2)/L2(0);
            
            a = (A0*p-Two*A1)*p + A2;
            b = (A1 - A0*p)*q + A3*p - A4;
            c = (A0*q-Two*A3)*q + A5;
            
            disc = b*b - a*c;
            
            if (a*a==0.)  //  a=0, so the equation is linear
            {
                Yint(k) = c/b/Two;
                Xint(k) = -p*Yint(k) - q;
                k++;
            }
            else if (disc >= 0.)
            {
                rootsum = (b > 0.) ? (b + sqrt(disc)) : (b - sqrt(disc));
                Yint(k) = rootsum/a;
                Xint(k) = -p*Yint(k) - q;
                k++;
                Yint(k) = c/rootsum;
                Xint(k) = -p*Yint(k) - q;
                k++;
            }
        }

        if ((abs(L2(1)) > abs(L2(0))) && abs(L2(1)) > small)
        {
            p = L2(0)/L2(1);  q = L2(2)/L2(1);
            
            a = (A2*p-Two*A1)*p + A0;
            b = (A1 - A2*p)*q + A4*p - A3;
            c = (A2*q-Two*A4)*q + A5;
           
            disc = b*b - a*c;
             
            if (a*a==0.)  //  a=0, so the equation is linear
            {
                Xint(k) = c/b/Two;
                Yint(k) = -p*Xint(k) - q;
                k++;
           }
            else if (disc >= 0.)
            {
                rootsum = (b > 0.) ? (b + sqrt(disc)) : (b - sqrt(disc));
                Xint(k) = rootsum/a;
                Yint(k) = -p*Xint(k) - q;
                k++;
                Xint(k) = c/rootsum;
                Yint(k) = -p*Xint(k) - q;
                k++;
            }
        }
     
//            Last step: select the intersection point closest to the point (u,v)

        if (k>0)
        {
            jmin = 0;  
            dmin = SQR(Xint(0)-u) + SQR(Yint(0)-v);
     
            for (j=1; j<k; j++)
            {
                dj = SQR(Xint(j)-u) + SQR(Yint(j)-v);
                if (dmin > dj)
                {
                    dmin = dj;
                    jmin = j;
                }
            }
//                                   Record the selected point of intersection     
            Xproj(i) = Xint(jmin);
            Yproj(i) = Yint(jmin);
        }
        else
        {
            Xproj(i) = u;
            Yproj(i) = v;
            cout << "  WEPeq3NR fails - no intersection points" << endl;
        }
    }
}          //  end of function ProjectPointsOntoConicByWEPeq3NR

} // namespace mosya

