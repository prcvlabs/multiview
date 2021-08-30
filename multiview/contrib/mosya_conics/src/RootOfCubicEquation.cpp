
namespace mosya {

reals RootOfCubicEquation(reals& a, reals& b, reals& c)
/*

       Finding a real root of a cubic equation x^3+ax^2+bx+c=0
       
       in case of three real roots, the function
       returns the one most distant from the other two
       (this is the most well-conditioned root)
       
       Input:  a,b,c - coefficients of the cubic polinomial

       Output:  a real root x
       
       Algorithm taken from "Numerical Recepies", section 5.6

       with added checkpoints against possible overflow 
       according to article by D. Herbison-Evans
       "Solving quartics and cubics for graphics"
       TR94-487, Basser Dept. Comp. Sci., Univ. of Sydney
       
       Nikolai Chernov,  August 2012

*/
{
    reals Q,R,A,B,QQQ,RR,aover3,factor,theta,disc,x,x1,x2,x3,x12,x13,x23;
    const reals TwentySeven=27.,OneThird=One/Three,pi23=Two*3.141592653589793238462643383L/Three;
    const reals REAL_MAX=numeric_limits<reals>::max(),MR13 = pow(REAL_MAX,OneThird);

    if (abs(a) > TwentySeven*MR13)  return (-a);      //  guard against overflow
    
    aover3 = a/Three;
    Q = aover3*aover3-b/Three;
    
    if (abs(Q) > MR13/Four)  return (-Q*pow(Four,OneThird));   //  guard against overflow
    
    QQQ = Q*Q*Q;
    R = aover3*(aover3*aover3-b/Two)+c/Two;
    if (abs(R)>sqrt(REAL_MAX))                //  guard against overflow
    {
        x = pow(abs(R),OneThird);
        if (R>=0.)  return  (x);
        else        return (-x);
    }
    RR = R*R;
    if (QQQ == 0. && RR == 0.)  x = -aover3;   //      3 equal roots
    else if (RR < QQQ)                         //      3 real roots
    {
        factor = -Two*sqrt(Q);
        theta  =  acos(R/sqrt(QQQ))/Three;
        x1 = factor*cos(theta     ) - aover3;
        x2 = factor*cos(theta+pi23) - aover3;
        x3 = factor*cos(theta-pi23) - aover3;
        
        //  roots are supposed to come in the order x1 < x3 < x2
        //  this case is treated below
        
        x13 = abs(x1-x3);
        x23 = abs(x2-x3);
        if (x13<x23)  x = x2;
        else  x = x1;
        
        //  but due to numerical errors, the order may be different
        //  so we check other cases, too

        if ((x1>x3)||(x3>x2))
        {
            x12 = abs(x1-x2);
            if ((x12<=x13)&&(x12<=x23))  x = x3;
            if ((x13<=x12)&&(x13<=x23))  x = x2;
            if ((x23<=x12)&&(x23<=x13))  x = x1;
        }
    }
    else                    //     1 real root and 2 complex (or 2 coincident real) roots
    {
        disc = sqrt(RR-QQQ);
        A = (R>0.) ? -pow(R+disc,OneThird) : pow(abs(R)+disc,OneThird);
        B = (A==0.) ? 0. : Q/A;
        x = A + B - aover3;
    }
    return x;
}

} // namespace mosya

