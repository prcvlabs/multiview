
namespace mosya {

void eigen2x2(reals a, reals b, reals c, reals& d1, reals& d2, reals& Vx, reals& Vy)
/*            <------- Input ----------> <--------------- Output ----------------->

       Eigendecomposition of a symmetric 2x2 matrix
          faster and more accurate than the library function
       
       Input:  a,b,c - components of the matrix [a c
                                                 c b]
       Output:  d1,d2 - eigenvalues
                Vx,Vy - eigenvectors (normalized) for d1
                
                The eigenvector for d2 need not be computed, it is (-Vy,Vx)
                
       Note:  d1 is the leading e-value, i.e., |d1|>=|d2|
       
       Nikolai Chernov,  June 2012

*/
{
    reals disc,f;

    disc = (a-b)*(a-b) + Four*c*c;    // discriminant

    d1 = (a+b > 0.) ? (a + b + sqrt(disc))/Two : (a + b - sqrt(disc))/Two;
    d2 = (a*b - c*c)/d1;

    if (abs(a-d1) > abs(b-d1))
    {
        f = sqrt(c*c + (d1-a)*(d1-a));
        if (f==0.) 
        {
            Vx = 1.; Vy = 0.;  return;
        }
        else       
        {
            Vx = c/f;  Vy = (d1-a)/f;
        }
    }
    else
    {
        f = sqrt(c*c + (d1-b)*(d1-b));
        if (f==0.) 
        {
            Vx = 1.; Vy = 0.;  return;
        }
        else       
        {
            Vx = (d1-b)/f;  Vy = c/f;
        }
    }
    
    return;
}


} // namespace mosya

