
namespace mosya {

void RandomNormalPair( reals& x, reals& y )
/*
    Generator of pseudo-random numbers
    with standard normal distribution
    based on Box-Muller transformation
    
    "reals" can be replaced by "float", 
    "double", or "long double"; or it 
    can be predefined as one of these types
    
    Input:  none
    Output:  two real values, x and y,
    that are random, independent, and 
    have standard normal distribution 
    (with mean 0 and variance 1)
    
    Call:
    
        RandomNormalPair(x,y);
        
    Uses standard C++ random generator rand()
    
    To reseed the generator rand(), call  
    
        srand ( (unsigned)time(NULL) );
    
    before you start calling this function
    
       Nikolai Chernov, November 2011
*/
{
    reals rand1,rand2,wrand,One=1.,Two=2.;
/*
//       version 1, by direct calculation (slower)
       
    reals pi=3.141592653589793;
    rand1 = (reals)rand()/RAND_MAX;
    rand2 = (reals)rand()/RAND_MAX;
    x = sqrt(-Two*log(rand1))*cos(Two*pi*rand2);
    y = sqrt(-Two*log(rand1))*sin(Two*pi*rand2);
*/
//       version 2, in polar form 
//         (faster and more stable numerically)

    do {
         rand1 = Two*rand()/RAND_MAX - One;
         rand2 = Two*rand()/RAND_MAX - One;
         wrand = rand1*rand1 + rand2*rand2;
       } while (wrand >= One);
    wrand = sqrt( (-Two*log(wrand) ) / wrand );
    x = rand1 * wrand;
    y = rand2 * wrand;

}

} // namespace mosya

