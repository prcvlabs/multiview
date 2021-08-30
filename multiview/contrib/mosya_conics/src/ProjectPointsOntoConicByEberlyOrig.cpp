
namespace mosya {

void ProjectPointsOntoConicByEberlyOrig(Mnx1& X, Mnx1& Y, M6x1& ParA, Mnx1& Xproj, Mnx1& Yproj)
/*                                      <---------- Input --------->  <------ Output -------->

   Projecting a given set of points onto a quadratic curve (conic)
   
   Method: Eberly's (original) for ellipses and 
           its adaptation by Chernov and Ma for hyperbolas and parabolas

   The conic is defined by quadratic equation 
      Ax^2 + 2Bxy + Cy^2 + 2Dx + 2Ey + F = 0

   Input:  ParA = (A,B,C,D,E,F)' is the column vector of the parameters of the conic
           XY(n,2) is the array of coordinates of n points x(i)=XY(i,1), y(i)=XY(i,2)

   Output: XYproj is the array of coordinates of projections

   The algorithm consists of several steps:
       1. Determine the type of conic and find its geometric parameters
       2. Transform the conic to its canonical coordinate system
       3. Find the projections  in the canonical coordinates
       4. Transform the projected points back to the original coordinates
       5. (optional) Adjust the coordinates of the projected points

   Step 1 is done in the function "AtoG"
   Steps 2,3,4 are done in the conic-specific functions 
               "ProjectPointsOntoEllipse", "ProjectPointsOntoHyperbola", etc.
        This module "coordinates" the procedure
   
        Nikolai Chernov,  February 2012
*/
{
    int code;
    M5x1 ParG;         //  column vector of size 5x1
    
//      Determine the type of conic and find its geometric parameters

    AtoG(ParA, ParG, code);   //  ParG is vector of geometric parameters; code is the code (type) of the conic

    if (code>3)
    {
        cout << code << endl;
        Xproj = X;  Yproj = Y;
        return;
    }
    
    //cout << code << endl;

    if (code==1)  ProjectPointsOntoEllipse(X,Y,ParG,Xproj,Yproj);      //  conic is an ellipse
    
    if (code==2)  ProjectPointsOntoHyperbola(X,Y,ParG,Xproj,Yproj);    //  conic is a hyperbola
    
    if (code==3)  ProjectPointsOntoParabola(X,Y,ParG,Xproj,Yproj);     //  conic is a parabola
    
    //   other types to be added later
    
    return;
}          //  end of function ProjectPointsOntoConicByEberlyOrig

} // namespace mosya

