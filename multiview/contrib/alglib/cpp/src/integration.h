/*************************************************************************
ALGLIB 3.12.0 (source code generated 2017-08-22)
Copyright (c) Sergey Bochkanov (ALGLIB project).

>>> SOURCE LICENSE >>>
This  program  is  a  Commercial Edition  of the ALGLIB package  licensed
to Perceive, Inc. (Licensee), agreement ID is AGR-20170916-1

As of 2017-09-16:
* DEV-1 license plan is purchased (1 developer)
* following developers are registered for this license:
  * amichaux@perceiveinc.com

========================== GENERAL INFORMATION ==========================

1. Only Licensee and  its Sublicensees  can  use/distribute  it according
to  the  ALGLIB  License  Agreement (see  below) between  Licensor  (Sole
Proprietor Bochkanov Sergey Anatolyevich) and Licensee.

2. All developers working  for  Licensee  should  register  themselves at
alglib.net. Quick access link for Licensee's account can be found at  the
section 9 of scanned copy of ALGLIB License Agreement or in e-mails  from
ALGLIB Project.

3. This  source  code  may  contain  modifications  made  by  Licensee or
Sublicensees  which  fall under the terms of the ALGLIB License Agreement
too.

4. Text  below  is  an excerpt from complete ALGLIB License Agreement,  a
part which governs usage and redistribution rights granted  to  Licensee.
See agreement-v6.pdf at the root of ALGLIB distribution for complete text
of license agreement.

================ ALGLIB LICENSE AGREEMENT ( APPENDIX A ) ================

DEFINITIONS:
* "ALGLIB" – software delivered by Licensor  to  Licensee  under  present
  Agreement. ALGLIB may include  Binary  Components  (delivered  only  in
  binary form) and Source  Code  Components  (with  optional  precompiled
  binary form).
  ALGLIB  may  include  optional  third-party  components, which may have
  their own licensing terms. Specific list of components  and  additional
  licensing terms (if there are any) is included in the license.txt  file
  in the root of archive containing ALGLIB. If you  decide  not  to  link
  ALGLIB with these optional components, their  licensing  terms  do  not
  apply to you.
* "Application"  -  program  developed  by  Licensee  (either  standalone
  application or software development library) which includes  ALGLIB  as
  one of its parts .
* "Sublicensee"  -  any  party  (including  resellers)   which   receives
  Application from Licensee or another Sublicensee.
* "Application License Agreement"  -  agreement  which   governs   usage/
  redistribution of the Application.
  
LICENSE GRANT:
Subject to the License Restrictions below, Licensor  grants  to  Licensee
the following non-exclusive royalty-free licenses:
A. To modify Source Code Components of ALGLIB and to use modified version
   on the terms of this Agreement.
B. To  develop  Applications  which  use  ALGLIB  and  to distribute such
   Applications in Binary and/or Source Code forms,  with  ALGLIB  either
   statically or dynamically linked. This right is granted provided that:
   * distribution of Source Code forms of Application/ALGLIB is performed
     subject to additional conditions set by clause H (this clause is not
     applied to binary-only distribution)
   * such Applications add significant  primary  functionality  different
     from that of the ALGLIB.
   * such Applications do not expose ALGLIB API (application  programming
     interface) either directly or indirectly
   * Sublicensee  has  no   right   to  use  ALGLIB  except  as  part  of
     the Application
   * any  subsequent  redistribution   respects    conditions    of   the
     present Agreement
   * all developers working for Licensee  should  register  at  company's
     account at www.alglib.net (in order to find login link, see  section
     9 of scanned copy of ALGLIB License Agreement -or find it in e-mails
     from ALGLIB Project).
C. To use Resellers for distribution of the  Application  (in  Binary  or
   Source Code forms), provided that the only activity Reseller  performs
   with Application is redistribution.
   
LICENSE RESTRICTIONS:
D. Licensee/Sublicensee may NOT use, copy or distribute ALGLIB except  as
   provided in this Agreement.
D2. Licensee/Sublicensee may NOT rent or lease ALGLIB to any third party.
E. Licensee/Sublicensee may NOT disassemble, reverse engineer, decompile,
   modify Binary Components of ALGLIB or compiled forms  of  Source  Code
   components.
F. Licensee/Sublicensee  may  NOT  remove  any  copyright notice from the
   Source Code / Binary Components.
G. Licensee/Sublicensee may NOT  disable/remove  code  which  checks  for
   presence of license keys (if such code is included in ALGLIB) from the
   Source Code / Binary Components.
H. Distribution of  Source  Code  forms  of  Application/ALGLIB  must  be
   performed subject to additional conditions:
   * Source Code Components of ALGLIB are distributed only as part of the
     Application. They are not  publicly  distributed.  Sublicensee  must
     explicitly accept Application License Agreement in order  to  access
     ALGLIB source code.
   * Sublicensee has no right to redistribute Application/ALGLIB (in  any
     form, Binary or Source Code), unless Sublicensee is Reseller who  is
     fully compliant with conditions set by clause C.
   * Sublicensee has no right to modify ALGLIB Source  Code,  except  for
     the purpose of fixing bugs
   * Sublicensee has no right to workaround "use ALGLIB only as  part  of
     the Application" limitation by sequentially modifying Application in
     a way which effectively creates new program with different  purpose.
     Application   License  Agreement  may  (a)  explicitly  forbid  such
     modifications, or (b) allow only limited set of "safe" modifications
     (developing plugins, fixing bugs, modifying only specific  parts  of
     the Application).
     
COPYRIGHT:
Title to the ALGLIB and all copies  thereof  remain  with  Licensor.  The
ALGLIB is copyrighted and is protected  by  Russian  copyright  laws  and
international treaty provisions. You will not remove any copyright notice
from the ALGLIB files. You agree to prevent any unauthorized  copying  of
the ALGLIB. Except as expressly provided herein, Licensor does not  grant
any express or implied right to you under Licensor  patents,  copyrights,
trademarks, or trade secret information.
>>> END OF LICENSE >>>
*************************************************************************/
#ifndef _integration_pkg_h
#define _integration_pkg_h
#include "ap.h"
#include "alglibinternal.h"
#include "linalg.h"
#include "alglibmisc.h"
#include "specialfunctions.h"

/////////////////////////////////////////////////////////////////////////
//
// THIS SECTION CONTAINS COMPUTATIONAL CORE DECLARATIONS (DATATYPES)
//
/////////////////////////////////////////////////////////////////////////
namespace alglib_impl
{
typedef struct
{
    ae_int_t terminationtype;
    ae_int_t nfev;
    ae_int_t nintervals;
} autogkreport;
typedef struct
{
    double a;
    double b;
    double eps;
    double xwidth;
    double x;
    double f;
    ae_int_t info;
    double r;
    ae_matrix heap;
    ae_int_t heapsize;
    ae_int_t heapwidth;
    ae_int_t heapused;
    double sumerr;
    double sumabs;
    ae_vector qn;
    ae_vector wg;
    ae_vector wk;
    ae_vector wr;
    ae_int_t n;
    rcommstate rstate;
} autogkinternalstate;
typedef struct
{
    double a;
    double b;
    double alpha;
    double beta;
    double xwidth;
    double x;
    double xminusa;
    double bminusx;
    ae_bool needf;
    double f;
    ae_int_t wrappermode;
    autogkinternalstate internalstate;
    rcommstate rstate;
    double v;
    ae_int_t terminationtype;
    ae_int_t nfev;
    ae_int_t nintervals;
} autogkstate;

}

/////////////////////////////////////////////////////////////////////////
//
// THIS SECTION CONTAINS C++ INTERFACE
//
/////////////////////////////////////////////////////////////////////////
namespace alglib
{





/*************************************************************************
Integration report:
* TerminationType = completetion code:
    * -5    non-convergence of Gauss-Kronrod nodes
            calculation subroutine.
    * -1    incorrect parameters were specified
    *  1    OK
* Rep.NFEV countains number of function calculations
* Rep.NIntervals contains number of intervals [a,b]
  was partitioned into.
*************************************************************************/
class _autogkreport_owner
{
public:
    _autogkreport_owner();
    _autogkreport_owner(const _autogkreport_owner &rhs);
    _autogkreport_owner& operator=(const _autogkreport_owner &rhs);
    virtual ~_autogkreport_owner();
    alglib_impl::autogkreport* c_ptr();
    alglib_impl::autogkreport* c_ptr() const;
protected:
    alglib_impl::autogkreport *p_struct;
};
class autogkreport : public _autogkreport_owner
{
public:
    autogkreport();
    autogkreport(const autogkreport &rhs);
    autogkreport& operator=(const autogkreport &rhs);
    virtual ~autogkreport();
    ae_int_t &terminationtype;
    ae_int_t &nfev;
    ae_int_t &nintervals;

};


/*************************************************************************
This structure stores state of the integration algorithm.

Although this class has public fields,  they are not intended for external
use. You should use ALGLIB functions to work with this class:
* autogksmooth()/AutoGKSmoothW()/... to create objects
* autogkintegrate() to begin integration
* autogkresults() to get results
*************************************************************************/
class _autogkstate_owner
{
public:
    _autogkstate_owner();
    _autogkstate_owner(const _autogkstate_owner &rhs);
    _autogkstate_owner& operator=(const _autogkstate_owner &rhs);
    virtual ~_autogkstate_owner();
    alglib_impl::autogkstate* c_ptr();
    alglib_impl::autogkstate* c_ptr() const;
protected:
    alglib_impl::autogkstate *p_struct;
};
class autogkstate : public _autogkstate_owner
{
public:
    autogkstate();
    autogkstate(const autogkstate &rhs);
    autogkstate& operator=(const autogkstate &rhs);
    virtual ~autogkstate();
    ae_bool &needf;
    double &x;
    double &xminusa;
    double &bminusx;
    double &f;

};

/*************************************************************************
Computation of nodes and weights for a Gauss quadrature formula

The algorithm generates the N-point Gauss quadrature formula  with  weight
function given by coefficients alpha and beta  of  a  recurrence  relation
which generates a system of orthogonal polynomials:

P-1(x)   =  0
P0(x)    =  1
Pn+1(x)  =  (x-alpha(n))*Pn(x)  -  beta(n)*Pn-1(x)

and zeroth moment Mu0

Mu0 = integral(W(x)dx,a,b)

INPUT PARAMETERS:
    Alpha   -   array[0..N-1], alpha coefficients
    Beta    -   array[0..N-1], beta coefficients
                Zero-indexed element is not used and may be arbitrary.
                Beta[I]>0.
    Mu0     -   zeroth moment of the weight function.
    N       -   number of nodes of the quadrature formula, N>=1

OUTPUT PARAMETERS:
    Info    -   error code:
                * -3    internal eigenproblem solver hasn't converged
                * -2    Beta[i]<=0
                * -1    incorrect N was passed
                *  1    OK
    X       -   array[0..N-1] - array of quadrature nodes,
                in ascending order.
    W       -   array[0..N-1] - array of quadrature weights.

  -- ALGLIB --
     Copyright 2005-2009 by Bochkanov Sergey
*************************************************************************/
void gqgeneraterec(const real_1d_array &alpha, const real_1d_array &beta, const double mu0, const ae_int_t n, ae_int_t &info, real_1d_array &x, real_1d_array &w);


/*************************************************************************
Computation of nodes and weights for a Gauss-Lobatto quadrature formula

The algorithm generates the N-point Gauss-Lobatto quadrature formula  with
weight function given by coefficients alpha and beta of a recurrence which
generates a system of orthogonal polynomials.

P-1(x)   =  0
P0(x)    =  1
Pn+1(x)  =  (x-alpha(n))*Pn(x)  -  beta(n)*Pn-1(x)

and zeroth moment Mu0

Mu0 = integral(W(x)dx,a,b)

INPUT PARAMETERS:
    Alpha   -   array[0..N-2], alpha coefficients
    Beta    -   array[0..N-2], beta coefficients.
                Zero-indexed element is not used, may be arbitrary.
                Beta[I]>0
    Mu0     -   zeroth moment of the weighting function.
    A       -   left boundary of the integration interval.
    B       -   right boundary of the integration interval.
    N       -   number of nodes of the quadrature formula, N>=3
                (including the left and right boundary nodes).

OUTPUT PARAMETERS:
    Info    -   error code:
                * -3    internal eigenproblem solver hasn't converged
                * -2    Beta[i]<=0
                * -1    incorrect N was passed
                *  1    OK
    X       -   array[0..N-1] - array of quadrature nodes,
                in ascending order.
    W       -   array[0..N-1] - array of quadrature weights.

  -- ALGLIB --
     Copyright 2005-2009 by Bochkanov Sergey
*************************************************************************/
void gqgenerategausslobattorec(const real_1d_array &alpha, const real_1d_array &beta, const double mu0, const double a, const double b, const ae_int_t n, ae_int_t &info, real_1d_array &x, real_1d_array &w);


/*************************************************************************
Computation of nodes and weights for a Gauss-Radau quadrature formula

The algorithm generates the N-point Gauss-Radau  quadrature  formula  with
weight function given by the coefficients alpha and  beta  of a recurrence
which generates a system of orthogonal polynomials.

P-1(x)   =  0
P0(x)    =  1
Pn+1(x)  =  (x-alpha(n))*Pn(x)  -  beta(n)*Pn-1(x)

and zeroth moment Mu0

Mu0 = integral(W(x)dx,a,b)

INPUT PARAMETERS:
    Alpha   -   array[0..N-2], alpha coefficients.
    Beta    -   array[0..N-1], beta coefficients
                Zero-indexed element is not used.
                Beta[I]>0
    Mu0     -   zeroth moment of the weighting function.
    A       -   left boundary of the integration interval.
    N       -   number of nodes of the quadrature formula, N>=2
                (including the left boundary node).

OUTPUT PARAMETERS:
    Info    -   error code:
                * -3    internal eigenproblem solver hasn't converged
                * -2    Beta[i]<=0
                * -1    incorrect N was passed
                *  1    OK
    X       -   array[0..N-1] - array of quadrature nodes,
                in ascending order.
    W       -   array[0..N-1] - array of quadrature weights.


  -- ALGLIB --
     Copyright 2005-2009 by Bochkanov Sergey
*************************************************************************/
void gqgenerategaussradaurec(const real_1d_array &alpha, const real_1d_array &beta, const double mu0, const double a, const ae_int_t n, ae_int_t &info, real_1d_array &x, real_1d_array &w);


/*************************************************************************
Returns nodes/weights for Gauss-Legendre quadrature on [-1,1] with N
nodes.

INPUT PARAMETERS:
    N           -   number of nodes, >=1

OUTPUT PARAMETERS:
    Info        -   error code:
                    * -4    an  error   was   detected   when  calculating
                            weights/nodes.  N  is  too  large   to  obtain
                            weights/nodes  with  high   enough   accuracy.
                            Try  to   use   multiple   precision  version.
                    * -3    internal eigenproblem solver hasn't  converged
                    * -1    incorrect N was passed
                    * +1    OK
    X           -   array[0..N-1] - array of quadrature nodes,
                    in ascending order.
    W           -   array[0..N-1] - array of quadrature weights.


  -- ALGLIB --
     Copyright 12.05.2009 by Bochkanov Sergey
*************************************************************************/
void gqgenerategausslegendre(const ae_int_t n, ae_int_t &info, real_1d_array &x, real_1d_array &w);


/*************************************************************************
Returns  nodes/weights  for  Gauss-Jacobi quadrature on [-1,1] with weight
function W(x)=Power(1-x,Alpha)*Power(1+x,Beta).

INPUT PARAMETERS:
    N           -   number of nodes, >=1
    Alpha       -   power-law coefficient, Alpha>-1
    Beta        -   power-law coefficient, Beta>-1

OUTPUT PARAMETERS:
    Info        -   error code:
                    * -4    an  error  was   detected   when   calculating
                            weights/nodes. Alpha or  Beta  are  too  close
                            to -1 to obtain weights/nodes with high enough
                            accuracy, or, may be, N is too large.  Try  to
                            use multiple precision version.
                    * -3    internal eigenproblem solver hasn't converged
                    * -1    incorrect N/Alpha/Beta was passed
                    * +1    OK
    X           -   array[0..N-1] - array of quadrature nodes,
                    in ascending order.
    W           -   array[0..N-1] - array of quadrature weights.


  -- ALGLIB --
     Copyright 12.05.2009 by Bochkanov Sergey
*************************************************************************/
void gqgenerategaussjacobi(const ae_int_t n, const double alpha, const double beta, ae_int_t &info, real_1d_array &x, real_1d_array &w);


/*************************************************************************
Returns  nodes/weights  for  Gauss-Laguerre  quadrature  on  [0,+inf) with
weight function W(x)=Power(x,Alpha)*Exp(-x)

INPUT PARAMETERS:
    N           -   number of nodes, >=1
    Alpha       -   power-law coefficient, Alpha>-1

OUTPUT PARAMETERS:
    Info        -   error code:
                    * -4    an  error  was   detected   when   calculating
                            weights/nodes. Alpha is too  close  to  -1  to
                            obtain weights/nodes with high enough accuracy
                            or, may  be,  N  is  too  large.  Try  to  use
                            multiple precision version.
                    * -3    internal eigenproblem solver hasn't converged
                    * -1    incorrect N/Alpha was passed
                    * +1    OK
    X           -   array[0..N-1] - array of quadrature nodes,
                    in ascending order.
    W           -   array[0..N-1] - array of quadrature weights.


  -- ALGLIB --
     Copyright 12.05.2009 by Bochkanov Sergey
*************************************************************************/
void gqgenerategausslaguerre(const ae_int_t n, const double alpha, ae_int_t &info, real_1d_array &x, real_1d_array &w);


/*************************************************************************
Returns  nodes/weights  for  Gauss-Hermite  quadrature on (-inf,+inf) with
weight function W(x)=Exp(-x*x)

INPUT PARAMETERS:
    N           -   number of nodes, >=1

OUTPUT PARAMETERS:
    Info        -   error code:
                    * -4    an  error  was   detected   when   calculating
                            weights/nodes.  May be, N is too large. Try to
                            use multiple precision version.
                    * -3    internal eigenproblem solver hasn't converged
                    * -1    incorrect N/Alpha was passed
                    * +1    OK
    X           -   array[0..N-1] - array of quadrature nodes,
                    in ascending order.
    W           -   array[0..N-1] - array of quadrature weights.


  -- ALGLIB --
     Copyright 12.05.2009 by Bochkanov Sergey
*************************************************************************/
void gqgenerategausshermite(const ae_int_t n, ae_int_t &info, real_1d_array &x, real_1d_array &w);

/*************************************************************************
Computation of nodes and weights of a Gauss-Kronrod quadrature formula

The algorithm generates the N-point Gauss-Kronrod quadrature formula  with
weight  function  given  by  coefficients  alpha  and beta of a recurrence
relation which generates a system of orthogonal polynomials:

    P-1(x)   =  0
    P0(x)    =  1
    Pn+1(x)  =  (x-alpha(n))*Pn(x)  -  beta(n)*Pn-1(x)

and zero moment Mu0

    Mu0 = integral(W(x)dx,a,b)


INPUT PARAMETERS:
    Alpha       -   alpha coefficients, array[0..floor(3*K/2)].
    Beta        -   beta coefficients,  array[0..ceil(3*K/2)].
                    Beta[0] is not used and may be arbitrary.
                    Beta[I]>0.
    Mu0         -   zeroth moment of the weight function.
    N           -   number of nodes of the Gauss-Kronrod quadrature formula,
                    N >= 3,
                    N =  2*K+1.

OUTPUT PARAMETERS:
    Info        -   error code:
                    * -5    no real and positive Gauss-Kronrod formula can
                            be created for such a weight function  with  a
                            given number of nodes.
                    * -4    N is too large, task may be ill  conditioned -
                            x[i]=x[i+1] found.
                    * -3    internal eigenproblem solver hasn't converged
                    * -2    Beta[i]<=0
                    * -1    incorrect N was passed
                    * +1    OK
    X           -   array[0..N-1] - array of quadrature nodes,
                    in ascending order.
    WKronrod    -   array[0..N-1] - Kronrod weights
    WGauss      -   array[0..N-1] - Gauss weights (interleaved with zeros
                    corresponding to extended Kronrod nodes).

  -- ALGLIB --
     Copyright 08.05.2009 by Bochkanov Sergey
*************************************************************************/
void gkqgeneraterec(const real_1d_array &alpha, const real_1d_array &beta, const double mu0, const ae_int_t n, ae_int_t &info, real_1d_array &x, real_1d_array &wkronrod, real_1d_array &wgauss);


/*************************************************************************
Returns   Gauss   and   Gauss-Kronrod   nodes/weights  for  Gauss-Legendre
quadrature with N points.

GKQLegendreCalc (calculation) or  GKQLegendreTbl  (precomputed  table)  is
used depending on machine precision and number of nodes.

INPUT PARAMETERS:
    N           -   number of Kronrod nodes, must be odd number, >=3.

OUTPUT PARAMETERS:
    Info        -   error code:
                    * -4    an  error   was   detected   when  calculating
                            weights/nodes.  N  is  too  large   to  obtain
                            weights/nodes  with  high   enough   accuracy.
                            Try  to   use   multiple   precision  version.
                    * -3    internal eigenproblem solver hasn't converged
                    * -1    incorrect N was passed
                    * +1    OK
    X           -   array[0..N-1] - array of quadrature nodes, ordered in
                    ascending order.
    WKronrod    -   array[0..N-1] - Kronrod weights
    WGauss      -   array[0..N-1] - Gauss weights (interleaved with zeros
                    corresponding to extended Kronrod nodes).


  -- ALGLIB --
     Copyright 12.05.2009 by Bochkanov Sergey
*************************************************************************/
void gkqgenerategausslegendre(const ae_int_t n, ae_int_t &info, real_1d_array &x, real_1d_array &wkronrod, real_1d_array &wgauss);


/*************************************************************************
Returns   Gauss   and   Gauss-Kronrod   nodes/weights   for   Gauss-Jacobi
quadrature on [-1,1] with weight function

    W(x)=Power(1-x,Alpha)*Power(1+x,Beta).

INPUT PARAMETERS:
    N           -   number of Kronrod nodes, must be odd number, >=3.
    Alpha       -   power-law coefficient, Alpha>-1
    Beta        -   power-law coefficient, Beta>-1

OUTPUT PARAMETERS:
    Info        -   error code:
                    * -5    no real and positive Gauss-Kronrod formula can
                            be created for such a weight function  with  a
                            given number of nodes.
                    * -4    an  error  was   detected   when   calculating
                            weights/nodes. Alpha or  Beta  are  too  close
                            to -1 to obtain weights/nodes with high enough
                            accuracy, or, may be, N is too large.  Try  to
                            use multiple precision version.
                    * -3    internal eigenproblem solver hasn't converged
                    * -1    incorrect N was passed
                    * +1    OK
                    * +2    OK, but quadrature rule have exterior  nodes,
                            x[0]<-1 or x[n-1]>+1
    X           -   array[0..N-1] - array of quadrature nodes, ordered in
                    ascending order.
    WKronrod    -   array[0..N-1] - Kronrod weights
    WGauss      -   array[0..N-1] - Gauss weights (interleaved with zeros
                    corresponding to extended Kronrod nodes).


  -- ALGLIB --
     Copyright 12.05.2009 by Bochkanov Sergey
*************************************************************************/
void gkqgenerategaussjacobi(const ae_int_t n, const double alpha, const double beta, ae_int_t &info, real_1d_array &x, real_1d_array &wkronrod, real_1d_array &wgauss);


/*************************************************************************
Returns Gauss and Gauss-Kronrod nodes for quadrature with N points.

Reduction to tridiagonal eigenproblem is used.

INPUT PARAMETERS:
    N           -   number of Kronrod nodes, must be odd number, >=3.

OUTPUT PARAMETERS:
    Info        -   error code:
                    * -4    an  error   was   detected   when  calculating
                            weights/nodes.  N  is  too  large   to  obtain
                            weights/nodes  with  high   enough   accuracy.
                            Try  to   use   multiple   precision  version.
                    * -3    internal eigenproblem solver hasn't converged
                    * -1    incorrect N was passed
                    * +1    OK
    X           -   array[0..N-1] - array of quadrature nodes, ordered in
                    ascending order.
    WKronrod    -   array[0..N-1] - Kronrod weights
    WGauss      -   array[0..N-1] - Gauss weights (interleaved with zeros
                    corresponding to extended Kronrod nodes).

  -- ALGLIB --
     Copyright 12.05.2009 by Bochkanov Sergey
*************************************************************************/
void gkqlegendrecalc(const ae_int_t n, ae_int_t &info, real_1d_array &x, real_1d_array &wkronrod, real_1d_array &wgauss);


/*************************************************************************
Returns Gauss and Gauss-Kronrod nodes for quadrature with N  points  using
pre-calculated table. Nodes/weights were  computed  with  accuracy  up  to
1.0E-32 (if MPFR version of ALGLIB is used). In standard double  precision
accuracy reduces to something about 2.0E-16 (depending  on your compiler's
handling of long floating point constants).

INPUT PARAMETERS:
    N           -   number of Kronrod nodes.
                    N can be 15, 21, 31, 41, 51, 61.

OUTPUT PARAMETERS:
    X           -   array[0..N-1] - array of quadrature nodes, ordered in
                    ascending order.
    WKronrod    -   array[0..N-1] - Kronrod weights
    WGauss      -   array[0..N-1] - Gauss weights (interleaved with zeros
                    corresponding to extended Kronrod nodes).


  -- ALGLIB --
     Copyright 12.05.2009 by Bochkanov Sergey
*************************************************************************/
void gkqlegendretbl(const ae_int_t n, real_1d_array &x, real_1d_array &wkronrod, real_1d_array &wgauss, double &eps);

/*************************************************************************
Integration of a smooth function F(x) on a finite interval [a,b].

Fast-convergent algorithm based on a Gauss-Kronrod formula is used. Result
is calculated with accuracy close to the machine precision.

Algorithm works well only with smooth integrands.  It  may  be  used  with
continuous non-smooth integrands, but with  less  performance.

It should never be used with integrands which have integrable singularities
at lower or upper limits - algorithm may crash. Use AutoGKSingular in such
cases.

INPUT PARAMETERS:
    A, B    -   interval boundaries (A<B, A=B or A>B)

OUTPUT PARAMETERS
    State   -   structure which stores algorithm state

SEE ALSO
    AutoGKSmoothW, AutoGKSingular, AutoGKResults.


  -- ALGLIB --
     Copyright 06.05.2009 by Bochkanov Sergey
*************************************************************************/
void autogksmooth(const double a, const double b, autogkstate &state);


/*************************************************************************
Integration of a smooth function F(x) on a finite interval [a,b].

This subroutine is same as AutoGKSmooth(), but it guarantees that interval
[a,b] is partitioned into subintervals which have width at most XWidth.

Subroutine  can  be  used  when  integrating nearly-constant function with
narrow "bumps" (about XWidth wide). If "bumps" are too narrow, AutoGKSmooth
subroutine can overlook them.

INPUT PARAMETERS:
    A, B    -   interval boundaries (A<B, A=B or A>B)

OUTPUT PARAMETERS
    State   -   structure which stores algorithm state

SEE ALSO
    AutoGKSmooth, AutoGKSingular, AutoGKResults.


  -- ALGLIB --
     Copyright 06.05.2009 by Bochkanov Sergey
*************************************************************************/
void autogksmoothw(const double a, const double b, const double xwidth, autogkstate &state);


/*************************************************************************
Integration on a finite interval [A,B].
Integrand have integrable singularities at A/B.

F(X) must diverge as "(x-A)^alpha" at A, as "(B-x)^beta" at B,  with known
alpha/beta (alpha>-1, beta>-1).  If alpha/beta  are  not known,  estimates
from below can be used (but these estimates should be greater than -1 too).

One  of  alpha/beta variables (or even both alpha/beta) may be equal to 0,
which means than function F(x) is non-singular at A/B. Anyway (singular at
bounds or not), function F(x) is supposed to be continuous on (A,B).

Fast-convergent algorithm based on a Gauss-Kronrod formula is used. Result
is calculated with accuracy close to the machine precision.

INPUT PARAMETERS:
    A, B    -   interval boundaries (A<B, A=B or A>B)
    Alpha   -   power-law coefficient of the F(x) at A,
                Alpha>-1
    Beta    -   power-law coefficient of the F(x) at B,
                Beta>-1

OUTPUT PARAMETERS
    State   -   structure which stores algorithm state

SEE ALSO
    AutoGKSmooth, AutoGKSmoothW, AutoGKResults.


  -- ALGLIB --
     Copyright 06.05.2009 by Bochkanov Sergey
*************************************************************************/
void autogksingular(const double a, const double b, const double alpha, const double beta, autogkstate &state);


/*************************************************************************
This function provides reverse communication interface
Reverse communication interface is not documented or recommended to use.
See below for functions which provide better documented API
*************************************************************************/
bool autogkiteration(const autogkstate &state);


/*************************************************************************
This function is used to launcn iterations of the 1-dimensional integrator

It accepts following parameters:
    func    -   callback which calculates f(x) for given x
    ptr     -   optional pointer which is passed to func; can be NULL


  -- ALGLIB --
     Copyright 07.05.2009 by Bochkanov Sergey

*************************************************************************/
void autogkintegrate(autogkstate &state,
    void (*func)(double x, double xminusa, double bminusx, double &y, void *ptr),
    void *ptr = NULL);


/*************************************************************************
Adaptive integration results

Called after AutoGKIteration returned False.

Input parameters:
    State   -   algorithm state (used by AutoGKIteration).

Output parameters:
    V       -   integral(f(x)dx,a,b)
    Rep     -   optimization report (see AutoGKReport description)

  -- ALGLIB --
     Copyright 14.11.2007 by Bochkanov Sergey
*************************************************************************/
void autogkresults(const autogkstate &state, double &v, autogkreport &rep);
}

/////////////////////////////////////////////////////////////////////////
//
// THIS SECTION CONTAINS COMPUTATIONAL CORE DECLARATIONS (FUNCTIONS)
//
/////////////////////////////////////////////////////////////////////////
namespace alglib_impl
{
void gqgeneraterec(/* Real    */ ae_vector* alpha,
     /* Real    */ ae_vector* beta,
     double mu0,
     ae_int_t n,
     ae_int_t* info,
     /* Real    */ ae_vector* x,
     /* Real    */ ae_vector* w,
     ae_state *_state);
void gqgenerategausslobattorec(/* Real    */ ae_vector* alpha,
     /* Real    */ ae_vector* beta,
     double mu0,
     double a,
     double b,
     ae_int_t n,
     ae_int_t* info,
     /* Real    */ ae_vector* x,
     /* Real    */ ae_vector* w,
     ae_state *_state);
void gqgenerategaussradaurec(/* Real    */ ae_vector* alpha,
     /* Real    */ ae_vector* beta,
     double mu0,
     double a,
     ae_int_t n,
     ae_int_t* info,
     /* Real    */ ae_vector* x,
     /* Real    */ ae_vector* w,
     ae_state *_state);
void gqgenerategausslegendre(ae_int_t n,
     ae_int_t* info,
     /* Real    */ ae_vector* x,
     /* Real    */ ae_vector* w,
     ae_state *_state);
void gqgenerategaussjacobi(ae_int_t n,
     double alpha,
     double beta,
     ae_int_t* info,
     /* Real    */ ae_vector* x,
     /* Real    */ ae_vector* w,
     ae_state *_state);
void gqgenerategausslaguerre(ae_int_t n,
     double alpha,
     ae_int_t* info,
     /* Real    */ ae_vector* x,
     /* Real    */ ae_vector* w,
     ae_state *_state);
void gqgenerategausshermite(ae_int_t n,
     ae_int_t* info,
     /* Real    */ ae_vector* x,
     /* Real    */ ae_vector* w,
     ae_state *_state);
void gkqgeneraterec(/* Real    */ ae_vector* alpha,
     /* Real    */ ae_vector* beta,
     double mu0,
     ae_int_t n,
     ae_int_t* info,
     /* Real    */ ae_vector* x,
     /* Real    */ ae_vector* wkronrod,
     /* Real    */ ae_vector* wgauss,
     ae_state *_state);
void gkqgenerategausslegendre(ae_int_t n,
     ae_int_t* info,
     /* Real    */ ae_vector* x,
     /* Real    */ ae_vector* wkronrod,
     /* Real    */ ae_vector* wgauss,
     ae_state *_state);
void gkqgenerategaussjacobi(ae_int_t n,
     double alpha,
     double beta,
     ae_int_t* info,
     /* Real    */ ae_vector* x,
     /* Real    */ ae_vector* wkronrod,
     /* Real    */ ae_vector* wgauss,
     ae_state *_state);
void gkqlegendrecalc(ae_int_t n,
     ae_int_t* info,
     /* Real    */ ae_vector* x,
     /* Real    */ ae_vector* wkronrod,
     /* Real    */ ae_vector* wgauss,
     ae_state *_state);
void gkqlegendretbl(ae_int_t n,
     /* Real    */ ae_vector* x,
     /* Real    */ ae_vector* wkronrod,
     /* Real    */ ae_vector* wgauss,
     double* eps,
     ae_state *_state);
void autogksmooth(double a,
     double b,
     autogkstate* state,
     ae_state *_state);
void autogksmoothw(double a,
     double b,
     double xwidth,
     autogkstate* state,
     ae_state *_state);
void autogksingular(double a,
     double b,
     double alpha,
     double beta,
     autogkstate* state,
     ae_state *_state);
ae_bool autogkiteration(autogkstate* state, ae_state *_state);
void autogkresults(autogkstate* state,
     double* v,
     autogkreport* rep,
     ae_state *_state);
void _autogkreport_init(void* _p, ae_state *_state);
void _autogkreport_init_copy(void* _dst, void* _src, ae_state *_state);
void _autogkreport_clear(void* _p);
void _autogkreport_destroy(void* _p);
void _autogkinternalstate_init(void* _p, ae_state *_state);
void _autogkinternalstate_init_copy(void* _dst, void* _src, ae_state *_state);
void _autogkinternalstate_clear(void* _p);
void _autogkinternalstate_destroy(void* _p);
void _autogkstate_init(void* _p, ae_state *_state);
void _autogkstate_init_copy(void* _dst, void* _src, ae_state *_state);
void _autogkstate_clear(void* _p);
void _autogkstate_destroy(void* _p);

}
#endif

