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
#ifndef _diffequations_pkg_h
#define _diffequations_pkg_h
#include "ap.h"
#include "alglibinternal.h"

/////////////////////////////////////////////////////////////////////////
//
// THIS SECTION CONTAINS COMPUTATIONAL CORE DECLARATIONS (DATATYPES)
//
/////////////////////////////////////////////////////////////////////////
namespace alglib_impl
{
typedef struct
{
    ae_int_t n;
    ae_int_t m;
    double xscale;
    double h;
    double eps;
    ae_bool fraceps;
    ae_vector yc;
    ae_vector escale;
    ae_vector xg;
    ae_int_t solvertype;
    ae_bool needdy;
    double x;
    ae_vector y;
    ae_vector dy;
    ae_matrix ytbl;
    ae_int_t repterminationtype;
    ae_int_t repnfev;
    ae_vector yn;
    ae_vector yns;
    ae_vector rka;
    ae_vector rkc;
    ae_vector rkcs;
    ae_matrix rkb;
    ae_matrix rkk;
    rcommstate rstate;
} odesolverstate;
typedef struct
{
    ae_int_t nfev;
    ae_int_t terminationtype;
} odesolverreport;

}

/////////////////////////////////////////////////////////////////////////
//
// THIS SECTION CONTAINS C++ INTERFACE
//
/////////////////////////////////////////////////////////////////////////
namespace alglib
{

/*************************************************************************

*************************************************************************/
class _odesolverstate_owner
{
public:
    _odesolverstate_owner();
    _odesolverstate_owner(const _odesolverstate_owner &rhs);
    _odesolverstate_owner& operator=(const _odesolverstate_owner &rhs);
    virtual ~_odesolverstate_owner();
    alglib_impl::odesolverstate* c_ptr();
    alglib_impl::odesolverstate* c_ptr() const;
protected:
    alglib_impl::odesolverstate *p_struct;
};
class odesolverstate : public _odesolverstate_owner
{
public:
    odesolverstate();
    odesolverstate(const odesolverstate &rhs);
    odesolverstate& operator=(const odesolverstate &rhs);
    virtual ~odesolverstate();
    ae_bool &needdy;
    real_1d_array y;
    real_1d_array dy;
    double &x;

};


/*************************************************************************

*************************************************************************/
class _odesolverreport_owner
{
public:
    _odesolverreport_owner();
    _odesolverreport_owner(const _odesolverreport_owner &rhs);
    _odesolverreport_owner& operator=(const _odesolverreport_owner &rhs);
    virtual ~_odesolverreport_owner();
    alglib_impl::odesolverreport* c_ptr();
    alglib_impl::odesolverreport* c_ptr() const;
protected:
    alglib_impl::odesolverreport *p_struct;
};
class odesolverreport : public _odesolverreport_owner
{
public:
    odesolverreport();
    odesolverreport(const odesolverreport &rhs);
    odesolverreport& operator=(const odesolverreport &rhs);
    virtual ~odesolverreport();
    ae_int_t &nfev;
    ae_int_t &terminationtype;

};

/*************************************************************************
Cash-Karp adaptive ODE solver.

This subroutine solves ODE  Y'=f(Y,x)  with  initial  conditions  Y(xs)=Ys
(here Y may be single variable or vector of N variables).

INPUT PARAMETERS:
    Y       -   initial conditions, array[0..N-1].
                contains values of Y[] at X[0]
    N       -   system size
    X       -   points at which Y should be tabulated, array[0..M-1]
                integrations starts at X[0], ends at X[M-1],  intermediate
                values at X[i] are returned too.
                SHOULD BE ORDERED BY ASCENDING OR BY DESCENDING!
    M       -   number of intermediate points + first point + last point:
                * M>2 means that you need both Y(X[M-1]) and M-2 values at
                  intermediate points
                * M=2 means that you want just to integrate from  X[0]  to
                  X[1] and don't interested in intermediate values.
                * M=1 means that you don't want to integrate :)
                  it is degenerate case, but it will be handled correctly.
                * M<1 means error
    Eps     -   tolerance (absolute/relative error on each  step  will  be
                less than Eps). When passing:
                * Eps>0, it means desired ABSOLUTE error
                * Eps<0, it means desired RELATIVE error.  Relative errors
                  are calculated with respect to maximum values of  Y seen
                  so far. Be careful to use this criterion  when  starting
                  from Y[] that are close to zero.
    H       -   initial  step  lenth,  it  will  be adjusted automatically
                after the first  step.  If  H=0,  step  will  be  selected
                automatically  (usualy  it  will  be  equal  to  0.001  of
                min(x[i]-x[j])).

OUTPUT PARAMETERS
    State   -   structure which stores algorithm state between  subsequent
                calls of OdeSolverIteration. Used for reverse communication.
                This structure should be passed  to the OdeSolverIteration
                subroutine.

SEE ALSO
    AutoGKSmoothW, AutoGKSingular, AutoGKIteration, AutoGKResults.


  -- ALGLIB --
     Copyright 01.09.2009 by Bochkanov Sergey
*************************************************************************/
void odesolverrkck(const real_1d_array &y, const ae_int_t n, const real_1d_array &x, const ae_int_t m, const double eps, const double h, odesolverstate &state);
void odesolverrkck(const real_1d_array &y, const real_1d_array &x, const double eps, const double h, odesolverstate &state);


/*************************************************************************
This function provides reverse communication interface
Reverse communication interface is not documented or recommended to use.
See below for functions which provide better documented API
*************************************************************************/
bool odesolveriteration(const odesolverstate &state);


/*************************************************************************
This function is used to launcn iterations of ODE solver

It accepts following parameters:
    diff    -   callback which calculates dy/dx for given y and x
    ptr     -   optional pointer which is passed to diff; can be NULL


  -- ALGLIB --
     Copyright 01.09.2009 by Bochkanov Sergey

*************************************************************************/
void odesolversolve(odesolverstate &state,
    void (*diff)(const real_1d_array &y, double x, real_1d_array &dy, void *ptr),
    void *ptr = NULL);


/*************************************************************************
ODE solver results

Called after OdeSolverIteration returned False.

INPUT PARAMETERS:
    State   -   algorithm state (used by OdeSolverIteration).

OUTPUT PARAMETERS:
    M       -   number of tabulated values, M>=1
    XTbl    -   array[0..M-1], values of X
    YTbl    -   array[0..M-1,0..N-1], values of Y in X[i]
    Rep     -   solver report:
                * Rep.TerminationType completetion code:
                    * -2    X is not ordered  by  ascending/descending  or
                            there are non-distinct X[],  i.e.  X[i]=X[i+1]
                    * -1    incorrect parameters were specified
                    *  1    task has been solved
                * Rep.NFEV contains number of function calculations

  -- ALGLIB --
     Copyright 01.09.2009 by Bochkanov Sergey
*************************************************************************/
void odesolverresults(const odesolverstate &state, ae_int_t &m, real_1d_array &xtbl, real_2d_array &ytbl, odesolverreport &rep);
}

/////////////////////////////////////////////////////////////////////////
//
// THIS SECTION CONTAINS COMPUTATIONAL CORE DECLARATIONS (FUNCTIONS)
//
/////////////////////////////////////////////////////////////////////////
namespace alglib_impl
{
void odesolverrkck(/* Real    */ ae_vector* y,
     ae_int_t n,
     /* Real    */ ae_vector* x,
     ae_int_t m,
     double eps,
     double h,
     odesolverstate* state,
     ae_state *_state);
ae_bool odesolveriteration(odesolverstate* state, ae_state *_state);
void odesolverresults(odesolverstate* state,
     ae_int_t* m,
     /* Real    */ ae_vector* xtbl,
     /* Real    */ ae_matrix* ytbl,
     odesolverreport* rep,
     ae_state *_state);
void _odesolverstate_init(void* _p, ae_state *_state);
void _odesolverstate_init_copy(void* _dst, void* _src, ae_state *_state);
void _odesolverstate_clear(void* _p);
void _odesolverstate_destroy(void* _p);
void _odesolverreport_init(void* _p, ae_state *_state);
void _odesolverreport_init_copy(void* _dst, void* _src, ae_state *_state);
void _odesolverreport_clear(void* _p);
void _odesolverreport_destroy(void* _p);

}
#endif

