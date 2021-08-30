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
#ifndef _fasttransforms_pkg_h
#define _fasttransforms_pkg_h
#include "ap.h"
#include "alglibinternal.h"

/////////////////////////////////////////////////////////////////////////
//
// THIS SECTION CONTAINS COMPUTATIONAL CORE DECLARATIONS (DATATYPES)
//
/////////////////////////////////////////////////////////////////////////
namespace alglib_impl
{

}

/////////////////////////////////////////////////////////////////////////
//
// THIS SECTION CONTAINS C++ INTERFACE
//
/////////////////////////////////////////////////////////////////////////
namespace alglib
{


/*************************************************************************
1-dimensional complex FFT.

Array size N may be arbitrary number (composite or prime).  Composite  N's
are handled with cache-oblivious variation of  a  Cooley-Tukey  algorithm.
Small prime-factors are transformed using hard coded  codelets (similar to
FFTW codelets, but without low-level  optimization),  large  prime-factors
are handled with Bluestein's algorithm.

Fastests transforms are for smooth N's (prime factors are 2, 3,  5  only),
most fast for powers of 2. When N have prime factors  larger  than  these,
but orders of magnitude smaller than N, computations will be about 4 times
slower than for nearby highly composite N's. When N itself is prime, speed
will be 6 times lower.

Algorithm has O(N*logN) complexity for any N (composite or prime).

INPUT PARAMETERS
    A   -   array[0..N-1] - complex function to be transformed
    N   -   problem size

OUTPUT PARAMETERS
    A   -   DFT of a input array, array[0..N-1]
            A_out[j] = SUM(A_in[k]*exp(-2*pi*sqrt(-1)*j*k/N), k = 0..N-1)


  -- ALGLIB --
     Copyright 29.05.2009 by Bochkanov Sergey
*************************************************************************/
void fftc1d(complex_1d_array &a, const ae_int_t n);
void fftc1d(complex_1d_array &a);


/*************************************************************************
1-dimensional complex inverse FFT.

Array size N may be arbitrary number (composite or prime).  Algorithm  has
O(N*logN) complexity for any N (composite or prime).

See FFTC1D() description for more information about algorithm performance.

INPUT PARAMETERS
    A   -   array[0..N-1] - complex array to be transformed
    N   -   problem size

OUTPUT PARAMETERS
    A   -   inverse DFT of a input array, array[0..N-1]
            A_out[j] = SUM(A_in[k]/N*exp(+2*pi*sqrt(-1)*j*k/N), k = 0..N-1)


  -- ALGLIB --
     Copyright 29.05.2009 by Bochkanov Sergey
*************************************************************************/
void fftc1dinv(complex_1d_array &a, const ae_int_t n);
void fftc1dinv(complex_1d_array &a);


/*************************************************************************
1-dimensional real FFT.

Algorithm has O(N*logN) complexity for any N (composite or prime).

INPUT PARAMETERS
    A   -   array[0..N-1] - real function to be transformed
    N   -   problem size

OUTPUT PARAMETERS
    F   -   DFT of a input array, array[0..N-1]
            F[j] = SUM(A[k]*exp(-2*pi*sqrt(-1)*j*k/N), k = 0..N-1)

NOTE:
    F[] satisfies symmetry property F[k] = conj(F[N-k]),  so just one half
of  array  is  usually needed. But for convinience subroutine returns full
complex array (with frequencies above N/2), so its result may be  used  by
other FFT-related subroutines.


  -- ALGLIB --
     Copyright 01.06.2009 by Bochkanov Sergey
*************************************************************************/
void fftr1d(const real_1d_array &a, const ae_int_t n, complex_1d_array &f);
void fftr1d(const real_1d_array &a, complex_1d_array &f);


/*************************************************************************
1-dimensional real inverse FFT.

Algorithm has O(N*logN) complexity for any N (composite or prime).

INPUT PARAMETERS
    F   -   array[0..floor(N/2)] - frequencies from forward real FFT
    N   -   problem size

OUTPUT PARAMETERS
    A   -   inverse DFT of a input array, array[0..N-1]

NOTE:
    F[] should satisfy symmetry property F[k] = conj(F[N-k]), so just  one
half of frequencies array is needed - elements from 0 to floor(N/2).  F[0]
is ALWAYS real. If N is even F[floor(N/2)] is real too. If N is odd,  then
F[floor(N/2)] has no special properties.

Relying on properties noted above, FFTR1DInv subroutine uses only elements
from 0th to floor(N/2)-th. It ignores imaginary part of F[0],  and in case
N is even it ignores imaginary part of F[floor(N/2)] too.

When you call this function using full arguments list - "FFTR1DInv(F,N,A)"
- you can pass either either frequencies array with N elements or  reduced
array with roughly N/2 elements - subroutine will  successfully  transform
both.

If you call this function using reduced arguments list -  "FFTR1DInv(F,A)"
- you must pass FULL array with N elements (although higher  N/2 are still
not used) because array size is used to automatically determine FFT length


  -- ALGLIB --
     Copyright 01.06.2009 by Bochkanov Sergey
*************************************************************************/
void fftr1dinv(const complex_1d_array &f, const ae_int_t n, real_1d_array &a);
void fftr1dinv(const complex_1d_array &f, real_1d_array &a);

/*************************************************************************
1-dimensional Fast Hartley Transform.

Algorithm has O(N*logN) complexity for any N (composite or prime).

INPUT PARAMETERS
    A   -   array[0..N-1] - real function to be transformed
    N   -   problem size

OUTPUT PARAMETERS
    A   -   FHT of a input array, array[0..N-1],
            A_out[k] = sum(A_in[j]*(cos(2*pi*j*k/N)+sin(2*pi*j*k/N)), j=0..N-1)


  -- ALGLIB --
     Copyright 04.06.2009 by Bochkanov Sergey
*************************************************************************/
void fhtr1d(real_1d_array &a, const ae_int_t n);


/*************************************************************************
1-dimensional inverse FHT.

Algorithm has O(N*logN) complexity for any N (composite or prime).

INPUT PARAMETERS
    A   -   array[0..N-1] - complex array to be transformed
    N   -   problem size

OUTPUT PARAMETERS
    A   -   inverse FHT of a input array, array[0..N-1]


  -- ALGLIB --
     Copyright 29.05.2009 by Bochkanov Sergey
*************************************************************************/
void fhtr1dinv(real_1d_array &a, const ae_int_t n);

/*************************************************************************
1-dimensional complex convolution.

For given A/B returns conv(A,B) (non-circular). Subroutine can automatically
choose between three implementations: straightforward O(M*N)  formula  for
very small N (or M), overlap-add algorithm for  cases  where  max(M,N)  is
significantly larger than min(M,N), but O(M*N) algorithm is too slow,  and
general FFT-based formula for cases where two previois algorithms are  too
slow.

Algorithm has max(M,N)*log(max(M,N)) complexity for any M/N.

INPUT PARAMETERS
    A   -   array[0..M-1] - complex function to be transformed
    M   -   problem size
    B   -   array[0..N-1] - complex function to be transformed
    N   -   problem size

OUTPUT PARAMETERS
    R   -   convolution: A*B. array[0..N+M-2].

NOTE:
    It is assumed that A is zero at T<0, B is zero too.  If  one  or  both
functions have non-zero values at negative T's, you  can  still  use  this
subroutine - just shift its result correspondingly.

  -- ALGLIB --
     Copyright 21.07.2009 by Bochkanov Sergey
*************************************************************************/
void convc1d(const complex_1d_array &a, const ae_int_t m, const complex_1d_array &b, const ae_int_t n, complex_1d_array &r);


/*************************************************************************
1-dimensional complex non-circular deconvolution (inverse of ConvC1D()).

Algorithm has M*log(M)) complexity for any M (composite or prime).

INPUT PARAMETERS
    A   -   array[0..M-1] - convolved signal, A = conv(R, B)
    M   -   convolved signal length
    B   -   array[0..N-1] - response
    N   -   response length, N<=M

OUTPUT PARAMETERS
    R   -   deconvolved signal. array[0..M-N].

NOTE:
    deconvolution is unstable process and may result in division  by  zero
(if your response function is degenerate, i.e. has zero Fourier coefficient).

NOTE:
    It is assumed that A is zero at T<0, B is zero too.  If  one  or  both
functions have non-zero values at negative T's, you  can  still  use  this
subroutine - just shift its result correspondingly.

  -- ALGLIB --
     Copyright 21.07.2009 by Bochkanov Sergey
*************************************************************************/
void convc1dinv(const complex_1d_array &a, const ae_int_t m, const complex_1d_array &b, const ae_int_t n, complex_1d_array &r);


/*************************************************************************
1-dimensional circular complex convolution.

For given S/R returns conv(S,R) (circular). Algorithm has linearithmic
complexity for any M/N.

IMPORTANT:  normal convolution is commutative,  i.e.   it  is symmetric  -
conv(A,B)=conv(B,A).  Cyclic convolution IS NOT.  One function - S - is  a
signal,  periodic function, and another - R - is a response,  non-periodic
function with limited length.

INPUT PARAMETERS
    S   -   array[0..M-1] - complex periodic signal
    M   -   problem size
    B   -   array[0..N-1] - complex non-periodic response
    N   -   problem size

OUTPUT PARAMETERS
    R   -   convolution: A*B. array[0..M-1].

NOTE:
    It is assumed that B is zero at T<0. If  it  has  non-zero  values  at
negative T's, you can still use this subroutine - just  shift  its  result
correspondingly.

  -- ALGLIB --
     Copyright 21.07.2009 by Bochkanov Sergey
*************************************************************************/
void convc1dcircular(const complex_1d_array &s, const ae_int_t m, const complex_1d_array &r, const ae_int_t n, complex_1d_array &c);


/*************************************************************************
1-dimensional circular complex deconvolution (inverse of ConvC1DCircular()).

Algorithm has M*log(M)) complexity for any M (composite or prime).

INPUT PARAMETERS
    A   -   array[0..M-1] - convolved periodic signal, A = conv(R, B)
    M   -   convolved signal length
    B   -   array[0..N-1] - non-periodic response
    N   -   response length

OUTPUT PARAMETERS
    R   -   deconvolved signal. array[0..M-1].

NOTE:
    deconvolution is unstable process and may result in division  by  zero
(if your response function is degenerate, i.e. has zero Fourier coefficient).

NOTE:
    It is assumed that B is zero at T<0. If  it  has  non-zero  values  at
negative T's, you can still use this subroutine - just  shift  its  result
correspondingly.

  -- ALGLIB --
     Copyright 21.07.2009 by Bochkanov Sergey
*************************************************************************/
void convc1dcircularinv(const complex_1d_array &a, const ae_int_t m, const complex_1d_array &b, const ae_int_t n, complex_1d_array &r);


/*************************************************************************
1-dimensional real convolution.

Analogous to ConvC1D(), see ConvC1D() comments for more details.

INPUT PARAMETERS
    A   -   array[0..M-1] - real function to be transformed
    M   -   problem size
    B   -   array[0..N-1] - real function to be transformed
    N   -   problem size

OUTPUT PARAMETERS
    R   -   convolution: A*B. array[0..N+M-2].

NOTE:
    It is assumed that A is zero at T<0, B is zero too.  If  one  or  both
functions have non-zero values at negative T's, you  can  still  use  this
subroutine - just shift its result correspondingly.

  -- ALGLIB --
     Copyright 21.07.2009 by Bochkanov Sergey
*************************************************************************/
void convr1d(const real_1d_array &a, const ae_int_t m, const real_1d_array &b, const ae_int_t n, real_1d_array &r);


/*************************************************************************
1-dimensional real deconvolution (inverse of ConvC1D()).

Algorithm has M*log(M)) complexity for any M (composite or prime).

INPUT PARAMETERS
    A   -   array[0..M-1] - convolved signal, A = conv(R, B)
    M   -   convolved signal length
    B   -   array[0..N-1] - response
    N   -   response length, N<=M

OUTPUT PARAMETERS
    R   -   deconvolved signal. array[0..M-N].

NOTE:
    deconvolution is unstable process and may result in division  by  zero
(if your response function is degenerate, i.e. has zero Fourier coefficient).

NOTE:
    It is assumed that A is zero at T<0, B is zero too.  If  one  or  both
functions have non-zero values at negative T's, you  can  still  use  this
subroutine - just shift its result correspondingly.

  -- ALGLIB --
     Copyright 21.07.2009 by Bochkanov Sergey
*************************************************************************/
void convr1dinv(const real_1d_array &a, const ae_int_t m, const real_1d_array &b, const ae_int_t n, real_1d_array &r);


/*************************************************************************
1-dimensional circular real convolution.

Analogous to ConvC1DCircular(), see ConvC1DCircular() comments for more details.

INPUT PARAMETERS
    S   -   array[0..M-1] - real signal
    M   -   problem size
    B   -   array[0..N-1] - real response
    N   -   problem size

OUTPUT PARAMETERS
    R   -   convolution: A*B. array[0..M-1].

NOTE:
    It is assumed that B is zero at T<0. If  it  has  non-zero  values  at
negative T's, you can still use this subroutine - just  shift  its  result
correspondingly.

  -- ALGLIB --
     Copyright 21.07.2009 by Bochkanov Sergey
*************************************************************************/
void convr1dcircular(const real_1d_array &s, const ae_int_t m, const real_1d_array &r, const ae_int_t n, real_1d_array &c);


/*************************************************************************
1-dimensional complex deconvolution (inverse of ConvC1D()).

Algorithm has M*log(M)) complexity for any M (composite or prime).

INPUT PARAMETERS
    A   -   array[0..M-1] - convolved signal, A = conv(R, B)
    M   -   convolved signal length
    B   -   array[0..N-1] - response
    N   -   response length

OUTPUT PARAMETERS
    R   -   deconvolved signal. array[0..M-N].

NOTE:
    deconvolution is unstable process and may result in division  by  zero
(if your response function is degenerate, i.e. has zero Fourier coefficient).

NOTE:
    It is assumed that B is zero at T<0. If  it  has  non-zero  values  at
negative T's, you can still use this subroutine - just  shift  its  result
correspondingly.

  -- ALGLIB --
     Copyright 21.07.2009 by Bochkanov Sergey
*************************************************************************/
void convr1dcircularinv(const real_1d_array &a, const ae_int_t m, const real_1d_array &b, const ae_int_t n, real_1d_array &r);

/*************************************************************************
1-dimensional complex cross-correlation.

For given Pattern/Signal returns corr(Pattern,Signal) (non-circular).

Correlation is calculated using reduction to  convolution.  Algorithm with
max(N,N)*log(max(N,N)) complexity is used (see  ConvC1D()  for  more  info
about performance).

IMPORTANT:
    for  historical reasons subroutine accepts its parameters in  reversed
    order: CorrC1D(Signal, Pattern) = Pattern x Signal (using  traditional
    definition of cross-correlation, denoting cross-correlation as "x").

INPUT PARAMETERS
    Signal  -   array[0..N-1] - complex function to be transformed,
                signal containing pattern
    N       -   problem size
    Pattern -   array[0..M-1] - complex function to be transformed,
                pattern to search withing signal
    M       -   problem size

OUTPUT PARAMETERS
    R       -   cross-correlation, array[0..N+M-2]:
                * positive lags are stored in R[0..N-1],
                  R[i] = sum(conj(pattern[j])*signal[i+j]
                * negative lags are stored in R[N..N+M-2],
                  R[N+M-1-i] = sum(conj(pattern[j])*signal[-i+j]

NOTE:
    It is assumed that pattern domain is [0..M-1].  If Pattern is non-zero
on [-K..M-1],  you can still use this subroutine, just shift result by K.

  -- ALGLIB --
     Copyright 21.07.2009 by Bochkanov Sergey
*************************************************************************/
void corrc1d(const complex_1d_array &signal, const ae_int_t n, const complex_1d_array &pattern, const ae_int_t m, complex_1d_array &r);


/*************************************************************************
1-dimensional circular complex cross-correlation.

For given Pattern/Signal returns corr(Pattern,Signal) (circular).
Algorithm has linearithmic complexity for any M/N.

IMPORTANT:
    for  historical reasons subroutine accepts its parameters in  reversed
    order:   CorrC1DCircular(Signal, Pattern) = Pattern x Signal    (using
    traditional definition of cross-correlation, denoting cross-correlation
    as "x").

INPUT PARAMETERS
    Signal  -   array[0..N-1] - complex function to be transformed,
                periodic signal containing pattern
    N       -   problem size
    Pattern -   array[0..M-1] - complex function to be transformed,
                non-periodic pattern to search withing signal
    M       -   problem size

OUTPUT PARAMETERS
    R   -   convolution: A*B. array[0..M-1].


  -- ALGLIB --
     Copyright 21.07.2009 by Bochkanov Sergey
*************************************************************************/
void corrc1dcircular(const complex_1d_array &signal, const ae_int_t m, const complex_1d_array &pattern, const ae_int_t n, complex_1d_array &c);


/*************************************************************************
1-dimensional real cross-correlation.

For given Pattern/Signal returns corr(Pattern,Signal) (non-circular).

Correlation is calculated using reduction to  convolution.  Algorithm with
max(N,N)*log(max(N,N)) complexity is used (see  ConvC1D()  for  more  info
about performance).

IMPORTANT:
    for  historical reasons subroutine accepts its parameters in  reversed
    order: CorrR1D(Signal, Pattern) = Pattern x Signal (using  traditional
    definition of cross-correlation, denoting cross-correlation as "x").

INPUT PARAMETERS
    Signal  -   array[0..N-1] - real function to be transformed,
                signal containing pattern
    N       -   problem size
    Pattern -   array[0..M-1] - real function to be transformed,
                pattern to search withing signal
    M       -   problem size

OUTPUT PARAMETERS
    R       -   cross-correlation, array[0..N+M-2]:
                * positive lags are stored in R[0..N-1],
                  R[i] = sum(pattern[j]*signal[i+j]
                * negative lags are stored in R[N..N+M-2],
                  R[N+M-1-i] = sum(pattern[j]*signal[-i+j]

NOTE:
    It is assumed that pattern domain is [0..M-1].  If Pattern is non-zero
on [-K..M-1],  you can still use this subroutine, just shift result by K.

  -- ALGLIB --
     Copyright 21.07.2009 by Bochkanov Sergey
*************************************************************************/
void corrr1d(const real_1d_array &signal, const ae_int_t n, const real_1d_array &pattern, const ae_int_t m, real_1d_array &r);


/*************************************************************************
1-dimensional circular real cross-correlation.

For given Pattern/Signal returns corr(Pattern,Signal) (circular).
Algorithm has linearithmic complexity for any M/N.

IMPORTANT:
    for  historical reasons subroutine accepts its parameters in  reversed
    order:   CorrR1DCircular(Signal, Pattern) = Pattern x Signal    (using
    traditional definition of cross-correlation, denoting cross-correlation
    as "x").

INPUT PARAMETERS
    Signal  -   array[0..N-1] - real function to be transformed,
                periodic signal containing pattern
    N       -   problem size
    Pattern -   array[0..M-1] - real function to be transformed,
                non-periodic pattern to search withing signal
    M       -   problem size

OUTPUT PARAMETERS
    R   -   convolution: A*B. array[0..M-1].


  -- ALGLIB --
     Copyright 21.07.2009 by Bochkanov Sergey
*************************************************************************/
void corrr1dcircular(const real_1d_array &signal, const ae_int_t m, const real_1d_array &pattern, const ae_int_t n, real_1d_array &c);
}

/////////////////////////////////////////////////////////////////////////
//
// THIS SECTION CONTAINS COMPUTATIONAL CORE DECLARATIONS (FUNCTIONS)
//
/////////////////////////////////////////////////////////////////////////
namespace alglib_impl
{
void fftc1d(/* Complex */ ae_vector* a, ae_int_t n, ae_state *_state);
void fftc1dinv(/* Complex */ ae_vector* a, ae_int_t n, ae_state *_state);
void fftr1d(/* Real    */ ae_vector* a,
     ae_int_t n,
     /* Complex */ ae_vector* f,
     ae_state *_state);
void fftr1dinv(/* Complex */ ae_vector* f,
     ae_int_t n,
     /* Real    */ ae_vector* a,
     ae_state *_state);
void fftr1dinternaleven(/* Real    */ ae_vector* a,
     ae_int_t n,
     /* Real    */ ae_vector* buf,
     fasttransformplan* plan,
     ae_state *_state);
void fftr1dinvinternaleven(/* Real    */ ae_vector* a,
     ae_int_t n,
     /* Real    */ ae_vector* buf,
     fasttransformplan* plan,
     ae_state *_state);
void fhtr1d(/* Real    */ ae_vector* a, ae_int_t n, ae_state *_state);
void fhtr1dinv(/* Real    */ ae_vector* a, ae_int_t n, ae_state *_state);
void convc1d(/* Complex */ ae_vector* a,
     ae_int_t m,
     /* Complex */ ae_vector* b,
     ae_int_t n,
     /* Complex */ ae_vector* r,
     ae_state *_state);
void convc1dinv(/* Complex */ ae_vector* a,
     ae_int_t m,
     /* Complex */ ae_vector* b,
     ae_int_t n,
     /* Complex */ ae_vector* r,
     ae_state *_state);
void convc1dcircular(/* Complex */ ae_vector* s,
     ae_int_t m,
     /* Complex */ ae_vector* r,
     ae_int_t n,
     /* Complex */ ae_vector* c,
     ae_state *_state);
void convc1dcircularinv(/* Complex */ ae_vector* a,
     ae_int_t m,
     /* Complex */ ae_vector* b,
     ae_int_t n,
     /* Complex */ ae_vector* r,
     ae_state *_state);
void convr1d(/* Real    */ ae_vector* a,
     ae_int_t m,
     /* Real    */ ae_vector* b,
     ae_int_t n,
     /* Real    */ ae_vector* r,
     ae_state *_state);
void convr1dinv(/* Real    */ ae_vector* a,
     ae_int_t m,
     /* Real    */ ae_vector* b,
     ae_int_t n,
     /* Real    */ ae_vector* r,
     ae_state *_state);
void convr1dcircular(/* Real    */ ae_vector* s,
     ae_int_t m,
     /* Real    */ ae_vector* r,
     ae_int_t n,
     /* Real    */ ae_vector* c,
     ae_state *_state);
void convr1dcircularinv(/* Real    */ ae_vector* a,
     ae_int_t m,
     /* Real    */ ae_vector* b,
     ae_int_t n,
     /* Real    */ ae_vector* r,
     ae_state *_state);
void convc1dx(/* Complex */ ae_vector* a,
     ae_int_t m,
     /* Complex */ ae_vector* b,
     ae_int_t n,
     ae_bool circular,
     ae_int_t alg,
     ae_int_t q,
     /* Complex */ ae_vector* r,
     ae_state *_state);
void convr1dx(/* Real    */ ae_vector* a,
     ae_int_t m,
     /* Real    */ ae_vector* b,
     ae_int_t n,
     ae_bool circular,
     ae_int_t alg,
     ae_int_t q,
     /* Real    */ ae_vector* r,
     ae_state *_state);
void corrc1d(/* Complex */ ae_vector* signal,
     ae_int_t n,
     /* Complex */ ae_vector* pattern,
     ae_int_t m,
     /* Complex */ ae_vector* r,
     ae_state *_state);
void corrc1dcircular(/* Complex */ ae_vector* signal,
     ae_int_t m,
     /* Complex */ ae_vector* pattern,
     ae_int_t n,
     /* Complex */ ae_vector* c,
     ae_state *_state);
void corrr1d(/* Real    */ ae_vector* signal,
     ae_int_t n,
     /* Real    */ ae_vector* pattern,
     ae_int_t m,
     /* Real    */ ae_vector* r,
     ae_state *_state);
void corrr1dcircular(/* Real    */ ae_vector* signal,
     ae_int_t m,
     /* Real    */ ae_vector* pattern,
     ae_int_t n,
     /* Real    */ ae_vector* c,
     ae_state *_state);

}
#endif

