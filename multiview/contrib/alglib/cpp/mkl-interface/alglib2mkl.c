#include <mkl.h>

#define ae_bool  char
#define ae_true  1
#define ae_false 0
#define ae_int_t ptrdiff_t

/*
 * This function is used to perform automatic inplace conversion
 * from ae_int_to to lapack_int:
 * * under LP64 function performs inplace conversion from 64-bit ALGLIB int to 32-bit LAPACK int
 * * under 32-bit environments or ILP64 nothing is done (no conversion is required)
 */
void alglibint_to_lapackint(void *ptr, ptrdiff_t n, char abort_on_failure)
{
    /* volatile is used to trick compiler, to prevent warnings about 'expression is always true' */
    volatile int sz_alglib = 0;
    volatile int sz_lapack = 0;
    
    sz_alglib = sizeof(ptrdiff_t);
    sz_lapack = sizeof(lapack_int);
    
    /* no conversion is required */
    if( sz_alglib==sz_lapack )
        return;
    
    /* ALGLIB int is wider */
    if( sz_alglib==8 && sz_lapack==4 )
    {
        ptrdiff_t i;
        volatile ptrdiff_t  *p_alglib = (ptrdiff_t*)ptr;
        volatile lapack_int *p_lapack = (lapack_int*)ptr;
        
        for(i=0; i<n; i++)
        {
            volatile lapack_int v;
            v = p_alglib[i];
            if( abort_on_failure && v!=p_alglib[i] )
                abort();
            p_lapack[i] = v;
        }
        
        return;
    }
    
    /* unexpected code branch */
    abort();
}


/*
 * This function is used to perform automatic inplace conversion
 * from lapack_int to ae_int_to:
 * * under LP64 function performs inplace conversion from 32-bit LAPACK int to 64-bit ALGLIB int
 * * under 32-bit environments or ILP64 nothing is done (no conversion is required)
 */
void lapackint_to_alglibint(void *ptr, ptrdiff_t n)
{
    /* volatile is used to trick compiler, to prevent warnings about 'expression is always true' */
    volatile int sz_alglib = 0;
    volatile int sz_lapack = 0;
    
    sz_alglib = sizeof(ptrdiff_t);
    sz_lapack = sizeof(lapack_int);
    
    /* no conversion is required */
    if( sz_alglib==sz_lapack )
        return;
    
    /* ALGLIB int is wider */
    if( sz_alglib==8 && sz_lapack==4 )
    {
        ptrdiff_t i;
        volatile ptrdiff_t  *p_alglib = (ptrdiff_t*)ptr;
        volatile lapack_int *p_lapack = (lapack_int*)ptr;
        
        for(i=n-1; i>=0; i--)
        {
            volatile ptrdiff_t v;
            v = p_lapack[i];
            p_alglib[i] = v;
        }
        
        return;
    }
    
    /* unexpected code branch */
    abort();
}

ae_bool _alglib2mkl_rmatrixsyrkmkl(
     ae_int_t n,
     ae_int_t k,
     double alpha,
     double *a_ptr,
     ae_int_t a_stride,
     ae_int_t optypea,
     double beta,
     double *c_ptr,
     ae_int_t c_stride,
     ae_bool isupper)
{
    int prev_nthreads;
    CBLAS_UPLO uplo = isupper ? CblasUpper : CblasLower;
    CBLAS_TRANSPOSE trans = optypea==0 ? CblasNoTrans : CblasTrans;
    prev_nthreads = mkl_set_num_threads_local(1);
    cblas_dsyrk(
        CblasRowMajor, uplo, trans, n, k,
        alpha, a_ptr, a_stride,
        beta,  c_ptr, c_stride);
    mkl_set_num_threads_local(prev_nthreads);
    return ae_true;
}

ae_bool _alglib2mkl_cmatrixherkmkl(
     ae_int_t n,
     ae_int_t k,
     double alpha,
     void *a_ptr,
     ae_int_t a_stride,
     ae_int_t optypea,
     double beta,
     void *c_ptr,
     ae_int_t c_stride,
     ae_bool isupper)
{
    int prev_nthreads;
    CBLAS_UPLO uplo = isupper ? CblasUpper : CblasLower;
    CBLAS_TRANSPOSE trans = optypea==0 ? CblasNoTrans : CblasConjTrans;
    prev_nthreads = mkl_set_num_threads_local(1);
    cblas_zherk(
        CblasRowMajor, uplo, trans, n, k,
        alpha, a_ptr, a_stride,
        beta,  c_ptr, c_stride);
    mkl_set_num_threads_local(prev_nthreads);
    return ae_true;
}

ae_bool _alglib2mkl_rmatrixgemmmkl(
     ae_int_t m,
     ae_int_t n,
     ae_int_t k,
     double alpha,
     double *a_ptr,
     ae_int_t a_stride,
     ae_int_t optypea,
     double *b_ptr,
     ae_int_t b_stride,
     ae_int_t optypeb,
     double beta,
     double *c_ptr,
     ae_int_t c_stride)
{
    CBLAS_TRANSPOSE trans_a = optypea==0 ? CblasNoTrans : CblasTrans;
    CBLAS_TRANSPOSE trans_b = optypeb==0 ? CblasNoTrans : CblasTrans;
    int prev_nthreads;
    prev_nthreads = mkl_set_num_threads_local(1);
    cblas_dgemm(
        CblasRowMajor, trans_a, trans_b,
        m, n, k,
        alpha,
        a_ptr, a_stride,
        b_ptr, b_stride,
        beta,
        c_ptr, c_stride);
    mkl_set_num_threads_local(prev_nthreads);
    return ae_true;
}

ae_bool _alglib2mkl_cmatrixgemmmkl(
     ae_int_t m,
     ae_int_t n,
     ae_int_t k,
     void* alpha,
     void* a_ptr,
     ae_int_t a_stride,
     ae_int_t optypea,
     void* b_ptr,
     ae_int_t b_stride,
     ae_int_t optypeb,
     void* beta,
     void* c_ptr,
     ae_int_t c_stride)
{
    CBLAS_TRANSPOSE trans_a = optypea==0 ? CblasNoTrans : (optypea==1 ? CblasTrans : CblasConjTrans);
    CBLAS_TRANSPOSE trans_b = optypeb==0 ? CblasNoTrans : (optypeb==1 ? CblasTrans : CblasConjTrans);
    int prev_nthreads;
    prev_nthreads = mkl_set_num_threads_local(1);
    cblas_zgemm(
        CblasRowMajor, trans_a, trans_b,
        m, n, k,
        alpha,
        a_ptr, a_stride,
        b_ptr, b_stride,
        beta,
        c_ptr, c_stride);
    mkl_set_num_threads_local(prev_nthreads);
    return ae_true;
}

ae_bool _alglib2mkl_rmatrixtrsmmkl(
     char is_rightside,
     ptrdiff_t m,
     ptrdiff_t n,
     double* a,
     ptrdiff_t a_stride,
     char isupper,
     char isunit,
     ptrdiff_t optype,
     double* x,
     ptrdiff_t x_stride)
{
    CBLAS_SIDE cb_side = is_rightside ? CblasRight : CblasLeft;
    CBLAS_UPLO cb_uplo = isupper ? CblasUpper : CblasLower;
    CBLAS_DIAG cb_diag = isunit ? CblasUnit : CblasNonUnit;
    CBLAS_TRANSPOSE cb_trans_a = optype==0 ? CblasNoTrans : CblasTrans;
    int prev_nthreads;
    prev_nthreads = mkl_set_num_threads_local(1);
    cblas_dtrsm(
        CblasRowMajor,
        cb_side, cb_uplo, cb_trans_a, cb_diag,
        m, n,
        1.0,
        a, a_stride,
        x, x_stride);
    mkl_set_num_threads_local(prev_nthreads);
    return ae_true;
}

ae_bool _alglib2mkl_cmatrixtrsmmkl(
     char is_rightside,
     ptrdiff_t m,
     ptrdiff_t n,
     void* a,
     ptrdiff_t a_stride,
     char isupper,
     char isunit,
     ptrdiff_t optype,
     void* x,
     ptrdiff_t x_stride)
{
    CBLAS_SIDE cb_side = is_rightside ? CblasRight : CblasLeft;
    CBLAS_UPLO cb_uplo = isupper ? CblasUpper : CblasLower;
    CBLAS_DIAG cb_diag = isunit ? CblasUnit : CblasNonUnit;
    CBLAS_TRANSPOSE cb_trans_a = optype==0 ? CblasNoTrans : (optype==1 ? CblasTrans : CblasConjTrans);
    int prev_nthreads;
    double alpha[2] = { 1, 0 };
    prev_nthreads = mkl_set_num_threads_local(1);
    cblas_ztrsm(
        CblasRowMajor,
        cb_side, cb_uplo, cb_trans_a, cb_diag,
        m, n,
        (void*)alpha,
        a, a_stride,
        x, x_stride);
    mkl_set_num_threads_local(prev_nthreads);
    return ae_true;
}

ae_bool _alglib2mkl_spdmatrixcholeskymkl(
     double* a,
     ptrdiff_t a_stride,
     ptrdiff_t n,
     ae_bool  isupper,
     ae_bool* cholresult)
{
    char uplo = isupper ? 'U' : 'L';
    int prev_nthreads;
    prev_nthreads = mkl_set_num_threads_local(1);
    *cholresult = LAPACKE_dpotrf(LAPACK_ROW_MAJOR, uplo, n, a, a_stride)==0;
    mkl_set_num_threads_local(prev_nthreads);
    return ae_true;
}

ae_bool _alglib2mkl_rmatrixplumkl(
     double* a,
     ptrdiff_t a_stride,
     ptrdiff_t m,
     ptrdiff_t n,
     ptrdiff_t offs,
     ptrdiff_t *pivots)
{
    int minmn, i;
    int prev_nthreads;
    
    /* call LAPACK code */
    prev_nthreads = mkl_set_num_threads_local(1);
    LAPACKE_dgetrf(LAPACK_ROW_MAJOR, m, n, a, a_stride, (lapack_int*)pivots);
    mkl_set_num_threads_local(prev_nthreads);
    
    /* convert from LAPACK numeration back to ALGLIB */
    minmn = m<n ? m : n;
    lapackint_to_alglibint(pivots, minmn);
    for(i=0; i<minmn; i++)
        pivots[i] = (pivots[i]-1)+offs;
    return ae_true;
}

char _alglib2mkl_rmatrixbdmkl(
     double    *a,
     ptrdiff_t a_stride,
     ptrdiff_t m,
     ptrdiff_t n,
     double    *d,
     double    *e,
     double    *tauq,
     double    *taup)
{
    int prev_nthreads;
    
    prev_nthreads = mkl_set_num_threads_local(1);
    LAPACKE_dgebrd(LAPACK_ROW_MAJOR, m, n, a, a_stride, d, e, tauq, taup);
    mkl_set_num_threads_local(prev_nthreads);
    
    return ae_true;
}

char _alglib2mkl_rmatrixbdmultiplybymkl(
     double *qp,
     ptrdiff_t qp_stride,
     ptrdiff_t m,
     ptrdiff_t n,
     double *tauq,
     double *taup,
     double *z,
     ptrdiff_t z_stride,
     ptrdiff_t zrows,
     ptrdiff_t zcolumns,
     char byq,
     char fromtheright,
     char dotranspose)
{
    int prev_nthreads;
    
    prev_nthreads = mkl_set_num_threads_local(1);
    LAPACKE_dormbr(
        LAPACK_ROW_MAJOR,
        byq ? 'Q' : 'P',
        fromtheright ? 'R' : 'L',
        dotranspose  ? 'T' : 'N',
        zrows, zcolumns,
        byq ? n : m,
        qp, qp_stride,
        byq ? tauq : taup,
        z, z_stride);
    mkl_set_num_threads_local(prev_nthreads);
    
    return ae_true;
}

char _alglib2mkl_rmatrixhessenbergmkl(
     double *a,
     ptrdiff_t a_stride,
     ptrdiff_t n,
     double* tau)
{
    int prev_nthreads;
    
    prev_nthreads = mkl_set_num_threads_local(1);
    LAPACKE_dgehrd(LAPACK_ROW_MAJOR, n, 1, n, a, a_stride, tau);
    mkl_set_num_threads_local(prev_nthreads);
    
    return ae_true;
}

char _alglib2mkl_rmatrixhessenbergunpackqmkl(
     double    *a,
     ptrdiff_t a_stride,
     ptrdiff_t n,
     double   *tau,
     double    *q,
     ptrdiff_t q_stride)
{
    int prev_nthreads, i, j;
    
    for(i=0; i<n; i++)
    {
        double *p_arow = a+a_stride*i;
        double *p_qrow = q+q_stride*i;
        for(j=0; j<n; j++)
            p_qrow[j] = p_arow[j];
    }
    prev_nthreads = mkl_set_num_threads_local(1);
    LAPACKE_dorghr(LAPACK_ROW_MAJOR, n, 1, n, q, q_stride, tau);
    mkl_set_num_threads_local(prev_nthreads);
    
    return ae_true;
}

char _alglib2mkl_smatrixtdmkl(
     double *a,
     ptrdiff_t a_stride,
     ptrdiff_t n,
     char isupper,
     double* tau,
     double* d,
     double* e)
{
    int prev_nthreads;
    
    prev_nthreads = mkl_set_num_threads_local(1);
    LAPACKE_dsytrd(
        LAPACK_ROW_MAJOR,
        isupper ? 'U' : 'L',
        n,
        a, a_stride,
        d, e, tau);
    mkl_set_num_threads_local(prev_nthreads);
    
    return ae_true;
}

char _alglib2mkl_smatrixtdunpackqmkl(
     double* a,
     ptrdiff_t a_stride,
     ptrdiff_t n,
     char isupper,
     double* tau,
     double* q,
     ptrdiff_t q_stride)
{
    int prev_nthreads, i, j;
    
    for(i=0; i<n; i++)
    {
        double *p_arow = a+a_stride*i;
        double *p_qrow = q+q_stride*i;
        for(j=0; j<n; j++)
            p_qrow[j] = p_arow[j];
    }
    prev_nthreads = mkl_set_num_threads_local(1);
    LAPACKE_dorgtr(
        LAPACK_ROW_MAJOR,
        isupper ? 'U' : 'L',
        n,
        q, q_stride,
        tau);
    mkl_set_num_threads_local(prev_nthreads);
    
    return ae_true;
}

char _alglib2mkl_hmatrixtdmkl(
     void *a,
     ptrdiff_t a_stride,
     ptrdiff_t n,
     char isupper,
     void* tau,
     double* d,
     double* e)
{
    int prev_nthreads;
    
    prev_nthreads = mkl_set_num_threads_local(1);
    LAPACKE_zhetrd(
        LAPACK_ROW_MAJOR,
        isupper ? 'U' : 'L',
        n,
        (lapack_complex_double*)a, a_stride,
        d, e, (lapack_complex_double*)tau);
    mkl_set_num_threads_local(prev_nthreads);
    
    return ae_true;
}

char _alglib2mkl_hmatrixtdunpackqmkl(
     void* a,
     ptrdiff_t a_stride,
     ptrdiff_t n,
     char isupper,
     void* tau,
     void* q,
     ptrdiff_t q_stride)
{
    int prev_nthreads, i, j;
    
    for(i=0; i<n; i++)
    {
        double *p_arow = (double*)a+2*a_stride*i;
        double *p_qrow = (double*)q+2*q_stride*i;
        for(j=0; j<2*n; j++)
            p_qrow[j] = p_arow[j];
    }
    prev_nthreads = mkl_set_num_threads_local(1);
    LAPACKE_zungtr(
        LAPACK_ROW_MAJOR,
        isupper ? 'U' : 'L',
        n,
        (lapack_complex_double*)q, q_stride,
        (lapack_complex_double*)tau);
    mkl_set_num_threads_local(prev_nthreads);
    
    return ae_true;
}

char _alglib2mkl_rmatrixbdsvdmkl(
     double* d,
     double* e,
     ptrdiff_t n,
     char isupper,
     double* u,
     ptrdiff_t u_stride,
     ptrdiff_t nru,
     double* c,
     ptrdiff_t c_stride,
     ptrdiff_t ncc,
     double* vt,
     ptrdiff_t vt_stride,
     ptrdiff_t ncvt,
     char* svdresult)
{
    int prev_nthreads, result;
    
    prev_nthreads = mkl_set_num_threads_local(1);
    result = LAPACKE_dbdsqr(
        LAPACK_ROW_MAJOR,
        isupper ? 'U' : 'L',
        n, ncvt, nru, ncc,
        d, e, vt, vt_stride, u, u_stride, c, c_stride);
    *svdresult = result==0;
    mkl_set_num_threads_local(prev_nthreads);
    
    return ae_true;
}

char _alglib2mkl_rmatrixinternalschurdecompositionmkl(
     double* h,
     ptrdiff_t h_stride,
     ptrdiff_t n,
     ptrdiff_t tneeded,
     ptrdiff_t zneeded,
     double* wr,
     double* wi,
     double* z,
     ptrdiff_t z_stride,
     ptrdiff_t* info)
{
    int prev_nthreads, result;
    char compz;
    
    compz = 'N';
    if( zneeded==1 )
        compz = 'V';
    if( zneeded==2 )
        compz = 'I';
    prev_nthreads = mkl_set_num_threads_local(1);
    result = LAPACKE_dhseqr(
        LAPACK_ROW_MAJOR,
        tneeded ? 'S' : 'E',
        compz,
        n, 1, n,
        h, h_stride,
        wr, wi,
        z, z_stride);
    *info = result;
    mkl_set_num_threads_local(prev_nthreads);
    
    return ae_true;
}

char _alglib2mkl_rmatrixinternaltrevcmkl(
     double* t,
     ptrdiff_t t_stride,
     ptrdiff_t n,
     ptrdiff_t side,
     ptrdiff_t howmny,
     double* vl,
     ptrdiff_t vl_stride,
     double* vr,
     ptrdiff_t vr_stride,
     ptrdiff_t* m,
     ptrdiff_t* info)
{
    int prev_nthreads;
    lapack_int loc_m;
    char c_side, c_howmny;

    if( howmny==3 ) /* not supported! */
        return ae_false;
    c_side = 'B';
    if( side==1 )
        c_side = 'R';
    if( side==2 )
        c_side = 'L';
    c_howmny = howmny==1 ? 'B' : 'A';
    prev_nthreads = mkl_set_num_threads_local(1);
    *info = LAPACKE_dtrevc(
        LAPACK_ROW_MAJOR,
        c_side,
        c_howmny,
        NULL,
        n, 
        t, t_stride,
        vl, vl_stride,
        vr, vr_stride,
        n,
        &loc_m);
    *m = loc_m;
    mkl_set_num_threads_local(prev_nthreads);
    
    return ae_true;
}

char _alglib2mkl_smatrixtdevdmkl(
     double* d,
     double* e,
     ptrdiff_t n,
     ptrdiff_t zneeded,
     double* z,
     ptrdiff_t z_stride,
     char* evdresult)
{
    int r, prev_nthreads;
    char compz;
    
    /* only zneeded=0,1,2 are supported */
    if( zneeded==3 )
        return ae_false;
        
    /* general case */
    prev_nthreads = mkl_set_num_threads_local(1);
    compz = 'N';
    if( zneeded==1 )
        compz = 'V';
    if( zneeded==2 )
        compz = 'I';
    r = LAPACKE_dsteqr(
        LAPACK_ROW_MAJOR,
        compz,
        n, d, e,
        z, z_stride);
    *evdresult = r==0;
    mkl_set_num_threads_local(prev_nthreads);
    
    return ae_true;
}