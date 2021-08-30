
#pragma once

namespace perceive
{
/**
 * fn,             function to be minimized
 * n,              number of variables
 * start[n],       start-point.
 * xmin[n],        (output) the output estimate that minimizes 'fn'
 * ynewlo,         (output) same as fn(&xmin[0])
 * reqmin,
 * diffsetp,
 * kcount,         maximum number of times to evaluate 'fn'
 * icount,         (output) number of function iterations used
 * ifault,         (output) number of function iterations used
 *        -9    exception
 *        -8    optimizer detected NAN/INF values either in the function itself,
 *              or in its Jacobian
 *        -7    derivative correctness check failed;
 *              see rep.funcidx, rep.varidx for
 *              more information.
 *        -5    inappropriate solver was used:
 *              solver created with minlmcreatefgh() used  on  problem  with
 *              general linear constraints (set with minlmsetlc() call).
 *        -3    constraints are inconsistent
 *         2    relative step is no more than EpsX.
 *         5    MaxIts steps was taken
 *         7    stopping conditions are too stringent,
 *              further improvement is impossible
 *         8    terminated   by  user  who  called  MinLMRequestTermination().
 *              X contains point which was "current accepted" when termination
 *              request was submitted.
 */
void levenberg_marquardt(std::function<double(const double*)> fn, unsigned n,
                         double start[], double xmin[], double reqmin,
                         double diffstep, int kcount, int& icount, int& ifault);

void levenberg_marquardt(std::function<float(const float*)> fn, unsigned n,
                         float start[], float xmin[], float reqmin,
                         float diffstep, int kcount, int& icount, int& ifault);

void levenberg_marquardt(std::function<double(const double*)> fn, unsigned n,
                         double start[], double xmin[], double reqmin,
                         double diffstep, int numres, int kcount, int& icount,
                         int& ifault);

void levenberg_marquardt(std::function<float(const float*)> fn, unsigned n,
                         float start[], float xmin[], float reqmin,
                         float diffstep,
                         int numres, // diffstep *= 0.1 between each restart
                         int kcount, int& icount, int& ifault);

// Returns textual description of fault (i.e., exit) code
const char* levenberg_marquardt_fault_str(int ifault);

} // namespace perceive
