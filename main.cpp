
#include "linalg.h"
#include "methods.h"
#include "ik.h"
#include "structs.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
    
    void _IK_LM_Chan(
        ETS *ets, Matrix4dc Tep,
        MapVectorX q0, int ilimit, int slimit, double tol, int reject_jl,
        MapVectorX q, int *it, int *search, int *solution, double *E,
        double lambda, MapVectorX we)
    {
        int iter = 1;

        // std::cout << Tep << "\n";
        // std::cout << std::endl;

        double *np_Te = (double *)PyMem_RawCalloc(16, sizeof(double));
        MapMatrix4dc Te(np_Te);

        double *np_J = (double *)PyMem_RawCalloc(6 * ets->n, sizeof(double));
        Eigen::Map<Eigen::MatrixXd> J(np_J, 6, ets->n);

        double *np_e = (double *)PyMem_RawCalloc(6, sizeof(double));
        MapVectorX e(np_e, 6);

        Matrix6dc We;

        Eigen::MatrixXd Wn(ets->n, ets->n);
        Eigen::MatrixXd EyeN = Eigen::MatrixXd::Identity(ets->n, ets->n);

        VectorX g(ets->n);

        // Set we
        if (we.size() == 6)
        {
            We = we.asDiagonal();
        }
        else
        {
            We = Matrix6dc::Identity();
        }

        // Set the first q0
        if (q0.size() == ets->n)
        {
            q = q0;
        }
        else
        {
            _rand_q(ets, q);
        }

        // Global search up to slimit
        while (*search <= slimit)
        {

            while (iter <= ilimit)
            {
                // Current pose Te
                _ETS_fkine(ets, q.data(), (double *)NULL, NULL, Te);

                // Angle axis error e
                _angle_axis(Te, Tep, e);

                // Squared error E
                *E = 0.5 * e.transpose() * We * e;

                if (*E < tol)
                {
                    // We have arrived

                    // wrap q to +- pi
                    for (int i = 0; i < ets->n; i++)
                    {
                        q(i) = std::fmod(q(i) + PI, PI_x2) - PI;
                    }

                    // Check for joint limit violation
                    if (reject_jl)
                    {
                        *solution = _check_lim(ets, q);
                    }
                    else
                    {
                        *solution = 1;
                    }

                    break;
                }

                // Jacobian Matric J
                _ETS_jacob0(ets, q.data(), (double *)NULL, J);

                // Weighting matrix Wn
                Wn = lambda * *E * EyeN;

                // The vector g
                g = J.transpose() * We * e;

                // Work out the joint velocity qd
                q += (J.transpose() * We * J + Wn).inverse() * g;
                // q += (J.transpose() * We * J + Wn).colPivHouseholderQr().solve(g);

                iter += 1;
            }

            if (*solution)
            {
                *it += iter;
                break;
            }

            *it += iter;
            iter = 0;
            *search += 1;
            _rand_q(ets, q);
        }

        free(np_e);
        free(np_Te);
        free(np_J);
    }