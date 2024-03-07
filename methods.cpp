/* methods.cpp */

#include "linalg.h"
#include "methods.h"
#include "structs.h"

//#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/QR>



void _ETS_jacob0(ETS *ets, double *q, double *tool, MapMatrixJc &eJ)
{


    ET *et;
    Eigen::Matrix<double, 6, Eigen::Dynamic> tJ(6, ets->n);
    double T[16];
    MapMatrix4dc eT(T);
    Matrix4dc U = Eigen::Matrix4d::Identity();
    Matrix4dc invU;
    Matrix4dc temp;
    Matrix4dc ret;
    int j = ets->n - 1;
    double sign = 1.0;

    if (tool != NULL)
    {
        Matrix4dc e_tool(tool);
        temp = e_tool * U;
        U = temp;
    }

    for (int i = ets->m - 1; i >= 0; i--)
    {
        et = ets->ets[i];

        if (et->isjoint)
        {

            if (et->axis == 0)
            {
                tJ(Eigen::seq(0, 2), j) = U(2, Eigen::seq(0, 2)) * U(1, 3) - U(1, Eigen::seq(0, 2)) * U(2, 3);
                tJ(Eigen::seq(3, 5), j) = U(0, Eigen::seq(0, 2));

                if (et->isflip)
                {
                    tJ(Eigen::seq(0, 5), j) = -tJ(Eigen::seq(0, 5), j);
                }
            }
            else if (et->axis == 1)
            {
                tJ(Eigen::seq(0, 2), j) = U(0, Eigen::seq(0, 2)) * U(2, 3) - U(2, Eigen::seq(0, 2)) * U(0, 3);
                tJ(Eigen::seq(3, 5), j) = U(1, Eigen::seq(0, 2));

                if (et->isflip)
                {
                    tJ(Eigen::seq(0, 5), j) = -tJ(Eigen::seq(0, 5), j);
                }
            }
            else if (et->axis == 2)
            {
                tJ(Eigen::seq(0, 2), j) = U(1, Eigen::seq(0, 2)) * U(0, 3) - U(0, Eigen::seq(0, 2)) * U(1, 3);
                tJ(Eigen::seq(3, 5), j) = U(2, Eigen::seq(0, 2));

                if (et->isflip)
                {
                    tJ(Eigen::seq(0, 5), j) = -tJ(Eigen::seq(0, 5), j);
                }
            }
            else if (et->axis == 3)
            {
                tJ(Eigen::seq(0, 2), j) = U(0, Eigen::seq(0, 2));
                tJ(Eigen::seq(3, 5), j) = Eigen::Vector3d::Zero();

                if (et->isflip)
                {
                    tJ(Eigen::seq(0, 2), j) = -tJ(Eigen::seq(0, 5), j);
                }
            }
            else if (et->axis == 4)
            {
                tJ(Eigen::seq(0, 2), j) = U(1, Eigen::seq(0, 2));
                tJ(Eigen::seq(3, 5), j) = Eigen::Vector3d::Zero();

                if (et->isflip)
                {
                    tJ(Eigen::seq(0, 2), j) = -tJ(Eigen::seq(0, 5), j);
                }
            }
            else if (et->axis == 5)
            {
                tJ(Eigen::seq(0, 2), j) = U(2, Eigen::seq(0, 2));
                tJ(Eigen::seq(3, 5), j) = Eigen::Vector3d::Zero();

                if (et->isflip)
                {
                    tJ(Eigen::seq(0, 2), j) = -tJ(Eigen::seq(0, 5), j);
                }
            }

            _ET_T(et, &ret(0), q[et->jindex]);
            temp = ret * U;
            U = temp;
            j--;
        }
        else
        {
            _ET_T(et, &ret(0), q[et->jindex]);
            temp = ret * U;
            U = temp;
        }
    }

    Eigen::Matrix<double, 6, 6> ev;
    ev.topLeftCorner<3, 3>() = U.topLeftCorner<3, 3>();
    ev.topRightCorner<3, 3>() = Eigen::Matrix3d::Zero();
    ev.bottomLeftCorner<3, 3>() = Eigen::Matrix3d::Zero();
    ev.bottomRightCorner<3, 3>() = U.topLeftCorner<3, 3>();
    eJ = ev * tJ;
}



void _ETS_fkine(ETS *ets, double *q, double *base, double *tool, MapMatrix4dc &e_ret)
{
    ET *et;
    Matrix4dc temp;
    Matrix4dc current;

    if (base != NULL)
    {
        MapMatrix4dc e_base(base);
        current = e_base;
    }
    else
    {
        current = Eigen::Matrix4d::Identity();
    }

    for (int i = 0; i < ets->m; i++)
    {
        et = ets->ets[i];

        _ET_T(et, &e_ret(0), q[et->jindex]);
        temp = current * e_ret;
        current = temp;
    }

    if (tool != NULL)
    {
        MapMatrix4dc e_tool(tool);
        e_ret = current * e_tool;
    }
    else
    {
        e_ret = current;
    }
}

void _ET_T(ET *et, double *ret, double eta)
{
    // Check if static and return static transform
    if (!et->isjoint)
    {
        _copy(et->T, ret);
        return;
    }

    if (et->isflip)
    {
        eta = -eta;
    }

    // // Calculate ET trasform based on eta

    et->op(ret, eta);

}
