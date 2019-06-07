/**
 * This file is part of iRotAvg.
 *
 * Created by Alvaro Parra on 19/3/19.
 * Copyright © 2019 Alvaro Parra <alvaro dot parrabustos at adelaide
 * dot edu dot au> (The University of Adelaide)
 * For more information see <https://github.com/ajparra/iRotAvg>
 *
 * This work was supported by Maptek (http://maptek.com) and the
 * ARC Grant DP160103490.
 *
 * iRotAvg is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * iRotAvg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with iRotAvg. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef l1_irls_hpp
#define l1_irls_hpp

#include <stdio.h>
#include "SuiteSparseQR.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <limits>


namespace irotavg
{
    #define EPS 2.2204e-16
    const double DBL_MAX_ = std::numeric_limits<double>::max();
    
    typedef SuiteSparse_long Long;
    typedef Eigen::SparseMatrix<double, Eigen::ColMajor, Long> SpMat;
    typedef Eigen::MatrixXd Mat;
    typedef Eigen::VectorXd Vec;
    typedef Eigen::Vector3d Vec3;
    typedef Eigen::Vector4d Vec4;
    typedef Eigen::Quaterniond Quat;
    typedef Eigen::Triplet<double> T;
    typedef std::vector< std::pair <int,int> > I_t;

    // ----------------------------------------------------------------------
    //     Cost
    // ----------------------------------------------------------------------
    enum Cost {L2, L1, L15, L05, Geman_McClure, Huber, Pseudo_Huber, Andrews,
        Bisquare, Cauchy, Fair, Logistic, Talwar, Welsch};
    
    inline
    std::ostream &operator<<( std::ostream &os, const Cost cost )
    {
        switch (cost) {
            case L2: os << "L2"; break;
            case L1: os << "L1"; break;
            case L15: os << "L1.5"; break;
            case L05: os << "L0.5"; break;
            case Geman_McClure: os << "Geman-McClure"; break;
            case Huber: os << "Huber"; break;
            case Pseudo_Huber: os << "Pseudo-Huber"; break;
            case Andrews: os << "Andrews"; break;
            case Bisquare: os << "Bisquare"; break;
            case Cauchy: os << "Cauchy"; break;
            case Fair:  os << "Fair"; break;
            case Logistic: os << "Logistic"; break;
            case Talwar: os << "Talwar"; break;
            case Welsch: os << "Welsch"; break;
        }
        return os;
    }
    
    // --------------------------------------------------------------------
    //   Function interfaces
    // --------------------------------------------------------------------
    // n    --  number of unknown rotations.
    // f    --  number of fixed rotations.
    // Q    --  (n+f)x4 matrix with absolute rotations. First f rotations are fixed.
    // QQ   --  mX4 matrix with relative rotations.
    // I    --  Camera connections.
    void init_mst(Mat &Q, const Mat &QQ, const I_t &I, const int f);

    SpMat make_A(const int n, const int f, const I_t &I);
    
    
    // ----  rotation averaging methods -------------------------------------
    // max_iter      -- maximum number of iterations.
    // change_th     -- treshold stoping criterion.
    // iter (OUT)    -- performed iterations.
    // runtime (OUT) -- runtime in seconds.
    // weights (OUT) -- IRLS weight's vector for each connection.
    void l1ra(const Mat &QQ, const I_t &I, const SpMat &A,
              Mat &Q, const int f, const int max_iters, double change_th,
              int &iter, double &runtime);
    
    void irls(const Mat &QQ, const I_t &I, const SpMat &A,
              Cost cost, double sigma, Mat &Q, const int f,
              const int max_iters, double change_th, Vec &weights,
              int &iteration, double &runtime);
    
    // ---- extra functions -------------------------------------------------
    
    // normalise non-fixed quaternions in Q
    void quat_normalised(Mat &Q, const int f);

}


#endif /* l1_irls_hpp */
