/**
 * This file is part of iRotAvg.
 *
 * Created by Alvaro Parra on 19/3/19.
 * Copyright © 2019 Alvaro Parra <alvaro dot parrabustos at adelaide
 * dot edu dot au> (The University of Adelaide)
 * For more information see <https://github.com/ajparra/iRotAvg>
 *
 * This work was supported by Maptek (http://maptek.com) and the
 * ARC Linkage Project LP140100946.
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


#include <iostream>
#include "l1_irls.hpp"
#include <fstream>
#include <set>

using namespace irotavg;


Cost parse_cost(const char *cost_name)
{
    if (cost_name == NULL)
    {
        std::cerr << "cost could not convert to string." << std::endl;
        std::exit(-1);
    }
    char buf[256];
    strcpy(buf, cost_name);
    // convert to lower case
    for (int i=0; buf[i] != 0; i++)
    {
        buf[i] = tolower(buf[i]);
    }
    
    Cost cost = Geman_McClure;
    if (     strcmp(buf, "l2"           )==0) cost = Cost::L2;
    else if (strcmp(buf, "l1"           )==0) cost = Cost::L1;
    else if (strcmp(buf, "l1.5"         )==0) cost = Cost::L15;
    else if (strcmp(buf, "l0.5"         )==0) cost = Cost::L05;
    else if (strcmp(buf, "geman-mcclure")==0) cost = Cost::Geman_McClure;
    else if (strcmp(buf, "huber"        )==0) cost = Cost::Huber;
    else if (strcmp(buf, "pseudo-huber" )==0) cost = Cost::Pseudo_Huber;
    else if (strcmp(buf, "andrews"      )==0) cost = Cost::Andrews;
    else if (strcmp(buf, "bisquare"     )==0) cost = Cost::Bisquare;
    else if (strcmp(buf, "cauchy"       )==0) cost = Cost::Cauchy;
    else if (strcmp(buf, "fair"         )==0) cost = Cost::Fair;
    else if (strcmp(buf, "logistic"     )==0) cost = Cost::Logistic;
    else if (strcmp(buf, "talwar"       )==0) cost = Cost::Talwar;
    else if (strcmp(buf, "welsch"       )==0) cost = Cost::Welsch;
    else
    {
        std::cerr << "Unknown string. " << cost_name << std::endl;
        std::exit(-1);
    }
    
    return cost;
}


int main(int argc, const char * argv[])
{
    std::string licence_notice =
    "iRotAvg Copyright (C) 2019  Alvaro Parra\n"
    "iRotAvg comes with ABSOLUTELY NO WARRANTY.\n"
    "    This is free software, and you are welcome to redistribute it\n"
    "    under certain conditions; visit\n"
    "    https://github.com/ajparra/iRotAvg#License for details.\n"
    "\n"
    "This work was supported by Maptek (http://maptek.com) and the \n"
    "ARC Linkage Project LP140100946..\n"
    "\n";
    
    std::string usage =
    "Usage:\n"
    "\n"
    "test input_file [output_file [cost [sigma [irls_iters [l1_iters [change_th]]]]]]\n"
    "\n"
    "  input_file      --  input file with format:\n"
    "      m n f\n"
    "      [relative rotations block]\n"
    "      [absolute rotations block]\n"
    "\n"
    "      where:\n"
    "          m  --  Number of relative rotations.\n"
    "          n  --  Number of absolute rotations.\n"
    "          f  --  Number of fixed absolute rotations. The first f rotations are fixed.\n"
    "                 If f=0, the first rotations is set to I; otherwise, the first f rotations\n"
    "                 are fixed with the first f rotations in the block of absolute rotations.\n"
    "\n"
    "      relative rotation block  --  Each line contains a relative rotation [w x y z] between\n"
    "                                   views i and j (with i,j integers and i<j) in the format:\n"
    "          i j w x y z\n"
    "\n"
    "      absolute rotation block  --  Each line contains an absolute rotation [w x y z] in the\n"
    "                                   order of the views. At least f rotations must be given. \n"
    "                                   Rotations from line (f+1) are used for initialisation if given.\n"
    "                                   Each line has the format:\n"
    "          w x y z\n"
    "\n"
    "  output_file     --  Output file with an absolute rotations [w x y z] and IRLS weights.\n"
    "                      The first n lines are the absolute rotations in the same order\n"
    "                      of the views). The m next lines are the weights. Default output file \n"
    "                      is l1_irls_out.txt. Rotation format:\n"
    "      w x y z\n"
    "\n"
    "  cost            --  IRLS cost function. Can take one of the following values: L2, L1, L1.5, L0.5,\n"
    "                      Geman-McClure, Huber, Pseudo-Huber, Andrews ,Bisquare, Cauchy, Fair,\n"
    "                      Logistic, Talwar, and Welsch. Default cost is Geman-McClure.\n"
    "\n"
    "  sigma           --  IRLS sigma in degrees. Default value is 5 [deg].\n"
    "\n"
    "  irls_iters      --  IRLS maximum number of iterations. Default value is 50.\n"
    "\n"
    "  l1_iters        --  L1-RA maximum number of itarations. Default value is 5.\n"
    "\n"
    "  change_th       --  Change threshold stopping criterion for L1-RA and IRLS. Default value is 0.001.\n"
    "\n";
    
    
    
    std::cout << licence_notice << std::endl;
    
    if (argc-1<1)
    {
        std::cout << usage <<std::endl;
        std::exit(-1);
    }
    
    //---------------------------------------------------------------------
    // Check input and output size
    //---------------------------------------------------------------------
    if (argc-1<1 || argc-1>7)
    {
        std::cerr << "Invalid number of arguments. Expected at least 1";
        std::cerr << "and at most 7 arguments." << std::endl;
        std::cerr << usage << std::endl;
        std::exit(-1);
    }
    
    //---------------------------------------------------------------------
    //    Check and read input
    //---------------------------------------------------------------------
    const char *input_file = argv[1];
    std::cout<< "input file: "<< argv[1] << std::endl;
    
    std::ifstream myfile (input_file);
    if (!myfile.is_open())
    {
        std::cerr << "Unable to open file " << input_file << std::endl;
        std::exit(-1);
    }
    
    int m, n, f; // # rel rots, # abs rots, # fixed abs rots
    myfile >> m >> n >> f;
    
    std::cout<< "# rel rots ..... = " << m << std::endl;
    std::cout<< "# abs rots ..... = " << n << std::endl;
    std::cout<< "# fixed abs rots = " << f << std::endl;
    
    I_t I;
    I.reserve(m);
    Mat QQ = Mat::Zero(m,4);
    Mat Q = Mat::Zero(n,4);
    
    int i=0, j=0;
    
    // read rel rots
    std::set<int> vertices;
    int e1, e2;
    double x, y, z, w;
    while (j < m)
    {
        if (myfile >> e1 >> e2 >> w >> x >> y >> z)
        {
            I.push_back(std::make_pair(e1, e2) ); // NOT READY TO BE USED
            vertices.insert(e1);
            vertices.insert(e2);
            QQ.row(j++) << x, y, z, w;
        }
        else
        {
            std::cerr << "Corrupt input file: inconsistent number of connections." << std::endl;
            std::exit(-1);
        }
    }
    
    e1 = 0;
    std::map<int,int> vertex_to_idx;
    for (auto const& x : vertices)
    {
        vertex_to_idx[x] = e1++;
    }

    for (auto &c: I)
    {
        c.first = vertex_to_idx[c.first];
        c.second = vertex_to_idx[c.second];
    }
    
    
    // read abs rots
    while (i < n) // f
    {
        if (myfile >> w >> x >> y >> z)
        {
            Q.row(i++) << x, y, z, w;
        }
        else
        {
            break;
        }
    }
    myfile.close();
    
    if (i<f)
    {
        std::cerr << "Insuficient number of absolute rotations. At least "<<f<< " must be given." << std::endl;
        std::exit(-1);
    }
    
    //check input_file
    int max = -1;
    for (auto e: I)
    {
        if (e.second > max)
            max = e.second;
    }
    if ( n != max+1)
    {
        std::cerr << "Corrupt input file: check abs rotations" << std::endl;
        std::exit(-1);
    }
    
    //---- output file ----------------------------------------------------
    const char *output_file = (argc-1>1) ? argv[2]: "l1_irls_out.txt";
    std::cout << "output file: " << output_file <<std::endl;
    
    //---- cost -----------------------------------------------------------
    const Cost cost = (argc-1>2) ? parse_cost(argv[3]) : Cost::Geman_McClure;
    std::cout<< "cost: " << cost << std::endl;
    
    //---- sigma ----------------------------------------------------------
    const double sigma = (argc-1>3) ?
        std::atof(argv[4])*EIGEN_PI/180.0 : 5*EIGEN_PI/180.0;
    std::cout<< "sigma [deg]: "<< sigma*180./EIGEN_PI << std::endl;
    
    //---- irls_iters -----------------------------------------------------
    const int irls_iters = (argc-1>4) ? std::atoi(argv[5]) : 50;
    std::cout<< "IRLS max. iterations: " << irls_iters << std::endl;
    
    //---- l1_iters -------------------------------------------------------
    const int l1_iters = (argc-1>5) ? std::atoi(argv[6]) : 5;
    std::cout<< "L1-RA max. iterations: " << l1_iters << std::endl;
    
    //---- change_threshold -----------------------------------------------
    const double change_th = (argc-1>6) ? std::atof(argv[7]) : 1e-3;
    std::cout<< "change threshold: " << change_th << std::endl;
    
    // --------------------------------------------------------------------
    // Compute initial Q from a Spanning Tree
    // --------------------------------------------------------------------
    if (f==0)
    {
        Q.row(0) << 0, 0, 0, 1;
        std::cout << "set first abs rot = I" << std::endl;
        f=1;
    }
    
    std::cout << "# initial absolute rots " << i << std::endl;
    const int init_f = (i>f) ? i : f;
    init_mst(Q, QQ, I, init_f);
    
    SpMat A = make_A(n, f, I);
    
    // --------------------------------------------------------------------
    // Optimise
    // --------------------------------------------------------------------
    int l1_iters_out;
    double l1_runtime;
    l1ra(QQ, I, A, Q, f, l1_iters, change_th, l1_iters_out, l1_runtime);
    
    int irls_iters_out;
    double irls_runtime;
    Vec weights(m);
    irls(QQ, I, A, cost, sigma, Q, f, irls_iters, change_th,
         weights, irls_iters_out, irls_runtime);
    quat_normalised(Q,f);
    
    // --------------------------------------------------------------------
    //  Report time
    // --------------------------------------------------------------------
    std::cout<<"L1-RA runtime [s] = " << l1_runtime << std::endl;
    std::cout<<"IRLS  runtime [s] = " << irls_runtime << std::endl;
    std::cout<<"total runtime [s] = " << (l1_runtime + irls_runtime) << std::endl;
    
    // --------------------------------------------------------------------
    //  Save output
    // --------------------------------------------------------------------
    std::ofstream out(output_file);
    if (out.is_open())
    {
        Mat Q2 = Q;
        Q2.col(0) = Q.col(3);
        Q2.col(1) = Q.col(0);
        Q2.col(2) = Q.col(1);
        Q2.col(3) = Q.col(2);
        Eigen::IOFormat HeavyFmt(Eigen::FullPrecision);
        out << Q2.format(HeavyFmt) << "\n";
        out << weights.format(HeavyFmt) << "\n";
        out.close();
    }
    else
    {
        std::cerr << "Unable to save results." << std::endl;
    }
    
    return 0;
}
