// Copyright (c) 2019 University of Lyon and CNRS (France).
// All rights reserved.
//
// This file is part of pc_msdm; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3.0
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.

#ifndef UTILITIES_H
#define UTILITIES_H

#include <cstdlib>
#include <cmath>
#include <cstring>
#include <fstream>
#include <random>
#include "PointSet.h"
#include "nanoflann.hpp"

using namespace nanoflann;

typedef KDTreeSingleIndexAdaptor<
 L2_Simple_Adaptor<double, PointSet > ,
 PointSet,
 3 /* dim */
 > KdTree;
 
static void save(std::vector<double> &scalars, const char *filename)
{
  std::ofstream out;
  out.open(filename);
  
  std::vector<double>::const_iterator it;
  for(it = scalars.begin(); it!=scalars.end();++it)
  {
    out<<*it<<"\n";
  }
  out.close();
} 

static void save(std::vector<Point> &pts, const char *filename)
{
  std::ofstream out;
  out.open(filename);
  
  std::vector<Point>::const_iterator it;
  for(it = pts.begin(); it!=pts.end();++it)
  {
    out<<it->x<<"\t"<<it->y<<"\t"<<it->z<<"\n";
  }
  out.close();
}

#endif
