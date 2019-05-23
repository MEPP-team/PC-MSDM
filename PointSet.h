#ifndef POINT_SET
#define POINT_SET
#include<iostream>
#include <cstdlib>
#include <vector>
#include <Eigen/Dense>

class Point
{
public:
  double x,y,z,nx,ny,nz;
};

Point operator+(const Point &p1, const Point &p2)
{
  Point p;
  p.x = p1.x + p2.x;
  p.y = p1.y + p2.y;
  p.z = p1.z + p2.z;
  return p;
}

Point operator+(const Point &p1, const Eigen::Vector3d &n)
{
  Point p;
  p.x = p1.x + n(0);
  p.y = p1.y + n(1);
  p.z = p1.z + n(2);
  return p;
}

Point operator/(const Point &p1, const double a)
{
  Point p;
  p.x = p1.x/a;
  p.y = p1.y/a;
  p.z = p1.z/a;
  return p;
}

Point operator*(const Point &p1, const double a)
{
  Point p;
  p.x = p1.x*a;
  p.y = p1.y*a;
  p.z = p1.z*a;
  return p;
}

Eigen::Matrix3d operator *(const Point &p1, const Point &p2)
{
    Eigen::Matrix3d M(3,3);
    M(0,0) = p1.x*p2.x;
    M(0,1) = p1.x*p2.y;
    M(0,2) = p1.x*p2.z;
    M(1,1) = p1.y*p2.y;
    M(1,2) = p1.y*p2.z;
    M(2,2) = p1.z*p2.z;
    M(1,0) = M(0,1);
    M(2,0) = M(0,2);
    M(2,1) = M(1,2);
    return M;
    
}

class PointSet
{
public :
  std::vector<Point>  pts;
  
  PointSet()
  {
    xmin=xmax = 0.0;
  }

  // Must return the number of data points
  inline size_t kdtree_get_point_count() const { return pts.size(); }

  // Returns the distance between the vector "p1[0:size-1]" and the data point 
  //with index "idx_p2" stored in the class:
  inline double kdtree_distance(const double *p1, const size_t idx_p2,size_t 
size) const
  {
    const double d0=p1[0]-pts[idx_p2].x;
    const double d1=p1[1]-pts[idx_p2].y;
    const double d2=p1[2]-pts[idx_p2].z;
    return d0*d0+d1*d1+d2*d2;
  }

  // Returns the dim'th component of the idx'th point in the class:
  // Since this is inlined and the "dim" argument is typically an immediate 
  //value, the  "if/else's" are actually solved at compile time.
  inline double kdtree_get_pt(const size_t idx, int dim) const
  {
    if (dim==0) return pts[idx].x;
    else if (dim==1) return pts[idx].y;
    else return pts[idx].z;
  }

  // Optional bounding-box computation: return false to default to a standard 
  //bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned 
  //in "bb" so it can be avoided to redo it again.
  //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 
  //for point clouds)
  template <class BBOX>
  bool kdtree_get_bbox(BBOX &bb) const { return false; }

  void readPointCloud(std::ifstream &f)
  {
    Point p;

	f >> p.x >> p.y >> p.z;
	//f >> p.x >> p.y >> p.z >> p.nx >> p.ny >> p.nz;//other formats

    unsigned int i=0;
    xmin = xmax = p.x;
    ymin = ymax = p.y;
    zmin = zmax = p.z;

    while(f)
    {
      xmax = xmax > p.x ? xmax : p.x;
      ymax = ymax > p.y ? ymax : p.y;
      zmax = zmax > p.z ? zmax : p.z;
      xmin = xmin < p.x ? xmin : p.x;
      ymin = ymin < p.y ? ymin : p.y;
      zmin = zmin < p.z ? zmin : p.z;
      pts.push_back(p);
      ++i;
	  f >> p.x >> p.y >> p.z;
	//  f >> p.x >> p.y >> p.z >> p.nx >> p.ny >> p.nz;//other formats
    }
  }
 
  double xmin,ymin,zmin;
  double xmax,ymax,zmax;
  
  
  void loadPointCloud(std::vector<Point> &points)
  {
    pts.clear();
    std::vector<Point>::const_iterator pi = points.begin();
    unsigned int i=0;
    xmin = xmax = pi->x;
    ymin = ymax = pi->y;
    zmin = zmax = pi->z;
 
    for(;pi!= points.end(); ++pi)
    {
      xmax = xmax > pi->x ? xmax : pi->x;
      ymax = ymax > pi->y ? ymax : pi->y;
      zmax = zmax > pi->z ? zmax : pi->z;
      xmin = xmin < pi->x ? xmin : pi->x;
      ymin = ymin < pi->y ? ymin : pi->y;
      zmin = zmin < pi->z ? zmin : pi->z;
      pts.push_back(*pi);
      ++i;
    }
  }
  
  void addPoint(Point &p)
  {
      if(xmax - xmin <1e-16)
      {
        xmin = xmax = p.x;
        ymin = ymax = p.y;
        zmin = zmax = p.z;
      }
      else
      {
        xmax = xmax > p.x ? xmax : p.x;
        ymax = ymax > p.y ? ymax : p.y;
        zmax = zmax > p.z ? zmax : p.z;
        xmin = xmin < p.x ? xmin : p.x;
        ymin = ymin < p.y ? ymin : p.y;
        zmin = zmin < p.z ? zmin : p.z;
      }
      pts.push_back(p);
  }
  
  int npts() const
  {
    return (int)pts.size();
  }
};

#endif
