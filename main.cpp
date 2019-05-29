// Copyright (c) 2019 University of Lyon and CNRS (France).
// All rights reserved.
//
// This file is part of pc_msdm; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3.0
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.


/** @file */ 
#include <chrono> //time computation
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include "nanoflann.hpp" //Point lib
#include "utilities.h"
#include <Eigen/Dense>


using std::cerr;
using std::cout;
using std::endl;
using std::string;

using namespace Eigen;
using namespace nanoflann;

using Eigen::Matrix3d;
using Eigen::JacobiSVD;

/**
* \fn void computeProjectionAndCurvature(const Point &origin, const std::vector<Point> &refpoints, std::vector<size_t> indices, Point &proj, double &H)
* \brief Compute the projection of an origin point onto the polynomial approximation of a set of neighbors given by a list of indices.
*
* \param origin : Point to be projected.
* \param refpoints : Contains all points from ref points cloud.
* \param indices : Index of points in refpoints cloud used to compute the projection.
* \param proj : Reference containing the point resulting from the projection.
* \param H : Reference containing the mean curvature of the projected point.
* \return Returns both the projection and the mean curvature (Referenced variables).
*/
void computeProjectionAndCurvature(const Point &origin, const std::vector<Point> &refpoints, std::vector<size_t> indices, Point &proj, double &H)
{
	Matrix3d M;
	M.setZero();
	Vector3d mu;
	mu.setZero();
	int nneighbors = indices.size();

	for (int i = 0; i<nneighbors; ++i)
	{
		Point p = refpoints[indices[i]];
		Vector3d neighbor(p.x, p.y, p.z);
		mu = mu + neighbor;
		M = M + neighbor*neighbor.transpose();
	}

	mu = mu / ((double)nneighbors);
	M = 1. / ((double)nneighbors)*M - mu*mu.transpose();

	//get local frame
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(M);


	Eigen::Vector3d t1 = eig.eigenvectors().col(2);
	Eigen::Vector3d t2 = eig.eigenvectors().col(1);
	Eigen::Vector3d  n = eig.eigenvectors().col(0);

	MatrixXd A(nneighbors, 6);
	VectorXd B(nneighbors);

	//build linear system
	for (int i = 0; i<nneighbors; ++i)
	{
		double xglob = refpoints[indices[i]].x - origin.x;
		double yglob = refpoints[indices[i]].y - origin.y;
		double zglob = refpoints[indices[i]].z - origin.z;
		Vector3d v(xglob, yglob, zglob);
		double x = v.transpose()*t1;
		double y = v.transpose()*t2;
		double z = v.transpose()*n;

		A(i, 0) = x*x;
		A(i, 1) = y*y;
		A(i, 2) = x*y;
		A(i, 3) = x;
		A(i, 4) = y;
		A(i, 5) = 1;

		B(i) = z;
	}


	VectorXd coeffs = A.colPivHouseholderQr().solve(B);

	//corresponding point:
	Vector3d delta = coeffs(5)*n;
	proj = origin + delta;


	//corresponding curvature
	double fxx = 2 * coeffs(0);
	double fyy = 2 * coeffs(1);
	double fxy = coeffs(2);
	double fx = coeffs(3);
	double fy = coeffs(4);

	H = 0.5*((1 + fx*fx)*fyy + (1 + fy*fy)*fxx - 2 * fxy*fx*fy) / pow(1 + fx*fx + fy*fy, 1.5);

}



/**
* \fn double compute_distance(Point &a, Point &b)
* \brief Compute the Euclidean distance between two points.
*
* \param a : Point a.
* \param b : Point b.
* \return Returns the Euclidean distance between a and b.
*/
double compute_distance(Point &a, Point &b)
{
	return std::sqrt(std::pow(b.x - a.x, 2.0) + std::pow(b.y - a.y, 2.0) + std::pow(b.z - a.z, 2.0));
}



/**
* \fn std::string remove_extension(const std::string& filename) 
* \brief Remove the extension of an input filename
*
* \param filename : filename
* \return The filename's string without extension
*/
std::string remove_extension(const std::string& filename) {
	size_t lastdot = filename.find_last_of(".");
	if (lastdot == std::string::npos) return filename;
	return filename.substr(0, lastdot);
}



/**
* \fn bool write_to_csv(const string regfile, const string reffile,
				  const double PC_MSDM, const double RadiusCurvature,
				  const int threshold_knnsearch, const double a,const string destination)
* \brief Write experiment informations and result at the end of the file. (append)
* \param regfile : Registered point cloud filename
* \param reffile : Reference point cloud filename
* \param PC_MSDM : Value of the metric
* \param RadiusCurvature : Radius used to compute curvature 
* \param threshold_knnsearch : Number of points used to compute curvature
* \param destination : Output filename
* \return True if the operation is a success, false otherwise.
*/
bool write_to_csv(const string regfile, const string reffile,
				  const double PC_MSDM, const double RadiusCurvature,
				  const int threshold_knnsearch, const double a,const string destination)
{
	//Write metric value and parameters to file
	std::ofstream PC_MSDM_out_f(destination, std::ios::app);

	if (PC_MSDM_out_f.is_open())
	{
		PC_MSDM_out_f << remove_extension(regfile) << ";";
		PC_MSDM_out_f << remove_extension(reffile) << ";";
		PC_MSDM_out_f << PC_MSDM << ";";
		PC_MSDM_out_f << RadiusCurvature << ";";
		PC_MSDM_out_f << threshold_knnsearch << ";";
		PC_MSDM_out_f << a << "\n";
		PC_MSDM_out_f.close();
		cout << "Writing to " << destination << " : success" << std::endl;
		return true;
	}
	else cout << "Unable to open "<<destination<<" for writting";
	PC_MSDM_out_f.close();
	return false;

}



/**
* \fn void compute_statistics(const double RadiusCurvature, const double maxDim, PointSet &regptset, KdTree &m_kdtree2,
	std::vector<Point> &projectedpointsOnRef, std::vector<Point> &projectedpointsOnMe,
	std::vector<double> &meancurvaturesProj, std::vector<double> &meancurvaturesMe,
	std::vector<double> &luminanceMe, std::vector<double> &contrastMe,
	std::vector<double> &structureMe, std::vector<double> &LMSDM,
	double &PC_MSDM, const double a)

* \brief This function contain the loop computing the statistics for each point.
* \param RadiusCurvature : Radius used to compute curvature.
* \param maxDim : Longest dimension of the bounding box.
* \param regptset : Registered pointset.
* \param m_kdtree2 : kdtree describing Registered point cloud.
* \param projectedpointsOnRef : Vector of points containing projected points on reference point cloud.
* \param projectedpointsOnMe  :  Vector of points containing projected points on registered point cloud.
* \param meancurvaturesProj : Vector containing meanCurvature value of each projected of the other point cloud.
* \param meancurvaturesMe  :  Vector containing meanCurvature value of each point projected on itself.
* \param luminanceMe : Vector containing luminance value for each point.
* \param contrastMe  : Vector containing contrast value for each point.
* \param structureMe  : Vector containing structure value for each point.
* \param LMSDM :  Vector containing local_distortion value for each point.
* \param PC_MSDM  :  Result variable.
* \param a  : Power parameter of the sum.
* \return The global score in PC_MSDM
*/
void compute_statistics(const double RadiusCurvature, const double maxDim, PointSet &regptset, KdTree &m_kdtree2,
	std::vector<Point> &projectedpointsOnRef, std::vector<Point> &projectedpointsOnMe,
	std::vector<double> &meancurvaturesProj, std::vector<double> &meancurvaturesMe,
	std::vector<double> &luminanceMe, std::vector<double> &contrastMe,
	std::vector<double> &structureMe, std::vector<double> &LMSDM,
	double &PC_MSDM, const double a
)
{
	double alpha = 1.0; //Luminance factor
	double beta = 1.0; //Contrast factor
	double gamma = 0.5; //Structure factor
	double const_k = 1.0;
	double search_radius_neighborhood = static_cast<double>(RadiusCurvature*maxDim*5.0);  //r = h/5 
																					  //Computing PC_MSDM
	std::cout << "Computing PC_MSDM" << std::endl;
	
	#pragma omp parallel for
	for (int i = 0; i < regptset.npts(); i++)//loop on each point of registered
	{

		Point origin = regptset.pts[i];
		double luminance = 0.0;
		double contrast = 0.0;
		double structure = 0.0;

		//Structure containing indexes and distances returned from KNN radius search (!= of knnsearch)
		std::vector<std::pair<size_t, double> >   ret_matches_Reg;
		ret_matches_Reg.clear();

		//Distances structures
		std::vector<double> ret_distance_Reg;
		std::vector<double> ret_distance_Ref;

		//Weights structures
		std::vector<double> ret_weight_Reg;
		std::vector<double> ret_weight_Ref;
		double sum_distances_me = 0.0;
		double sum_distances_proj = 0.0;

		nanoflann::SearchParams params;
		params.sorted = false;

		double query_pt[3] = { origin.x, origin.y, origin.z };

		//Looking for neighbors of REGISTERED to compute statistics
		const size_t nMatches_Reg = m_kdtree2.radiusSearch(&query_pt[0], std::pow(search_radius_neighborhood,2.0), ret_matches_Reg, params); //Powered distances  => pow(radius,2)

		double debug_variance = search_radius_neighborhood / 2.0;
		//Computing weightsl
		for (size_t cpt_reg = 0; cpt_reg < nMatches_Reg; cpt_reg++)
		{
			//Get distances for REGISTERED
			ret_distance_Reg.push_back(std::sqrt(ret_matches_Reg[cpt_reg].second)); //Powered distances  => sqrt(distance) 

			//manually computing distance REFERENCE
			Point p_orig_proj = projectedpointsOnRef[i];
			Point p_neigh_proj = projectedpointsOnRef[ret_matches_Reg[cpt_reg].first];
			ret_distance_Ref.push_back(compute_distance(p_orig_proj, p_neigh_proj));

			//Gaussian-weighting computation
			double wi1 = 1 / debug_variance / sqrt(2 * 3.141592)*exp(-(ret_distance_Reg[cpt_reg] * ret_distance_Reg[cpt_reg]) / 2 / debug_variance / debug_variance);
			double wi2 = 1 / debug_variance / sqrt(2 * 3.141592)*exp(-(ret_distance_Ref[cpt_reg] * ret_distance_Ref[cpt_reg]) / 2 / debug_variance / debug_variance);
			ret_weight_Reg.push_back(wi1);
			ret_weight_Ref.push_back(wi2);

			//Sum the weight
			sum_distances_me += ret_weight_Reg[cpt_reg];
			sum_distances_proj += ret_weight_Ref[cpt_reg];

		}


		//Average of curvature computed on neighbors on both reference and registered cloud
		double mu_me = 0.0;
		double mu_proj = 0.0;


		//Looking for neighbors
		for (int cpt_neigh = 0; cpt_neigh < nMatches_Reg; cpt_neigh++)
		{
			size_t index_reg = ret_matches_Reg[cpt_neigh].first;
			mu_me += std::abs(meancurvaturesMe[index_reg] * ret_weight_Reg[cpt_neigh]);
			mu_proj += std::abs(meancurvaturesProj[index_reg] * ret_weight_Ref[cpt_neigh]);
		}

		mu_me = mu_me / sum_distances_me;
		mu_proj = mu_proj / sum_distances_proj;



		//Variance
		double standard_dev_me = 0.0;
		double standard_dev_proj = 0.0;

		//Covariance
		double covariance = 0.0;
		for (int cpt_neigh = 0; cpt_neigh < nMatches_Reg; cpt_neigh++)
		{
			size_t index_reg = ret_matches_Reg[cpt_neigh].first;
			standard_dev_me += pow((std::abs(meancurvaturesMe[index_reg]) - mu_me), 2.0) * ret_weight_Reg[cpt_neigh];
			standard_dev_proj += pow((std::abs(meancurvaturesProj[index_reg]) - mu_proj), 2.0) * ret_weight_Ref[cpt_neigh];
			covariance += ((std::abs(meancurvaturesMe[index_reg]) - mu_me)   * (std::abs(meancurvaturesProj[index_reg]) - mu_proj))  * ret_weight_Reg[cpt_neigh];

		}

		standard_dev_me = std::sqrt(standard_dev_me / sum_distances_me);
		standard_dev_proj = std::sqrt(standard_dev_proj / sum_distances_proj);

		covariance = covariance / sum_distances_me;

		luminance = std::abs(mu_me - mu_proj) / (std::max(mu_me, mu_proj) + const_k);
		contrast = std::abs(standard_dev_me - standard_dev_proj) / (std::max(standard_dev_me, standard_dev_proj) + const_k);
		structure = std::abs(standard_dev_me*standard_dev_proj - covariance) / ((standard_dev_me*standard_dev_proj) + const_k);

		//clamping structure field
		if (structure > 1.0) structure = 1.0;


		luminanceMe[i] = luminance;
		contrastMe[i] = contrast;
		structureMe[i] = structure;
	
		LMSDM[i] = (alpha * luminance + beta * contrast + gamma *structure) / (alpha + beta + gamma); //LD - Local distortion

		#pragma omp atomic
		PC_MSDM += std::pow(LMSDM[i], a);

	}
}

/**
* \fn main(int argc, char** argv)

* \brief Entry point of the program

* \return EXIT_SUCCESS if the code executed successfuly.
*/
int main(int argc, char** argv) {

	//Use interface
	bool use_interface = true;
	//Keep console open if false
	bool fast_quit = false;


	//This code is based on the MSDM2 module from the MEPP team.
	//Some parameters are kept in order to compare both solutions and results.

	//Default parameters
	double RadiusCurvature = 0.007; //Radius used to compute curvature in MSDM2 => We only use it for the radius search in compute_statistics
	int threshold_knnsearch = 15; //Minimum threshold for the Knn-search 
	double a = 3.0; //Minkowski sum parameter.
	if (argc < 3) {
		std::cerr << "Error with arguments usage : \n\t REF_file REG_file Output_file \n\t (Options) (--fastquit || -fq) (-r radius) (-knn nb_point) (-a power) (-i inverse)" << std::endl;
		return EXIT_FAILURE;
	}
	//First and second arguments are the filenames of your point cloud
	std::string reffile = argv[1]; 
	std::string regfile = argv[2];
	std::string out_filename = argv[3]; //Output file

	/* Example of command line
	Note that you need to execute the code in both forward and backward (-i) and do your own Mean on the PC_MSDM field from the output files.
		Windows : 
			Forward  =>	PC-MSDM.exe cube.xyz cube_Octree_perc0_3.xyz pc_msdm  -fq -r 0.007 -knn 5 -a 2
			Backward =>	PC-MSDM.exe cube.xyz cube_Octree_perc0_3.xyz pc_msdm  -fq -r 0.007 -knn 5 -a 2 -i
		Linux : 
			Forward  =>	./PC-MSDM     cube.xyz cube_Octree_perc0_3.xyz pc_msdm  -fq -r 0.007 -knn 5 -a 2
			Backward =>	./PC-MSDM     cube.xyz cube_Octree_perc0_3.xyz pc_msdm  -fq -r 0.007 -knn 5 -a 2 -i

	*/

	//Loop on each argument
	for (int i = 4; i < argc; ++i)
	{

		if (std::string(argv[i]) == "--fastquit" || std::string(argv[i]) == "-fq") //-fq if you use test script
		{

			fast_quit = true;
			std::cout << "fast_quit set to : " << fast_quit << std::endl;
		}

		if (std::string(argv[i]) == "-r") 
		{
			if (i + 1 < argc) { // Make sure we aren't at the end of argv!
				i++;            // Increment 'i' so we don't get the argument as the next argv[i].
				RadiusCurvature = std::stod(std::string(argv[i])); 
				std::cout << "RadiusCurvature set to : "<< RadiusCurvature << std::endl;
			}
			else {
				std::cerr << "Error : -r option requires one argument." << std::endl;
				return EXIT_FAILURE;
			}
		}

		if (std::string(argv[i]) == "-knn") 
		{
			if (i + 1 < argc) { // Make sure we aren't at the end of argv!
				i++;
				threshold_knnsearch = std::stoi(std::string(argv[i])); // Increment 'i' so we don't get the argument as the next argv[i].

				std::cout << "threshold_knnsearch set to : " << threshold_knnsearch << std::endl;
			}
			else {
				std::cerr << "Error : -knn option requires one argument." << std::endl;
				return EXIT_FAILURE;
			}
		}

		if (std::string(argv[i]) == "-a")
		{
			if (i + 1 < argc) { // Make sure we aren't at the end of argv!
				i++;
				a = std::stoi(std::string(argv[i])); // Increment 'i' so we don't get the argument as the next argv[i].

				std::cout << "a set to : " << a << std::endl;
			}
			else {
				std::cerr << "Error : -a option requires one argument." << std::endl;
				return EXIT_FAILURE;
			}
		}

		if (std::string(argv[i]) == "-i") //
		{
			std::cout << "Inversing Ref <=> Reg" << std::endl;
			reffile = argv[2];
			regfile = argv[1];
			std::stringstream ss;
			ss << std::string(argv[3]) <<"_b";
			out_filename = ss.str();
		}

	}


	std::cout << "Input reference point set file:  " << reffile << std::endl;
	std::cout << "Input registered point set file:  " << regfile << std::endl;

	PointSet refptset;
	PointSet regptset;

	std::ifstream f;
	f.open(reffile.c_str()); //open file
	if (f.is_open())
	{
		refptset.readPointCloud(f);//Read file and set data to Pointset - Check the function to match your file format. Default : x,y,z
		f.close();
		std::cout << refptset.npts() << " points in the reference point set" << std::endl;
	}
	else
	{
		std::cout << "Error opening file : " << reffile.c_str() << std::endl;
		return EXIT_FAILURE;
	}



	f.open(regfile.c_str());
	if (f.is_open())
	{
		regptset.readPointCloud(f);
		f.close();
		std::cout << regptset.npts() << " points in the registered point set" << std::endl;
	}
	else
	{
		std::cout << "Error opening file : " << regfile.c_str() << std::endl;
		return EXIT_FAILURE;
	}

	//building kdtree structures
	KdTree m_kdtree(3, refptset, KDTreeSingleIndexAdaptorParams(10));
	m_kdtree.buildIndex();


	KdTree m_kdtree2(3, regptset, KDTreeSingleIndexAdaptorParams(10));
	m_kdtree2.buildIndex();


	//structure initializing
	std::vector<Point> projectedpointsOnRef;
	std::vector<Point> projectedpointsOnMe;
	std::vector<double> meancurvaturesProj;
	std::vector<double> meancurvaturesMe;
	std::vector<double> luminanceMe;
	std::vector<double> contrastMe;
	std::vector<double> structureMe;
	std::vector<double> LMSDM;
	projectedpointsOnRef.assign(regptset.npts(), Point());
	projectedpointsOnMe.assign(regptset.npts(), Point());
	meancurvaturesProj.assign(regptset.npts(), 0.0);
	meancurvaturesMe.assign(regptset.npts(), 0.0);
	luminanceMe.assign(regptset.npts(), 0.0);
	contrastMe.assign(regptset.npts(), 0.0);
	structureMe.assign(regptset.npts(), 0.0);
	LMSDM.assign(regptset.npts(), 0.0);


	//computation of the bounding box
	double rangeX, rangeY, rangeZ = 0.0;
	rangeX = std::abs(regptset.xmin) + std::abs(regptset.xmax);
	rangeY = std::abs(regptset.ymin) + std::abs(regptset.ymax);
	rangeZ = std::abs(regptset.zmin) + std::abs(regptset.zmax);

	//LMSDM params
	double M = 1.0;
	double maxDim = std::max(rangeX, std::max(rangeY, rangeZ));
	double PC_MSDM = 0.0;

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();//Chrono start


	//CURVATURE COMPUTATION
	#pragma omp parallel for
	for (int i = 0; i < regptset.npts(); ++i) //loop on each points of registered point set
	{
		Point origin = regptset.pts[i];
		double H = 0;
		double K = 0;
		Point proj;
		Point projOnMe;

		double query_pt[3] = { origin.x, origin.y, origin.z }; //basic format transformation



		// KNNSEARCH
		//Indexes structures
		std::vector<size_t>   ret_index_ref(threshold_knnsearch);
		std::vector<size_t>   ret_index_reg(threshold_knnsearch);
		//Distances structures
		std::vector<double> out_dist_sqr_ref(threshold_knnsearch);
		std::vector<double> out_dist_sqr_reg(threshold_knnsearch);

		//Get neighborhood of the query for each point cloud
		m_kdtree.knnSearch(&query_pt[0], threshold_knnsearch, &ret_index_ref[0], &out_dist_sqr_ref[0]);
		m_kdtree2.knnSearch(&query_pt[0], threshold_knnsearch, &ret_index_reg[0], &out_dist_sqr_reg[0]);
	
		computeProjectionAndCurvature(origin, refptset.pts, ret_index_ref, proj, H);
		computeProjectionAndCurvature(origin, regptset.pts, ret_index_reg, projOnMe, K);

		meancurvaturesProj[i] = std::abs(H);
		projectedpointsOnRef[i] = proj;
		meancurvaturesMe[i] = std::abs(K);
		projectedpointsOnMe[i] = projOnMe;
	}



	compute_statistics(RadiusCurvature, maxDim, regptset, m_kdtree2,
		projectedpointsOnRef, projectedpointsOnMe,
		meancurvaturesProj, meancurvaturesMe,
		luminanceMe, contrastMe, structureMe,
		LMSDM, PC_MSDM, a);

	//Final result
	PC_MSDM = std::pow(PC_MSDM / double(regptset.npts()), 1.0 / a);
	std::cout << " PC_MSDM value : " << PC_MSDM << std::endl;


	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now(); //Chrono end
	std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()/1000000.0 << " sec elpased"<< std::endl;
	
	//Write PC_MSDM to file
	write_to_csv(regfile, reffile, PC_MSDM, RadiusCurvature, threshold_knnsearch, a, out_filename);


	if (!fast_quit)
	{
		std::cout << "Press enter to exit" << std::endl;
		std::getchar();
	}
	return EXIT_SUCCESS;
}
