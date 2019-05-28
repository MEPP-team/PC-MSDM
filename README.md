# PC-MSDM
---

PC-MSDM, an objective metric for visual quality assessment of 3D point clouds. This full-reference metric is based on local curvature statistics and can be viewed as an extension for point clouds of the MSDM metric suited for 3D meshes that you can find on the [MEPP plateform](https://github.com/MEPP-team/MEPP). 


Here is the list of the parameters : 

* -r   Set the radius for the Knn-RadiusSearch.
* -knn Set the number of points used for the quadric surface construction.
* -a   Set the power value of the metric calculus.
* -fq Keep the console open for single testing

To use the compiled binary  :
	
Note that you need to execute the code in both forward and backward (-i) mode and compute your own mean on the PC_MSDM field from the output files to compute the metric.
```
Windows : 
	PC-MSDM.exe PointC_A.xyz PointC_B.xyz outputfile -r 0.007 -knn 5 -a 2	  			 
	PC-MSDM.exe PointC_A.xyz PointC_B.xyz outputfile -r 0.007 -knn 5 -a 2 -i 			  
	
Linux : 
	./PC-MSDM PointC_A.xyz PointC_B.xyz outputfile -r 0.007 -knn 5 -a 2				
	./PC-MSDM PointC_A.xyz PointC_B.xyz outputfile -r 0.007 -knn 5 -a 2 -i    			
```

Input files must be formated this way: x_pos y_pos z_pos
```
-0.067 0.061558 0.020109
-0.0782 0.071279 0.021032
-0.062116 0.045145 0.037802
-0.065473 0.039513 0.037964
-0.06725 0.03742 0.033413
-0.072702 0.065008 0.018701
-0.06145 0.059165 0.018731
-0.0675 0.061479 0.019221
-0.057411 0.054114 0.0038257
-0.079222 0.070654 0.017735
-0.062473 0.04468 0.01111		
```

You can set your own file parser by modifying this function in PointSet.h : 
```C++
	void readPointCloud(std::ifstream &f) 
```

You can build the documentation using [Doxygen](http://www.doxygen.nl/).

---

Development of this software is part of the of the LIRIS research team [M2DISCO](https://liris.cnrs.fr/equipe/m2disco).
