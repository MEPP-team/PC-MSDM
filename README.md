# PC-MSDM
---

PC-MSDM, an objective metric for visual quality assessment of 3D point clouds. This full-reference metric is based on local curvature statistics and can be viewed as an extension for point clouds of the MSDM metric suited for 3D meshes that you can find on the [MEPP plateform](https://github.com/MEPP-team/MEPP). 


Here is the list of the parameters : 

* -r   Set the radius for the Knn-RadiusSearch.
* -knn Set the number of points used for the quadric surface construction.
* -a   Set the power value of the metric calculus.
* -fq Keep the console open for single testing

To use the compiled binary  :

Example of command line
Note that you need to execute the code in both forward and backward (-i) mode and compute your own mean on the PC_MSDM field from the output files to compute the metric.
```
	Windows : 
		PC-MSDM.exe cube.xyz cube_Octree_perc0_3.xyz pc_msdm  -r 0.007 -knn 5 -a 2	  			  //Forward
		PC-MSDM.exe cube.xyz cube_Octree_perc0_3.xyz pc_msdm  -r 0.007 -knn 5 -a 2 -i 			  //Backward
		
	Linux : 
		./PC-MSDM cube.xyz cube_Octree_perc0_3.xyz pc_msdm  -r 0.007 -knn 5 -a 2				  //Forward 
		./PC-MSDM cube.xyz cube_Octree_perc0_3.xyz pc_msdm  -r 0.007 -knn 5 -a 2 -i    			  //Backward
```

You can build the documentation using [Doxygen](http://www.doxygen.nl/).

---

Development of this software is part of the of the LIRIS research team [M2DISCO] https://liris.cnrs.fr/equipe/m2disco.
