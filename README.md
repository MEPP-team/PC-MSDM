# PC-MSDM
---

PC-MSDM, an objective metric for visual quality assessment of 3D point clouds. This full-reference metric is based on local curvature statistics and can be viewed as an extension for point clouds of the MSDM metric suited for 3D meshes that you can find on the [MEPP plateform](https://github.com/MEPP-team/MEPP). 


To use the compiled binary  :

```
	Example of command line
	Note that you need to execute the code in both forward and backward (-i) and do your own Mean on the PC_MSDM field from the output files to compute the metric.
	
		Windows : 
			PC-MSDM.exe cube.xyz cube_Octree_perc0_3.xyz pc_msdm  -fq -r 0.007 -knn 5 -a 2	  			  //Forward
			PC-MSDM.exe cube.xyz cube_Octree_perc0_3.xyz pc_msdm  -fq -r 0.007 -knn 5 -a 2 -i 			  //Backward
			
		Linux : 
			./PC-MSDM cube.xyz cube_Octree_perc0_3.xyz pc_msdm  -fq -r 0.007 -knn 5 -a 2				  //Forward 
			./PC-MSDM cube.xyz cube_Octree_perc0_3.xyz pc_msdm  -fq -r 0.007 -knn 5 -a 2 -i    			  //Backward

```

You can build the documentation using [Doxygen](http://www.doxygen.nl/).

---

