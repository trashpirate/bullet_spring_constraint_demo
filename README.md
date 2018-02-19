# bullet_spring_constraint_demo
This Bullet example demonstrates an issue of the btGeneric6Dof**Spring2**Constraint regarding the relationship between pendulum length and frequency. The simulation shows 6 pendulums oscillating about a point on the vertical axis through the origin. The 3 upper pendulums (pendulum 1-3) are constrained by btGeneric6Dof**Spring**Constraint, while the 3 lower ones (pendulum 4-6) are constrained by btGeneric6Dof**Spring2**Constraint. It is obvious that pendulum 1-3 oscillate at different frequencies, the shortest with the highest and the longest with the lowest frequency. This is physically correct. However, pendulum 4-6 oscillate at the same frequency indicating a bug in the btGeneric6Dof**Spring2**Constraint resulting in incorrectly simulated dynamics.

## Installation Instructions:

1. Clone bullet from https://github.com/bulletphysics/bullet3

2. Follow installation instructions on http://bulletphysics.org/mediawiki-1.5.8/index.php/Installation or on Linux do:
```
	cd path/to/bullet
	mkdir bullet_build
	cd bullet_build
	cmake .. -G "Unix Makefiles" -DINSTALL_LIBS=ON -DBUILD_SHARED_LIBS=ON
	make -j4
	sudo make install
```

4. Clone bullet_spring_constraint_demo

5. Compile bullet_spring_constraint_demo with:
```
	cd your/path/to/bullet_spring_constraint_demo
	mkdir build
	cd build
	cmake -DBULLET_SOURCE_DIR:STRING=/your/path/to/bullet/source/directory -DBULLET_BUILD_DIR:STRING=/your/path/to/bullet/build/directory ..
	make
```
6. Run `AppSpringConstraintDemo`
 


 
