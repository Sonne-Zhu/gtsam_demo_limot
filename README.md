# gtsam_demo_limot
This gtsam demo contains the implementation of curve fitting and the definition and usage of ternary factors with node type SE(3). 
It can illustrate and verify that gtsam is used correctly in our work [LIMOT](https://github.com/tiev-tongji/LIMOT).
* Curve fitting: In LIMOT, we use curve fitting to predict the object's position for object tracking. The optimization problem of curve fitting is solved by gtsam.
* Ternary factor: Ternary factors are used to construct the collaborative optimization factor graph in LIMOT.
In gtsam, the partial derivatives of the error function with respect to each node variable are required to customize a factor.
The relevant derivation procedure for the ternary factor with node type SE(3) can be found in /math/math.pdf.
The rest of the factors in the collaborative optimization factor graph can be used directly in gtsam.
## Dependency
* [gtsam]([https://gtsam.org/get_started/](https://github.com/borglab/gtsam/releases/tag/4.0.3))(Georgia Tech Smoothing and Mapping library)
  ```
  git clone -b 4.0.3 https://github.com/borglab/gtsam && cd gtsam
  mkdir build && cd build
  cmake ..
  sudo make install
  ```
## Run
```
git clone git@github.com:Sonne-Zhu/gtsam_demo_limot.git
cd gtsam_demo_limot/
mkdir build && cd build
cmake ..
make -j
./ternaryfactors_test     # run ternaryfactors_test
./curve_fit               # run curve_fit
