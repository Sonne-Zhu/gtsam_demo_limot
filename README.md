# gtsam_demo_limot
A gtsam demo contains the implementation of curve fitting and the definition and usage of ternary factors with node type SE(3).
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
