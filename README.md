# icpcuda_helloworld

Requires CUDA, [Pangolin](https://github.com/stevenlovegrove/Pangolin), [Eigen](https://github.com/stevenlovegrove/eigen) and [Sophus](https://github.com/stevenlovegrove/Sophus), and [ICPCUDA](https://github.com/theobslhc/ICPCUDA).

Build instructions :

```bash
https://github.com/theobslhc/icpcuda_helloworld.git
cd ICPCUDA

mkdir build
cd build

cmake ../src
make
```

Run like;

```bash
./ICPHW ~/Desktop/rgbd_dataset_freiburg1_desk/ -v
```
See [ICPCUDA](https://github.com/theobslhc/ICPCUDA) repository for details
