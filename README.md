# icpcuda_helloworld

Requires CUDA, [Pangolin](https://github.com/theobslhc/Pangolin), [Eigen](https://github.com/stevenlovegrove/eigen), [Sophus](https://github.com/stevenlovegrove/Sophus), [libfreenect2](https://github.com/OpenKinect/libfreenect2), [Opt](https://github.com/niessner/Opt) and [ICPCUDA](https://github.com/theobslhc/ICPCUDA).

Build instructions :

```bash
git clone https://github.com/theobslhc/icpcuda_helloworld.git
cd icpcuda_helloworld

mkdir build
cd build

cmake ..
make
```

Run like;

```bash
./ICPHW ~/Desktop/rgbd_dataset_freiburg1_desk/ -v
```
See [ICPCUDA](https://github.com/theobslhc/ICPCUDA) repository for details
