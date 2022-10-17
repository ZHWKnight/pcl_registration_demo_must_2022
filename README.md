# pcl_registration_demo

## Brief introduction

A demo for point cloud registration, is a part of the presentation in Fundamentals of Artificial Intelligence (2022) MUST.

## Usage

```shell
## 1. Install dependencies
# for Ubuntu
sudo apt install build-essential cmake git libpcl-dev -y

## 2. clone source code
git clone

## 3. Complie
cd pcl_registration_demo_must_2022
mkdir build && cd build
cmake ..
make

## 4. Run
# parameter: ply model file, iteration every time.
./interactive_icp_registration monkey_head.ply 1
./normal_distributions_transform
```

## Reference materials

### ICP

[Paul J. Besl and Neil D. McKay. A method for registration of 3-D shapes.
IEEE Transactions on Pattern Analysis and Machine Intelligence, 14(2):
239 – 256, February 1992.](https://ieeexplore.ieee.org/document/121791)

[Yang Chen and Gérard Medioni. Object modelling by registration of
multiple range images. Image and Vision Computing, 10(3):145–155,
April 1992.](https://www.sciencedirect.com/science/article/abs/pii/026288569290066C?via%3Dihub)

### NDT

[Peter Biber and Wolfgang Straßer. The normal distributions transform:
A new approach to laser scan matching. In Proceedings of the IEEE In-
ternational Conference on Intelligent Robots and Systems (IROS), pages
2743–2748, Las Vegas, USA, October 2003](https://ieeexplore.ieee.org/document/1249285)

[Magnusson, Martin (2009). The three-dimensional normal-distributions transform: an efficient representation for registration, surface analysis, and loop detection (Ph.D.). Örebro universitet](https://www.researchgate.net/publication/229213868_The_Three-Dimensional_Normal-Distributions_Transform_---_an_Efficient_Representation_for_Registration_Surface_Analysis_and_Loop_Detection#:~:text=ThesisPDF%20Available-,The%20Three%2DDimensional%20Normal%2DDistributions%20Transform%20%2D%2D%2D%20an%20Efficient%20Representation,Surface%20Analysis%2C%20and%20Loop%20Detection)
