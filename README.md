Ray Intersection without MVE Environment
-------------------------------------------------------------------------------

Notice
-------------------------------------------------------------------------------
The copyright is belong to the original author!


This repo is only a demo to show how to use rayint without MVE.

Build & run
-------------------------------------------------------------------------------
opencv is needed if you want to run the demo.

```
mkdir build
cd build
cmake ..
make
cd apps/raycast
./raycast
```

Result
-------------------------------------------------------------------------------

sphere obj
![sphere](https://github.com/Yannnnnnnnnnnn/rayint_nomve/blob/cuda/data/sphere_obj.png)

depth map
![depth](https://github.com/Yannnnnnnnnnnn/rayint_nomve/blob/cuda/data/depth.png)

-------------------------------------------------------------------------------
This project offers a header only acceleration structure library including
implementations for a BVH- and KD-Tree. Applications may include ray
intersection tests or nearest neighbor searches.

As an example application a simple raycasting app for
[MVE](https://github.com/simonfuhrmann/mve) scenes is provided.

The application was inspired by
[coldet](http://sourceforge.net/projects/coldet/).

License
-------------------------------------------------------------------------------
The software is licensed under the BSD 3-Clause license,
for more details see the LICENSE.txt file.
