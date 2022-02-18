# SEE-CSOM
SEE-CSOM: Sharp-Edged and Efficient Continous Semantic Occupancy Mapping through Multi-entropy Kernel Inference 

<img src="https://github.com/BIT-DYN/SEE-CSOM/blob/master/image/CSM.png" width="250"><img src="https://github.com/BIT-DYN/SEE-CSOM/blob/master/image/BKI.png" width="250"><img src="https://github.com/BIT-DYN/SEE-CSOM/blob/master/image/DYN.png" width="250">

This is a novel continuous semantic mapping algorithm, which can complete dense but not thick semantic map reconstruction efficiently.

## Getting Started

### Building with catkin

```bash
catkin_ws/src$ git clone https://github.com/BIT-DYN/SEE-CSOM
catkin_ws/src$ cd ..
catkin_ws$ catkin_make
catkin_ws$ source ~/catkin_ws/devel/setup.bash
```

### Building using Intel C++ compiler (optional for better speed performance)
```bash
catkin_ws$ source /opt/intel/compilers_and_libraries/linux/bin/compilervars.sh intel64
catkin_ws$ catkin_make -DCMAKE_C_COMPILER=icc -DCMAKE_CXX_COMPILER=icpc
catkin_ws$ source ~/catkin_ws/devel/setup.bash
```

### Running the Demo

```bash
$ roslaunch see_csom toy_example_node.launch
```

### Running kitti (change the kitti dataset loction in launch)
```bash
$ roslaunch see_csom kitti_node.launch
```


<img src="https://github.com/BIT-DYN/SEE-CSOM/blob/master/image/25truth.png" width="1000">
<img src="https://github.com/BIT-DYN/SEE-CSOM/blob/master/image/25.png" width="1000">

### Running SemanticKITTI 
```bash
$ roslaunch see_csom semantickitti_quan.launch
```
<img src="https://github.com/BIT-DYN/SEE-CSOM/blob/master/image/semantickitti.png" width="3000">


### Running Stanford
```bash
$ roslaunch see_csom stanford_node.launch
```
<img src="https://github.com/BIT-DYN/SEE-CSOM/blob/master/image/stanford.png" width="3000">

