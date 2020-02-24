## This Repo is for CVPR Course Work

This repository is copyright to Hongbinzhou

### 1. Camera Calibration (First homework) && 2. KLT

#### Depends && Requirements:
- [fmt](https://github.com/fmtlib/fmt) (A modern formatting library)  
- OpenCV 3.0 above  
- [glog](https://github.com/google/glog)
- cmake

### Install  

1. 进入代码文件路径下  
`cd $Repo_DIR`  
2. 创建`build`文件夹，并进入`build`文件夹  
`mkdir build && cd build`  
3. 编译生成  
`cmake .. && make`
4. 执行   
`./camera_calib $image_dir $images_num $parameter.xml`  
其中`$image_dir`为标定图片存储路径， `$images_num`一次随机加载图片数目， `$parameter.xml`输出标定参数文件名  
`./klt_node` 执行klt第二次作业， 需将对应video放置video文件夹内
