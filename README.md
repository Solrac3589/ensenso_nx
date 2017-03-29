
### Overview
This repository holds code of a [ROS](http://www.ros.org) package for point cloud and depth image acquisition with  [EnsensoNx](https://en.ids-imaging.com/ensenso-stereo-3d-camera.html) 3D cameras. It is basically a ROS wrapper of the low-level API provided by [IDS](https://en.ids-imaging.com), the manufacturer of the camera. The ROS node can be configured to act as a server or as publisher at a given rate. 

![Camera and cloud at rviz](media/20160801_ensenso_ros_cropped.png)

### Dependencies
The package has been tested with the following dependencies:
* Ubuntu 16.04
* CMake + gcc
* [ROS Kinetic](http://wiki.ros.org/kinetic)
* [Point Cloud Library v1.7](http://www.pointclouds.org/) (shipped with ROS Kinetic)
* [OpenCV v3.1](http://www.opencv.org/) (shipped with ROS Kinetic)
* UEYE driver (camera interface from manufacturer IDS)
* Ensenso SDK (propietary library from manufacturer IDS)

To install the ueye driver and tools:

1. Download the UEYE from the [IDS website](http://www.ensenso.com/support/sdk-download/) (file uEye_4.80.2_Linux_64.tgz)
2. Uncompress, move to the folder and run the script (ethernet or usb as needed)
```shell 
$ sudo sh ./ueyesdk-setup-4.80-eth-amd64.gz.run
```

To install Ensenso SDK dependency:

1. Download the SDK from the [IDS website](http://www.ensenso.com/support/sdk-download/) (file EnsensoSDK-1.3.180-x64.deb)
2. Install it with
```shell 
$ sudo dpkg -i EnsensoSDK-1.3.180-x64.deb
```
3. The installation above does not copy the file FindEnsenso.cmake to the CMake Modules folder, so we have to do that manually:
```shell 
$ to do with a single command ....
```


### Download and Build This ROS package
Download to your ROS workspace /src, with the command:
```shell
$ git clone https://github.com/beta-robots/ensenso_nx.git
```
and from your ROS workspace, build it with:
```shell
$ catkin_make --only-pkg-with-deps ensenso_nx
```

### Camera Operation
Start the ueye daemon (in case it didn't started on system boot):
```shell
$ sudo /etc/init.d/ueyeethdrc start
```
Check the camera is there with the IDS application nxView
```shell
$ nxView
```
Run the node (by default a rviz window will appear)
```shell
$ roslaunch ensenso_nx ensenso_nx.launch 
```
If you are operating the node in run mode "SERVER", from another terminal please request a Point Cloud capture with a given exposure value: 
```shell
$ rosservice call /ensenso_server "exposure: 30"
```

### Troubleshooting
Check that the IP of the computer and that of the camera are in the same network. To manually set the IP of the camera, or manage other configurations, go to: 
```shell
$ ueyecameramanager
```

In some cases you have to manually edit the file **/usr/local/share/ueye/ueyeethd/ueyeethd.conf** with sudo privileges and set the interface and port. For instance, if you are using eth0 interface, the file should look like:
```shell
;ueye configuration file

[Parameters]
Interfaces = eth0

[eth0]
Port_Base = 50000
```

Sometimes you have to stop the ueye daemon an then start it again 
```shell
$ sudo /etc/init.d/ueyeethdrc stop
$ sudo /etc/init.d/ueyeethdrc start
```




