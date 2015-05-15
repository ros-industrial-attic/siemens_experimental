# siemens_experimental

This repository contains following drivers and additional software for Siemens components used in ROS-PROFINET wrapper project:  
 
 - Linux SW for CP1616 V2.6 - original Siemens driver + IO Base, Layer2 and Serv library
 - CP1616 GSD files for integration into Simatic NET configuration
 - STEP 7 example projects for S7-1200 PLC

#CP1616 
[ROS-PROFINET wrapper][] utilizes communications module [Siemens CP1616][], which enables PGs/PCs equipped with a PCI slot to be connected to PROFINET IO. With existing Linux support, real-time capabilities and both IO Controller/IO device communications options, we consider this device an ideally suited candidate for connecting [ROS-Industrial][] systems to [PROFINET IO][]. For additional details about Linux support please refer to [DK16xx PN IO software][]. 
<p align="center">
<img src="https://github.com/durovsky/siemens_experimental/blob/master/rep/cp1616.jpeg" />
</p>

#S7-1200 PLC
For initial phase of development we decided to use new "TIA generation" PLC [S7-1200][]. The S7-1200 compact controller with built-in PROFINET as a standard interface enables us to experiment with both IO Controller/IO device options. Example projects for testing basic Profinet communication between CP1616 and S7-1200 will be available in Step7 folder soon.

<p align="center">
<img src="https://github.com/durovsky/siemens_experimental/blob/master/rep/s7-1200_sys.jpg" />
</p>

Image sources
- CP1616  https://mall.industry.siemens.com/collaterals/files/14/jpg/P_IK10_XX_00591i.jpg
- S7-1200 https://cache.industry.siemens.com/dl/files/057/57027057/img_23108/v1/s7-1200_sys.jpg

[ROS-PROFINET wrapper]: https://github.com/ros-industrial/ros_profinet_experimental
[ROS-Industrial]: http://www.ros.org/wiki/Industrial
[Siemens CP1616]: http://w3.siemens.com/mcms/industrial-communication/en/ie/system-interfacing/system-interfacing-pg-pc/cp1616/pages/cp1616.aspx
[S7-1200]: http://w3.siemens.com/mcms/programmable-logic-controller/en/basic-controller/s7-1200/pages/default.aspx
[DK16xx PN IO software]: http://w3.siemens.com/mcms/industrial-communication/en/ie/system-interfacing/system-interfacing-pg-pc/development-kit-dk16xx/Documents/PROFINET_DK_16xx_PN_IO_en_Web_mit_KF.pdf
[PROFINET IO]: http://us.profinet.com/technology/profinet/
