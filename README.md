# LSLAM

A LiDAR SLAM algorithm implemented by python

## Installation

    git clone https://github.com/scomup/lslam.git
    
## Usage:
    
    Eidt the lslam.py
	bagreader = BagReader(${your.bag}, ${scan topic}, ${odom topic}, ${start time}, ${start time})  
 	 - `{your.bag}`   : your bag file  
	 - `${scan topic}`: The scan topic name.  
 	 - `${odom topic}`: The odometry topic name.  
 	 - `${start time}`: The start time(secs) of your bag file  
     - `${end time}`  : The end time(secs) of your bag file  
    ./lslam.py

