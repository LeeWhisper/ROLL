#!/bin/sh
if [ $# != 2 ]; then
echo "usage: $0 [bag_dir] [bag_name]"
exit;
fi
echo ${PWD}
echo "bag path: $1/$2"
bag_dir=$1 # no arbitrary space
bag_nameI=$2
bag_path="${bag_dir} ${bag_nameI}"

# rosparam load ../config/velydyne_nclt.yaml # not working
# rosparam load /home/haisenberg/Documents/ROLL/src/FAST_LIO/config/velodyne_nclt.yaml
cd /home/haisenberg/Documents/ROLL/devel/lib/fast_lio/
# rosrun ./fastlio_mapping bag_name:=${bag_nameI//./_} # replace . with _: bad substitution , why??

# cd ../../../src/FAST_LIO
# rviz -d rviz_cfg/loam_livox.rviz

rosbag play ${bag_path} --clock
sleep 2s
./fastlio_mapping
echo "end bag: ${bag_path}"
rosnode kill -a







