#!/bin/sh

# bash sh difference ???
if [ $# != 1 ]; then
echo "usage: $0 [bag_dir]"
exit;
fi

source ~/.bashrc 

echo ${ROS_PKG_DIR}
echo "bag directory: $1"
bag_dir=$1 # no arbitrary space
baglists=$(ls ${bag_dir}) #() {} difference?
for bag in ${baglists}
do
    # roscore
    sleep 5s
    echo "start bag: ${bag}"
    roslaunch fastlio mapping_nclt.launch bag_name:=${bag}
    rosbag play ${bag_dir}/${bag} --clock
    echo "end bag: ${bag}"
    rosnode kill -a
done







