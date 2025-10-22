pushd `pwd` > /dev/null
cd `dirname $0`
cd ..
clear
echo "Working Path: "`pwd`
colcon build \
  --symlink-install \
  --cmake-args \
    -DCMAKE_CXX_FLAGS="-O3" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_BUILD_TYPE=Release \
    -DROS_EDITION=ROS2 \
  --parallel-workers 16 \
  --packages-select laser_to_foot
#   --packages-skip livox_ros_driver2\
#   --event-handlers console_direct+ \
popd > /dev/null