rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
# TODO Replace with proper init
sleep infinity