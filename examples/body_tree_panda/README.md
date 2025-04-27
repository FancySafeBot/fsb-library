# Example Forward Kinematics

Run forward kinematics

```sh
cmake --build --preset "fsb-examples"
cd ./build/debug/examples/fsb_forward_kinematics_ur5
./fsb_forward_kinematics_ur5 data/ur5.urdf data/joint_data.csv output_ee.csv
```
