# Example Forward Kinematics

Load a Universal Robots UR5 urdf file and an input CSV file with a list of joint positions. Given a body name, output a CSV file containing the position and orientation (quaternions) of that body. For example:

```sh
cmake --build --preset "fsb-examples"
cd ./build/debug/examples/forward_kinematics_ur5
./fsb_forward_kinematics_ur5 data/ur5.urdf ee_link data/joint_data.csv output_ee.csv
```

The output file `output_ee.csv` contents are:

```sh
cat ./output_ee.csv
0.81725000000000003,0.19145000000000001,-0.00549099999999980,0.00000000000000014,0.70710678118481640,0.70710678118827885,0.00000000000000014
0.88898649185143896,0.17410781590400914,0.01270501845016872,0.01756923413687131,0.95422918329845485,0.29193768615743676,0.06253299252754330
```

Help output:

```sh
./fsb_forward_kinematics_ur5 --help
Usage: fsb_example_forward_kinematics [--help] [--version] urdf_path body_name joint_csv output_csv

Positional arguments:
  urdf_path      Path to the URDF file [required]
  body_name      Body URDF name [required]
  joint_csv      Path to the CSV file containing joint positions [required]
  output_csv     Path to the output CSV file where end effector kinematics will be written [required]

Optional arguments:
  -h, --help     shows help message and exits
  -v, --version  prints version information and exits
```
