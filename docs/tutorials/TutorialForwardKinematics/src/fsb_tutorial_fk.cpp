#include <iostream>
#include "fsb_urdf.h"
#include "fsb_compute_kinematics.h"

int main(void) {
    std::cout <<
        "FancySafeBot Tutorial for forward kinematics with a UR5 Manipulator\n";

    // parse URDF
    const std::string urdf_path = "data/ur5.urdf";
    fsb::urdf::UrdfError   urdf_err = {};
    fsb::urdf::UrdfNameMap name_map = {};
    const fsb::BodyTree    body_tree = fsb::urdf::parse_urdf_file(urdf_path, name_map, urdf_err);
    if (urdf_err.is_error())
    {
        std::cerr << "Error parsing URDF file: " << urdf_err.get_description() << "\n";
        return EXIT_FAILURE;
    }

    // Initialize kinematics object
    fsb::ComputeKinematics kinematics = {};
    kinematics.initialize(body_tree);

    // Input joint input position
    const fsb::JointSpacePosition joint_position = {};
    // compute position forward kinematics
    fsb::BodyCartesianPva cartesian_pva = {};
    kinematics.compute_forward_kinematics_pose(joint_position, cartesian_pva);

    const std::string end_effector_name = "ee_link";
    fsb::urdf::NameMapError name_err = {};
    const size_t ee_index = name_map.get_body_index(end_effector_name, name_err);
    if (name_err != fsb::urdf::NameMapError::SUCCESS)
    {
        std::cerr << "Body name '" << end_effector_name << "' not found in URDF file " << urdf_path << "\n";
        return EXIT_FAILURE;
    }

    // Print the pose of the end effector
    const fsb::Transform& ee_pose = cartesian_pva.body[ee_index].pose;
    std::cout << "Input Joint Position:\n";
    for (size_t i = 0; i < body_tree.get_num_coordinates(); ++i) {
        std::cout << "  q" << i << ": " << joint_position.q[i] << "\n";
    }
    std::cout << "End effector pose:\n";
    std::cout << "  Position: (" << ee_pose.translation.x << ", " << ee_pose.translation.y << ", " << ee_pose.translation.z << ")\n";
    std::cout << "  Orientation: (" << ee_pose.rotation.qx << ", " << ee_pose.rotation.qy << ", " << ee_pose.rotation.qz << ", " << ee_pose.rotation.qw << ")\n";

    return EXIT_SUCCESS;
}
