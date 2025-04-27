#include "fsb_jacobian.h"

#include <iostream>
#include <cmath>
#include <fsb_body_tree.h>
#include <fsb_joint.h>
#include <fsb_kinematics.h>
#include <fsb_rotation.h>
#include <iomanip>

static fsb::BodyTree create_panda_body_tree(fsb::BodyTreeError& err, size_t& ee_index) {
    err = fsb::BodyTreeError::SUCCESS;
    fsb::BodyTree body_tree = {};
    ee_index = 0U;

	// │   0 │ panda_link0  │       │ BASE        │                                                │
	// │   1 │ panda_link1  │     0 │ panda_link0 │ SE3(0, 0, 0.333) ⊕ Rz(q0)                      │
	// │   2 │ panda_link2  │     1 │ panda_link1 │ SE3(-90°, -0°, 0°) ⊕ Rz(q1)                    │
	// │   3 │ panda_link3  │     2 │ panda_link2 │ SE3(0, -0.316, 0; 90°, -0°, 0°) ⊕ Rz(q2)       │
	// │   4 │ panda_link4  │     3 │ panda_link3 │ SE3(0.0825, 0, 0; 90°, -0°, 0°) ⊕ Rz(q3)       │
	// │   5 │ panda_link5  │     4 │ panda_link4 │ SE3(-0.0825, 0.384, 0; -90°, -0°, 0°) ⊕ Rz(q4) │
	// │   6 │ panda_link6  │     5 │ panda_link5 │ SE3(90°, -0°, 0°) ⊕ Rz(q5)                     │
	// │   7 │ panda_link7  │     6 │ panda_link6 │ SE3(0.088, 0, 0; 90°, -0°, 0°) ⊕ Rz(q6)        │
	// │   8 │ @panda_link8 │       │ panda_link7 │ SE3(0, 0, 0.107)

    // Add bodies and joints for the Panda robot
    fsb::Transform tr1 = fsb::transform_identity();
    tr1.translation.z = 0.333;
    size_t link_index = body_tree.add_massless_body(fsb::BodyTree::base_index, fsb::JointType::REVOLUTE_Z, tr1, {}, err);

    if (err == fsb::BodyTreeError::SUCCESS)
    {
        fsb::Transform tr2 = fsb::transform_identity();
        link_index = body_tree.add_massless_body(link_index, fsb::JointType::REVOLUTE_Y, tr2, {}, err);
    }

    if (err == fsb::BodyTreeError::SUCCESS)
    {
        fsb::Transform tr3 = fsb::transform_identity();
        tr3.translation.z = 0.316;
        link_index = body_tree.add_massless_body(link_index, fsb::JointType::REVOLUTE_Z, tr3, {}, err);
    }

    if (err == fsb::BodyTreeError::SUCCESS)
    {
        fsb::Transform tr4 = fsb::transform_identity();
        tr4.translation.x = 0.0825;
        link_index = body_tree.add_massless_body(link_index, fsb::JointType::REVOLUTE_Y, tr4, {}, err);
    }

    if (err == fsb::BodyTreeError::SUCCESS)
    {
        fsb::Transform tr5 = fsb::transform_identity();
        tr5.translation.x = -0.0825;
        tr5.translation.z = 0.384;
        link_index = body_tree.add_massless_body(link_index, fsb::JointType::REVOLUTE_Z, tr5, {}, err);
    }

    if (err == fsb::BodyTreeError::SUCCESS)
    {
        fsb::Transform tr6 = fsb::transform_identity();
        link_index = body_tree.add_massless_body(link_index, fsb::JointType::REVOLUTE_Y, tr6, {}, err);
    }

    if (err == fsb::BodyTreeError::SUCCESS)
    {
        fsb::Transform tr7 = {};
        tr7.translation.x = 0.088;
        tr7.rotation = fsb::quat_rx(M_PI);
        link_index = body_tree.add_massless_body(link_index, fsb::JointType::REVOLUTE_Z, tr7, {}, err);
    }

    if (err == fsb::BodyTreeError::SUCCESS)
    {
        fsb::Transform tr8 = fsb::transform_identity();
        tr8.translation.z = 0.107;
        ee_index = body_tree.add_massless_body(link_index, fsb::JointType::FIXED, tr8, {}, err);
    }

    return body_tree;
}

int main() {
    // Create the Panda robot's BodyTree
    fsb::BodyTreeError err = {};
    size_t ee_index = 0;
    fsb::BodyTree panda_tree = create_panda_body_tree(err, ee_index);
    if (err != fsb::BodyTreeError::SUCCESS) {
        std::cerr << "Failed to create panda body tree, fsb::BodyTreeError=" << static_cast<int>(err) << "\n";
        return EXIT_FAILURE;
    }

    const fsb::ForwardKinematicsOption opt = fsb::ForwardKinematicsOption::POSE;
    fsb::JointPva joint_pva = {};
    joint_pva.position.q[0] = 0.0;
    joint_pva.position.q[1] = -0.3;
    joint_pva.position.q[2] = 0.0;
    joint_pva.position.q[3] = 2.2;
    joint_pva.position.q[4] = 0.0;
    joint_pva.position.q[5] = -2.0;
    joint_pva.position.q[6] = 0.79;
    fsb::CartesianPva base_pva = {};
    base_pva.pose = fsb::transform_identity();

    fsb::BodyCartesianPva cartesian_pva = {};
    fsb::forward_kinematics(panda_tree, joint_pva, base_pva, opt, cartesian_pva);

    std::cout << std::fixed << std::setprecision(4) << std::setfill(' ');
    for (size_t ind  = 0; ind < panda_tree.get_num_bodies(); ++ind)
    {
        const fsb::Vec3 ee_pos = cartesian_pva.body[ind].pose.translation;
        const fsb::Mat3 ee_rot = fsb::quat_to_rot(cartesian_pva.body[ind].pose.rotation);
        std::cout << "\nBody " << ind << " transform:\n";
        std::cout << ee_rot.m00 << " " << ee_rot.m01 << " " << ee_rot.m02 << " " << ee_pos.x << "\n";
        std::cout << ee_rot.m10 << " " << ee_rot.m11 << " " << ee_rot.m12 << " " << ee_pos.y << "\n";
        std::cout << ee_rot.m20 << " " << ee_rot.m21 << " " << ee_rot.m22 << " " << ee_pos.z << "\n";
        std::cout << "0 0 0 1\n";
    }

    // jacobian
    fsb::Jacobian jac = {};
    const fsb::JacobianError jac_err = fsb::calculate_jacobian(ee_index, panda_tree, cartesian_pva, jac);
    if (jac_err == fsb::JacobianError::SUCCESS)
    {
        std::cout << "\nJacobian:\n";
        for (size_t row = 0; row < 6U; ++row)
        {
            for (size_t col = 0; col < panda_tree.get_num_dofs(); ++col)
            {
                std::cout << jac.j[fsb::jacobian_index(row, col)] << " ";
            }
            std::cout << "\n";
        }
    }

    return EXIT_SUCCESS;
}
