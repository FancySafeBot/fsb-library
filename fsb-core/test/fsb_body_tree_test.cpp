
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_body_tree.h"

TEST_SUITE_BEGIN("body_tree");

TEST_CASE("Add body to tree with errors" * doctest::description("[fsb_kinematics][fsb::BodyTree::add_body]"))
{
    SUBCASE("error: PARENT_BODY_NONEXISTENT")
    {
        // parent index input
        const size_t parent_body_index = 1U;
        // joint input
        const auto joint_type = fsb::JointType::FIXED;
        const fsb::Transform parent_joint_tr = fsb::transform_identity();
        // body input
        const fsb::MotionVector offset = {};
        const fsb::MassProps mass_props = {1.0, {}, {1.0, 1.0, 1.0, 0.0, 0.0, 0.0}};
        const fsb::PrincipalInertia principal_inertia = {};
        const fsb::Body body = {
            offset, mass_props, principal_inertia, 0U
        };
        // expected error value
        const size_t body_index_expected = 0U;
        const auto err_expected = fsb::BodyTreeError::PARENT_BODY_NONEXISTENT;
        // process
        auto err_actual = fsb::BodyTreeError::SUCCESS;
        fsb::BodyTree body_tree = {};
        const size_t body_index_actual = body_tree.add_body(parent_body_index, joint_type, parent_joint_tr, body, err_actual);
        // check
        REQUIRE(body_index_actual == body_index_expected);
        REQUIRE(err_actual == err_expected);
    }

    SUBCASE("error: MASS_ZERO")
    {
        // parent index input
        const size_t parent_body_index = 0U;
        // joint input
        const auto joint_type = fsb::JointType::REVOLUTE_Z;
        const fsb::Transform parent_joint_tr = fsb::transform_identity();
        // body input
        const fsb::MotionVector offset = {};
        const fsb::MassProps mass_props = {0.5 * FSB_TOL, {}, {1.0, 1.0, 1.0, 0.0, 0.0, 0.0}};
        const fsb::PrincipalInertia principal_inertia = {};
        const fsb::Body body = {
            offset, mass_props, principal_inertia, 0U
        };
        // expected error value
        const size_t body_index_expected = 0U;
        const auto err_expected = fsb::BodyTreeError::MASS_ZERO;
        // process
        auto err_actual = fsb::BodyTreeError::SUCCESS;
        fsb::BodyTree body_tree = {};
        const size_t body_index_actual = body_tree.add_body(parent_body_index, joint_type, parent_joint_tr, body, err_actual);
        // check
        REQUIRE(body_index_actual == body_index_expected);
        REQUIRE(err_actual == err_expected);
    }

    SUBCASE("error: INERTIA_ZERO")
    {
        // parent index input
        const size_t parent_body_index = 0U;
        // joint input
        const auto joint_type = fsb::JointType::REVOLUTE_Z;
        const fsb::Transform parent_joint_tr = fsb::transform_identity();
        // body input
        const fsb::MotionVector offset = {};
        const fsb::MassProps mass_props = {1.0, {}, {0.5 * FSB_TOL, 1.0, 1.0, 0.0, 0.0, 0.0}};
        const fsb::PrincipalInertia principal_inertia = {};
        const fsb::Body body = {
            offset, mass_props, principal_inertia, 0U
        };
        // expected error value
        const size_t body_index_expected = 0U;
        const auto err_expected = fsb::BodyTreeError::INERTIA_ZERO;
        // process
        auto err_actual = fsb::BodyTreeError::SUCCESS;
        fsb::BodyTree body_tree = {};
        const size_t body_index_actual = body_tree.add_body(parent_body_index, joint_type, parent_joint_tr, body, err_actual);
        // check
        REQUIRE(body_index_actual == body_index_expected);
        REQUIRE(err_actual == err_expected);
    }

    SUBCASE("error: MASS_ZERO_WITH_INERTIA")
    {
        // parent index input
        const size_t parent_body_index = 0U;
        // joint input
        const auto joint_type = fsb::JointType::FIXED;
        const fsb::Transform parent_joint_tr = fsb::transform_identity();
        // body input
        const fsb::MotionVector offset = {};
        const fsb::MassProps mass_props = {0.0, {}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.5 * FSB_TOL}};
        const fsb::PrincipalInertia principal_inertia = {};
        const fsb::Body body = {
            offset, mass_props, principal_inertia, 0U
        };
        // expected error value
        const size_t body_index_expected = 0U;
        const auto err_expected = fsb::BodyTreeError::MASS_ZERO_WITH_INERTIA;
        // process
        auto err_actual = fsb::BodyTreeError::SUCCESS;
        fsb::BodyTree body_tree = {};
        const size_t body_index_actual = body_tree.add_body(parent_body_index, joint_type, parent_joint_tr, body, err_actual);
        // check
        REQUIRE(body_index_actual == body_index_expected);
        REQUIRE(err_actual == err_expected);
    }

    SUBCASE("error: INERTIA_NOT_POS_DEF")
    {
        // parent index input
        const size_t parent_body_index = 0U;
        // joint input
        const auto joint_type = fsb::JointType::REVOLUTE_Z;
        const fsb::Transform parent_joint_tr = fsb::transform_identity();
        // body input
        const fsb::MotionVector offset = {};
        const fsb::MassProps mass_props = {1.0, {}, {1.0, -1.0, -1.0, 0.0, 0.0, 0.0}};
        const fsb::PrincipalInertia principal_inertia = {};
        const fsb::Body body = {
            offset, mass_props, principal_inertia, 0U
        };
        // expected error value
        const size_t body_index_expected = 0U;
        const auto err_expected = fsb::BodyTreeError::INERTIA_NOT_POS_DEF;
        // process
        auto err_actual = fsb::BodyTreeError::SUCCESS;
        fsb::BodyTree body_tree = {};
        const size_t body_index_actual = body_tree.add_body(parent_body_index, joint_type, parent_joint_tr, body, err_actual);
        // check
        REQUIRE(body_index_actual == body_index_expected);
        REQUIRE(err_actual == err_expected);
    }

    SUBCASE("error: MAX_BODIES_REACHED")
    {
        // parent index input
        const size_t parent_body_index = 0U;
        // joint input
        const auto joint_type = fsb::JointType::REVOLUTE_Z;
        const fsb::Transform parent_joint_tr = fsb::transform_identity();
        // body input
        const fsb::MotionVector offset = {};
        const fsb::MassProps mass_props = {1.0, {}, {1.0, 1.0, 1.0, 0.0, 0.0, 0.0}};
        const fsb::PrincipalInertia principal_inertia = {};
        const fsb::Body body = {
            offset, mass_props, principal_inertia, 0U
        };
        // expected error value
        const size_t body_index_expected = 0U;
        const auto err_expected = fsb::BodyTreeError::MAX_BODIES_REACHED;
        // process
        auto err_actual = fsb::BodyTreeError::SUCCESS;
        fsb::BodyTree body_tree = {};
        for (size_t index = 0; index < (fsb::MaxSize::bodies - 1U); ++index)
        {
            fsb::BodyTreeError err = fsb::BodyTreeError::SUCCESS;
            const size_t body_index = body_tree.add_body(parent_body_index, joint_type, parent_joint_tr, body, err);
            REQUIRE(body_index == (index + 1));
            REQUIRE(err == fsb::BodyTreeError::SUCCESS);
        }
        const size_t body_index_actual = body_tree.add_body(parent_body_index, joint_type, parent_joint_tr, body, err_actual);
        // check
        REQUIRE(body_index_actual == body_index_expected);
        REQUIRE(err_actual == err_expected);
    }

    // SUBCASE("error: MAX_JOINTS_REACHED")
    // {
    //     // parent index input
    //     const size_t parent_body_index = 0U;
    //     // joint input
    //     const auto joint_type = fsb::JointType::REVOLUTE_Z;
    //     const fsb::Transform parent_joint_tr = fsb::transform_identity();
    //     // body input
    //     const fsb::MotionVector offset = {};
    //     const fsb::MassProps mass_props = {1.0, {}, {1.0, 1.0, 1.0, 0.0, 0.0, 0.0}};
    //     const fsb::PrincipalInertia principal_inertia = {};
    //     const fsb::Body body = {
    //         offset, mass_props, principal_inertia, 0U
    //     };
    //     // expected error value
    //     const size_t body_index_expected = 0U;
    //     const auto err_expected = fsb::BodyTreeError::MAX_JOINTS_REACHED;
    //     // process
    //     auto err_actual = fsb::BodyTreeError::SUCCESS;
    //     fsb::BodyTree body_tree = {};
    //     for (size_t index = 0; index < (fsb::MaxSize::bodies - 1U); ++index)
    //     {
    //         fsb::BodyTreeError err = fsb::BodyTreeError::SUCCESS;
    //         const size_t body_index = body_tree.add_body(parent_body_index, joint_type, parent_joint_tr, body, err);
    //         REQUIRE(body_index == (index + 1));
    //         REQUIRE(err == fsb::BodyTreeError::SUCCESS);
    //     }
    //     const size_t body_index_actual = body_tree.add_body(parent_body_index, joint_type, parent_joint_tr, body, err_actual);
    //     // check
    //     REQUIRE(body_index_actual == body_index_expected);
    //     REQUIRE(err_actual == err_expected);
    // }

    SUBCASE("error: MAX_JOINT_COORDINATES_REACHED")
    {
        // parent index input
        const size_t parent_body_index = 0U;
        // joint input
        const auto joint_type = fsb::JointType::CARTESIAN;
        const fsb::Transform parent_joint_tr = fsb::transform_identity();
        // body input
        const fsb::MotionVector offset = {};
        const fsb::MassProps mass_props = {1.0, {}, {1.0, 1.0, 1.0, 0.0, 0.0, 0.0}};
        const fsb::PrincipalInertia principal_inertia = {};
        const fsb::Body body = {
            offset, mass_props, principal_inertia, 0U
        };
        // expected error value
        const auto err_expected = fsb::BodyTreeError::MAX_JOINT_COORDINATES_REACHED;
        // process
        auto err_actual = fsb::BodyTreeError::SUCCESS;
        fsb::BodyTree body_tree = {};
        while (err_actual == fsb::BodyTreeError::SUCCESS)
        {
            const size_t body_index = body_tree.add_body(parent_body_index, joint_type, parent_joint_tr, body, err_actual);
            (void)body_index;
        }
        // check
        REQUIRE(err_actual == err_expected);
    }

    SUBCASE("error: MAX_JOINT_DOFS_REACHED")
    {
        // parent index input
        const size_t parent_body_index = 0U;
        // joint input
        const auto joint_type = fsb::JointType::PLANAR;
        const fsb::Transform parent_joint_tr = fsb::transform_identity();
        // body input
        const fsb::MotionVector offset = {};
        const fsb::MassProps mass_props = {1.0, {}, {1.0, 1.0, 1.0, 0.0, 0.0, 0.0}};
        const fsb::PrincipalInertia principal_inertia = {};
        const fsb::Body body = {
            offset, mass_props, principal_inertia, 0U
        };
        // expected error value
        const auto err_expected = fsb::BodyTreeError::MAX_JOINT_DOFS_REACHED;
        // process
        auto err_actual = fsb::BodyTreeError::SUCCESS;
        fsb::BodyTree body_tree = {};
        while (err_actual == fsb::BodyTreeError::SUCCESS)
        {
            const size_t body_index = body_tree.add_body(parent_body_index, joint_type, parent_joint_tr, body, err_actual);
            (void)body_index;
        }
        // check
        REQUIRE(err_actual == err_expected);
    }

}

TEST_CASE("Add body to tree" * doctest::description("[fsb_kinematics][fsb::BodyTree::add_body]"))
{

    // work
    fsb::BodyTree body_tree = {};

    SUBCASE("Add revolute body")
    {
        // parent index input
        const size_t parent_body_index = 0U;
        // joint input
        const auto joint_type = fsb::JointType::REVOLUTE_Z;
        const fsb::Transform parent_joint_tr = fsb::transform_identity();
        // body input
        const fsb::MotionVector offset = {};
        const fsb::MassProps mass_props = {1.0, {}, {1.0, 1.0, 1.0, 0.0, 0.0, 0.0}};
        const fsb::PrincipalInertia principal_inertia = {};
        const fsb::Body body = {
            offset, mass_props, principal_inertia, 0U
        };
        // expected values
        const size_t body1_index_expected = 1U;
        const size_t body1_parent_joint_index_expected = 0U;
        const size_t joint1_parent_body_index_expected = 0U;
        const size_t joint1_child_body_index_expected = 1U;
        const size_t num_coordinates_expected = 1U;
        const size_t num_dofs_expected = 1U;
        // process
        auto err = fsb::BodyTreeError::SUCCESS;
        const size_t body1_index_actual = body_tree.add_body(parent_body_index, joint_type, parent_joint_tr, body, err);
        REQUIRE(err == fsb::BodyTreeError::SUCCESS);
        // check first body and joint
        REQUIRE(body1_index_actual == body1_index_expected);
        const fsb::Body body1 = body_tree.get_body(body1_index_actual, err);
        REQUIRE(err == fsb::BodyTreeError::SUCCESS);
        REQUIRE(body1.joint_index == body1_parent_joint_index_expected);
        const fsb::Joint joint1 = body_tree.get_joint(body1.joint_index, err);
        REQUIRE(err == fsb::BodyTreeError::SUCCESS);
        REQUIRE(joint1.parent_body_index == joint1_parent_body_index_expected);
        REQUIRE(joint1.child_body_index == joint1_child_body_index_expected);
        REQUIRE(body_tree.get_num_coordinates() == num_coordinates_expected);
        REQUIRE(body_tree.get_num_dofs() == num_dofs_expected);
    }

    SUBCASE("Add prismatic body")
    {
        // parent index input
        const size_t parent_body_index = 0U;
        // joint input
        const auto joint_type = fsb::JointType::REVOLUTE_Z;
        const fsb::Transform parent_joint_tr = fsb::transform_identity();
        // body input
        const fsb::MotionVector offset = {};
        const fsb::MassProps mass_props = {1.0, {}, {1.0, 1.0, 1.0, 0.0, 0.0, 0.0}};
        const fsb::PrincipalInertia principal_inertia = {};
        const fsb::Body body = {
            offset, mass_props, principal_inertia, 0U
        };
        // expected values
        const size_t body1_index_expected = 1U;
        const size_t body1_parent_joint_index_expected = 0U;
        const size_t joint1_parent_body_index_expected = 0U;
        const size_t joint1_child_body_index_expected = 1U;
        const size_t num_coordinates_expected = 1U;
        const size_t num_dofs_expected = 1U;
        // process
        auto err = fsb::BodyTreeError::SUCCESS;
        const size_t body1_index_actual = body_tree.add_body(parent_body_index, joint_type, parent_joint_tr, body, err);
        REQUIRE(err == fsb::BodyTreeError::SUCCESS);
        // check first body and joint
        REQUIRE(body1_index_actual == body1_index_expected);
        const fsb::Body body1 = body_tree.get_body(body1_index_actual, err);
        REQUIRE(err == fsb::BodyTreeError::SUCCESS);
        REQUIRE(body1.joint_index == body1_parent_joint_index_expected);
        const fsb::Joint joint1 = body_tree.get_joint(body1.joint_index, err);
        REQUIRE(err == fsb::BodyTreeError::SUCCESS);
        REQUIRE(joint1.parent_body_index == joint1_parent_body_index_expected);
        REQUIRE(joint1.child_body_index == joint1_child_body_index_expected);
        REQUIRE(body_tree.get_num_coordinates() == num_coordinates_expected);
        REQUIRE(body_tree.get_num_dofs() == num_dofs_expected);
    }

    SUBCASE("Add spherical body")
    {
        // parent index input
        const size_t parent_body_index = 0U;
        // joint input
        const auto joint_type = fsb::JointType::SPHERICAL;
        const fsb::Transform parent_joint_tr = fsb::transform_identity();
        // body input
        const fsb::MotionVector offset = {};
        const fsb::MassProps mass_props = {1.0, {}, {1.0, 1.0, 1.0, 0.0, 0.0, 0.0}};
        const fsb::PrincipalInertia principal_inertia = {};
        const fsb::Body body = {
            offset, mass_props, principal_inertia, 0U
        };
        // expected values
        const size_t body1_index_expected = 1U;
        const size_t body1_parent_joint_index_expected = 0U;
        const size_t joint1_parent_body_index_expected = 0U;
        const size_t joint1_child_body_index_expected = 1U;
        const size_t num_coordinates_expected = 4U;
        const size_t num_dofs_expected = 3U;
        // process
        auto err = fsb::BodyTreeError::SUCCESS;
        const size_t body1_index_actual = body_tree.add_body(parent_body_index, joint_type, parent_joint_tr, body, err);
        REQUIRE(err == fsb::BodyTreeError::SUCCESS);
        // check first body and joint
        REQUIRE(body1_index_actual == body1_index_expected);
        const fsb::Body body1 = body_tree.get_body(body1_index_actual, err);
        REQUIRE(err == fsb::BodyTreeError::SUCCESS);
        REQUIRE(body1.joint_index == body1_parent_joint_index_expected);
        const fsb::Joint joint1 = body_tree.get_joint(body1.joint_index, err);
        REQUIRE(err == fsb::BodyTreeError::SUCCESS);
        REQUIRE(joint1.parent_body_index == joint1_parent_body_index_expected);
        REQUIRE(joint1.child_body_index == joint1_child_body_index_expected);
        REQUIRE(body_tree.get_num_coordinates() == num_coordinates_expected);
        REQUIRE(body_tree.get_num_dofs() == num_dofs_expected);
    }

}

TEST_SUITE_END();
