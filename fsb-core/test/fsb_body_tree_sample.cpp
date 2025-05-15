
#include <cmath>
#include <doctest/doctest.h>
#include "fsb_body_tree_sample.h"

using namespace fsb;

BodyTree body_tree_sample_rpr(
    const Transform& joint1_tr, const Transform& joint2_tr, const Transform& joint3_tr,
    const MassProps& body1_massprops, const MassProps& body2_massprops, const MassProps& body3_massprops,
    size_t& last_body_index)
{
    auto     err = BodyTreeError::SUCCESS;
    BodyTree body_tree = {};

    MotionVector origin_offset = {};
    Body body1 = {origin_offset, body1_massprops, {}, 0U, false};
    Body body2 = {origin_offset, body2_massprops, {}, 0U, false};
    Body body3 = {origin_offset, body3_massprops, {}, 0U, true};

    const size_t body1_index
        = body_tree.add_body(BodyTree::base_index, JointType::REVOLUTE_Z, joint1_tr, body1, err);
    REQUIRE(err == BodyTreeError::SUCCESS);
    const size_t body2_index
        = body_tree.add_body(body1_index, JointType::PRISMATIC_Z, joint2_tr, body2, err);
    REQUIRE(err == BodyTreeError::SUCCESS);
    last_body_index = body_tree.add_body(body2_index, JointType::REVOLUTE_Z, joint3_tr, body3, err);
    REQUIRE(err == BodyTreeError::SUCCESS);

    return body_tree;
}

BodyTree create_panda_body_tree(size_t& ee_index)
{
    BodyTreeError err = BodyTreeError::SUCCESS;
    BodyTree body_tree = {};
    ee_index = 0U;

    // Add bodies and joints for the Panda robot
    Transform tr1 = transform_identity();
    tr1.translation.z = 0.333;
    size_t link_index
        = body_tree.add_massless_body(BodyTree::base_index, JointType::REVOLUTE_Z, tr1, {}, err);
    REQUIRE(err == BodyTreeError::SUCCESS);

    Transform tr2 = transform_identity();
    tr2.rotation = quat_rx(-M_PI_2);
    link_index = body_tree.add_massless_body(link_index, JointType::REVOLUTE_Z, tr2, {}, err);
    REQUIRE(err == BodyTreeError::SUCCESS);

    Transform tr3 = transform_identity();
    tr3.translation.y = -0.316;
    tr3.rotation = quat_rx(M_PI_2);
    link_index = body_tree.add_massless_body(link_index, JointType::REVOLUTE_Z, tr3, {}, err);
    REQUIRE(err == BodyTreeError::SUCCESS);

    Transform tr4 = transform_identity();
    tr4.translation.x = 0.0825;
    tr4.rotation = quat_rx(M_PI_2);
    link_index = body_tree.add_massless_body(link_index, JointType::REVOLUTE_Z, tr4, {}, err);
    REQUIRE(err == BodyTreeError::SUCCESS);

    Transform tr5 = transform_identity();
    tr5.translation.x = -0.0825;
    tr5.translation.y = 0.384;
    tr5.rotation = quat_rx(-M_PI_2);
    link_index = body_tree.add_massless_body(link_index, JointType::REVOLUTE_Z, tr5, {}, err);
    REQUIRE(err == BodyTreeError::SUCCESS);

    Transform tr6 = transform_identity();
    tr6.rotation = quat_rx(M_PI_2);
    link_index = body_tree.add_massless_body(link_index, JointType::REVOLUTE_Z, tr6, {}, err);
    REQUIRE(err == BodyTreeError::SUCCESS);

    Transform tr7 = {};
    tr7.translation.x = 0.088;
    tr7.rotation = quat_rx(M_PI_2);
    link_index = body_tree.add_massless_body(link_index, JointType::REVOLUTE_Z, tr7, {}, err);
    REQUIRE(err == BodyTreeError::SUCCESS);

    Transform tr8 = transform_identity();
    tr8.translation.z = 0.107;
    ee_index = body_tree.add_massless_body(link_index, JointType::FIXED, tr8, {}, err);
    REQUIRE(err == BodyTreeError::SUCCESS);

    return body_tree;
}

//BodyTree create_panda_body_tree_fk(size_t& ee_index)
//{
//    BodyTreeError err = BodyTreeError::SUCCESS;
//    BodyTree body_tree = {};
//    ee_index = 0U;
//
//    // Add bodies and joints for the Panda robot
//    Transform tr1 = transform_identity();
//    tr1.translation.z = 0.333;
//    size_t link_index
//        = body_tree.add_massless_body(BodyTree::base_index, JointType::REVOLUTE_Z, tr1, {}, err);
//    REQUIRE(err == BodyTreeError::SUCCESS);
//
//    Transform tr2 = transform_identity();
//    link_index = body_tree.add_massless_body(link_index, JointType::REVOLUTE_Y, tr2, {}, err);
//    REQUIRE(err == BodyTreeError::SUCCESS);
//
//    Transform tr3 = transform_identity();
//    tr3.translation.z = 0.316;
//    link_index = body_tree.add_massless_body(link_index, JointType::REVOLUTE_Z, tr3, {}, err);
//    REQUIRE(err == BodyTreeError::SUCCESS);
//
//    Transform tr4 = transform_identity();
//    tr4.translation.x = 0.0825;
//    link_index = body_tree.add_massless_body(link_index, JointType::REVOLUTE_Y, tr4, {}, err);
//    REQUIRE(err == BodyTreeError::SUCCESS);
//
//    Transform tr5 = transform_identity();
//    tr5.translation.x = -0.0825;
//    tr5.translation.z = 0.384;
//    link_index = body_tree.add_massless_body(link_index, JointType::REVOLUTE_Z, tr5, {}, err);
//    REQUIRE(err == BodyTreeError::SUCCESS);
//
//    Transform tr6 = transform_identity();
//    link_index = body_tree.add_massless_body(link_index, JointType::REVOLUTE_Y, tr6, {}, err);
//    REQUIRE(err == BodyTreeError::SUCCESS);
//
//    Transform tr7 = {};
//    tr7.translation.x = 0.088;
//    tr7.rotation = quat_rx(M_PI);
//    link_index = body_tree.add_massless_body(link_index, JointType::REVOLUTE_Z, tr7, {}, err);
//    REQUIRE(err == BodyTreeError::SUCCESS);
//
//    Transform tr8 = transform_identity();
//    tr8.translation.z = 0.107;
//    ee_index = body_tree.add_massless_body(link_index, JointType::FIXED, tr8, {}, err);
//    REQUIRE(err == BodyTreeError::SUCCESS);
//
//    return body_tree;
//}
//
//BodyTree create_panda_body_tree_jac(size_t& ee_index)
//{
//    BodyTreeError err = BodyTreeError::SUCCESS;
//    BodyTree body_tree = {};
//    ee_index = 0U;
//
//    // Add bodies and joints for the Panda robot
//    Transform tr1 = transform_identity();
//    tr1.translation.z = 0.333;
//    size_t link_index
//        = body_tree.add_massless_body(BodyTree::base_index, JointType::REVOLUTE_Z, tr1, {}, err);
//    REQUIRE(err == BodyTreeError::SUCCESS);
//
//    Transform tr2 = transform_identity();
//    link_index = body_tree.add_massless_body(link_index, JointType::REVOLUTE_Y, tr2, {}, err);
//    REQUIRE(err == BodyTreeError::SUCCESS);
//
//    Transform tr3 = transform_identity();
//    tr3.translation.z = 0.316;
//    link_index = body_tree.add_massless_body(link_index, JointType::REVOLUTE_Z, tr3, {}, err);
//    REQUIRE(err == BodyTreeError::SUCCESS);
//
//    Transform tr4 = transform_identity();
//    tr4.translation.x = 0.0825;
//    link_index = body_tree.add_massless_body(link_index, JointType::REVOLUTE_Y, tr4, {}, err);
//    REQUIRE(err == BodyTreeError::SUCCESS);
//
//    Transform tr5 = transform_identity();
//    tr5.translation.x = -0.0825;
//    tr5.translation.z = 0.384;
//    link_index = body_tree.add_massless_body(link_index, JointType::REVOLUTE_Z, tr5, {}, err);
//    REQUIRE(err == BodyTreeError::SUCCESS);
//
//    Transform tr6 = transform_identity();
//    link_index = body_tree.add_massless_body(link_index, JointType::REVOLUTE_Y, tr6, {}, err);
//    REQUIRE(err == BodyTreeError::SUCCESS);
//
//    Transform tr7 = {};
//    tr7.translation.x = 0.088;
//    tr7.rotation = quat_rx(M_PI_2);
//    link_index = body_tree.add_massless_body(link_index, JointType::FIXED, tr7, {}, err);
//    REQUIRE(err == BodyTreeError::SUCCESS);
//
//    Transform tr8 = transform_identity();
//    tr8.translation.z = 0.107;
//    ee_index = body_tree.add_massless_body(link_index, JointType::REVOLUTE_Z, tr8, {}, err);
//    REQUIRE(err == BodyTreeError::SUCCESS);
//
//    return body_tree;
//}
