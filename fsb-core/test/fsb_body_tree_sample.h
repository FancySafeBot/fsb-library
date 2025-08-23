#pragma once

#include "fsb_body_tree.h"

fsb::BodyTree body_tree_sample_rpr(
    const fsb::Transform& joint1_tr, const fsb::Transform& joint2_tr, const fsb::Transform& joint3_tr,
    const fsb::MassProps& body1_massprops, const fsb::MassProps& body2_massprops, const fsb::MassProps& body3_massprops,
    size_t& last_body_index);

fsb::BodyTree body_tree_sample_srs(
    const fsb::Transform& joint1_tr, const fsb::Transform& joint2_tr, const fsb::Transform& joint3_tr,
    const fsb::MassProps& body1_massprops, const fsb::MassProps& body2_massprops, const fsb::MassProps& body3_massprops,
    size_t& last_body_index);

// ┌──────┬──────────────┬───────┬─────────────┬────────────────────────────────────────────────┐
// │ link │     link     │ joint │   parent    │              ETS: parent to link               │
// ├──────┼──────────────┼───────┼─────────────┼────────────────────────────────────────────────┤
// │    0 │ panda_link0  │       │ BASE        │ SE3()                                          │
// │    1 │ panda_link1  │     0 │ panda_link0 │ SE3(0, 0, 0.333) ⊕ Rz(q0)                      │
// │    2 │ panda_link2  │     1 │ panda_link1 │ SE3(-90°, -0°, 0°) ⊕ Rz(q1)                    │
// │    3 │ panda_link3  │     2 │ panda_link2 │ SE3(0, -0.316, 0; 90°, -0°, 0°) ⊕ Rz(q2)       │
// │    4 │ panda_link4  │     3 │ panda_link3 │ SE3(0.0825, 0, 0; 90°, -0°, 0°) ⊕ Rz(q3)       │
// │    5 │ panda_link5  │     4 │ panda_link4 │ SE3(-0.0825, 0.384, 0; -90°, -0°, 0°) ⊕ Rz(q4) │
// │    6 │ panda_link6  │     5 │ panda_link5 │ SE3(90°, -0°, 0°) ⊕ Rz(q5)                     │
// │    7 │ panda_link7  │     6 │ panda_link6 │ SE3(0.088, 0, 0; 90°, -0°, 0°) ⊕ Rz(q6)        │
// │    8 │ @panda_link8 │       │ panda_link7 │ SE3(0, 0, 0.107)                               │
// └──────┴──────────────┴───────┴─────────────┴────────────────────────────────────────────────┘

fsb::BodyTree create_panda_body_tree(size_t& ee_index);

// fsb::BodyTree create_ur5_body_tree(size_t& ee_index);
