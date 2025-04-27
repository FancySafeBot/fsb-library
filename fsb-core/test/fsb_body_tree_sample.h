#pragma once

#include "fsb_body_tree.h"

fsb::BodyTree body_tree_sample_rpr(const fsb::Transform& joint1_tr, const fsb::Transform& joint2_tr, const fsb::Transform& joint3_tr,
    size_t& body3_index);

fsb::BodyTree create_panda_body_tree(size_t& ee_index);
