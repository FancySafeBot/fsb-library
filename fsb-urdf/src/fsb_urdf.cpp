
#include <cstddef>
#include <string>
#include <tinyxml2.h>
#include <list>
#include <vector>

#include "fsb_urdf.h"
#include "fsb_body.h"
#include "fsb_body_tree.h"
#include "fsb_urdf_body.h"
#include "fsb_urdf_joint.h"
#include "fsb_urdf_error.h"
#include "fsb_urdf_name_map.h"

namespace fsb::urdf
{

static UrdfError urdf_parse_robot_name(const std::string& fname, const tinyxml2::XMLElement& robot_xml, UrdfNameMap& urdf_names)
{
    UrdfError err = {};

    if (const auto& robot_name_item = robot_xml.Attribute("name"); robot_name_item == nullptr)
    {
        err = {UrdfErrorType::MISSING_NAME, "Missing robot name in URDF file: " + fname};
    }
    else
    {
        urdf_names.set_robot_name(std::string(robot_name_item));
    }
    return err;
}

static UrdfError urdf_parse_links_joints(const std::string& fname, const tinyxml2::XMLElement& robot_xml, UrdfNameMap& urdf_names,
                                         std::vector<UrdfLink>& links, std::vector<UrdfJoint>& joints)
{
    UrdfError err = {};

    // links
    if (!err.is_error())
    {
        const tinyxml2::XMLElement* link_xml = robot_xml.FirstChildElement("link");
        while (link_xml != nullptr)
        {
            const UrdfLink link = urdf_parse_link(fname, link_xml, err);
            if (!err.is_error())
            {
                if (const NameMapError name_err = urdf_names.add_body(links.size(), link.link_name);
                    name_err != NameMapError::SUCCESS)
                {
                    err = {UrdfErrorType::REPEATED_NAME, "Link name '" + link.link_name + "' is not unique in URDF file: " + fname};
                }
            }
            if (!err.is_error())
            {
                links.push_back(link);
                link_xml = link_xml->NextSiblingElement("link");
            }
        }
    }

    // joints
    if (!err.is_error())
    {
        const tinyxml2::XMLElement* joint_xml = robot_xml.FirstChildElement("joint");
        while ((!err.is_error()) && (joint_xml != nullptr))
        {
            const UrdfJoint joint = urdf_parse_joint(fname, joint_xml, err);
            if (!err.is_error())
            {
                if (const NameMapError name_err
                    = urdf_names.add_joint(joints.size(), joint.joint_name);
                    name_err != NameMapError::SUCCESS)
                {
                    err = {UrdfErrorType::REPEATED_NAME, "Joint name '" + joint.joint_name + "' is not unique in URDF file: " + fname};
                }
            }
            if (!err.is_error())
            {
                joints.push_back(joint);
                joint_xml = joint_xml->NextSiblingElement("joint");
            }
        }
    }
    return err;
}

static std::list<size_t> sort_joints(
    const std::string& fname, const UrdfNameMap& urdf_names, const std::vector<UrdfJoint>& joints, UrdfError& err)
{
    std::list<size_t> joints_sorted = {};

    for (size_t joint_index = 0U; joint_index < joints.size(); ++joint_index)
    {
        // current joint
        const auto& joint = joints[joint_index];
        // parent index from body tree map
        auto         name_err = NameMapError::SUCCESS;
        const size_t parent_index = urdf_names.get_body_index(joint.parent_name, name_err);
        // check parent and child exist
        if (name_err != NameMapError::SUCCESS)
        {
            err = {UrdfErrorType::JOINT_PARENT_BODY_NOT_FOUND, "Joint '" + joint.joint_name + "' parent body '" + joint.parent_name + "' not included in URDF file '" + fname + "'"};
        }
        if (!urdf_names.body_exists(joint.child_name))
        {
            err = {UrdfErrorType::JOINT_CHILD_BODY_NOT_FOUND, "Joint '" + joint.joint_name + "' child body '" + joint.child_name + "' not included in URDF file '" + fname + "'"};
        }

        // look in sorted joints for matching child index
        bool inserted_joint = false;
        for (auto sorted_iter = joints_sorted.begin(); sorted_iter != joints_sorted.end(); ++sorted_iter)
        {
            // get sorted child index
            const auto& sorted_joint = joints[*sorted_iter];
            // if test joint parent matches sorted child index
            if (const size_t sorted_child_index = urdf_names.get_body_index(sorted_joint.child_name, name_err);
                parent_index == sorted_child_index)
            {
                // insert joint with parent body after sorted joint child body
                joints_sorted.insert(std::next(sorted_iter), joint_index);
                inserted_joint = true;
                break;
            }
        }
        if (!inserted_joint)
        {
            joints_sorted.insert(joints_sorted.begin(), joint_index);
        }
    }

    return joints_sorted;
}

static BodyTree urdf_generate_tree(
    const std::string& fname, const UrdfNameMap& urdf_names, const std::vector<UrdfLink>& links,
    const std::vector<UrdfJoint>& joints, UrdfNameMap& body_tree_map, UrdfError& err)
{
    BodyTree body_tree = {};
    body_tree_map = {};

    if (!err.is_error() && !joints.empty())
    {
        const std::list<size_t> joints_sorted = sort_joints(fname, urdf_names, joints, err);

        if (!err.is_error())
        {
            // add base body name
            body_tree_map.add_body(BodyTree::base_index, joints[joints_sorted.front()].parent_name);
            // add bodies to tree
            size_t joint_count = 0U;
            for (const auto joint_index : joints_sorted)
            {
                const auto& joint = joints[joint_index];
                // parent from body tree map
                auto name_err = NameMapError::SUCCESS;
                const size_t body_tree_parent_index = body_tree_map.get_body_index(joint.parent_name, name_err);
                // child from URDF map
                const size_t child_body_index = urdf_names.get_body_index(joint.child_name, name_err);
                // Add body to tree
                Body child_body = {};
                child_body.mass_props = links[child_body_index].mass_props;
                child_body.origin_offset = links[child_body_index].origin_offset;
                child_body.principal_inertia = links[child_body_index].principal_inertia;
                auto tree_err = BodyTreeError::SUCCESS;
                const size_t body_tree_child_index = body_tree.add_body(body_tree_parent_index, joint.joint_type, joint.parent_child_transform, child_body, tree_err);
                if (tree_err != BodyTreeError::SUCCESS)
                {
                    err = {UrdfErrorType::BODY_TREE_ERROR, "Failed to add body '" + joint.child_name + "' for joint '" + + "' to body tree from URDF '" + fname + "'"};
                }
                else
                {
                    body_tree_map.add_joint(joint_count, joint.joint_name);
                    body_tree_map.add_body(body_tree_child_index, joint.child_name);
                    joint_count++;
                }
            }
        }
    }
    return body_tree;
}

static BodyTree parse_urdf(
    const std::string& urdf_string, UrdfNameMap& body_tree_map, UrdfError& err,
    const bool input_string_is_filename = false)
{
    err = {};
    BodyTree body_tree = {};

    // Robot xml element
    const tinyxml2::XMLElement* robot_xml = nullptr;
    const std::string fname = (input_string_is_filename ? urdf_string : "[urdf string]");

    // open urdf and get <robot> element
    tinyxml2::XMLDocument doc = {};
    if (const tinyxml2::XMLError xml_err
        = (input_string_is_filename ? doc.LoadFile(urdf_string.c_str()) :
                                      doc.Parse(urdf_string.c_str()));
        xml_err != tinyxml2::XMLError::XML_SUCCESS)
    {
        err = {UrdfErrorType::PARSE_ERROR, "Failed to parse URDF string with error: " + std::string(doc.ErrorStr()) + ""};
    }
    else
    {
        robot_xml = doc.FirstChildElement("robot");
        if (robot_xml == nullptr)
        {
            err = {UrdfErrorType::MISSING_ROBOT, "Missing <robot> element from URDF file: " + fname};
        }
    }

    // robot name
    if (!err.is_error() && (robot_xml != nullptr))
    {
        err = urdf_parse_robot_name(fname, (*robot_xml), body_tree_map);
    }

    // all links and joints
    UrdfNameMap name_map = {};
    std::vector<UrdfLink> links = {};
    std::vector<UrdfJoint> joints = {};
    if (!err.is_error() && (robot_xml != nullptr))
    {
        err = urdf_parse_links_joints(fname, (*robot_xml), name_map, links, joints);
    }

    // generate tree
    if (!err.is_error() && (robot_xml != nullptr))
    {
        body_tree = urdf_generate_tree(fname, name_map, links, joints, body_tree_map, err);
    }

    return body_tree;
}

BodyTree parse_urdf_string(const std::string& urdf_string, UrdfNameMap& name_map, UrdfError& err)
{
    return parse_urdf(urdf_string, name_map, err, false);
}

BodyTree parse_urdf_file(const std::string& urdf_filename, UrdfNameMap& name_map, UrdfError& err)
{
    return parse_urdf(urdf_filename, name_map, err, true);
}

} // namespace fsb
