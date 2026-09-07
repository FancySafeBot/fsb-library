

#include <cstddef>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <argparse/argparse.hpp>

#include "fsb_joint.h"
#include "fsb_motion.h"
#include <fsb_urdf.h>
#include <fsb_urdf_name_map.h>
#include <fsb_body.h>
#include <fsb_body_tree.h>
#include <fsb_kinematics.h>
#include <fsb_urdf_error.h>
#include <fsb_urdf_utilities.h>

static std::vector<fsb::JointSpacePosition> csv_read(const std::string& csv_path)
{
    std::vector<fsb::JointSpacePosition> result = {};

    // Read joint data from CSV file
    std::ifstream infile(csv_path);
    if (!infile.is_open())
    {
        std::cerr << "Failed to open input file: " << csv_path << "\n";
        return result;
    }

    fsb::urdf::UrdfError parse_err = {};
    std::string line = {};
    while (std::getline(infile, line))
    {
        std::istringstream  input_stream(line);
        std::string         input_value = {};
        fsb::JointSpacePosition row_data = {};
        size_t joint_index = 0;
        while (std::getline(input_stream, input_value, ','))
        {
            const fsb::Real value = fsb::urdf::string_to_real(input_value, parse_err);
            if (parse_err.is_error())
            {
                std::cerr << "Error parsing value '" << input_value << "' in row " << result.size() + 1U
                          << ": " << parse_err.get_description() << "\n";
                break;
            }
            if (joint_index >= row_data.size())
            {
                std::cerr << "Too many values in row " << result.size() + 1U
                          << ": expected at most " << row_data.size() << " values\n";
                break;
            }
            row_data[joint_index] = value;
            ++joint_index;
        }
        if (!parse_err.is_error())
        {
            result.push_back(row_data);
        }
    }

    infile.close();
    return result;
}

static void csv_write(const std::string& output_csv, const std::vector<fsb::Transform>& data)
{
    std::ofstream outfile(output_csv);
    if (!outfile.is_open())
    {
        std::cerr << "Failed to open output file: " << output_csv << "\n";
        return;
    }

    // Header
    outfile << "translation_x,translation_y,translation_z,rotation_w,rotation_x,rotation_y,rotation_z\n";

    // setting precision to max precision for double
    outfile << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (const auto& row : data)
    {
        outfile << row.translation.x << "," << row.translation.y << "," << row.translation.z << ","
                << row.rotation.qw << "," << row.rotation.qx << "," << row.rotation.qy << "," << row.rotation.qz;
        outfile << "\n";
    }
    outfile.close();
}

static std::vector<fsb::Transform>
compute_kinematics(const size_t body_index, const fsb::BodyTree& body_tree, const std::vector<fsb::JointSpacePosition>& joint_data)
{
    // preallocate result vector
    auto result = std::vector<fsb::Transform>(joint_data.size());

    for (size_t row = 0; row < joint_data.size(); ++row)
    {
        // compute forward kinematics
        const fsb::CartesianPva base_pva = {};
        const fsb::JointPva joint_pva = {
            joint_data[row], {}, {}
        };
        fsb::BodyCartesianPva cartesian_pva = {};
        fsb::forward_kinematics(body_tree, joint_pva, base_pva, fsb::ForwardKinematicsOption::POSE, cartesian_pva);
        result[row] = cartesian_pva.body[body_index].pose;
    }

    return result;
}

int main(int argc, char* argv[])
{
    argparse::ArgumentParser program("fsb_example_forward_kinematics", "1.0");

    program.add_argument("urdf_path").help("Path to the URDF file").required();
    program.add_argument("body_name").help("Body URDF name").required();
    program.add_argument("joint_csv")
        .help("Path to the CSV file containing joint positions")
        .required();
    program.add_argument("output_csv")
        .help("Path to the output CSV file where end effector kinematics will be written")
        .required();

    try
    {
        program.parse_args(argc, argv);
    }
    catch (const std::runtime_error& err)
    {
        std::cerr << err.what() << "\n";
        std::cerr << program;
        return EXIT_FAILURE;
    }

    const auto urdf_path = program.get<std::string>("urdf_path");
    const auto body_name = program.get<std::string>("body_name");
    const auto joint_csv = program.get<std::string>("joint_csv");
    const auto output_csv = program.get<std::string>("output_csv");

    // parse URDF
    fsb::urdf::UrdfError   urdf_err = {};
    fsb::urdf::UrdfNameMap name_map = {};
    const fsb::BodyTree    body_tree = fsb::urdf::parse_urdf_file(urdf_path, name_map, urdf_err);
    if (urdf_err.is_error())
    {
        std::cerr << "Error parsing URDF file: " << urdf_err.get_description() << "\n";
        return EXIT_FAILURE;
    }

    fsb::urdf::NameMapError name_err = {};
    const size_t body_index = name_map.get_body_index(body_name, name_err);
    if (name_err != fsb::urdf::NameMapError::SUCCESS)
    {
        std::cerr << "Body name '" << body_name << "' not found in URDF file " << urdf_path << "\n";
        return EXIT_FAILURE;
    }

    const std::vector<fsb::JointSpacePosition> joint_data = csv_read(joint_csv);
    const std::vector<fsb::Transform> cartesian_data = compute_kinematics(body_index, body_tree, joint_data);
    csv_write(output_csv, cartesian_data);

    return EXIT_SUCCESS;
}
