

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

#include <fsb_urdf.h>
#include <fsb_urdf_name_map.h>
#include <fsb_body.h>
#include <fsb_body_tree.h>
#include <fsb_kinematics.h>
#include <fsb_compute_kinematics.h>
#include <fsb_urdf_error.h>
#include <fsb_urdf_utilities.h>

static std::vector<std::vector<double>> csv_read(const std::string& csv_path)
{
    std::vector<std::vector<double>> result = {};

    // Read joint data from CSV file
    std::ifstream infile(csv_path);
    if (!infile.is_open())
    {
        std::cerr << "Failed to open input file: " << csv_path << "\n";
        return result;
    }

    fsb::urdf::UrdfError urdf_err = {};
    std::string line = {};
    while (std::getline(infile, line))
    {
        std::istringstream  input_stream(line);
        std::string         input_value = {};
        std::vector<double> row_data = {};
        while (std::getline(input_stream, input_value, ','))
        {
            const fsb::real_t value = fsb::urdf::string_to_real(input_value, urdf_err);
            if (!urdf_err.is_error())
            {
                row_data.push_back(value);
            }
            else
            {
                std::cerr << "Error converting row  " << result.size() + 1U
                          << " and column " << row_data.size() + 1 << ": "
                          << urdf_err.get_description() << "\n";
                break;
            }
        }
        if (!urdf_err.is_error())
        {
            result.push_back(row_data);
        }
    }

    infile.close();
    return result;
}

static void csv_write(const std::string& output_csv, const std::vector<std::vector<double>>& data)
{
    std::ofstream outfile(output_csv);
    if (!outfile.is_open())
    {
        std::cerr << "Failed to open output file: " << output_csv << "\n";
        return;
    }

    // setting precision to max precision for double
    outfile << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10);

    for (const auto& row : data)
    {
        if (!row.empty())
        {
            for (size_t i = 0; i < (row.size() - 1U); ++i)
            {
                outfile << row[i] << ",";
            }
            outfile << row.back();
        }
        outfile << "\n";
    }
    outfile.close();
}

static std::vector<std::vector<double>>
compute_kinematics(const size_t body_index, const fsb::ComputeKinematics& kinematics, const std::vector<std::vector<double>>& joint_data)
{
    // preallocate result vector
    auto result = std::vector<std::vector<double>>(joint_data.size(), std::vector<double>(7U, 0.0));

    const size_t num_joint_coords = kinematics.get_num_coordinates();
    const size_t num_bodies = kinematics.get_num_bodies();
    if (body_index >= num_bodies)
    {
        std::cerr << "Invalid body index: " << body_index << "\n";
        return result;
    }

    for (size_t row = 0; row < joint_data.size(); ++row)
    {
        const auto& joint = joint_data[row];
        // Assign joint position from current row of csv data
        fsb::JointSpacePosition joint_position = {};
        for (size_t i = 0; (i < num_joint_coords) && (i < joint.size()); ++i)
        {
            joint_position.q[i] = joint[i];
        }
        // compute forward kinematics
        fsb::BodyCartesianPva cartesian_pva = {};
        kinematics.compute_forward_kinematics_pose(joint_position, cartesian_pva);
        const fsb::Transform& body_pose = cartesian_pva.body[body_index].pose;
        // assign result to output data row
        result[row][0] = body_pose.translation.x;
        result[row][1] = body_pose.translation.y;
        result[row][2] = body_pose.translation.z;
        result[row][3] = body_pose.rotation.qw;
        result[row][4] = body_pose.rotation.qx;
        result[row][5] = body_pose.rotation.qy;
        result[row][6] = body_pose.rotation.qz;
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

    fsb::ComputeKinematics kinematics = {};
    const fsb::ComputeKinematicsError kin_err = kinematics.initialize(body_tree);
    if (kin_err != fsb::ComputeKinematicsError::SUCCESS)
    {
        std::cerr << "Error initializing kinematics computation\n";
        return EXIT_FAILURE;
    }

    const std::vector<std::vector<double>> joint_data = csv_read(joint_csv);
    const std::vector<std::vector<double>> cartesian_data = compute_kinematics(body_index, kinematics, joint_data);
    csv_write(output_csv, cartesian_data);

    return EXIT_SUCCESS;
}
