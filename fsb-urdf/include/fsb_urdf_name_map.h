
#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>

namespace fsb::urdf
{

/**
 * @addtogroup UrdfTopic
 * @{
 */

/**
 * @brief Name map errors
 */
enum class NameMapError
{
    /** @brief No error */
    SUCCESS,
    /** @brief Joint or body name not found */
    NOT_FOUND,
    /** @brief Joint or body already exists */
    EXISTS
};

/**
 * @brief Name map relating names to object indices
 */
class NameMap
{
public:
    NameMap() = default;

    /**
     * @brief Add named object with index
     * @param index Index of named object
     * @param name Name of object
     * @return Result of operation
     */
    NameMapError add(size_t index, const std::string& name);

    /**
     * @brief Get index of named object.
     * @param name Name of object
     * @param err Error status of operation
     * @return Index of named object
     */
    size_t get_index(const std::string& name, NameMapError& err) const;

    /**
     * @brief Get name of indexed object
     * @param index Index
     * @param err Error status of operation
     * @return Name of object at index
     */
    std::string get_name(size_t index, NameMapError& err) const;

    /**
     * @brief Check if name exists in map
     * @param name Name of object in map
     * @return @c true if name exists in map, @c false otherwise
     */
    [[nodiscard]] bool name_exists(const std::string& name) const
    {
        const auto iter = m_index_map.find(name);
        return (iter != m_index_map.end());
    }

    /**
     * @brief Get number of name-index pairs in map
     * @return Number of objects in map
     */
    [[nodiscard]] size_t size() const
    {
        return m_index_map.size();
    }

private:
    std::unordered_map<size_t, std::string> m_name_map;
    std::unordered_map<std::string, size_t> m_index_map;
};

/** @brief Name-index map of joints and bodies in rigid body tree */
class UrdfNameMap
{
public:
    UrdfNameMap() = default;

    /**
     * @brief Set robot name
     * @param name Robot name
     */
    void set_robot_name(const std::string& name)
    {
        m_robot_name = name;
    }

    /**
     *
     * @param index Body index in tree
     * @param name Body name
     * @return Error result of operation
     */
    NameMapError add_body(const size_t index, const std::string& name)
    {
        return m_bodies.add(index, name);
    }

    /**
     *
     * @param index Joint index in tree
     * @param name Joint name
     * @return Error result of operation
     */
    NameMapError add_joint(const size_t index, const std::string& name)
    {
        return m_joints.add(index, name);
    }

    /**
     * @brief Get index of body in tree
     * @param name Body name
     * @param err Error status
     * @return Index of body in body tree
     */
    size_t get_body_index(const std::string& name, NameMapError& err) const
    {
        return m_bodies.get_index(name, err);
    }

    /**
     * @brief Get index of joint in body tree
     *
     * @param name Joint name
     * @param err Error status
     * @return Index of joint in body tree
     */
    size_t get_joint_index(const std::string& name, NameMapError& err) const
    {
        return m_joints.get_index(name, err);
    }

    /**
     * @brief Check if body name is set in map
     * @param name Name of body to check
     * @return @c true if body exists in map
     */
    [[nodiscard]] bool body_exists(const std::string& name) const
    {
        return m_bodies.name_exists(name);
    }

    /**
     * @brief Check if joint name is set in map
     * @param name Name of joint to check
     * @return @c true if joint exists in map
     */
    [[nodiscard]] bool joint_exists(const std::string& name) const
    {
        return m_joints.name_exists(name);
    }

    /**
     * @brief Get number of bodies in map.
     * @return Number of bodies in map
     */
    [[nodiscard]] size_t get_num_bodies() const
    {
        return m_bodies.size();
    }

    /**
     * @brief Get number of joints in map
     * @return Number of joints in map
     */
    [[nodiscard]] size_t get_num_joints() const
    {
        return m_joints.size();
    }

private:
    std::string m_robot_name;
    NameMap     m_bodies;
    NameMap     m_joints;
};

/**
 *@}
 */

inline NameMapError NameMap::add(const size_t index, const std::string& name)
{
    auto err = NameMapError::SUCCESS;

    auto name_err = NameMapError::SUCCESS;
    const std::string existing_name = get_name(index, name_err);
    auto index_err = NameMapError::SUCCESS;
    const size_t existing_index = get_index(name, index_err);
    if (name_err == NameMapError::NOT_FOUND && index_err == NameMapError::NOT_FOUND)
    {
        // Name and index do not exist - Add to map
        m_index_map[name] = index;
        m_name_map[index] = name;
    }
    else if ((name_err == NameMapError::EXISTS) &&
             (existing_name == name) &&
             (index_err == NameMapError::EXISTS) &&
             (existing_index == index))
    {
        // no error, already in map
    }
    else
    {
        // name-index pair already exists but does not have the same index and/or name
        err = NameMapError::EXISTS;
    }
    return err;
}

inline size_t NameMap::get_index(const std::string& name, NameMapError& err) const
{
    size_t index = 0U;
    if (const auto iter = m_index_map.find(name); iter == m_index_map.end())
    {
        err = NameMapError::NOT_FOUND;
    }
    else
    {
        err = NameMapError::SUCCESS;
        index = iter->second;
    }
    return index;
}

inline std::string NameMap::get_name(const size_t index, NameMapError& err) const
{
    std::string name = {};
    if (const auto iter = m_name_map.find(index); iter == m_name_map.end())
    {
        err = NameMapError::NOT_FOUND;
    }
    else
    {
        err = NameMapError::SUCCESS;
        name = iter->second;
    }
    return name;
}

} // namespace fsb::urdf
