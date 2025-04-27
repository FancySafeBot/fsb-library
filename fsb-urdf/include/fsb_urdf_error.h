
#pragma once

#include <string>
#include <utility>

namespace fsb::urdf
{

/**
 * @addtogroup UrdfTopic
 * @{
 */

/**
 * @brief Errors from parsing URDF
 */
enum class UrdfErrorType
{
    /** @brief No error */
    SUCCESS,
    /** @brief Failed to convert to numerical value */
    VALUE_CONVERSION_FAILED,
    /** @brief Value out of range */
    RANGE_ERROR,
    /** @brief File failed to load */
    LOAD_FILE_ERROR,
    /** @brief Failed to parse URDF XML */
    PARSE_ERROR,
    /** @brief Missing robot element */
    MISSING_ROBOT,
    /** @brief Missing name attribute */
    MISSING_NAME,
    /** @brief Repeated element name */
    REPEATED_NAME,
    /** @brief Missing joint type attribute */
    MISSING_JOINT_TYPE,
    /** @brief Invalid joint type attribute */
    INVALID_JOINT_TYPE,
    /** @brief Invalid joint axis */
    JOINT_INVALID_AXIS,
    /** @brief Joint parent not found */
    MISSING_JOINT_PARENT,
    /** @brief Missing parent attribute on joint element */
    MISSING_PARENT_LINK_ATTRIBUTE,
    /** @brief Missing child attribute on joint element */
    MISSING_JOINT_CHILD,
    /** @brief Missing child attribute on joint element */
    MISSING_CHILD_LINK_ATTRIBUTE,
    /** @brief Missing inertial element for body */
    MISSING_BODY_INERTIAL,
    /** @brief Missing mass element */
    MISSING_BODY_MASS,
    /** @brief Missing inertia element */
    MISSING_BODY_INERTIA,
    /** @brief Missing inertia attribute ixx, iyy, or izz */
    MISSING_INERTIA_ATTRIBUTE,
    /** @brief Missing mass value attribute */
    MISSING_MASS_VALUE_ATTRIBUTE,
    BODY_PRINCIPAL_INERTIA, ///< Failed to determine inertia principal axis
    BODY_INERTIA_NOT_POSITIVE_DEFINITE, ///< Inertia is not positive definite
    MAX_JOINTS_EXCEEDED, ///< Maximum number of joints exceeded
    MAX_BODIES_EXCEEDED, ///< Maximum number of bodies exceeded
    MAX_COORDINATES_EXCEEDED, ///< Maximum number of position coordinates exceeded
    MAX_DOFS_EXCEEDED, ///< Maximum number of degrees of freedom exceeded
    BODY_TREE_ERROR, ///< Failed to add body to tree
    JOINT_PARENT_BODY_NOT_FOUND, ///< Joint parent body not found
    JOINT_CHILD_BODY_NOT_FOUND ///< Joint child body not found
};

/**
 * @brief Urdf parsing error
 */
class UrdfError
{
public:
    UrdfError() = default;

    /**
     * @brief Constructor with type and error description.
     *
     * @param type Error type
     * @param description Error description
     */
    UrdfError(const UrdfErrorType type, std::string  description) :
            m_type(type),
            m_description(std::move(description))
    {}

    /**
     * @brief Get error status
     *
     * @return @c true if error, @c false otherwise.
     */
    [[nodiscard]] bool is_error() const
    {
        return (m_type != UrdfErrorType::SUCCESS);
    }

    /**
     * @brief Get error description
     *
     * @return Error description
     */
    [[nodiscard]] std::string get_description() const
    {
        return m_description;
    }

    /**
     * @brief Get error type
     * @return Error type
     */
    [[nodiscard]] UrdfErrorType get_type() const
    {
        return m_type;
    }

private:
    UrdfErrorType m_type = UrdfErrorType::SUCCESS;
    std::string   m_description;
};

/**
 * @}
 */

} // namespace fsb::urdf
