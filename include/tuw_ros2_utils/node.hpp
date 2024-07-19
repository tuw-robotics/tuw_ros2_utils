#ifndef TUW_ROS2_UTILS__TUW_ROS2_NODE_HPP_
#define TUW_ROS2_UTILS__TUW_ROS2_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
namespace tuw
{
    class Node : public rclcpp::Node
    {
    public:
        using rclcpp::Node::Node;

        /**
         * function to update parameters on changes and prints a log message if the value was updated
         * @param name name of the shared parameter
         * @param value value to store the parameter content
         * @param flag_changed will be set on true of the parameter changed or the value was read the first time
         * @param force_update forces a value update on ture
         */
        void update_parameter_and_log(const char *name,
                              double &value,
                              bool &flag_changed,
                              bool force_update)
        {
            double tmp = std::numeric_limits<double>::quiet_NaN();
            this->get_parameter<double>(name, tmp);
            if (force_update || (tmp != value))
            {
                flag_changed = true;
                RCLCPP_INFO(this->get_logger(), "%s: %f", name, tmp);
            }
            value = tmp;
        };

        /**
         * function to update parameters on changes and prints a log message if the value was updated
         * @param name name of the shared parameter
         * @param value value to store the parameter content
         * @param flag_changed will be set on true of the parameter changed or the value was read the first time
         * @param force_update forces a value update on ture
         */
        void update_parameter_and_log(const char *name,
                              bool &value,
                              bool &flag_changed,
                              bool force_update)
        {
            bool tmp = false;
            this->get_parameter<bool>(name, tmp);
            if (force_update || (tmp != value))
            {
                flag_changed = true;
                RCLCPP_INFO(this->get_logger(), "%s: %s", name, (value ? "true" : "false"));
            }
            value = tmp;
        };

        /**
         * function to update parameters on changes and prints a log message if the value was updated
         * @param name name of the shared parameter
         * @param value value to store the parameter content
         * @param flag_changed will be set on true of the parameter changed or the value was read the first time
         * @param force_update forces a value update on ture
         */
        void update_parameter_and_log(const char *name,
                              int &value,
                              bool &flag_changed,
                              bool force_update)
        {
            int tmp = 0;
            this->get_parameter<int>(name, tmp);
            if (force_update || (tmp != value))
            {
                flag_changed = true;
                RCLCPP_INFO(this->get_logger(), "%s: %4d", name, tmp);
            }
            value = tmp;
        };
        /**
         * function to read a parameter and prints a log message
         * @param name name of the shared parameter
         * @param value value to store the parameter content
         */
        void get_parameter_and_log(const char *name, bool &value){
            this->get_parameter<bool>(name, value);
            RCLCPP_INFO(this->get_logger(), "%s: %s", name, (value ? "true" : "false"));
        };
        /**
         * function to read a parameter and prints a log message
         * @param name name of the shared parameter
         * @param value value to store the parameter content
         */
        void get_parameter_and_log(const char *name, int &value){
            this->get_parameter<int>(name, value);
            RCLCPP_INFO(this->get_logger(), "%s: %4d", name, value);
        };
        /**
         * function to read a parameter and prints a log message
         * @param name name of the shared parameter
         * @param value value to store the parameter content
         */
        void get_parameter_and_log(const char *name, double &value){
            this->get_parameter<double>(name, value);
            RCLCPP_INFO(this->get_logger(), "%s: %f", name, value);
        };
        /**
         * function to read a parameter and prints a log message
         * @param name name of the shared parameter
         * @param value value to store the parameter content
         */
        void get_parameter_and_log(const char *name, std::string &value){
            this->get_parameter<std::string>(name, value);
            RCLCPP_INFO(this->get_logger(), "%s: %s", name, value.c_str());
        };

        /**
         * declares parameters with description and defines a default value 
         * @param name name of the shared parameter
         * @param default_value default_value
         * @param description description
         */
        void declare_parameters_with_description(const char *name, const char *default_value, const char *description)
        {
            auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
            descriptor.description = description;
            this->declare_parameter<std::string>(name, default_value, descriptor);
        };

        /**
         * declares parameters with description and defines a default value 
         * @param name name of the shared parameter
         * @param default_value default_value
         * @param description description
         */
        void declare_parameters_with_description(const char *name, bool default_value, const char *description)
        {
            auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
            descriptor.description = description;
            this->declare_parameter<bool>(name, default_value, descriptor);
        };
        /**
         * declares parameters with description and defines a default value 
         * @param name name of the shared parameter
         * @param default_value default_value
         * @param description description
         */
        void declare_parameters_with_description(const char *name, double default_value, const char *description)
        {
            auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
            descriptor.description = description;
            this->declare_parameter<double>(name, default_value, descriptor);
        };
        /**
         * declares parameters with description and defines a default value 
         * @param name name of the shared parameter
         * @param default_value default_value
         * @param description description
         * @param min min value
         * @param max max value
         * @param step_size step size
         */
        void declare_parameters_with_description(const char *name, double default_value, const char *description, double min, double max, double step_size = 0.01)
        {
            auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
            rcl_interfaces::msg::FloatingPointRange range;
            range.set__from_value(min).set__to_value(max).set__step(step_size);
            descriptor.floating_point_range = {range};
            descriptor.description = description;
            this->declare_parameter<double>(name, default_value, descriptor);
        };
        /**
         * declares parameters with description and defines a default value 
         * @param name name of the shared parameter
         * @param default_value default_value
         * @param description description
         */
        void declare_parameters_with_description(const char *name, int default_value, const char *description)
        {
            auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
            descriptor.description = description;
            this->declare_parameter<int>(name, default_value, descriptor);
        };
        /**
         * declares parameters with description and defines a default value 
         * @param name name of the shared parameter
         * @param default_value default_value
         * @param description description
         * @param min min value
         * @param max max value
         * @param step_size step size
         */
        void declare_parameters_with_description(const char *name, int default_value, const char *description, int min, int max, int step_size = 1)
        {
            auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
            rcl_interfaces::msg::IntegerRange range;
            range.set__from_value(min).set__to_value(max).set__step(step_size);
            descriptor.integer_range = {range};
            descriptor.description = description;
            this->declare_parameter<int>(name, default_value, descriptor);
        };
    };
}
#endif // TUW_ROS2_UTILS__TUW_ROS2_NODE_HPP_