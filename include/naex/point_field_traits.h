#pragma once

#include <sensor_msgs/PointField.h>

namespace naex
{

template<typename T>
class PointFieldTraits
{
public:
    static sensor_msgs::PointField::_datatype_type datatype();
    static size_t value_size()
    {
        return sizeof(T);
    }
};

// float32
template<>
sensor_msgs::PointField::_datatype_type PointFieldTraits<float>::datatype()
{
    return sensor_msgs::PointField::FLOAT32;
}

// float64
template<>
sensor_msgs::PointField::_datatype_type PointFieldTraits<double>::datatype()
{
    return sensor_msgs::PointField::FLOAT64;
}

// int8
template<>
sensor_msgs::PointField::_datatype_type PointFieldTraits<int8_t>::datatype()
{
    return sensor_msgs::PointField::INT8;
}

// int16
template<>
sensor_msgs::PointField::_datatype_type PointFieldTraits<int16_t>::datatype()
{
    return sensor_msgs::PointField::INT16;
}

// int32
template<>
sensor_msgs::PointField::_datatype_type PointFieldTraits<int32_t>::datatype()
{
    return sensor_msgs::PointField::INT32;
}

// uint8
template<>
sensor_msgs::PointField::_datatype_type PointFieldTraits<uint8_t>::datatype()
{
    return sensor_msgs::PointField::UINT8;
}

// uint16
template<>
sensor_msgs::PointField::_datatype_type PointFieldTraits<uint16_t>::datatype()
{
    return sensor_msgs::PointField::UINT16;
}

// uint32
template<>
sensor_msgs::PointField::_datatype_type PointFieldTraits<uint32_t>::datatype()
{
    return sensor_msgs::PointField::UINT32;
}

}  // namespace naex
