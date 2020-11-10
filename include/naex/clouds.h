
#ifndef NAEX_CLOUDS_H
#define NAEX_CLOUDS_H

#include <naex/types.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>

namespace naex
{
    template<typename T>
    class PointFieldTraits
    {
    public:
        static sensor_msgs::PointField::_datatype_type datatype();
        static size_t value_size();
    };

    // float32
    template<>
    sensor_msgs::PointField::_datatype_type PointFieldTraits<float>::datatype()
    {
        return sensor_msgs::PointField::FLOAT32;
    }
    template<>
    size_t PointFieldTraits<float>::value_size() { return 4; }

    // float64
    template<>
    sensor_msgs::PointField::_datatype_type PointFieldTraits<double>::datatype()
    {
        return sensor_msgs::PointField::FLOAT64;
    }
    template<>
    size_t PointFieldTraits<double>::value_size() { return 8; }

    // int8
    template<>
    sensor_msgs::PointField::_datatype_type PointFieldTraits<int8_t>::datatype()
    {
        return sensor_msgs::PointField::INT8;
    }
    template<>
    size_t PointFieldTraits<int8_t>::value_size() { return 1; }

    // int16
    template<>
    sensor_msgs::PointField::_datatype_type PointFieldTraits<int16_t>::datatype()
    {
        return sensor_msgs::PointField::INT16;
    }
    template<>
    size_t PointFieldTraits<int16_t>::value_size() { return 2; }

    // int32
    template<>
    sensor_msgs::PointField::_datatype_type PointFieldTraits<int32_t>::datatype()
    {
        return sensor_msgs::PointField::INT32;
    }
    template<>
    size_t PointFieldTraits<int32_t>::value_size() { return 4; }

    // uint8
    template<>
    sensor_msgs::PointField::_datatype_type PointFieldTraits<uint8_t>::datatype()
    {
        return sensor_msgs::PointField::UINT8;
    }
    template<>
    size_t PointFieldTraits<uint8_t>::value_size() { return 1; }

    // uint16
    template<>
    sensor_msgs::PointField::_datatype_type PointFieldTraits<uint16_t>::datatype()
    {
        return sensor_msgs::PointField::UINT16;
    }
    template<>
    size_t PointFieldTraits<uint16_t>::value_size() { return 2; }

    // uint32
    template<>
    sensor_msgs::PointField::_datatype_type PointFieldTraits<uint32_t>::datatype()
    {
        return sensor_msgs::PointField::UINT32;
    }
    template<>
    size_t PointFieldTraits<uint32_t>::value_size() { return 4; }

    const sensor_msgs::PointField* find_field(
            const sensor_msgs::PointCloud2& cloud,
            const std::string& name)
    {
        for (const auto& f: cloud.fields)
        {
            if (f.name == name)
            {
                return &f;
            }
        }
        return nullptr;
    }

    template<typename T>
    void fill_field(
            const std::string& name,
            const T* it,
            sensor_msgs::PointCloud2& cloud)
    {
        size_t n = cloud.height * cloud.width;
        sensor_msgs::PointCloud2Iterator<T> field_it(cloud, name);
        const auto end = it + n;
        for (; it != end; ++it, ++field_it)
        {
            *field_it = *it;
        }
    }

    template<typename T>
    void fill_const_field(
            const std::string& name,
            const T& value,
            sensor_msgs::PointCloud2& cloud)
    {
        size_t n = cloud.height * cloud.width;
        sensor_msgs::PointCloud2Iterator<T> field_it(cloud, name);
        const auto end = field_it + n;
        for (; field_it != end; ++field_it)
        {
            *field_it = value;
        }
    }

    void reset_fields(sensor_msgs::PointCloud2& cloud)
    {
        cloud.fields.clear();
        cloud.point_step = 0;
    }

    template<typename T>
    void append_field(
            const std::string& name,
            uint32_t count,
            sensor_msgs::PointCloud2& cloud)
    {
        sensor_msgs::PointField field;
        field.name = name;
        field.offset = cloud.point_step;
        field.datatype = PointFieldTraits<T>::datatype();
        field.count = count;
        cloud.fields.emplace_back(field);
        cloud.point_step += count * PointFieldTraits<T>::value_size();
        // Setting row_step is up to caller.
    }

    template<typename T = float>
    void append_position_fields(sensor_msgs::PointCloud2& cloud)
    {
        append_field<T>("x", 1, cloud);
        append_field<T>("y", 1, cloud);
        append_field<T>("z", 1, cloud);
    }

    template<typename T = float>
    void append_normal_fields(sensor_msgs::PointCloud2& cloud)
    {
        append_field<T>("normal_x", 1, cloud);
        append_field<T>("normal_y", 1, cloud);
        append_field<T>("normal_z", 1, cloud);
    }

    void append_occupancy_fields(sensor_msgs::PointCloud2& cloud)
    {
        append_field<uint16_t>("empty", 1, cloud);
        append_field<uint16_t>("occupied", 1, cloud);
        append_field<float>("dynamic", 1, cloud);
    }

    void append_traversability_fields(sensor_msgs::PointCloud2& cloud)
    {
        append_field<uint8_t>("num_normal_pts", 1, cloud);
        append_field<uint8_t>("num_obstacle_pts", 1, cloud);
        append_field<float>("ground_diff_std", 1, cloud);
        append_field<float>("ground_diff_min", 1, cloud);
        append_field<float>("ground_diff_max", 1, cloud);
        append_field<float>("ground_abs_diff_mean", 1, cloud);
        append_field<uint8_t>("normal_label", 1, cloud);
        append_field<uint8_t>("final_label", 1, cloud);
    }

    void append_planning_fields(sensor_msgs::PointCloud2& cloud)
    {
        append_field<float>("path_cost", 1, cloud);
        append_field<float>("utility", 1, cloud);
        append_field<float>("final_cost", 1, cloud);
    }

    void resize_cloud(
            sensor_msgs::PointCloud2& cloud,
            uint32_t height,
            uint32_t width)
    {
        cloud.data.resize(height * width * cloud.point_step);
        cloud.height = height;
        cloud.width = width;
        cloud.row_step = width * cloud.point_step;
    }

    template<typename P, typename N>
    void create_debug_cloud(
            const flann::Matrix<P>& points,
            const flann::Matrix<N>& normals,
            sensor_msgs::PointCloud2& cloud)
    {
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(19,
                "x", 1, sensor_msgs::PointField::FLOAT32,
                "y", 1, sensor_msgs::PointField::FLOAT32,
                "z", 1, sensor_msgs::PointField::FLOAT32,
                "normal_x", 1, sensor_msgs::PointField::FLOAT32,
                "normal_y", 1, sensor_msgs::PointField::FLOAT32,
                "normal_z", 1, sensor_msgs::PointField::FLOAT32,
                "num_normal_pts", 1, sensor_msgs::PointField::UINT8,
                "ground_diff_std", 1, sensor_msgs::PointField::FLOAT32,
                "ground_diff_min", 1, sensor_msgs::PointField::FLOAT32,
                "ground_diff_max", 1, sensor_msgs::PointField::FLOAT32,
                "ground_abs_diff_mean", 1, sensor_msgs::PointField::FLOAT32,
                "num_obstacle_pts", 1, sensor_msgs::PointField::UINT8,
                "normal_label", 1, sensor_msgs::PointField::UINT8,
                "final_label", 1, sensor_msgs::PointField::UINT8,
                "path_cost", 1, sensor_msgs::PointField::FLOAT32,
                "utility", 1, sensor_msgs::PointField::FLOAT32,
                "final_cost", 1, sensor_msgs::PointField::FLOAT32,
                "occupied", 1, sensor_msgs::PointField::UINT16,
                "empty", 1, sensor_msgs::PointField::UINT16);
        modifier.resize(points.rows);

        sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x"), nx_it(cloud, "normal_x");
        for (size_t i = 0; i < points.rows; ++i, ++x_it, ++nx_it)
        {
            x_it[0] = points[i][0];
            x_it[1] = points[i][1];
            x_it[2] = points[i][2];
            nx_it[0] = normals[i][0];
            nx_it[1] = normals[i][1];
            nx_it[2] = normals[i][2];
        }
        fill_const_field("num_normal_pts", uint8_t(0), cloud);
        fill_const_field("ground_diff_std", std::numeric_limits<float>::quiet_NaN(), cloud);
        fill_const_field("ground_diff_min", std::numeric_limits<float>::quiet_NaN(), cloud);
        fill_const_field("ground_diff_max", std::numeric_limits<float>::quiet_NaN(), cloud);
        fill_const_field("ground_abs_diff_mean", std::numeric_limits<float>::quiet_NaN(), cloud);
        fill_const_field("num_obstacle_pts", uint8_t(0), cloud);

        fill_const_field("normal_label", uint8_t(UNKNOWN), cloud);
        fill_const_field("final_label", uint8_t(UNKNOWN), cloud);
        fill_const_field("path_cost", std::numeric_limits<float>::quiet_NaN(), cloud);
        fill_const_field("utility", std::numeric_limits<float>::quiet_NaN(), cloud);
        fill_const_field("final_cost", std::numeric_limits<float>::quiet_NaN(), cloud);

        fill_const_field("occupied", uint16_t(0), cloud);
        fill_const_field("empty", uint16_t(0), cloud);
    }

    template<typename P>
    void create_xyz_cloud(
            const flann::Matrix<P>& points,
            sensor_msgs::PointCloud2& cloud)
    {
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(3,
                "x", 1, sensor_msgs::PointField::FLOAT32,
                "y", 1, sensor_msgs::PointField::FLOAT32,
                "z", 1, sensor_msgs::PointField::FLOAT32);
        modifier.resize(points.rows);

        sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
        for (size_t i = 0; i < points.rows; ++i, ++x_it)
        {
            x_it[0] = points[i][0];
            x_it[1] = points[i][1];
            x_it[2] = points[i][2];
        }
    }

}  // namespace naex

#endif //NAEX_CLOUDS_H
