
#ifndef NAEX_CLOUDS_H
#define NAEX_CLOUDS_H

#include <cmath>
#include <iomanip>
#include <iostream>
#include <naex/geom.h>
#include <naex/types.h>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <naex/timer.h>
#include <unordered_set>

namespace naex
{
bool bigendian()
{
    uint16_t num = 1;
    return !(*(uint8_t*)&num == 1);
}


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

    void reset_fields(sensor_msgs::PointCloud2 &cloud)
    {
        cloud.fields.clear();
        cloud.point_step = 0;
    }

    template<typename T>
    void append_field(
        const std::string& name,
        const uint32_t count,
        sensor_msgs::PointCloud2 &cloud)
//        const uint32_t offset = cloud.point_step)
    {
        typedef typename std::remove_reference<T>::type C;
        sensor_msgs::PointField field;
        field.name = name;
        field.offset = cloud.point_step;
//        field.offset = offset;
        field.datatype = PointFieldTraits<C>::datatype();
        field.count = count;
        cloud.fields.emplace_back(field);
        cloud.point_step += count * PointFieldTraits<C>::value_size();
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
        append_field<T>("nx", 1, cloud);
        append_field<T>("ny", 1, cloud);
        append_field<T>("nz", 1, cloud);
    }

    void append_occupancy_fields(sensor_msgs::PointCloud2& cloud)
    {
        append_field<uint8_t>("seen_thru", 1, cloud);
        append_field<uint8_t>("hit", 1, cloud);
//        append_field<uint16_t>("empty", 1, cloud);
//        append_field<uint16_t>("occupied", 1, cloud);
//        append_field<float>("dynamic", 1, cloud);
    }

    void append_traversability_fields(sensor_msgs::PointCloud2& cloud)
    {
//        8 bytes
        append_field<uint8_t>("normal_pts", 1, cloud);
        append_field<uint8_t>("obs_pts", 1, cloud);
//        append_field<float>("gnd_diff_std", 1, cloud);
//        append_field<float>("gnd_diff_min", 1, cloud);
//        append_field<float>("gnd_diff_max", 1, cloud);
//        append_field<float>("gnd_abs_diff_mean", 1, cloud);
        // Compact int8 representation in NN radius.
//        append_field<float>("trav_radius", 1, cloud);
        append_field<uint8_t>("gnd_diff_std", 1, cloud);
        append_field<int8_t>("gnd_diff_min", 1, cloud);
        append_field<int8_t>("gnd_diff_max", 1, cloud);
        append_field<uint8_t>("gnd_abs_diff_mean", 1, cloud);
        append_field<uint8_t>("nz_lbl", 1, cloud);
        append_field<uint8_t>("final_lbl", 1, cloud);
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

//    template<typename P, typename N>
//    void create_debug_cloud(
//            const flann::Matrix<P>& points,
//            const flann::Matrix<N>& normals,
//            sensor_msgs::PointCloud2& cloud)
//    {
//        sensor_msgs::PointCloud2Modifier modifier(cloud);
//        modifier.setPointCloud2Fields(19,
//                "x", 1, sensor_msgs::PointField::FLOAT32,
//                "y", 1, sensor_msgs::PointField::FLOAT32,
//                "z", 1, sensor_msgs::PointField::FLOAT32,
//                "normal_x", 1, sensor_msgs::PointField::FLOAT32,
//                "normal_y", 1, sensor_msgs::PointField::FLOAT32,
//                "normal_z", 1, sensor_msgs::PointField::FLOAT32,
//                "num_normal_pts", 1, sensor_msgs::PointField::UINT8,
//                "ground_diff_std", 1, sensor_msgs::PointField::FLOAT32,
//                "ground_diff_min", 1, sensor_msgs::PointField::FLOAT32,
//                "ground_diff_max", 1, sensor_msgs::PointField::FLOAT32,
//                "ground_abs_diff_mean", 1, sensor_msgs::PointField::FLOAT32,
//                "num_obstacle_pts", 1, sensor_msgs::PointField::UINT8,
//                "normal_label", 1, sensor_msgs::PointField::UINT8,
//                "final_label", 1, sensor_msgs::PointField::UINT8,
//                "path_cost", 1, sensor_msgs::PointField::FLOAT32,
//                "utility", 1, sensor_msgs::PointField::FLOAT32,
//                "final_cost", 1, sensor_msgs::PointField::FLOAT32,
//                "occupied", 1, sensor_msgs::PointField::UINT16,
//                "empty", 1, sensor_msgs::PointField::UINT16);
//        modifier.resize(points.rows);
//
//        sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x"), nx_it(cloud, "normal_x");
//        for (size_t i = 0; i < points.rows; ++i, ++x_it, ++nx_it)
//        {
//            x_it[0] = points[i][0];
//            x_it[1] = points[i][1];
//            x_it[2] = points[i][2];
//            nx_it[0] = normals[i][0];
//            nx_it[1] = normals[i][1];
//            nx_it[2] = normals[i][2];
//        }
//        fill_const_field("num_normal_pts", uint8_t(0), cloud);
//        fill_const_field("ground_diff_std", std::numeric_limits<float>::quiet_NaN(), cloud);
//        fill_const_field("ground_diff_min", std::numeric_limits<float>::quiet_NaN(), cloud);
//        fill_const_field("ground_diff_max", std::numeric_limits<float>::quiet_NaN(), cloud);
//        fill_const_field("ground_abs_diff_mean", std::numeric_limits<float>::quiet_NaN(), cloud);
//        fill_const_field("num_obstacle_pts", uint8_t(0), cloud);
//
//        fill_const_field("normal_label", uint8_t(UNKNOWN), cloud);
//        fill_const_field("final_label", uint8_t(UNKNOWN), cloud);
//        fill_const_field("path_cost", std::numeric_limits<float>::quiet_NaN(), cloud);
//        fill_const_field("utility", std::numeric_limits<float>::quiet_NaN(), cloud);
//        fill_const_field("final_cost", std::numeric_limits<float>::quiet_NaN(), cloud);
//
//        fill_const_field("occupied", uint16_t(0), cloud);
//        fill_const_field("empty", uint16_t(0), cloud);
//    }

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

    void print_cloud_summary(const sensor_msgs::PointCloud2& cloud)
    {
        sensor_msgs::PointCloud2ConstIterator<float> x_begin(cloud, "x");
        std::stringstream az_ss, el_ss;

        for (Index r = 0; r < cloud.height; r += cloud.height / 8)
        {
            if (r > 0)
            {
                az_ss << std::endl;
                el_ss << std::endl;
            }
            for (Index c = 0; c < cloud.width; c += cloud.width / 8)
            {
                const auto it = (x_begin + r * cloud.width + c);
                float az, el, radius;
                cartesian_to_spherical(it[0], it[1], it[2], az, el, radius);
                if (c > 0)
                {
                    az_ss << " ";
                    el_ss << " ";
                }
                az_ss << degrees(az);
                el_ss << degrees(el);
            }
        }

        ROS_INFO("Azimuth sample:\n%s", az_ss.str().c_str());
        ROS_INFO("Elevation sample:\n%s", el_ss.str().c_str());
    }

    class SphericalProjection
    {
    public:
        SphericalProjection()
        {}

        SphericalProjection(float azimuth_start,
                            float azimuth_step,
                            float elevation_start,
                            float elevation_step,
                            uint32_t height,
                            uint32_t width):
            azimuth_start_(azimuth_start),
            azimuth_step_(azimuth_step),
            elevation_start_(elevation_start),
            elevation_step_(elevation_step),
            height_(height),
            width_(width)
        {}

        bool check(const sensor_msgs::PointCloud2& cloud)
        {
            assert(height_ == cloud.height);
            assert(width_ == cloud.width);
            if (cloud.height != height_ || cloud.width != width_)
            {
                ROS_WARN("Cloud size (%i, %i) inconsistent with model size (%i, %i).",
                         cloud.height, cloud.width, height_, width_);
            }
            std::stringstream az_ss, el_ss;
            sensor_msgs::PointCloud2ConstIterator<float> x_it(cloud, "x");
            double residual_sum = 0.;
            Index n = 0;

            for (Index r = 0; r < height_; ++r)
            {
                for (Index c = 0; c < width_; ++c, ++x_it)
                {
                    if (!std::isfinite(x_it[0]) || !std::isfinite(x_it[1]) || !std::isfinite(x_it[2]))
                        continue;
                    Value r_model, c_model;
                    project(x_it[0], x_it[1], x_it[2], r_model, c_model);

                    if (r == std::round(r_model) && c == std::round(c_model))
                    {
                        continue;
                    }

                    Vec3 pt(x_it[0], x_it[1], x_it[2]);
                    pt.normalize();
                    Vec3 pt_model(0., 0., 0.);
                    unproject(Value(r), Value(c), pt_model(0), pt_model(1), pt_model(2));
                    Value residual = std::acos(pt.dot(pt_model));
                    if (std::isfinite(residual))
                    {
                        residual_sum += residual;
                        ++n;
                    }
                    continue;

                    if (residual <= std::min(std::abs(azimuth_step_), std::abs(elevation_step_)) / 2.f)
                    {
                        continue;
                    }

                    ROS_WARN("Model direction [%.3f, %.3f, %.3f] "
                             "inconsistent with data [%.3f, %.3f, %.3f], "
                             "residual %.3f [deg].",
                             pt_model(0), pt_model(1), pt_model(2),
                             pt.x(), pt.y(), pt.z(),
                             residual);
                }
            }
            double mean_residual = residual_sum / n;
            if (mean_residual > std::min(std::abs(azimuth_step_), std::abs(elevation_step_)) / 2.)
            {
                ROS_WARN("Mean angular error: %.3f [deg].", degrees(residual_sum / n));
            }
            else
            {
                ROS_DEBUG("Mean angular error: %.3f [deg].", residual_sum / n);
            }
            return true;
        }

        void print_model_summary()
        {
            std::stringstream az_ss, el_ss;
            for (Index r = 0; r < height_; r += height_ / 8)
            {
                if (r > 0)
                {
                    az_ss << std::endl;
                    el_ss << std::endl;
                }
                for (Index c = 0; c < width_; c += width_ / 8)
                {
                    Vec3 pt_model(0.f, 0.f, 0.f);
                    unproject(Value(r), Value(c), pt_model(0), pt_model(1), pt_model(2));
                    Value az, el, r;
                    cartesian_to_spherical(pt_model(0), pt_model(1), pt_model(2), az, el, r);
                    if (c > 0)
                    {
                        az_ss << " ";
                        el_ss << " ";
                    }
                    az_ss << degrees(az);
                    el_ss << degrees(el);
                }
            }
            ROS_INFO("Azimuth model sample:\n%s", az_ss.str().c_str());
            ROS_INFO("Elevation model sample:\n%s", el_ss.str().c_str());
        }

        bool fit_fast(const sensor_msgs::PointCloud2& cloud)
        {
            Timer t;
            assert(cloud.height >= 1);
            assert(cloud.width >= 1);

            const Index n_points = cloud.height * cloud.width;
            sensor_msgs::PointCloud2ConstIterator<float> x_begin(cloud, "x");
            sensor_msgs::PointCloud2ConstIterator<float> x_it = x_begin;
            Index i_r0 = INVALID_INDEX;
            Index i_r1 = INVALID_INDEX;
            Index i_c0 = INVALID_INDEX;
            Index i_c1 = INVALID_INDEX;
            for (Index i = 0; i < n_points; ++i, ++x_it)
            {
                if (!std::isfinite(x_it[0]) || !std::isfinite(x_it[1]) || !std::isfinite(x_it[2]))
                    continue;
                if (i_r0 == INVALID_INDEX || i / cloud.width < i_r0 / cloud.width)
                    i_r0 = i;
                if (i_r1 == INVALID_INDEX || i / cloud.width > i_r1 / cloud.width)
                    i_r1 = i;
                if (i_c0 == INVALID_INDEX || i % cloud.width < i_c0 % cloud.width)
                    i_c0 = i;
                if (i_c1 == INVALID_INDEX || i % cloud.width > i_c1 % cloud.width)
                    i_c1 = i;
                if (i_r0 < i_r1 && i_c0 < i_c1)
                    break;
            }
            if (i_r0 == INVALID_INDEX)
                return false;

            height_ = cloud.height;
            width_ = cloud.width;

            Value elevation_0 = elevation((x_begin + i_r0)[0], (x_begin + i_r0)[1], (x_begin + i_r0)[2]);
            Value elevation_1 = elevation((x_begin + i_r1)[0], (x_begin + i_r1)[1], (x_begin + i_r1)[2]);
            Index r0 = i_r0 / width_;
            Index r1 = i_r1 / width_;
            elevation_step_ = (elevation_1 - elevation_0) / (r1 - r0);
            elevation_start_ = elevation_0 - r0 * elevation_step_;

            Value azimuth_0 = azimuth((x_begin + i_c0)[0], (x_begin + i_c0)[1]);
            Value azimuth_1 = azimuth((x_begin + i_c1)[0], (x_begin + i_c1)[1]);
            Index c0 = i_c0 % width_;
            Index c1 = i_c1 % width_;
            azimuth_step_ = (azimuth_1 - azimuth_0) / (c1 - c0);
            azimuth_start_ = azimuth_0 - c0 * azimuth_step_;
            ROS_INFO("Spherical model: "
                      "elevation difference %.3f between rows %i and %i, "
                      "azimuth difference %.3f between cols %i and %i "
                      "(%.6f s).",
                      elevation_1 - elevation_0, int(r0), int(r1),
                      azimuth_1 - azimuth_0, int(c0), int(c1),
                      t.seconds_elapsed());

            // Check that the whole cloud conforms to the estimated parameters.
//            assert(check(cloud));
            return true;
        }

        bool fit_robust(const sensor_msgs::PointCloud2& cloud)
        {
            Timer t;
            assert(cloud.height >= 1);
            assert(cloud.width >= 1);

            const Index n_points = cloud.height * cloud.width;
            sensor_msgs::PointCloud2ConstIterator<float> x_begin(cloud, "x");

            // Collect valid indices.
            std::vector<Index> valid;
            valid.reserve(n_points);
            auto x = x_begin;
            for (Index i = 0; i < n_points; ++i, ++x)
            {
                if (!std::isfinite(x[0]) || !std::isfinite(x[1]) || !std::isfinite(x[2]))
                    continue;
                valid.push_back(i);
            }

            // Shuffle valid indices.
            std::random_device rd;
            std::mt19937 g(rd());
            std::shuffle(valid.begin(), valid.end(), g);

            // Generate models from pairs of points.
            // Model consists of (start, step).
            typedef std::pair<Value, Value> Model;
            const auto comp = [](const Model& a, const Model& b) { return a.second < b.second; };
            std::vector<Model> az_models, el_models;
            az_models.reserve(n_points);
            el_models.reserve(n_points);
            for (Index i = 0; i + 1 < valid.size(); ++i)
            {
                auto x0 = x_begin + valid[i];
                auto x1 = x_begin + valid[i + 1];

                Index c0 = valid[i] % cloud.width;
                Index c1 = valid[i + 1] % cloud.width;
                if (c0 != c1)
                {
                    Value az0 = azimuth(x0[0], x0[1]);
                    Value az1 = azimuth(x1[0], x1[1]);
                    auto az_step = (az1 - az0) / (int(c1) - int(c0));
                    az_models.push_back({az0 - c0 * az_step, az_step});
                }

                Index r0 = valid[i] / cloud.width;
                Index r1 = valid[i + 1] / cloud.width;
                if (r0 != r1)
                {
                    Value el0 = elevation(x0[0], x0[1], x0[2]);
                    Value el1 = elevation(x1[0], x1[1], x1[2]);
                    auto el_step = (el1 - el0) / (int(r1) - int(r0));
                    el_models.push_back({el0 - r0 * el_step, el_step});
                }

                if (az_models.size() >= 25 && el_models.size() >= 25)
                {
                    break;
                }
            }

            if (az_models.empty() || el_models.empty())
            {
                return false;
            }

            // Get median step models.
            std::sort(az_models.begin(), az_models.end(), comp);
            azimuth_start_ = az_models[az_models.size() / 2].first;
            azimuth_step_ = az_models[az_models.size() / 2].second;

            std::sort(el_models.begin(), el_models.end(), comp);
            elevation_start_ = el_models[el_models.size() / 2].first;
            elevation_step_ = el_models[el_models.size() / 2].second;

            height_ = cloud.height;
            width_ = cloud.width;

            ROS_DEBUG("Robust fit [deg]: "
                      "azimuth [%.1f, %.1f], step %.3f (from %lu models), "
                      "elevation [%.1f, %.1f], step %.3f (from %lu models) "
                      "(%.6f s).",
                     degrees(azimuth_start_), degrees(azimuth_start_ + (width_ - 1) * azimuth_step_),
                     degrees(azimuth_step_), az_models.size(),
                     degrees(elevation_start_), degrees(elevation_start_ + (height_ - 1) * elevation_step_),
                     degrees(elevation_step_), el_models.size(),
                     t.seconds_elapsed());

            return true;
        }

        bool fit(const sensor_msgs::PointCloud2& cloud)
        {
            return fit_robust(cloud);
        }

        template<typename T>
        inline void unproject(const T& r, const T& c, T& x, T& y, T& z)
        {
            const T azimuth = azimuth_start_ + c * azimuth_step_;
            const T elevation = elevation_start_ + r * elevation_step_;
            spherical_to_cartesian(azimuth, elevation, T(1), x, y, z);
        }

        template<typename T>
        void project(const T x, const T y, const T z, T& r, T& c)
        {
            T azimuth, elevation, radius;
            cartesian_to_spherical(x, y, z, azimuth, elevation, radius);
            r = (elevation - elevation_start_) / elevation_step_;
            c = (azimuth - azimuth_start_) / azimuth_step_;
        }

        template<typename PointIt, typename ProjIt>
        void project(PointIt x_begin, PointIt x_end, ProjIt u_begin)
        {
            for (; x_begin < x_end; ++x_begin, ++u_begin)
            {
                project(x_begin[0], x_begin[1], x_begin[2], u_begin[0], u_begin[1]);
            }
        }

        // Azimuth, angle in xy plane, positive for x to y direction;
        // azimuth at image[:, 0].
        float azimuth_start_;
        float azimuth_step_;
        // Elevation, angle from from xy plane to point;
        // elevation at image[0, :].
        float elevation_start_;
        float elevation_step_;
        // Cloud 2D grid size.
        uint32_t height_;
        uint32_t width_;
    };

void copy_cloud_metadata(const sensor_msgs::PointCloud2& input,
                         sensor_msgs::PointCloud2& output)
{
    output.header = input.header;
//    output.height = input.height;
//    output.width = input.width;
    output.fields = input.fields;
    output.is_bigendian = input.is_bigendian;
    output.point_step = input.point_step;
//    output.row_step = output.width * output.point_step;
//    output.data.resize(indices.size() * output.point_step);
    output.is_dense = input.is_dense;
}

/**
 * Copy selected points.
 * @tparam C A container type, with begin, end and size methods.
 * @param input Input cloud.
 * @param indices Container of indices.
 * @param output Output cloud.
 */
template<typename C>
void copy_points(const sensor_msgs::PointCloud2& input,
                 const C& indices,
                 sensor_msgs::PointCloud2& output)
{
    Timer t;
    output.header = input.header;
    output.height = 1;
    output.width = decltype(output.width)(indices.size());
    output.fields = input.fields;
    output.is_bigendian = input.is_bigendian;
    output.point_step = input.point_step;
    output.row_step = output.width * output.point_step;
    output.data.resize(indices.size() * output.point_step);
    output.is_dense = input.is_dense;
    const auto in_ptr = input.data.data();
    uint8_t* out_ptr = output.data.data();
    auto it = indices.begin();
    for (Index i = 0; i != indices.size(); ++i, ++it)
    {
        std::copy(in_ptr + (*it) * input.point_step,
                  in_ptr + (*it + 1) * input.point_step,
                  out_ptr + i * output.point_step);
    }
    ROS_DEBUG("%lu / %lu points copied (%.6f s).",
              indices.size(), size_t(input.height * input.width), t.seconds_elapsed());
}

void step_subsample(const sensor_msgs::PointCloud2& input,
                    const Index max_rows,
                    const Index max_cols,
                    sensor_msgs::PointCloud2& output)
{
    Timer t;
    output.header = input.header;
    Index row_step = (input.height - 1) / max_rows + 1;
    Index col_step = (input.width - 1) / max_cols + 1;
    output.height = input.height / row_step;
    output.width = input.width / col_step;
    output.fields = input.fields;
    output.is_bigendian = input.is_bigendian;
    output.point_step = input.point_step;
    output.row_step = output.width * input.point_step;
    output.is_dense = input.is_dense;

    output.data.resize(output.height * output.width * output.point_step);
    const uint8_t* in_ptr = input.data.data();
    uint8_t* out_ptr = output.data.data();
    for (Index r = 0; r < output.height; ++r)
    {
        for (Index c = 0; c < output.width; ++c)
        {
            std::copy(in_ptr + (r * row_step * input.row_step + (c * col_step) * input.point_step),
                      in_ptr + (r * row_step * input.row_step + (c * col_step + 1) * input.point_step),
                      out_ptr + (r * output.row_step + c * output.point_step));
        }
    }
    ROS_DEBUG("Step-subsample %u-by-%u cloud to %u-by-%u (%.6f s).",
              input.height, input.width, output.height, output.width, t.seconds_elapsed());
}

class Voxel
{
public:
    class Hash
    {
    public:
        size_t operator()(const Voxel& voxel) const noexcept
        {
            return voxel.hash();
        }
    };

    Voxel():
        x_(0), y_(0), z_(0)
    {}
    Voxel(int x, int y, int z):
        x_(x), y_(y), z_(z)
    {}
    Voxel(const Voxel& other):
        x_(other.x_), y_(other.y_), z_(other.z_)
    {}
    size_t hash() const noexcept
    {
        return (std::hash<int>{}(x_)
                ^ std::hash<int>{}(y_)
                ^ std::hash<int>{}(z_));
    }
    friend bool operator==(const Voxel& a, const Voxel& b)
    {
        return (a.x_ == b.x_) && (a.y_ == b.y_) && (a.z_ == b.z_);
    }
    int x_;
    int y_;
    int z_;
};

void voxel_filter(const sensor_msgs::PointCloud2& input,
                  const Value bin_size,
                  sensor_msgs::PointCloud2& output)
{
    Timer t, t_part;
    assert(input.row_step % input.point_step == 0);
    const Index n_pts = input.height * input.width;

    t_part.reset();
    std::vector<Index> indices;
    indices.reserve(n_pts);
    for (Index i = 0; i < n_pts; ++i)
    {
        indices.push_back(i);
    }
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(indices.begin(), indices.end(), g);
    ROS_DEBUG("Created random permutation of %lu points (%.6f s).",
              size_t(n_pts), t_part.seconds_elapsed());

    t_part.reset();
    std::vector<Index> keep;
    keep.reserve(indices.size());
    const uint8_t* in_ptr = input.data.data();
//    uint8_t* out_ptr = output.data.data();
    sensor_msgs::PointCloud2ConstIterator<float> pt_it(input, "x");
    std::unordered_set<Voxel, Voxel::Hash> voxels;
    voxels.reserve(keep.size());
    for (Index i = 0; i < n_pts; ++i, in_ptr += input.point_step, ++pt_it)
    {
        if (!std::isfinite(pt_it[0]) || !std::isfinite(pt_it[1]) || !std::isfinite(pt_it[2]))
        {
            continue;
        }
        Value x = std::floor(pt_it[0] / bin_size);
        Value y = std::floor(pt_it[1] / bin_size);
        Value z = std::floor(pt_it[2] / bin_size);
        if (x < std::numeric_limits<int>::min() || x > std::numeric_limits<int>::max()
            || y < std::numeric_limits<int>::min() || y > std::numeric_limits<int>::max()
            || z < std::numeric_limits<int>::min() || z > std::numeric_limits<int>::max())
        {
            continue;
        }
//        const Voxel voxel(int(x), int(y), int(z));
        const Voxel voxel((int)x, (int)y, (int)z);
        if (voxels.find(voxel) == voxels.end())
        {
//            voxels.insert(it, voxel);
            voxels.insert(voxel);
            keep.push_back(i);
        }
    }
    ROS_DEBUG("Getting %lu indices to keep (%.6f s).", keep.size(), t_part.seconds_elapsed());

    // Copy selected indices.
    t_part.reset();
    copy_points(input, keep, output);
    ROS_DEBUG("%lu / %lu points kept by voxel filter (%.6f s).",
              keep.size(), size_t(n_pts), t_part.seconds_elapsed());
}

}  // namespace naex

#endif //NAEX_CLOUDS_H
