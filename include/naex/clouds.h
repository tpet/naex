
#ifndef NAEX_CLOUDS_H
#define NAEX_CLOUDS_H

#include <cmath>
#include <naex/types.h>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <naex/timer.h>
#include <unordered_set>

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

    template<typename T>
    flann::Matrix<T> flann_matrix_view(
        sensor_msgs::PointCloud2& cloud,
        const std::string& field,
        const uint32_t count = 1)
    {
        sensor_msgs::PointCloud2Iterator<T> it(cloud, field);
        return flann::Matrix<T>(&it[0], cloud.height * cloud.width, count, cloud.point_step);
    }

    template<typename T>
    flann::Matrix<T> const_flann_matrix_view(
        const sensor_msgs::PointCloud2& cloud,
        const std::string& field,
        const uint32_t count = 1)
    {
        sensor_msgs::PointCloud2ConstIterator<T> it(cloud, field);
        return flann::Matrix<T>(&it[0], cloud.height * cloud.width, count, cloud.point_step);
    }

    class SphericalProjection
    {
    public:
        SphericalProjection()
        {}

//        SphericalProjection(float azimuth[2], float elevation[2], uint32_t size[2]):
//            azimuth_{azimuth[0], azimuth[1]},
//            elevation_{elevation[0], elevation[1]},
//            size_{size[0], size[1]}
//        {
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

        bool check(sensor_msgs::PointCloud2& cloud)
        {
//            assert(size_[0] == cloud.height);
//            assert(size_[1] == cloud.width);
            assert(height_ == cloud.height);
            assert(width_ == cloud.width);
            sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
            for (uint32_t i = 0; i < height_; ++i)
            {
                for (uint32_t j = 0; j < width_; ++j, ++x_it)
                {
                    auto az = azimuth(x_it[0], x_it[1]);
                    auto el = elevation(x_it[0], x_it[1], x_it[2]);
                    assert(std::abs(az - (azimuth_start_ + j * azimuth_step_)) <= 1e-3);
                    assert(std::abs(el - (elevation_start_ + i * elevation_step_)) <= 1e-3);
                }
            }
            return true;
        }

        bool fit(sensor_msgs::PointCloud2& cloud)
        {
            assert(cloud.height >= 1);
            assert(cloud.width >= 1);
            height_ = cloud.height;
            width_ = cloud.width;
            sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
            // Find start and step values for azimuth and elevation.

            // Iterate rows for azimuth and cols for elevation to find first
            // valid step value.
//            uint32_t prev_i = std::numeric_limits<uint32_t>::max();
//            float prev_az = std::numeric_limits<float>::quiet_NaN();
//            for (uint32_t i = 0; i < size_[0]; ++i)
//            {
//                for (uint32_t j = 0; j < size_[1]; ++j)
//                {
////                    Vec3Map x(&x_it[0]);
//                    if (std::isfinite(x_it[0]))
//                    {
//                        // Valid input point.
//                        auto az = azimuth(x_it[0], x_it[1]);
//                        if (std::isnan(prev_az))
//                        {
//                            prev_i = i;
//                            prev_az = az;
//                        }
//                        else if (prev_i != i)
//                        {
//                            azimuth_step_ = (az - prev_az) / (i - prev_i);
//                            goto azimuth_done;
//                        }
//                    }
//                }
//            }

            float ref_az = std::numeric_limits<float>::quiet_NaN();
            uint32_t ref_az_j = std::numeric_limits<uint32_t>::max();
            float ref_el = std::numeric_limits<float>::quiet_NaN();
            uint32_t ref_el_i = std::numeric_limits<uint32_t>::max();

            azimuth_step_ = std::numeric_limits<float>::quiet_NaN();
            elevation_step_ = std::numeric_limits<float>::quiet_NaN();

            for (uint32_t i = 0; i < height_; ++i)
            {
                for (uint32_t j = 0; j < width_; ++j, ++x_it)
                {
                    if (std::isfinite(x_it[0]))
                    {
                        // We have a valid input point.
                        auto az = azimuth(x_it[0], x_it[1]);
                        auto el = elevation(x_it[0], x_it[1], x_it[2]);

                        // Get a reference azimuth and elevation for comparison.
                        if (std::isnan(ref_az))
                        {
                            ref_az = az;
                            ref_az_j = j;
                            ref_el = el;
                            ref_el_i = i;
                            continue;
                        }

                        // Compute azimuth and elevation if possible (once).
                        if (j != ref_az_j && std::isnan(azimuth_step_))
                        {
                            azimuth_step_ = (az - ref_az) / (j - ref_az_j);
                        }
                        if (i != ref_el_i && std::isnan(elevation_step_))
                        {
                            elevation_step_ = (el - ref_el) / (i - ref_el_i);
                        }

                        // Stop early once we have everything.
                        if (!std::isnan(azimuth_step_) && !std::isnan(elevation_step_))
                        {
                            goto estimate_start_values;
                        }
                    }
                }
            }

            estimate_start_values:
            if (std::isnan(azimuth_step_))
            {
                azimuth_start_ = ref_az;
            }
            else
            {
                azimuth_start_ = ref_az - ref_az_j * azimuth_step_;
            }
            if (std::isnan(elevation_step_))
            {
                elevation_start_ = ref_el;
            }
            else
            {
                elevation_start_ = ref_el - ref_el_i * elevation_step_;
            }

            // Check that the whole cloud conforms to the estimated parameters.
            assert(check(cloud));
            return true;
        }

        template<typename T>
        T azimuth(const T x, const T y)
        {
            return std::atan2(y, x);
        }

        template<typename T>
        T elevation(const T x, const T y, const T z)
        {
            return std::atan2(z, std::hypot(x, y));
        }

        template<typename T>
        void project(const T x, const T y, const T z, T& az, T& el)
        {
            az = azimuth(x, y);
            el = elevation(x, y, z);
        }

        template<typename PointIt, typename ProjIt>
        void project(PointIt x_begin, PointIt x_end, ProjIt u_begin)
        {
            for (; x_begin < x_end; ++x_begin, ++u_begin)
            {
//                u_begin[0] = (azimuth(x_begin[0], x_begin[1])
//                    - azimuth_start_) / azimuth_step_;
//                u_begin[1] = (elevation(x_begin[0], x_begin[1], x_begin[2])
//                    - elevation_start_) / elevation_step_;
                project(x_begin[0], x_begin[1], x_begin[2], u_begin[0], u_begin[1]);
            }
        }

        // Azimuth, angle in xy plane, positive for x to y direction;
        // azimuth at image[:, 0], image[:, size_[1] - 1]
//        float azimuth_[2];
        float azimuth_start_;
        float azimuth_step_;
        // Elevation, angle from from xy plane to point;
        // elevation at image[0, :], image[size_[0] - 1, :]
//        float elevation_[2];
        float elevation_start_;
        float elevation_step_;
        // Cloud resolution in elevation and azimuth.
//        uint32_t size_[2];
        uint32_t height_;
        uint32_t width_;
    };

    void update_point_occupancy(
            const sensor_msgs::PointCloud2& map,
            const sensor_msgs::PointCloud2& input,
            const Vec3& origin,
            const Elem eps)
    {
        // get nearby points
        // reconstruct input surface
        // update occupancy:
        //   occupied if map point within [-eps, eps] distance,
        //   empty if within [-inf, -eps) distance from input mesh.
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
//                in_ptr += col_step * input.point_step,
//                out_ptr += output.point_step)
        {
//            std::copy(in_ptr, in_ptr + input.point_step, out_ptr);
            std::copy(in_ptr + (r * row_step * input.row_step + (c * col_step) * input.point_step),
                      in_ptr + (r * row_step * input.row_step + (c * col_step + 1) * input.point_step),
                      out_ptr + (r * output.row_step + c * output.point_step));
        }
    }
    ROS_INFO("Step-subsample %u-by-%u cloud to %u-by-%u (%.6f s).",
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
//    bool operator==(const Voxel& a, const Voxel& b)
//    {
//        return (a.x_ == b.x_) && (a.y_ == b.y_) && (a.z_ == b.z_);
//    }
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
    output.header = input.header;
//    output.height = 1;
//    output.width = 0;
    output.fields = input.fields;
    output.is_bigendian = input.is_bigendian;
    output.point_step = input.point_step;
//    output.row_step = output.width * input.point_step;
    output.is_dense = input.is_dense;

//    output.data.resize(output.height * output.width * output.point_step);
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
    ROS_INFO("Created random permutation of %lu points (%.6f s).",
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
//        const Voxel voxel = Voxel(int(x), int(y), int(z));
//        const auto it = voxels.find(voxel);
        if (voxels.find(voxel) == voxels.end())
        {
//            voxels.insert(it, voxel);
            voxels.insert(voxel);
            keep.push_back(i);
        }
    }
    ROS_INFO("Getting %lu indices to keep (%.6f s).", keep.size(), t_part.seconds_elapsed());

    // Copy selected indices.
    t_part.reset();
    output.height = 1;
    output.width = decltype(output.width)(keep.size());
    output.row_step = output.width * output.point_step;
    output.data.resize(keep.size() * output.point_step);
    in_ptr = input.data.data();
    uint8_t* out_ptr = output.data.data();
    for (Index i = 0; i < keep.size(); ++i)
    {
        std::copy(in_ptr + keep[i] * input.point_step,
                  in_ptr + (keep[i] + 1) * input.point_step,
                  out_ptr + i * output.point_step);
    }
    ROS_INFO("%lu / %lu points kept by voxel filter (%.6f s).",
             keep.size(), size_t(n_pts), t_part.seconds_elapsed());
}

}  // namespace naex

#endif //NAEX_CLOUDS_H
