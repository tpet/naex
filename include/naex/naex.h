//
// Created by petrito1 on 11/18/20.
//

#ifndef NAEX_NAEX_H
#define NAEX_NAEX_H

#include <naex/types.h>

// TODO: LO-RANSAC planes instead LS fit? Extra two points to get model.
//   LS fit on stairs: Top points would not have other points peaking above the
//   plane, points in corners could be penalized by having other points peeking
//   above the plane.
// TODO: Don't center mean for normal computation.
//   (It won't be single pass due to the need of assesing NNs already with the
//   plane.)
// TODO: Incremental index updates.
// TODO: Removing dynamic points from index.
// TODO: Incremental NN updates.

// NB: FLANN knnSearch is not thread-safe!
// https://stackoverflow.com/questions/39257005/is-opencvflannindex-knnsearch-thread-safe/39339909

/**
 * Classes
 * =======
 *
 * TODO: Use directly PointCloud2 backend with adhoc flann::Matrix wrappers
 *   for adding points (for Map and NearestNeighborGraph).
 *
 * NearestNeighborGraph
 *   Maintains
 *     list of source point matrices (position, descriptor),
 *       which can be transformed after pose-graph optimization
 *       (index should be rebuilt, but NN may be partialy usable).
 *     index-to-point mapping (used in NN index; e.g. start-to-matrix map?),
 *       including invalid point flags (nullptr) and the number of valid points
 *       per source matrix (to be able to discard whole matrices),
 *     NN index (with add/remove points),
 *     NN structure (M-by-K with invalid entries beyond search radius).
 *   Identifies valid entries after NN search (radius with limit on K).
 *
 * Map
 *   Maintains additional point vertex payload, such as
 *     normals, geometry features, traversability features, semantics
 *     occupancy, frontiers, etc.
 *   Enforces minimum point distance.
 *   Prunes dynamic points.
 *
 * Boost Graph interface?
 *   Only graph and property traits classes, and functions in boost ns.
 *   Costs need to have normals and labels.
 *
 * Planner
 *   Maintain planning configuration, map(s).
 *   Serves requests.
 *   Maintains planning results (as pred and cost lists).
 *
 * Plan
 *   Attributes: start pos(e), stop pos(e)
 *   Attributes: pred and cost lists from planning
 *
 */


// TODO: LO-RANSAC planes instead LS fit? Extra two points to get model.
//   LS fit on stairs: Top points would not have other points peaking above the
//   plane, points in corners could be penalized by having other points peeking
//   above the plane.
// TODO: Don't center mean for normal computation.
//   (It won't be single pass due to the need of assesing NNs already with the
//   plane.)
// TODO: Incremental index updates.
// TODO: Removing dynamic points from index.
// TODO: Incremental NN updates.

// NB: FLANN knnSearch is not thread-safe!
// https://stackoverflow.com/questions/39257005/is-opencvflannindex-knnsearch-thread-safe/39339909

// Input processing:
//   Voxel filter (maybe outside)
//   Surface reconstruction (normals from forward diffs)? (maybe not needed)
// Identify dynamic points for removal.
//   Projection based:
//     Project points into input frame.
//     Check plane from nearest neighbors or interpolate in inverse depth.
//   NN based:
//     Find directional NN.
//     Check plane from nearest neighbors or interpolate in inverse depth.
//   Limit incidence surface angle (surface skew).
//   Mark occluding map points as empty. (MAP WRITE)
// Update map
//   Map NN search for input query (INDEX READ)
//     Identify points to add.
//     Where the NN should be updated?
//   Copy identified points.
//   Add identified points (above) to index. (INDEX WRITE)
//   Recompute affected NN (a set from NN search) and corresponding features. (MAP WRITE)

// NB: Normal radius should be smaller than robot radius so that normals are not
// skewed in traversable points.
namespace naex
{

    // Not needed, a continuous buffer within NN graph will be better.
    template<typename Index, typename Elem>
    class PointSets
    {
    public:
        void add_points();
        Elem* get_point();
        Elem* remove_point(Index i);
        void prune_invalid();
//        std::map<Buffer<Elem>> x_buf_;
//        std::vector<flann::Matrix<Elem>> x_;
        std::vector<Index> pt_to_set_;
        std::vector<Index> set_to_start_pt_;
        std::vector<bool> valid_;
    };

    /**
     * NearestNeighborGraph
     *   Maintains
     *     list of source point matrices (position, descriptor),
     *       which can be transformed after pose-graph optimization
     *       (index should be rebuilt, but NN may be partialy usable).
     *     index-to-point mapping (used in NN index; e.g. start-to-matrix map?),
     *       including invalid point flags (nullptr) and the number of valid points
     *       per source matrix (to be able to discard whole matrices),
     *     NN index (with add/remove points),
     *     NN structure (M-by-K with invalid entries beyond search radius).
     *   Identifies valid entries after NN search (radius with limit on K).
     */
    class NearestNeighborGraph
    {
    public:
        void add_points();
        Type* get_point();
        Type* remove_point(Index i);
        void update_valid_neighbors();

//        std::map<Buffer<Type>> x_buf_;
//        std::vector<flann::Matrix<Type>> x_;
        flann::Index index_;  // FLANN point index.
        Buffer<Index> nn_buf_;
        flann::Matrix<Index> nn_;  // N-by-K index array, K nearest neighbors for each of N points.
        naex nn_count_;  // N-by-1 index array, denotes number of valid nearest neighbors.

//        sensor_msgs::PointCloud2 meta_c
    };

    /**
     * Map
     *   Maintains additional point vertex payload, such as
     *     normals, geometry features, traversability features, semantics
     *     occupancy, frontiers, etc.
     *   Enforces minimum point distance.
     *   Prunes dynamic points.
     */
    class Map
    {
    public:
//        enum MapField
//        {
//            POSITION = 1,
//        };
//        template<typename It>
//        void insert_points(It iter);  // iter[0], iter[1], iter[2]
        /**
         * Update occupancy, remove dynamic points, update map with new points.
         *
         * Occupancy update:
         * (1) based on projection model, no index -> faster,
         * (2) based on NN directions, need to create index -> slower, may be
         * more precise.
         *
         * Dynamic points:
         * Remove x if
         * (1) empty(x) - occupied(x) > thresh
         * (2) empty(X) / occupied(x) > thresh
         *
         * New points:
         * Add point if far enough from current measurements.
         *
         * Index update:
         * Fill in new points.
         * Add points to index with flann matrix wrapper.
         *
         * @param cloud
         */
        void insert_cloud(const sensor_msgs::PointCloud2& cloud)
        {

        }
//        void iterator(FieldId fid);

        flann::Index index_;  // FLANN point index.
        void add_points();
        Type* get_point();
        Type* remove_point(Index i);
        void update_valid_neighbors();

//        PointCloud2 cloud_;
//        std::vector<sensor_msgs::PointCloud2Iterator> iters_;
//        Buffer<uint8_t> data_;

        // PointCloud2 provides convenient multi-channel cloud representation
        // with arbitrary types and field iterators.
        sensor_msgs::PointCloud2 cloud_;
        // Iterators are initialized only once. Avoid using iter.end()!
        sensor_msgs::PointCloud2Iterator<Value> x_iter_;
        // NN Graph
        sensor_msgs::PointCloud2Iterator<Index> nn_count_iter_;
        sensor_msgs::PointCloud2Iterator<Index> nn_iter_;
        // Viewpoint (for occupancy assessment and measurement distance)
        sensor_msgs::PointCloud2Iterator<Value> vp_x_iter_;
        // Geometric features
        sensor_msgs::PointCloud2Iterator<Value> normal_x_iter_;
        // Normal scale is common
//        sensor_msgs::PointCloud2Iterator<Value> normal_scale_iter_;
        sensor_msgs::PointCloud2Iterator<uint8_t> normal_support_iter_;
        // Occupancy
        sensor_msgs::PointCloud2Iterator<uint8_t> empty_iter_;
        sensor_msgs::PointCloud2Iterator<uint8_t> occupied_iter_;
        // Traversability labels
        sensor_msgs::PointCloud2Iterator<uint8_t> normal_label_iter_;
        sensor_msgs::PointCloud2Iterator<uint8_t> func_label_iter_;
        // Planning costs and rewards
        sensor_msgs::PointCloud2Iterator<Value> path_cost_iter_;
        sensor_msgs::PointCloud2Iterator<Value> reward_iter_;
        sensor_msgs::PointCloud2Iterator<Value> rel_cost_iter_;
        // Index metadata
        // unindexed
        // indexed
        // to_update
        sensor_msgs::PointCloud2Iterator<uint8_t> index_state_iter_;
    };

    class Plan
    {
    public:
        Plan(size_t n);
        Buffer<Index> pred_;
        Buffer<Cost> cost_;
    };

    class Planner
    {
        void plan(const Map& map, Plan& plan);
    };
}

#endif //NAEX_NAEX_H
