#pragma once

#include <cassert>
#include <cmath>
#include <naex/hash.h>
#include <vector>
#include <unordered_map>

namespace naex
{
namespace grid
{

typedef uint32_t CellId;
typedef float Cost;

template<typename T>
struct Point2
{
    Point2(T x, T y):
        x(x),
        y(y)
    {}
    Point2();

    Point2& operator=(const Point2& other)
    {
        x = other.x;
        y = other.y;
        return *this;
    }

    bool operator==(const Point2& other) const
    {
        return x == other.x && y == other.y;
    }

    T x;
    T y;
};
template struct Point2<float>;
template struct Point2<int16_t>;

template<>
Point2<float>::Point2():
    Point2(std::numeric_limits<float>::quiet_NaN(),
           std::numeric_limits<float>::quiet_NaN())
{}
template<>
Point2<int16_t>::Point2():
    Point2(0, 0)
{}

typedef Point2<float> Point2f;
typedef Point2<int16_t> Point2s;
typedef Point2s Cell;

template<typename T>
struct Point2Hasher
{
    std::size_t operator()(const Point2<T>& v) const
    {
        std::size_t seed = 0;
        hash_combine(seed, v.x);
        hash_combine(seed, v.y);
        return seed;
    }
};

template struct Point2Hasher<float>;
template struct Point2Hasher<int16_t>;

typedef Point2Hasher<float> Point2fHasher;
typedef Point2Hasher<int16_t> Point2sHasher;
typedef Point2sHasher CellHasher;

struct Costs
{
    Costs(Cost c0 = std::numeric_limits<Cost>::quiet_NaN(),
          Cost c1 = std::numeric_limits<Cost>::quiet_NaN(),
          Cost c2 = std::numeric_limits<Cost>::quiet_NaN(),
          Cost c3 = std::numeric_limits<Cost>::quiet_NaN()):
        data{c0, c1, c2, c3}
    {}
    Cost data[4];

    Costs& operator=(const Costs& other)
    {
        std::copy(other.data, other.data + size(), data);
        return *this;
    }
    template<typename T>
    Costs& operator=(const std::vector<T>& costs)
    {
        for (size_t i = 0; i < size(); ++i)
        {
            data[i] = (i < costs.size())
                ? costs[i]
                : std::numeric_limits<Cost>::quiet_NaN();
        }
        return *this;
    }
    const Cost& operator[](size_t i) const
    {
        return data[i];
    }
    Cost& operator[](size_t i)
    {
        return data[i];
    }
    size_t size() const
    {
        return 4;
    }
    Cost total() const
    {
        Cost total = 0;
        for (size_t i = 0; i < size(); ++i)
        {
            if (!std::isnan(data[i]))
            {
                total += data[i];
            }
        }
        return total;
    }
};

// TODO: Move max costs and total to grid.
// TODO: Add costs weights for total.
// TODO: Add required flags for total.
class Grid
{
public:
    Grid(float cell_size = 1.f, float forget_factor = 1.f):
        cell_size_(cell_size),
        forget_factor_(forget_factor)
    {}

    bool hasCell(const Cell& c) const
    {
        return cell_to_id_.find(c) != cell_to_id_.end();
    }
    void createCell(const Cell& c)
    {
        cell_to_id_[c] = size();
        id_to_cell_.push_back(c);
        id_to_costs_.push_back(Costs());
    }

    const Cell& cell(const CellId& id) const
    {
        assert(id < size());
        return id_to_cell_[id];
    }
    Cell& cell(const CellId& id)
    {
        assert(id < size());
        return id_to_cell_[id];
    }
    Point2f point(const CellId& id) const
    {
        return cellToPoint(cell(id));
    }

    CellId& cellId(const Cell& c)
    {
        if (!hasCell(c))
        {
            createCell(c);
        }
        return cell_to_id_[c];
    }
    const CellId& cellId(const Cell& c) const
    {
        assert(hasCell(c));
        return cell_to_id_.find(c)->second;
    }

    Cell pointToCell(const Point2f& p) const
    {
        return Cell(std::floor(p.x / cell_size_), std::floor(p.y / cell_size_));
    }
    Point2f cellToPoint(const Cell& c) const
    {
        return Point2f((c.x + 0.5f) * cell_size_ , (c.y + 0.5f) * cell_size_);
    }

    const Costs& costs(const CellId& id) const
    {
        assert(id < size());
        return id_to_costs_[id];
    }
    Costs& costs(const CellId& id)
    {
        assert(id < size());
        return id_to_costs_[id];
    }
    Costs& cellCosts(const Cell& c)
    {
        return costs(cellId(c));
    }
    Costs& pointCosts(const Point2f& p)
    {
        return cellCosts(pointToCell(p));
    }

    Costs& updateCellCost(Cell c, int level, Cost cost)
    {
        Costs& costs = cellCosts(c);
        if (std::isfinite(costs.data[level]))
        {
            Cost w0 = (1. - forget_factor_);
            Cost w1 = forget_factor_;
            costs.data[level] = w0 * costs.data[level] + w1 * cost;
        }
        else
        {
            costs.data[level] = cost;
        }
        return costs;
    }
    Costs& updatePointCost(Point2f p, int level, Cost cost)
    {
        return updateCellCost(pointToCell(p), level, cost);
    }

    bool empty() const
    {
        return id_to_costs_.empty();
    }
    size_t size() const
    {
        return id_to_costs_.size();
    }
    
protected:
    float cell_size_;
    float forget_factor_;

    // CellId to Costs
    std::vector<Costs> id_to_costs_;
    // CellId to Cell
    std::vector<Cell> id_to_cell_;
    // Cell to CellId
    std::unordered_map<Cell, CellId, CellHasher> cell_to_id_;
};

}  // namespace grid
}  // namespace naex
