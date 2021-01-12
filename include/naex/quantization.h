
#ifndef NAEX_QUANTIZATION_H
#define NAEX_QUANTIZATION_H

namespace naex
{

template<typename F, typename I>
I quantize(F value, F scale)
{
    const auto lo = std::numeric_limits<I>::min();
    const auto hi = std::numeric_limits<I>::max();
    const F v = value / scale * hi;
    return (v < lo) ? lo : (v > hi) ? hi : static_cast<I>(v);
}

template<typename F, typename I>
F reconstruct(I value, F scale)
{
    const auto hi = std::numeric_limits<I>::max();
    return value / hi * scale;
}

template<typename F, typename I>
I quantize(F value, F min, F max)
{
    assert(min <= max);
    const auto lo = std::numeric_limits<I>::min();
    const auto hi = std::numeric_limits<I>::max();
    const F v = lo + (value - min) / (max - min) * (static_cast<F>(hi) - lo);
    return (v < lo) ? lo : (v > hi) ? hi : static_cast<I>(v);
}

template<typename F, typename I>
F reconstruct(I value, F min, F max)
{
    assert(min <= max);
    const auto lo = std::numeric_limits<I>::min();
    const auto hi = std::numeric_limits<I>::max();
    return min + (static_cast<F>(value) - lo) / (static_cast<F>(hi) - lo) * (max - min);
}

}  // namespace naex

#endif  // NAEX_QUANTIZATION_H
