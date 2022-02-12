#pragma once
#include <cstdint>

#ifdef SL_TRACY_ENABLE
#include "tracy/Tracy.hpp"
#define SL_ZONE_SCOPED(name) ZoneScopedN(name)
#define SL_ZONE_SCOPED_DYNAMIC(text, size) ZoneScoped ZoneName(text, size)
#define SL_FRAME_MARK FrameMark
#define SL_FRAME_MARK_START(name) FrameMarkStart(name)
#define SL_FRAME_MARK_END(name) FrameMarkEnd(name)
#else
#define SL_ZONE_SCOPED(name)
#define SL_ZONE_SCOPED_DYNAMIC(text, size)
#define SL_FRAME_MARK
#define SL_FRAME_MARK_START(name)
#define SL_FRAME_MARK_END(name)

#endif

namespace slope {

using i8  = int8_t;
using i16 = int16_t;
using i32 = int32_t;
using i64 = int64_t;

using u8  = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;

using f32 = float;
using f64 = double;

} // slope
