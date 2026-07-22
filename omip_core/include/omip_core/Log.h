// Trivial stderr logging shim replacing ROS_INFO/WARN/ERROR/DEBUG (+ _STREAM,
// _NAMED variants). See PORTING_NOTES.md, section 0.3 ("ROS logging" row):
// no ROS log call in the ported packages was found to have control-flow
// side effects, so a direct stderr write is behavior-equivalent.
#ifndef OMIP_CORE_LOG_H_
#define OMIP_CORE_LOG_H_

#include <cstdio>
#include <iostream>

// printf-style variants, for the (rarer) original ROS_*_NAMED call sites
// that used a format string + args instead of a stream (e.g.
// ROS_INFO_NAMED("name", "Lost features: %3d", n)).
#define OMIP_INFO_NAMED(name, fmt, ...) do { std::fprintf(stderr, "[INFO] (%s) " fmt "\n", name, ##__VA_ARGS__); } while (0)
#define OMIP_WARN_NAMED(name, fmt, ...) do { std::fprintf(stderr, "[WARN] (%s) " fmt "\n", name, ##__VA_ARGS__); } while (0)
#define OMIP_ERROR_NAMED(name, fmt, ...) do { std::fprintf(stderr, "[ERROR] (%s) " fmt "\n", name, ##__VA_ARGS__); } while (0)
#define OMIP_DEBUG_NAMED(name, fmt, ...) do { std::fprintf(stderr, "[DEBUG] (%s) " fmt "\n", name, ##__VA_ARGS__); } while (0)

#define OMIP_INFO_STREAM(msg) do { std::cerr << "[INFO] " << msg << std::endl; } while (0)
#define OMIP_WARN_STREAM(msg) do { std::cerr << "[WARN] " << msg << std::endl; } while (0)
#define OMIP_ERROR_STREAM(msg) do { std::cerr << "[ERROR] " << msg << std::endl; } while (0)
#define OMIP_DEBUG_STREAM(msg) do { std::cerr << "[DEBUG] " << msg << std::endl; } while (0)

// _NAMED variants: original ROS macros took a logger name as the first
// argument (used for per-component log filtering, a ROS-only feature).
// Preserved here as a prefix in the printed message for auditability.
#define OMIP_INFO_STREAM_NAMED(name, msg) do { std::cerr << "[INFO] (" << name << ") " << msg << std::endl; } while (0)
#define OMIP_WARN_STREAM_NAMED(name, msg) do { std::cerr << "[WARN] (" << name << ") " << msg << std::endl; } while (0)
#define OMIP_ERROR_STREAM_NAMED(name, msg) do { std::cerr << "[ERROR] (" << name << ") " << msg << std::endl; } while (0)
#define OMIP_DEBUG_STREAM_NAMED(name, msg) do { std::cerr << "[DEBUG] (" << name << ") " << msg << std::endl; } while (0)

#endif // OMIP_CORE_LOG_H_
