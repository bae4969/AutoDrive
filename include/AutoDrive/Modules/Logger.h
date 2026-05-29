#pragma once
#include <spdlog/spdlog.h>
#include <string>
#include <exception>

namespace Logger
{
	// Initializes the global default logger: rotating file + colored console.
	// Total retained log size ~= (rotatedFiles + 1) * fileSizeKB (current + backups).
	// Call once at program start (before other logging). Returns false on failure.
	bool InitLogger(const std::string &filePath, int fileSizeKB, int rotatedFiles);
}

// Convenience macros. Use the default logger and capture source file:line (for debugging).
// Format is fmt-style: LOG_INFO("value = {}", x);
#define LOG_TRACE(...) SPDLOG_TRACE(__VA_ARGS__)
#define LOG_DEBUG(...) SPDLOG_DEBUG(__VA_ARGS__)
#define LOG_INFO(...) SPDLOG_INFO(__VA_ARGS__)
#define LOG_WARN(...) SPDLOG_WARN(__VA_ARGS__)
#define LOG_ERROR(...) SPDLOG_ERROR(__VA_ARGS__)
#define LOG_CRITICAL(...) SPDLOG_CRITICAL(__VA_ARGS__)

// Use inside a catch (...) block: logs MSG followed by the in-flight exception's reason
// (e.what()), or MSG alone for non-std exceptions. Re-throws to inspect the active exception,
// so only valid while an exception is being handled.
#define LOG_EXC_ERROR(MSG, ...) do { try { throw; } \
	catch (const std::exception &e) { LOG_ERROR(MSG ": {}" __VA_OPT__(,) __VA_ARGS__, e.what()); } \
	catch (...) { LOG_ERROR(MSG __VA_OPT__(,) __VA_ARGS__); } } while (0)
#define LOG_EXC_WARN(MSG, ...) do { try { throw; } \
	catch (const std::exception &e) { LOG_WARN(MSG ": {}" __VA_OPT__(,) __VA_ARGS__, e.what()); } \
	catch (...) { LOG_WARN(MSG __VA_OPT__(,) __VA_ARGS__); } } while (0)
#define LOG_EXC_DEBUG(MSG, ...) do { try { throw; } \
	catch (const std::exception &e) { LOG_DEBUG(MSG ": {}" __VA_OPT__(,) __VA_ARGS__, e.what()); } \
	catch (...) { LOG_DEBUG(MSG __VA_OPT__(,) __VA_ARGS__); } } while (0)
