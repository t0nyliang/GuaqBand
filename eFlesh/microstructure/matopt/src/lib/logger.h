//
// Created by Davi Colli Tozoni on 3/12/22.
// Based on quadfoam code by Jeremie Dumas.
//

#ifndef MATOPTIMIZATION_LOGGER_H
#define MATOPTIMIZATION_LOGGER_H

#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <fmt/ranges.h>
#include <fmt/ostream.h>

struct Logger {
	static std::shared_ptr<spdlog::async_logger> logger_;

	// By default, write to stdout, but don't write to any file
	static void init(bool use_cout = true, const std::string &filename = "", bool truncate = true);
};

// Retrieve current logger, or create one if not available
inline spdlog::async_logger & logger() {
	if (!Logger::logger_) {
		Logger::init();
	}
	return *Logger::logger_;
}

#endif //MATOPTIMIZATION_LOGGER_H
