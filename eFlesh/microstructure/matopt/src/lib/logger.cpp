//
// Created by Davi Colli Tozoni on 3/12/22.
// Based on quadfoam code by Jeremie Dumas.
//

#include "logger.h"
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <memory>
#include <mutex>
#include <iostream>

std::shared_ptr<spdlog::async_logger> Logger::logger_;

// Asynchronous logger with multi sinks
void Logger::init(bool use_cout, const std::string &filename, bool truncate) {
	spdlog::init_thread_pool(8192, 1);
	std::vector<spdlog::sink_ptr> sinks;
	if (use_cout) {
		sinks.emplace_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt >());
	}
	if (!filename.empty()) {
		sinks.emplace_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>(filename, truncate));
	}
	logger_ = std::make_shared<spdlog::async_logger>("matopt", sinks.begin(), sinks.end(),
		spdlog::thread_pool(), spdlog::async_overflow_policy::block);
	spdlog::drop("matopt");
	spdlog::register_logger(logger_);
}
