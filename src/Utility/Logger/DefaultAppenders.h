//
// Created by User on 20.04.2025.
//

#ifndef DEFAULT_APPENDERS_H
#define DEFAULT_APPENDERS_H

#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>

#include "ILogAppender.h"

namespace details
{
class FileAppender final : public ILogAppender
{
public:
	explicit FileAppender(const std::string& filename)
	{
		using namespace std::filesystem;

		path logDir = current_path() / "logs";
		path fullPath = logDir / filename;
		m_filePath = fullPath.string();

		try
		{
			create_directories(logDir);
		}
		catch (const std::exception& e)
		{
			std::cerr << "Failed to create log directory \'" << logDir.string() << "\': " << e.what() << std::endl;
			throw;
		}

		m_logFile.open(m_filePath, std::ios::out | std::ios::app);

		if (!m_logFile.is_open())
		{
			throw std::runtime_error("Failed to open log file \'" + m_filePath + "\'");
		}
	}

	~FileAppender() override
	{
		if (m_logFile.is_open())
		{
			m_logFile.close();
		}
	}

	void Append(LogLevel level, const std::string& message) override
	{
		std::lock_guard lock(m_mutex);

		m_logFile << message << std::endl;

		m_logFile.flush();
	}

private:
	std::mutex m_mutex;
	std::ofstream m_logFile;
	std::string m_filePath;
};

class ConsoleAppender final : public ILogAppender
{
public:
	void Append(LogLevel level, const std::string& message) override
	{
		std::lock_guard lock(m_mutex);

		level > LogLevel::Warning
			? std::cerr << message << std::endl
			: std::cout << message << std::endl;
	}

private:
	std::mutex m_mutex;
};
} // namespace details

#endif // DEFAULT_APPENDERS_H
