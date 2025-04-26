#include "INIParser.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>

bool INIParser::Load(const std::string &filename) {
	m_filename = filename;
	std::ifstream file(m_filename);
	if (!file.is_open()) {
		std::cerr << "Error opening file: " << m_filename << std::endl;
		return false;
	}

	std::string line, currentSection;
	while (std::getline(file, line)) {
		// Remove comments
		size_t commentPos = line.find(';');
		if (commentPos != std::string::npos) {
			line.erase(commentPos);
		}

		// Trim whitespace
		line.erase(0, line.find_first_not_of(" \t"));
		line.erase(line.find_last_not_of(" \t") + 1);

		if (line.empty()) continue;

		// Check for section header
		if (line[0] == '[' && line[line.size() - 1] == ']') {
			currentSection = line.substr(1, line.size() - 2);
			m_data[currentSection] = {};
		} else {
			// Split key and value
			size_t equalPos = line.find('=');
			if (equalPos != std::string::npos) {
				std::string key = line.substr(0, equalPos);
				std::string value = line.substr(equalPos + 1);
				key.erase(0, key.find_first_not_of(" \t"));
				key.erase(key.find_last_not_of(" \t") + 1);
				value.erase(0, value.find_first_not_of(" \t"));
				value.erase(value.find_last_not_of(" \t") + 1);

				if (!currentSection.empty()) {
					m_data[currentSection][key] = value;
				}
			}
		}
	}

	file.close();
	return true;
}
void INIParser::Unload() {
	m_data.clear();
}

std::string INIParser::GetValue(const std::string &section, const std::string &key, const std::string &defaultValue) {
	auto sectionIt = m_data.find(section);
	if (sectionIt != m_data.end()) {
		auto keyIt = sectionIt->second.find(key);
		if (keyIt != sectionIt->second.end()) {
			return keyIt->second;
		}
	}
	return defaultValue;
}
int INIParser::GetInt(const std::string &section, const std::string &key, int defaultValue) {
	std::string value = GetValue(section, key);
	try {
		return std::stoi(value);
	} catch (const std::invalid_argument &) {
		return defaultValue;
	}
}
float INIParser::GetFloat(const std::string &section, const std::string &key, float defaultValue) {
	std::string value = GetValue(section, key);
	try {
		return std::stof(value);
	} catch (const std::invalid_argument &) {
		return defaultValue;
	}
}
bool INIParser::GetBool(const std::string &section, const std::string &key, bool defaultValue) {
	std::string value = GetValue(section, key);
	if (value == "true" || value == "1") {
		return true;
	} else if (value == "false" || value == "0") {
		return false;
	}
	return defaultValue;
}
