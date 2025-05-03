#pragma once
#include <string>
#include <map>

class INIParser
{
private:
	std::string m_filename;
	std::map<std::string, std::map<std::string, std::string>> m_data;

public:
	bool Load(const std::string &filename);
	void Unload();

	std::string GetValue(const std::string &section, const std::string &key, const std::string &defaultValue = "");
	int GetInt(const std::string &section, const std::string &key, int defaultValue = 0);
	float GetFloat(const std::string &section, const std::string &key, float defaultValue = 0.0f);
	bool GetBool(const std::string &section, const std::string &key, bool defaultValue = false);
};
