#pragma once
#include <vector>
#include <thread>
#include <chrono>
#include <queue>
#include <shared_mutex>
#include <atomic>

namespace LD06
{
	typedef unsigned char uchar;
	typedef unsigned short ushort;

	struct LidarData{
		float Degree;	// deg
		float Distance;	// mm
		float Intensity;	// around 200
	};

	class Lidar
	{
	private:
		int m_fd;
		std::atomic<bool> m_isStop;
		std::vector<LidarData> m_data;
		std::thread m_recvThread;
		std::shared_mutex m_dataMutex;

		bool isValidData(int* data, int len);
		int concatBytes(int& left, int& right);
		void reciveSerialDataThreadFunc();

	public:
		bool Init();
		void Release();

		std::vector<LidarData> GetData();
	};
}
