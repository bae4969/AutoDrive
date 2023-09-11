#pragma once
#include <zmq.hpp>
#include <zmq_addon.hpp>
#include <thread>
#include <boost/shared_ptr.hpp>

#define PROXY_XPUB_STR "inproc://SERVER_XPUB"
#define PROXY_XSUB_STR "inproc://SERVER_XSUB"

namespace Protocol
{
	bool InitProtocol();

	class PubSubClient
	{
		std::shared_ptr<zmq::socket_t> m_subSocket = NULL;
		std::shared_ptr<zmq::socket_t> m_pubSocket = NULL;
		std::string m_pubTopic;

	public:
		bool Init(std::string pubConnStr, std::string subConnStr);
		~PubSubClient();

		void ChangePubTopic(std::string topic);
		void PublishMessage(zmq::multipart_t &msg);
		void AddSubTopic(std::string topic);
		void RemoveSubTopic(std::string topic);
		bool SubscribeMessage(zmq::multipart_t &msg);
	};
	class PubSubServer
	{
		std::thread m_subThread;
		std::shared_ptr<zmq::socket_t> m_xSubSocket = NULL;
		std::shared_ptr<zmq::socket_t> m_xPubSocket = NULL;

	public:
		bool Init(std::vector<std::string> xPubConnStrs, std::vector<std::string> xSubConnStrs);
		~PubSubServer();
	};
}
