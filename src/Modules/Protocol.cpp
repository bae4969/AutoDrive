#include "Protocol.h"

namespace Protocol
{
	using namespace std;

	static shared_ptr<zmq::context_t> CONTEXT = NULL;

	bool InitProtocol()
	{
		CONTEXT = make_shared<zmq::context_t>(4);

		return true;
	}

	bool PubSubClient::Init(string pubConnStr, string subConnStr)
	{
		if (CONTEXT == NULL)
		{
			printf("Protocol is not initialized\n");
			return false;
		}

		m_subSocket = make_shared<zmq::socket_t>(*CONTEXT, zmq::socket_type::sub);
		m_pubSocket = make_shared<zmq::socket_t>(*CONTEXT, zmq::socket_type::pub);
		m_pubTopic = "NOT DEFINED";

		m_subSocket->set(zmq::sockopt::rcvtimeo, 1);
		m_subSocket->set(zmq::sockopt::rcvhwm, 5);
		m_pubSocket->set(zmq::sockopt::sndhwm, 5);

		m_subSocket->connect(subConnStr);
		m_pubSocket->connect(pubConnStr);

		return true;
	}
	PubSubClient::~PubSubClient()
	{
		if (m_subSocket)
			m_subSocket->close();
		if (m_pubSocket)
			m_pubSocket->close();
	}

	void PubSubClient::ChangePubTopic(std::string topic)
	{
		m_pubTopic = topic;
	}
	void PubSubClient::PublishMessage(zmq::multipart_t &msg)
	{
		msg.pushstr(m_pubTopic);
		msg.send(*m_pubSocket);
	}
	void PubSubClient::AddSubTopic(std::string topic)
	{
		m_subSocket->set(zmq::sockopt::subscribe, topic);
	}
	void PubSubClient::RemoveSubTopic(std::string topic)
	{
		m_subSocket->set(zmq::sockopt::unsubscribe, topic);
	}
	bool PubSubClient::SubscribeMessage(zmq::multipart_t &msg)
	{
		return msg.recv(*m_subSocket);
	}

	bool PubSubServer::Init(vector<string> xPubConnStrs, vector<string> xSubConnStrs)
	{
		m_xSubSocket = make_shared<zmq::socket_t>(*CONTEXT, zmq::socket_type::xsub);
		m_xPubSocket = make_shared<zmq::socket_t>(*CONTEXT, zmq::socket_type::xpub);
		m_xPubSocket->set(zmq::sockopt::rcvhwm, 20);
		m_xSubSocket->set(zmq::sockopt::sndhwm, 20);

		for (string xSubConnStr : xSubConnStrs)
			m_xSubSocket->bind(xSubConnStr);
		for (string xPubConnStr : xPubConnStrs)
			m_xPubSocket->bind(xPubConnStr);

		m_subThread = thread(
			[](shared_ptr<zmq::socket_t> sub,
			   shared_ptr<zmq::socket_t> pub)
			{
				try
				{
					zmq::proxy(*sub, *pub);
				}
				catch (...)
				{
				}
			},
			m_xSubSocket,
			m_xPubSocket);

		return true;
	}
	PubSubServer::~PubSubServer()
	{
		if (m_xSubSocket)
			m_xSubSocket->close();
		if (m_xPubSocket)
			m_xPubSocket->close();
		m_subThread.join();
	}
}
