#include "ns3/log.h"
#include "ns3/address.h"
#include "ns3/node.h"
#include "ns3/nstime.h"
#include "ns3/socket.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/tcp-socket-factory.h"
#include "tcp-client-application.h"
#include "ns3/names.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("TcpClientApplication");

NS_OBJECT_ENSURE_REGISTERED (TcpClientApplication);

TypeId
TcpClientApplication::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TcpClientApplication")
    .SetParent<Application> ()
    .SetGroupName("Applications") 
    .AddConstructor<TcpClientApplication> ()
    .AddAttribute ("Remote", "The address of the destination",
                   AddressValue (),
                   MakeAddressAccessor (&TcpClientApplication::m_peer),
                   MakeAddressChecker ())
    .AddAttribute ("MaxRxBytes",
                   "The total number of bytes expected to recieve",
                   UintegerValue (0),
                   MakeUintegerAccessor (&TcpClientApplication::m_maxRx),
                   MakeUintegerChecker<uint32_t> ())
    .AddTraceSource ("Tx", "A new packet is created and is sent",
                     MakeTraceSourceAccessor (&TcpClientApplication::m_txTrace),
                     "ns3::Packet::TracedCallback")
    .AddTraceSource ("Rx",
                     "A packet has been received",
                     MakeTraceSourceAccessor (&TcpClientApplication::m_rxTrace),
                     "ns3::Packet::PacketAddressTracedCallback")
  ;
  return tid;
}


TcpClientApplication::TcpClientApplication ()
  : m_socket (0),
    m_connected (false)
{
  NS_LOG_FUNCTION (this);
  m_totalRx = 0;
  m_completeRx = 0;
}

TcpClientApplication::~TcpClientApplication ()
{
  NS_LOG_FUNCTION (this);
}

uint32_t TcpClientApplication::GetTotalRx () const
{
  NS_LOG_FUNCTION (this);
  return m_totalRx;
}

uint32_t TcpClientApplication::GetCompleteRx () const
{
  NS_LOG_FUNCTION (this);
  return m_completeRx;
}

Ptr<Socket>
TcpClientApplication::GetSocket (void) const
{
  NS_LOG_FUNCTION (this);
  return m_socket;
}

void
TcpClientApplication::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_socket = 0;
  Application::DoDispose ();
}

void TcpClientApplication::StartConnection (void)
{
  NS_LOG_FUNCTION (this);

  m_socket = 0;

  /*if (m_socket != 0)
    {
      m_socket->Close ();
      m_connected = false;
    }*/

  m_connected = false;
  m_totalRx = 0;

  m_tid = TypeId::LookupByName ("ns3::TcpSocketFactory");
  // Create the socket if not already
  if (!m_socket)
    {
      m_socket = Socket::CreateSocket (GetNode (), m_tid);

      // Fatal error if socket type is not NS3_SOCK_STREAM or NS3_SOCK_SEQPACKET
      if (m_socket->GetSocketType () != Socket::NS3_SOCK_STREAM &&
          m_socket->GetSocketType () != Socket::NS3_SOCK_SEQPACKET)
        {
          NS_FATAL_ERROR ("Use TCP instead of UDP.");
        }
      /*if (InetSocketAddress::IsMatchingType (m_peer))
        {
          m_socket->Bind ();
        }*/

      m_socket->Connect (m_peer);
      //m_socket->ShutdownRecv ();
      m_socket->SetConnectCallback (
        MakeCallback (&TcpClientApplication::ConnectionSucceeded, this),
        MakeCallback (&TcpClientApplication::ConnectionFailed, this));
      m_socket->SetSendCallback (
        MakeCallback (&TcpClientApplication::DataSend, this));
      m_socket->SetRecvCallback (
        MakeCallback (&TcpClientApplication::HandleRead, this));
    }
  if (m_connected)
    {
      SendData ();
    }
}

// Application Methods
void TcpClientApplication::StartApplication (void) // Called at time specified by Start
{
  NS_LOG_FUNCTION (this);
}

void TcpClientApplication::StopApplication (void) // Called at time specified by Stop
{
  NS_LOG_FUNCTION (this);

  if (m_socket != 0)
    {
      m_socket->Close ();
      m_connected = false;
    }
  else
    {
      NS_LOG_WARN ("TcpClientApplication found null socket to close in StopApplication");
    }
}


// Private helpers

void TcpClientApplication::SendData (void)
{
  NS_LOG_FUNCTION (this);

  if( m_totalRx == 0 )
  {
    NS_LOG_LOGIC ("sending packet at " << Simulator::Now ());
    Ptr<Packet> packet = Create<Packet> (13);
    m_txTrace (packet);
    m_socket->Send (packet);
    HandleRead(m_socket);
  }
  
  //m_socket->Close ();
  //m_connected = false;
}

void TcpClientApplication::HandleRead (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
  Ptr<Packet> packet;
  Address from;
  while ((packet = socket->RecvFrom (from)))
    {
      if (packet->GetSize () == 0)
        { //EOF
          break;
        }
      m_totalRx += packet->GetSize ();
      m_completeRx += packet->GetSize ();
      if (InetSocketAddress::IsMatchingType (from))
        {
          NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds ()
                       << "s packet sink received "
                       <<  packet->GetSize () << " bytes from "
                       << InetSocketAddress::ConvertFrom(from).GetIpv4 ()
                       << " port " << InetSocketAddress::ConvertFrom (from).GetPort ()
                       << " total Rx " << m_totalRx << " bytes");
        }
      m_rxTrace (packet, from);
      NS_LOG_LOGIC ("sending packet at " << Simulator::Now ());
      Ptr<Packet> pkt = Create<Packet> (13);
      m_txTrace (pkt);
      socket->Send (pkt);
    }
}

void TcpClientApplication::ConnectionSucceeded (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
  NS_LOG_LOGIC ("TcpClientApplication Connection succeeded");
  m_connected = true;
  SendData ();
}

void TcpClientApplication::ConnectionFailed (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
  NS_LOG_LOGIC ("TcpClientApplication, Connection Failed");
}

void TcpClientApplication::DataSend (Ptr<Socket>, uint32_t)
{
  NS_LOG_FUNCTION (this);

  if (m_connected)
    { // Only send new data if the connection has completed
      SendData ();
    }
}

/************************************************************************************/

TcpClientApplicationHelper::TcpClientApplicationHelper (Address address)
{
  m_factory.SetTypeId (TcpClientApplication::GetTypeId());
  SetAttribute ("Remote", AddressValue (address));
}

void
TcpClientApplicationHelper::SetAttribute (std::string name, const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
TcpClientApplicationHelper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
TcpClientApplicationHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
TcpClientApplicationHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application>
TcpClientApplicationHelper::InstallPriv (Ptr<Node> node) const
{
  Ptr<Application> app = m_factory.Create<Application> ();
  node->AddApplication (app);

  return app;
}

} // Namespace ns3