#include "ns3/address.h"
#include "ns3/address-utils.h"
#include "ns3/log.h"
#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/node.h"
#include "ns3/socket.h"
#include "ns3/udp-socket.h"
#include "ns3/uinteger.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/packet.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/udp-socket-factory.h"
#include "tcp-server-application.h"
#include "ns3/string.h"
#include "ns3/inet-socket-address.h"
#include "ns3/names.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("TcpServerApplication");

//NS_OBJECT_ENSURE_REGISTERED (TcpServerApplication);

TypeId 
TcpServerApplication::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TcpServerApplication")
    .SetParent<Application> ()
    .SetGroupName("Applications")
    .AddConstructor<TcpServerApplication> ()
    .AddAttribute ("Local",
                   "The Address on which to Bind the rx socket.",
                   AddressValue (),
                   MakeAddressAccessor (&TcpServerApplication::m_local),
                   MakeAddressChecker ())
    .AddAttribute ("SendSize", "The amount of data to send each time.",
                   UintegerValue (512),
                   MakeUintegerAccessor (&TcpServerApplication::m_sendSize),
                   MakeUintegerChecker<uint32_t> (1))
    .AddAttribute ("MaxTxBytes",
                   "The total number of bytes to send. "
                   "Once these bytes are sent, "
                   "no data  is sent again. The value zero means "
                   "that there is no limit.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&TcpServerApplication::m_maxBytes),
                   MakeUintegerChecker<uint32_t> ())
    .AddTraceSource ("Tx", "A new packet is created and is sent",
                     MakeTraceSourceAccessor (&TcpServerApplication::m_txTrace),
                     "ns3::Packet::TracedCallback")
    .AddTraceSource ("Rx",
                     "A packet has been received",
                     MakeTraceSourceAccessor (&TcpServerApplication::m_rxTrace),
                     "ns3::Packet::PacketAddressTracedCallback")
  ;
  return tid;
}

TcpServerApplication::TcpServerApplication ()
{
  NS_LOG_FUNCTION (this);
  m_socket = 0;
  m_totalRx = 0;
  m_totBytes = 0;
}

TcpServerApplication::~TcpServerApplication()
{
  NS_LOG_FUNCTION (this);
}

uint32_t TcpServerApplication::GetTotalRx () const
{
  NS_LOG_FUNCTION (this);
  return m_totalRx;
}

void
TcpServerApplication::SetMaxBytes (uint32_t maxBytes)
{
  NS_LOG_FUNCTION (this << maxBytes);
  m_maxBytes = maxBytes;
}

Ptr<Socket>
TcpServerApplication::GetListeningSocket (void) const
{
  NS_LOG_FUNCTION (this);
  return m_socket;
}

std::list<Ptr<Socket> >
TcpServerApplication::GetAcceptedSockets (void) const
{
  NS_LOG_FUNCTION (this);
  return m_socketList;
}

void TcpServerApplication::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_socket = 0;
  m_socketList.clear ();
  Application::DoDispose ();
}


// Application Methods
void TcpServerApplication::StartApplication ()    // Called at time specified by Start
{
  NS_LOG_FUNCTION (this);
  // Create the socket if not already
  m_tid = TypeId::LookupByName ("ns3::TcpSocketFactory");
  if (!m_socket)
    {
      m_socket = Socket::CreateSocket (GetNode (), m_tid);
      m_socket->Bind (m_local);
      m_socket->Listen ();
      //m_socket->ShutdownSend ();
    }

  m_socket->SetRecvCallback (MakeCallback (&TcpServerApplication::HandleRead, this));
  m_socket->SetAcceptCallback (
    MakeNullCallback<bool, Ptr<Socket>, const Address &> (),
    MakeCallback (&TcpServerApplication::HandleAccept, this));
  m_socket->SetCloseCallbacks (
    MakeCallback (&TcpServerApplication::HandlePeerClose, this),
    MakeCallback (&TcpServerApplication::HandlePeerError, this));
}

void TcpServerApplication::StopApplication ()     // Called at time specified by Stop
{
  NS_LOG_FUNCTION (this);
  while(!m_socketList.empty ()) //these are accepted sockets, close them
    {
      Ptr<Socket> acceptedSocket = m_socketList.front ();
      m_socketList.pop_front ();
      acceptedSocket->Close ();
      sv_connected.clear();
    }
  if (m_socket) 
    {
      m_socket->Close ();
      m_socket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
    }
}

void TcpServerApplication::SendData (Ptr<Socket> sock, Address from)
{
  NS_LOG_FUNCTION (this << sock);

  if(sv_connected.find(from) == sv_connected.end())
  {
    // do nothing
  }
  else if(sv_connected[from])
  {
    while (m_maxBytes == 0 || m_totBytes < m_maxBytes)
      { // Time to send more
        uint32_t toSend = m_sendSize;
        // Make sure we don't send too many
        if (m_maxBytes > 0)
          {
            toSend = std::min (m_sendSize, m_maxBytes - m_totBytes);
          }
        NS_LOG_LOGIC ("sending packet at " << Simulator::Now ());
        Ptr<Packet> packet = Create<Packet> (toSend);
        m_txTrace (packet);
        int actual = sock->SendTo (packet, 0, from);
        if (actual > 0)
          {
            m_totBytes += actual;
          }
        // We exit this loop when actual < toSend as the send side
        // buffer is full. The "DataSent" callback will pop when
        // some buffer space has freed ip.
        if ((unsigned)actual != toSend)
          {
            break;
          }
      }
      // Check if time to close (all sent)
      /*if (m_totBytes == m_maxBytes && sv_connected[from])
        {
          sv_connected[from] = false;
          m_totBytes = 0;
          sock->Close ();
        }*/
    }
}

void TcpServerApplication::HandleRead (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
  Ptr<Packet> packet, pktSend;
  Address from;
  while ((packet = socket->RecvFrom (from)))
    {
      if (packet->GetSize () == 0)
        { //EOF
          break;
        }
      m_totalRx += packet->GetSize ();
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
      if (packet->GetSize () == 13)
        { //EOF
          SendData(socket, from);
        }
    }
}


void TcpServerApplication::HandlePeerClose (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
}
 
void TcpServerApplication::HandlePeerError (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
}
 

void TcpServerApplication::HandleAccept (Ptr<Socket> s, const Address& from)
{
  NS_LOG_FUNCTION (this << s << from);
  s->SetRecvCallback (MakeCallback (&TcpServerApplication::HandleRead, this));
  //s->SetSendCallback (MakeCallback (&TcpServerApplication::DataSend, this));
  m_socketList.push_back (s);
  sv_connected[from] = true;
}

TcpServerApplicationHelper::TcpServerApplicationHelper (Address address)
{
  m_factory.SetTypeId (TcpServerApplication::GetTypeId());
  SetAttribute ("Local", AddressValue (address));
}

void 
TcpServerApplicationHelper::SetAttribute (std::string name, const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
TcpServerApplicationHelper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
TcpServerApplicationHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
TcpServerApplicationHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application>
TcpServerApplicationHelper::InstallPriv (Ptr<Node> node) const
{
  Ptr<Application> app = m_factory.Create<Application> ();
  node->AddApplication (app);

  return app;
}

} // Namespace ns3