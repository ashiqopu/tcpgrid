/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Md Ashiqur Rahman: University of Arizona.
 * 
 * Status: Channel and IP change works
 **/

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/config-store-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"

#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"
#include "ns3/yans-wifi-phy.h"
#include "ns3/mobility-module.h"
#include "ns3/constant-velocity-mobility-model.h"

#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-nix-vector-helper.h"
#include "ns3/ipv4-nix-vector-routing.h"
#include "ns3/olsr-helper.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <map>

#include "tcp-server-application.h"
#include "tcp-client-application.h"

#define rtOffset 1
#define TOTAL_WIRED_LINKS 65
#define TOP_BOUNDARY 500.0
#define BOTTOM_BOUNDARY -500.0
#define RIGHT_BOUNDARY 500.0
#define LEFT_BOUNDARY -500.0
#define INTERSECTION_THRESHOLD 2.0
#define STATIONS 10
#define MAX_GAP 50.0
#define data_size 0.1
#define MAX_AP_TO_VISIT 10

NS_LOG_COMPONENT_DEFINE ("tcp-wifi-ap");

using namespace ns3;

uint16_t sport = 4000;
uint16_t cport = 5000;

double position_interval = 0.01;

typedef struct {
  double x, y, range;
} AccessPoint;

enum Direction { NORTH, EAST, WEST, SOUTH };

typedef struct {
  int curAP;
  int files15;
  int files50;
  int files100;
  int files400;
  int nbytes;
  int totBytes;
  std::vector< std::vector <std::string> > ipList;
} StationNode;

int staNodes = STATIONS;
int apNodes = 36;
int l1Routers = 12;
int l2Routers = 4;
//int l3Routers = 1; 
int totalRouters = l1Routers + l2Routers;// + l3Routers;
double spacing = 200.0;
double range = 100.0;
int fileSize = 100; // KB 15,50,100,400
double endtime = 100.0;
double speed = 24.5872; //setting speed to span full sim time
int seedVal = 1;
bool verbose = false;
uint32_t maxBytes = (15*1024);

std::vector<YansWifiPhyHelper> wifiPhy(apNodes,YansWifiPhyHelper::Default ());
std::vector< Ptr<YansWifiChannel> > wifiPhyChn(apNodes);
std::vector<NetDeviceContainer> passengerDevice(apNodes);
Ipv4InterfaceContainer wifiInterfaces;

std::vector<AccessPoint> apProp(apNodes);
std::vector<StationNode> staProp(staNodes);
//std::vector< std::vector<ApplicationContainer> > senderApps(staNodes, std::vector<ApplicationContainer>(apNodes));
//std::vector<ApplicationContainer> sinkApps(staNodes);
std::vector<ApplicationContainer> clientApps(apNodes);
ApplicationContainer serverApp;
//std::vector<std::vector<ApplicationContainer>> senderApps;

std::map<std::string,bool> ipMap;

std::vector< std::queue<Direction> > staTravelList(staNodes);

NodeContainer serverNode;
NodeContainer clientNodes;

std::vector<NodeContainer> passengerNodes (apNodes);

// consumer direction
std::vector<Direction> consumerDir(STATIONS);
std::vector<bool> calcDir(STATIONS);

// Topology checkpoints
void generateIntersections()
{
  int i=0;
  for(int Ypos = -500; Ypos <= 500; Ypos += 200)
  {
      for(int Xpos = -500; Xpos <= 500; Xpos += 200)
      {
          apProp[i].x = Xpos;
          apProp[i].y = Ypos;
          apProp[i].range = range;
          i++;
      }
  }
}

void generateStaTravelList()
{

}

void changeChannelAndAddress(Ptr<Node> node, int id, int apID, bool ipDeleted, bool ipAdded)
{
  //std::cout<< ">>>> Checkpoint 1\n";
  Vector thePos = node->GetObject<ConstantVelocityMobilityModel>()->GetPosition();
  Vector theVel = node->GetObject<ConstantVelocityMobilityModel>()->GetVelocity();
  
  //std::cout << "X: " << thePos.x <<  " ; Y: " << thePos.y <<"\n";
  
  int cID = id;

  //Vector pos = node->GetObject<ConstantVelocityMobilityModel>()->GetPosition();

  // Manhattan Mobility Model
  //srand((unsigned)time(NULL));
  
  double uniformRand01 = ((double) rand() / (RAND_MAX*1.0)) ;

  double dist = 999999.0;

  if(apID != -1)
  {
    dist = sqrt( (apProp[apID].x-thePos.x) * (apProp[apID].x-thePos.x)
                +(apProp[apID].y-thePos.y) * (apProp[apID].y-thePos.y) ); 
  }

  if(dist > range && ipAdded && apID != -1)
  {
    // Delete IP
    //std::cout << "stopping apId = " << apID << " staID = " << id << std::endl;
    Ptr<TcpClientApplication> client = DynamicCast<TcpClientApplication> (clientApps[apID].Get (id));
      
    int nowBytes = client->GetTotalRx ();

    //std::cout << nowBytes - staProp[id].nbytes << std::endl;
    staProp[id].files15 += (nowBytes) / (15 * 1024);
    staProp[id].files50 += (nowBytes) / (50 * 1024);
    staProp[id].files100 += (nowBytes) / (100 * 1024);
    staProp[id].files400 += (nowBytes) / (400 * 1024); 

    Vector psgPos = passengerNodes[apID].Get(id)->GetObject<ConstantVelocityMobilityModel>()->GetPosition();
    Vector psgVel = passengerNodes[apID].Get(id)->GetObject<ConstantVelocityMobilityModel>()->GetVelocity();
    
    psgPos.x = 800; psgPos.y = 800;
    psgVel.x = 0; psgVel.y = 0;
    
    ipDeleted = true;
    ipAdded = false;
    apID = -1;
    staProp[id].curAP = -1;
  }

  else if(ipDeleted)
  {
    double mindist = 999999999.0;
    int newAPID = -1;
    for(int i=0; i<apNodes; i++)
    {
      dist = sqrt( (apProp[i].x-thePos.x) * (apProp[i].x-thePos.x)
                 + (apProp[i].y-thePos.y) * (apProp[i].y-thePos.y) );

      if( dist < mindist )
      {
        mindist = dist;
        newAPID = i;
      }
    }

    if( mindist <= range )
    {
      //------------------------------- Channel + IP Change ------------------------------------//
      // Channel change
      /*
      PointerValue tmpPtr;
      staDevice.Get(id)->GetAttribute("Phy", tmpPtr);
      Ptr<Object> wifiObj = tmpPtr.GetObject();
      Ptr<YansWifiPhy> yansPhy = wifiObj->GetObject<YansWifiPhy>();
      yansPhy->SetChannel(wifiPhyChn[newAPID]);
      */
      
      Ptr<TcpClientApplication> client = DynamicCast<TcpClientApplication> (clientApps[newAPID].Get (id));
      
      staProp[id].nbytes = client->GetTotalRx ();
      //staProp[id].totBytes += client->GetTotalRx ();

      Vector psgPos = passengerNodes[newAPID].Get(id)->GetObject<ConstantVelocityMobilityModel>()->GetPosition();
      Vector psgVel = passengerNodes[newAPID].Get(id)->GetObject<ConstantVelocityMobilityModel>()->GetVelocity();
      
      psgPos.x = thePos.x; psgPos.y = thePos.y;
      psgVel.x = theVel.x; psgVel.y = theVel.y;
      
      client->StartConnection();

      ipDeleted = false;
      ipAdded = true;
      apID = newAPID;
      staProp[id].curAP = apID;
    }
  }
  
  //std::cout<< ">>>> Checkpoint 2\n";
  if (ipAdded && apID != -1)
  {
    Ptr<TcpClientApplication> client = DynamicCast<TcpClientApplication> (clientApps[apID].Get (id));
    if( client->GetTotalRx() >= maxBytes && ipAdded)
    {
      //client->StartConnection();
    }
  }

  double minDist = 999999.0;

  int curIntersection = 0;

  for(int i=0; i<36; i++)
  {
      double curDist = sqrt( (apProp[i].x - thePos.x) * (apProp[i].x - thePos.x)
                            +(apProp[i].y - thePos.y) * (apProp[i].y - thePos.y) );
      if( curDist < minDist )
      {
        minDist = curDist;
        curIntersection = i;
      }
  }
  
  //std::cout<< ">>>> Checkpoint 3\n";

  if( minDist > INTERSECTION_THRESHOLD && calcDir[cID] == false )
  {
    calcDir[cID] = true;
  }
  else if( minDist <= INTERSECTION_THRESHOLD && calcDir[cID] == true && apID != -1)
  {
    Vector pos (apProp[curIntersection].x, apProp[curIntersection].y, 0);
      Vector vel (0, 0, 0);

      // directions and boundary checkers and Manhattan_Mobility
      if ( consumerDir[cID] == NORTH )
      {
        if ( pos.y < TOP_BOUNDARY && pos.x > LEFT_BOUNDARY && pos.x < RIGHT_BOUNDARY )
        {
          if( uniformRand01 <= 0.50 )
        {
          vel.x = theVel.x;
          vel.y = theVel.y;
        }
        else if( uniformRand01 > 0.50 && uniformRand01 <= 0.75 )
        {
          vel.x = speed;
          vel.y = 0;
          consumerDir[cID] = EAST;
        }
        else {
          vel.x = -1.0 * speed;
          vel.y = 0;
          consumerDir[cID] = WEST;
        }
        }
        else if ( pos.y >= TOP_BOUNDARY && pos.x > LEFT_BOUNDARY && pos.x < RIGHT_BOUNDARY )
        {
          if( uniformRand01 <= 0.50 )
        {
          vel.x = speed;
          vel.y = 0;
          consumerDir[cID] = EAST;
        }
        else {
          vel.x = -1.0 * speed;
          vel.y = 0;
          consumerDir[cID] = WEST;
        }
        }
        else if ( pos.y < TOP_BOUNDARY && pos.x <= LEFT_BOUNDARY && pos.x < RIGHT_BOUNDARY )
        {
          if( uniformRand01 <= 0.50 )
        {
          vel.x = theVel.x;
          vel.y = theVel.y;
        }
        else {
          vel.x = speed;
          vel.y = 0;
          consumerDir[cID] = EAST;
        }
        }
        else if ( pos.y < TOP_BOUNDARY && pos.x > LEFT_BOUNDARY && pos.x >= RIGHT_BOUNDARY )
        {
          if( uniformRand01 <= 0.50 )
        {
          vel.x = theVel.x;
          vel.y = theVel.y;
        }
        else {
          vel.x = -1.0 * speed;
          vel.y = 0;
          consumerDir[cID] = WEST;
        }
        }
        else if ( pos.y >= TOP_BOUNDARY && pos.x <= LEFT_BOUNDARY && pos.x < RIGHT_BOUNDARY )
        {
        vel.x = speed;
        vel.y = 0;
        consumerDir[cID] = EAST;
        }
        else if ( pos.y >= TOP_BOUNDARY && pos.x > LEFT_BOUNDARY && pos.x >= RIGHT_BOUNDARY )
        {
        vel.x = -1.0 * speed;
        vel.y = 0;
        consumerDir[cID] = WEST;
        }
      }
      else if ( consumerDir[cID] == SOUTH )
      {
        if ( pos.y > BOTTOM_BOUNDARY && pos.x > LEFT_BOUNDARY && pos.x < RIGHT_BOUNDARY )
        {
          if( uniformRand01 <= 0.50 )
        {
          vel.x = theVel.x;
          vel.y = theVel.y;
        }
        else if( uniformRand01 > 0.50 && uniformRand01 <= 0.75 )
        {
          vel.x = speed;
          vel.y = 0;
          consumerDir[cID] = EAST;
        }
        else {
          vel.x = -1.0 * speed;
          vel.y = 0;
          consumerDir[cID] = WEST;
        }
        }
        else if ( pos.y <= BOTTOM_BOUNDARY && pos.x > LEFT_BOUNDARY && pos.x < RIGHT_BOUNDARY )
        {
          if( uniformRand01 <= 0.50 )
        {
          vel.x = speed;
          vel.y = 0;
          consumerDir[cID] = EAST;
        }
        else {
          vel.x = -1.0 * speed;
          vel.y = 0;
          consumerDir[cID] = WEST;
        }
        }
        else if ( pos.y > BOTTOM_BOUNDARY && pos.x == LEFT_BOUNDARY && pos.x < RIGHT_BOUNDARY )
        {
          if( uniformRand01 <= 0.50 )
        {
          vel.x = theVel.x;
          vel.y = theVel.y;
        }
        else {
          vel.x = speed;
          vel.y = 0;
          consumerDir[cID] = EAST;
        }
        }
        else if ( pos.y > BOTTOM_BOUNDARY && pos.x > LEFT_BOUNDARY && pos.x == RIGHT_BOUNDARY )
        {
          if( uniformRand01 <= 0.50 )
        {
          vel.x = theVel.x;
          vel.y = theVel.y;
        }
        else {
          vel.x = -1.0 * speed;
          vel.y = 0;
          consumerDir[cID] = WEST;
        }
        }
        else if ( pos.y == BOTTOM_BOUNDARY && pos.x == LEFT_BOUNDARY && pos.x < RIGHT_BOUNDARY )
        {
        vel.x = speed;
        vel.y = 0;
        consumerDir[cID] = EAST;
        }
        else if ( pos.y == BOTTOM_BOUNDARY && pos.x > LEFT_BOUNDARY && pos.x == RIGHT_BOUNDARY )
        {
        vel.x = -1.0 * speed;
        vel.y = 0;
        consumerDir[cID] = WEST;
        }
      }
      else if ( consumerDir[cID] == EAST )
      {
        if ( pos.x < RIGHT_BOUNDARY && pos.y < TOP_BOUNDARY && pos.y > BOTTOM_BOUNDARY )
        {
          if( uniformRand01 <= 0.50 )
        {
          vel.x = theVel.x;
          vel.y = theVel.y;
        }
        else if( uniformRand01 > 0.50 && uniformRand01 <= 0.75 )
        {
          vel.x = 0;
          vel.y = -1.0 * speed;
          consumerDir[cID] = SOUTH;
        }
        else {
          vel.x = 0;
          vel.y = speed;
          consumerDir[cID] = NORTH;
        }
        }
        else if ( pos.x == RIGHT_BOUNDARY && pos.y < TOP_BOUNDARY && pos.y > BOTTOM_BOUNDARY )
        {
          if( uniformRand01 <= 0.50 )
        {
          vel.x = 0;
          vel.y = -1.0 * speed;
          consumerDir[cID] = SOUTH;
        }
        else {
          vel.x = 0;
          vel.y = speed;
          consumerDir[cID] = NORTH;
        }
        }
        else if ( pos.x < RIGHT_BOUNDARY && pos.y == TOP_BOUNDARY && pos.y > BOTTOM_BOUNDARY )
        {
          if( uniformRand01 <= 0.50 )
        {
          vel.x = theVel.x;
          vel.y = theVel.y;
        }
        else {
          vel.x = 0;
          vel.y = -1.0 * speed;
          consumerDir[cID] = SOUTH;
        }
        }
        else if ( pos.x < RIGHT_BOUNDARY && pos.y < TOP_BOUNDARY && pos.y == BOTTOM_BOUNDARY )
        {
          if( uniformRand01 <= 0.50 )
        {
          vel.x = theVel.x;
          vel.y = theVel.y;
        }
        else {
          vel.x = 0;
          vel.y = speed;
          consumerDir[cID] = NORTH;
        }
        }
        else if ( pos.x == RIGHT_BOUNDARY && pos.y < TOP_BOUNDARY && pos.y == BOTTOM_BOUNDARY )
        {
        vel.x = 0;
        vel.y = speed;
        consumerDir[cID] = NORTH;
        }
        else if ( pos.x == RIGHT_BOUNDARY && pos.y == TOP_BOUNDARY && pos.y > BOTTOM_BOUNDARY )
        {
        vel.x = 0;
        vel.y = -1.0 * speed;
        consumerDir[cID] = SOUTH;
        }
      }
      else if( consumerDir[cID] == WEST )
      {
        if ( pos.x > LEFT_BOUNDARY && pos.y < TOP_BOUNDARY && pos.y > BOTTOM_BOUNDARY )
        {
          if( uniformRand01 <= 0.50 )
	        {
	          vel.x = theVel.x;
	          vel.y = theVel.y;
	        }
	        else if( uniformRand01 > 0.50 && uniformRand01 <= 0.75 )
	        {
	          vel.x = 0;
	          vel.y = -1.0 * speed;
	          consumerDir[cID] = SOUTH;
	        }
	        else {
	          vel.x = 0;
	          vel.y = speed;
	          consumerDir[cID] = NORTH;
	        }
        }
        else if ( pos.x == LEFT_BOUNDARY && pos.y < TOP_BOUNDARY && pos.y > BOTTOM_BOUNDARY )
        {
          if( uniformRand01 <= 0.50 )
        {
          vel.x = 0;
          vel.y = -1.0 * speed;
          consumerDir[cID] = SOUTH;
        }
        else {
          vel.x = 0;
          vel.y = speed;
          consumerDir[cID] = NORTH;
        }
        }
        else if ( pos.x > LEFT_BOUNDARY && pos.y == TOP_BOUNDARY && pos.y > BOTTOM_BOUNDARY )
        {
          if( uniformRand01 <= 0.50 )
        {
          vel.x = theVel.x;
          vel.y = theVel.y;
        }
        else {
          vel.x = 0;
          vel.y = -1.0 * speed;
          consumerDir[cID] = SOUTH;
        }
        }
        else if ( pos.x > LEFT_BOUNDARY && pos.y < TOP_BOUNDARY && pos.y == BOTTOM_BOUNDARY )
        {
          if( uniformRand01 <= 0.50 )
        {
          vel.x = theVel.x;
          vel.y = theVel.y;
        }
        else {
          vel.x = 0;
          vel.y = speed;
          consumerDir[cID] = NORTH;
        }
        }
        else if ( pos.x == LEFT_BOUNDARY && pos.y < TOP_BOUNDARY && pos.y == BOTTOM_BOUNDARY )
        {
        vel.x = 0;
        vel.y = speed;
        consumerDir[cID] = NORTH;
        }
        else if ( pos.x == LEFT_BOUNDARY && pos.y == TOP_BOUNDARY && pos.y > BOTTOM_BOUNDARY )
        {
        vel.x = 0;
        vel.y = -1.0 * speed;
        consumerDir[cID] = SOUTH;
        }
      }
      
      Ptr<ConstantVelocityMobilityModel> cvmmCurNode = node->GetObject<ConstantVelocityMobilityModel> ();
    
      Ptr<ConstantVelocityMobilityModel> cvmmCurPsgNode = passengerNodes[apID].Get(id)->GetObject<ConstantVelocityMobilityModel> ();

      cvmmCurNode->SetPosition(pos);
      cvmmCurNode->SetVelocity(vel);
    
    cvmmCurPsgNode->SetPosition(pos);
    cvmmCurPsgNode->SetVelocity(vel);
    
      calcDir[cID] = false;
  }

  //std:: cout << "Sta " << id+1 << ": " << pos.x << " -> " << pos.y << "\n";
  Simulator::Schedule(Seconds(position_interval), 
                      &changeChannelAndAddress,
                      node, id, apID, ipDeleted, ipAdded);
}

int
main( int argc, char *argv[])
{
  generateIntersections();

  generateStaTravelList();

  Config::SetDefault ("ns3::Ipv4GlobalRouting::RespondToInterfaceEvents", BooleanValue (true));

  std::string phyMode = "DsssRate1Mbps";

  std::string animFile = "ap-tcp-animation.xml";

  for (int i = 0; i < STATIONS; i++){
    staProp[i].ipList.resize(apNodes);
  }

  if(verbose)
  {
    LogComponentEnable("ApWifiMac", LOG_LEVEL_ALL);
    LogComponentEnable("OnOffApplication", LOG_LEVEL_INFO);
  }

  //Config::SetDefault ("ns3::OnOffApplication::PacketSize", UintegerValue (1024));
  //Config::SetDefault ("ns3::OnOffApplication::DataRate", StringValue ("5kb/s"));
  // Control TCP fragmentation
  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (2200));
  //Config::SetDefault ("ns3::UdpSocket::SegmentSize", UintegerValue (2200));
  //Config::SetDefault ("ns3::WifiPhy::ChannelNumber", UintegerValue (1));
  //Config::SetDefault("ns3::Ipv4GlobalRouting::RandomEcmpRouting", BooleanValue(true));
  //Config::SetDefault ("ns3::Ipv4GlobalRouting::FlowEcmpRouting", BooleanValue (false));

  CommandLine cmd;
  cmd.AddValue ("seedVal", "seed value", seedVal);
  cmd.AddValue ("speed", "vehicle speed", speed);
  cmd.AddValue ("fileSize", "file size", fileSize);
  cmd.AddValue ("animFile", "File Name for Animation Output", animFile);
  cmd.AddValue ("maxBytes", "Total number of bytes for application to send", maxBytes);
  cmd.Parse (argc, argv);

  endtime = (double)( (MAX_AP_TO_VISIT * 1.0 * spacing) / (speed * 1.0) );
  //endtime = 3.0;

  maxBytes = fileSize * 1024;
  
  // Here, we will create one server, N client and M wifinodes nodes in a star.
  NS_LOG_INFO ("Create nodes.");
  
  NodeContainer wifiApNodes;
  NodeContainer routerNodes;
  NodeContainer topRouter;
  wifiApNodes.Create (apNodes);
  routerNodes.Create (totalRouters);
  topRouter.Create (1);
  serverNode.Create (1);
  clientNodes.Create (staNodes);
  
  for (int i=0; i < apNodes; i++) {
    passengerNodes[i].Create(staNodes);
  }
  NodeContainer fixNodes = NodeContainer (wifiApNodes, routerNodes, topRouter, serverNode);
  NodeContainer mobNodes = NodeContainer (clientNodes);
  
  
  ////////////////////////////////////////////////////////////////////////////////////////
  // Wired topology
  ////////////////////////////////////////////////////////////////////////////////////////
  std::vector<NodeContainer> nodeAdjacencyList (TOTAL_WIRED_LINKS);
  int rtCnt = 0;
  
  nodeAdjacencyList[rtCnt++] = NodeContainer (serverNode, topRouter); // 
  
  // L2-L3 Links
  nodeAdjacencyList[rtCnt++] = NodeContainer (topRouter, routerNodes.Get(13-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (topRouter, routerNodes.Get(14-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (topRouter, routerNodes.Get(15-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (topRouter, routerNodes.Get(16-rtOffset));

  // L1-L2 Links
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(13-rtOffset), routerNodes.Get(1-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(13-rtOffset), routerNodes.Get(2-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(13-rtOffset), routerNodes.Get(5-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(13-rtOffset), routerNodes.Get(6-rtOffset));

  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(14-rtOffset), routerNodes.Get(3-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(14-rtOffset), routerNodes.Get(4-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(14-rtOffset), routerNodes.Get(7-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(14-rtOffset), routerNodes.Get(8-rtOffset));

  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(15-rtOffset), routerNodes.Get(5-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(15-rtOffset), routerNodes.Get(6-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(15-rtOffset), routerNodes.Get(9-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(15-rtOffset), routerNodes.Get(10-rtOffset));

  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(16-rtOffset), routerNodes.Get(7-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(16-rtOffset), routerNodes.Get(8-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(16-rtOffset), routerNodes.Get(11-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(16-rtOffset), routerNodes.Get(12-rtOffset));

  // L1-L1 Links
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(1-rtOffset), routerNodes.Get(2-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(2-rtOffset), routerNodes.Get(6-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(3-rtOffset), routerNodes.Get(4-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(3-rtOffset), routerNodes.Get(7-rtOffset));

  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(5-rtOffset), routerNodes.Get(9-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(9-rtOffset), routerNodes.Get(10-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(8-rtOffset), routerNodes.Get(12-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(11-rtOffset), routerNodes.Get(12-rtOffset));

  // L1-AP Links
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(1-rtOffset), wifiApNodes.Get(1-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(1-rtOffset), wifiApNodes.Get(2-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(1-rtOffset), wifiApNodes.Get(7-rtOffset));

  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(2-rtOffset), wifiApNodes.Get(3-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(2-rtOffset), wifiApNodes.Get(8-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(2-rtOffset), wifiApNodes.Get(9-rtOffset));
  
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(3-rtOffset), wifiApNodes.Get(4-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(3-rtOffset), wifiApNodes.Get(10-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(3-rtOffset), wifiApNodes.Get(11-rtOffset));
  
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(4-rtOffset), wifiApNodes.Get(5-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(4-rtOffset), wifiApNodes.Get(6-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(4-rtOffset), wifiApNodes.Get(12-rtOffset));
  
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(5-rtOffset), wifiApNodes.Get(13-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(5-rtOffset), wifiApNodes.Get(19-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(5-rtOffset), wifiApNodes.Get(20-rtOffset));
  
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(6-rtOffset), wifiApNodes.Get(14-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(6-rtOffset), wifiApNodes.Get(15-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(6-rtOffset), wifiApNodes.Get(21-rtOffset));
  
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(7-rtOffset), wifiApNodes.Get(16-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(7-rtOffset), wifiApNodes.Get(17-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(7-rtOffset), wifiApNodes.Get(22-rtOffset));
  
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(8-rtOffset), wifiApNodes.Get(18-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(8-rtOffset), wifiApNodes.Get(23-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(8-rtOffset), wifiApNodes.Get(24-rtOffset));
  
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(9-rtOffset), wifiApNodes.Get(25-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(9-rtOffset), wifiApNodes.Get(31-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(9-rtOffset), wifiApNodes.Get(32-rtOffset));
  
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(10-rtOffset), wifiApNodes.Get(26-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(10-rtOffset), wifiApNodes.Get(27-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(10-rtOffset), wifiApNodes.Get(33-rtOffset));
  
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(11-rtOffset), wifiApNodes.Get(28-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(11-rtOffset), wifiApNodes.Get(29-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(11-rtOffset), wifiApNodes.Get(34-rtOffset));
  
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(12-rtOffset), wifiApNodes.Get(30-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(12-rtOffset), wifiApNodes.Get(35-rtOffset));
  nodeAdjacencyList[rtCnt++] = NodeContainer (routerNodes.Get(12-rtOffset), wifiApNodes.Get(36-rtOffset));
  
  
  // We create the wired channels first without any IP addressing informatio
  NS_LOG_INFO ("Create channels.");
  PointToPointHelper p2p;
  //p2p.SetDeviceAttribute ("DataRate", StringValue ("10Mbps"));
  //p2p.SetChannelAttribute ("Delay", StringValue ("2ms"));
  
  std::vector<NetDeviceContainer> deviceAdjacencyList (TOTAL_WIRED_LINKS);
  // Switch to L1 routers-> r3, r4
  for( int i=0; i < TOTAL_WIRED_LINKS; ++i)
  {
    p2p.SetDeviceAttribute ("DataRate", StringValue ("10Mbps"));
    p2p.SetChannelAttribute ("Delay", StringValue ("5ms"));
    deviceAdjacencyList[i] = p2p.Install (nodeAdjacencyList[i]);
  }
  
  
  ////////////////////////////////////////////////////////////////////////////////////////
  // Wireless topology
  ////////////////////////////////////////////////////////////////////////////////////////
  // disable fragmentation, RTS/CTS and enable non-unicast data rate for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
  //Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue (phyMode));
  
  // Wifi helper setup
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
  wifi.SetRemoteStationManager ("ns3::AarfWifiManager"); // Use AARF rate control
  
  
  for(int i=0; i<apNodes; i++)
  {
    wifiPhy[i].SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    // The following has an absolute cutoff at distance > range (range == radius)
    wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel",  "MaxRange", DoubleValue(range));
    wifiPhyChn[i] = wifiChannel.Create ();
    wifiPhy[i].SetChannel (wifiPhyChn[i]);
    //wifiPhy[i].Set("ChannelNumber",UintegerValue(i));
  }
  
  /*wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue (phyMode),
                                "ControlMode", StringValue (phyMode));*/
  
  // Setup rest of upper MAC
  Ssid ssid = Ssid ("wifi-default");
  // AP
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  wifiMac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid),
                   "BeaconGeneration", BooleanValue(true));
                   //"BeaconInterval", TimeValue(Seconds(2.5)));
  // STA
  NqosWifiMacHelper wifiMacHelper = NqosWifiMacHelper::Default ();
  // Active associsation of STA to AP via probing.
  wifiMacHelper.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid),
                         "ActiveProbing", BooleanValue (true),
                         "ProbeRequestTimeout", TimeValue(Seconds(0.25)));

  NetDeviceContainer apDevices;

  for(int i=0; i<apNodes; i++)
  {
    // Setup AP.
    apDevices.Add(wifi.Install (wifiPhy[i], wifiMac, wifiApNodes.Get(i)));

    passengerDevice[i].Add(wifi.Install(wifiPhy[i], wifiMacHelper, passengerNodes[i]));
  }

  NetDeviceContainer devices;
  
  for (int i=0; i < apNodes; i++) {
    devices.Add(passengerDevice[i]);
  }
  
  devices.Add (apDevices);
  
  // Set positions for APs 
  MobilityHelper sessile;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  
  for(int Ypos = -500; Ypos <= 500; Ypos += 200)
	{
	    for(int Xpos = -500; Xpos <= 500; Xpos += 200)
	    {
	        positionAlloc->Add(Vector(Xpos, Ypos, 0.0));
	    }
	}
  
  positionAlloc->Add(Vector(-400, -400, 0.0)); // r1
  positionAlloc->Add(Vector(-200, -400, 0.0)); // r2
  positionAlloc->Add(Vector(200, -400, 0.0)); // r3
  positionAlloc->Add(Vector(400, -400, 0.0)); // r4

  positionAlloc->Add(Vector(-400, 1, 0.0)); // r5
  positionAlloc->Add(Vector(-200, 1, 0.0)); // r6
  positionAlloc->Add(Vector(200, 1, 0.0)); // r7
  positionAlloc->Add(Vector(400, 1, 0.0)); // r8

  positionAlloc->Add(Vector(-400, 400, 0.0)); // r9
  positionAlloc->Add(Vector(-200, 400, 0.0)); // r10
  positionAlloc->Add(Vector(200, 400, 0.0)); // r11
  positionAlloc->Add(Vector(400, 400, 0.0)); // r12

  positionAlloc->Add(Vector(-300, -200, 0.0)); // r13
  positionAlloc->Add(Vector(300, -200, 0.0)); // r14
  positionAlloc->Add(Vector(-300, 200, 0.0)); // r15
  positionAlloc->Add(Vector(300, 200, 0.0)); // r16

  positionAlloc->Add(Vector(0, 1, 0.0)); // switch
  positionAlloc->Add(Vector(0, 700, 0.0)); // Server
  
  
  sessile.SetPositionAllocator (positionAlloc);
  sessile.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  sessile.Install (wifiApNodes);
  sessile.Install (routerNodes);
  sessile.Install (topRouter);
  sessile.Install (serverNode);
  
  // Setting mobility model and movement parameters for mobile nodes
  // ConstantVelocityMobilityModel is a subclass of MobilityModel
  MobilityHelper mobile; 
  mobile.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  mobile.Install(clientNodes);
  
  for (int i=0; i < apNodes; i++) {
    mobile.Install(passengerNodes[i]);
  }
  // Setting each mobile client 100m apart from each other
  double nxt = 0;
  //srand((unsigned)seedVal);

  for (int i=0; i<staNodes/2 ; i++)
  {
    Ptr<ConstantVelocityMobilityModel> cvmm = clientNodes.Get(i)->GetObject<ConstantVelocityMobilityModel> ();
    Vector pos (0-nxt, -500, 0);
    Vector vel (speed, 0, 0);
    cvmm->SetPosition(pos);
    cvmm->SetVelocity(vel);
    consumerDir[i] = EAST;
    calcDir[i] = false;
    nxt += (double)( MAX_GAP * (rand() / (RAND_MAX*1.0)) );
  }

  
  nxt = 0;
  for (int i=staNodes/2; i<staNodes ; i++)
  {
    Ptr<ConstantVelocityMobilityModel> cvmm = clientNodes.Get(i)->GetObject<ConstantVelocityMobilityModel> ();
    Vector pos (nxt, 500, 0);
    Vector vel (-1*speed, 0, 0);
    cvmm->SetPosition(pos);
    cvmm->SetVelocity(vel);
    consumerDir[i] = WEST;
    calcDir[i] = false;
    nxt += (double)( MAX_GAP * (rand() / (RAND_MAX*1.0)) );
  }
  
  for (int i=0; i < apNodes ; i++)
  {
    for (int j=0; j<staNodes; j++) {
      Ptr<ConstantVelocityMobilityModel> cvmm = passengerNodes[i].Get(j)->GetObject<ConstantVelocityMobilityModel> ();
      Vector pos (800, 800, 0);
      Vector vel (0, 0, 0);
      cvmm->SetPosition(pos);
      cvmm->SetVelocity(vel);
    }
  }
  

  // Install netwok stack
  InternetStackHelper internet;
  internet.Install (fixNodes);
  //internet.Install (mobNodes);
  
  
  //////////////////////////////////////////////////////////////////////////////
  // Add IP addresses.
  //////////////////////////////////////////////////////////////////////////////
  NS_LOG_INFO ("Assign IP Addresses.");
  
  //////////////////////////////////////////////////////
  // Wired interfaces
  //////////////////////////////////////////////////////
  Ipv4AddressHelper ipv4;
  std::vector<Ipv4InterfaceContainer> interfaceAdjacencyList(deviceAdjacencyList.size());

  for (uint32_t i=0; i<deviceAdjacencyList.size(); i++)
  {
    std::ostringstream subnet;
    subnet<<"10.1."<<i+1<<".0";
    ipv4.SetBase (subnet.str ().c_str (), "255.255.255.0");
    interfaceAdjacencyList[i] = ipv4.Assign (deviceAdjacencyList[i]);
    //std:: cout << interfaceAdjacencyList.GetAddress (0) << " <-> ";
    //std:: cout << interfaceAdjacencyList.GetAddress (1) << std::endl;
  }
  
  
  //////////////////////////////////////////////////////
  // Wireless interfaces
  //////////////////////////////////////////////////////
  std::vector<Ipv4AddressHelper> ipv4Wifi(apNodes);
  Ipv4InterfaceContainer wifiApInterfaces;
  int dAL = deviceAdjacencyList.size();

  for(int i = 0; i < apNodes; ++i)
  {
    std::ostringstream subnet;
    subnet<<"10.1."<< i + dAL + 1 <<".0";
    ipv4Wifi[i].SetBase (subnet.str ().c_str (), "255.255.255.0");
    wifiApInterfaces = ipv4Wifi[i].Assign (apDevices.Get(i));
    //std:: cout << "AP -> " << i+1 << " : " << wifiApInterfaces.GetAddress (0) << std::endl;
    
    ipv4Wifi[i].Assign (passengerDevice[i]);
  }

  //Turn on global static routing
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  //Ipv4GlobalRoutingHelper globalRouting;
  
    
  // Routing for STA nodes
  Ipv4NixVectorHelper nixRouting;
  //OlsrHelper olsr;
  Ipv4StaticRoutingHelper staticRouting;
  Ipv4ListRoutingHelper list; list.Add (staticRouting, 1000); list.Add (nixRouting, 1);
  //Ipv4ListRoutingHelper list; list.Add (staticRouting, 1000); //list.Add (olsr, 0);
  InternetStackHelper stack; stack.SetRoutingHelper (list);
  for (int i=0; i < apNodes; i++) {
    stack.Install (passengerNodes[i]);
  }

  //Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
  
  for(int j=0; j<staNodes; j++)
  {
    staProp[j].curAP = 0;
    staProp[j].files15 = 0;
    staProp[j].files50 = 0;
    staProp[j].files100 = 0;
    staProp[j].files400 = 0;
    staProp[j].nbytes = 0;
    staProp[j].totBytes = 0;
    //std:: cout << "STA -> " << j+1 << " : " <<  wifiInterfaces.GetAddress (j) << std::endl;
  }
  
  // Install TCP application
  NS_LOG_INFO ("Create Applications.");

//
// Create a TcpServerApplicationApplication and install it on serverNode
//
  TcpServerApplicationHelper server (InetSocketAddress (Ipv4Address::GetAny (), sport));
  server.SetAttribute ("MaxTxBytes", UintegerValue (0));
  serverApp = server.Install (serverNode);
  serverApp.Start (Seconds (0.0));
  serverApp.Stop (Seconds (endtime));

//
// Create a TcpClientApplication and install it on clientNodes
//

  TcpClientApplicationHelper client (InetSocketAddress (
                                     interfaceAdjacencyList[0].GetAddress(0),
                                     sport )
                                    );

  client.SetAttribute ("MaxRxBytes", UintegerValue (maxBytes));
  
  
  for (int i = 0; i < apNodes; ++i)
  {
    for (int j=0; j < STATIONS; j++) {
      clientApps[i].Add(client.Install (passengerNodes[i].Get (j)));
    }
    clientApps[i].Start (Seconds (0.0));
    clientApps[i].Stop (Seconds (endtime));
  }

  // Generate animation file
  AnimationInterface anim (animFile);
  anim.SetMaxPktsPerTraceFile(9999999999);
  
  for (int i=0; i<staNodes; i++) 
  {
    Simulator::Schedule(Seconds(position_interval), 
                        &changeChannelAndAddress,
                        clientNodes.Get(i), i, 
                        staProp[i].curAP,
                        true, false);
  }

  Simulator::Stop(Seconds(endtime));

  //configure tracing
  AsciiTraceHelper ascii;
  p2p.EnableAsciiAll (ascii.CreateFileStream ("tcp-ap-server.tr"));
  //p2p.EnablePcapAll ("tcp-ap-server");
  p2p.EnablePcap ("tcp-ap-server", serverNode.Get(0)->GetId (), 0);
  
  /*
  for (int i=0; i < apNodes; i++) 
  {
    std::string str = std::string("tcp-ap-sta") + std::to_string(i);
    std::cout << str <<std::endl;
    wifiPhy[i].EnablePcapAll(str);
  }
  */
  
  NS_LOG_INFO ("Run Simulation.");
  Simulator::Run();
  Simulator::Destroy();
  NS_LOG_INFO ("Done.");

  for (int i=0; i<staNodes; i++)
  {
    int Rx = 0;
    for (int j=0; j<apNodes; j++) {
      Ptr<TcpClientApplication> client = DynamicCast<TcpClientApplication> (clientApps[j].Get (i));
      Rx += client->GetCompleteRx ();
    }
    std::cout << "Bytes-STA-" << i+1 << ": " << Rx << std::endl;
    std::cout << "Files-STA-" << i+1 << ": " << staProp[i].files15 << " " << staProp[i].files50 << " " << staProp[i].files100 << " " << staProp[i].files400 << std::endl;
  }
  
  return 0;
}

