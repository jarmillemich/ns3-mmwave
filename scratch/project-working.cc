#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store.h"
#include "ns3/mmwave-helper.h"
#include <ns3/buildings-helper.h>
#include "ns3/log.h"
#include <ns3/buildings-module.h>
#include <ns3/point-to-point-helper.h>
#include <ns3/internet-stack-helper.h>
#include <ns3/mmwave-point-to-point-epc-helper.h>
#include <ns3/ipv4-static-routing-helper.h>
#include <ns3/rng-seed-manager.h>
#include <ns3/applications-module.h>
#include <ns3/lte-ue-net-device.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdexcept>
#include <omp.h>
#include <execinfo.h>
#include <signal.h>
#include <unistd.h>

using namespace std;
using namespace ns3;
using namespace mmwave;

#define UEPerStreet 10

/* NOTES

Some good terminology at
https://www.artizanetworks.com/resources/tutorials/sae_tec.html



*/

/** Generate a single building inside the specified box */
Ptr<Building> makeBuilding(const Box& box) {
  Ptr<Building> building = Create<Building>();
  building->SetBoundaries(box);
  building->SetBuildingType(Building::Residential);
  building->SetExtWallsType(Building::ConcreteWithoutWindows);

  building->SetNFloors (1);
  building->SetNRoomsX (1);
  building->SetNRoomsY (1);

  return building;
}

/** Generate a grid of buildings of the specified size */
BuildingContainer generateManhattanGrid(int xBlocks, int yBlocks) {
  BuildingContainer ret;

  double buildingWidth = 75;
  double buildingHeight = 12;
  double streetWidth = 15;
      
  for (int xBlock = 0; xBlock < xBlocks; xBlock++) {
    for (int yBlock = 0; yBlock < yBlocks; yBlock++) {
      int xStart = (buildingWidth + streetWidth) * xBlock;
      int xStop = xStart + buildingWidth;

      int yStart = (buildingWidth + streetWidth) * yBlock;
      int yStop = yStart + buildingWidth;

      ret.Add(makeBuilding(Box(
        xStart, xStop,
        yStart, yStop,
        0, buildingHeight
      )));
    }
  }

  return ret;
}

static ns3::GlobalValue g_outPath ("outPath",
                                   "The path of output log files",
                                   ns3::StringValue ("./project-stats/"), ns3::MakeStringChecker ());
static ns3::GlobalValue g_runNumber ("runNumber", "Run number for rng",
                                     ns3::UintegerValue (10), ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_scenario ("scenario", "Scenario to use",
                                     ns3::UintegerValue (3), ns3::MakeUintegerChecker<uint32_t> ());


static ofstream *mobilityOs;

void SetupTraceConfig(string path, uint32_t runSet) {
  //get current time
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];
  time (&rawtime);
  timeinfo = localtime (&rawtime);
  strftime (buffer,80,"%Y%m%d_%I_%M_%S",timeinfo);
  std::string time_str (buffer);

  // rng things
  uint32_t seedSet = 5;
  RngSeedManager::SetSeed (seedSet);
  RngSeedManager::SetRun (runSet);
  UintegerValue scenarioValue;
  GlobalValue::GetValueByName ("scenario", scenarioValue);
  uint32_t scenario = scenarioValue.Get ();
  char seedSetStr[21];
  char runSetStr[21];
  char scenarioStr[4];
  sprintf (seedSetStr, "%d", seedSet);
  sprintf (runSetStr, "%d", runSet);
  sprintf (scenarioStr, "%d", scenario);

  std::string dlRlcOutName = "DlRlcStats";
  std::string dlPdcpOutName = "DlPdcpStats";
  std::string ulRlcOutName = "UlRlcStats";
  std::string ulPdcpOutName = "UlPdcpStats";
  std::string extension = ".txt";
  

  auto mkPath = [=](string subName) {
    return StringValue(path + time_str + "_scene" + scenarioStr + "_" + subName + "_seed" + seedSetStr + "_run" + runSetStr + extension);
  };

  
  Config::SetDefault ("ns3::MmWaveBearerStatsCalculator::DlRlcOutputFilename",              mkPath(dlRlcOutName));
  Config::SetDefault ("ns3::MmWaveBearerStatsCalculator::UlRlcOutputFilename",              mkPath(ulRlcOutName));
  Config::SetDefault ("ns3::MmWaveBearerStatsCalculator::DlPdcpOutputFilename",             mkPath(dlPdcpOutName));
  Config::SetDefault ("ns3::MmWaveBearerStatsCalculator::UlPdcpOutputFilename",             mkPath(ulPdcpOutName));
  
  // A dump of all packets sent/received, with size, SINR
  Config::SetDefault ("ns3::MmWavePhyRxTrace::OutputFilename",                              mkPath("PacketTrace"));
  /*
    With some help from https://www.etsi.org/deliver/etsi_ts/136200_136299/136213/08.03.00_60/ts_136213v080300p.pdf
    Columns in the log file
    time - time
    frame - frame # for ENB/UE pair
    subF - subframe (frames consist of subframes, first couple might be control ul/dl,
           the rest are data uplink/downlink)
    1stSymbol - not used
    symbol# - # of symbols in subframe
    cellId - ID of ENB
    rnti - radio network temporary identifier (ENB Id for UE)
    ccId - not used
    tbSize - transmission block size
  */

  mobilityOs = new ofstream();
  mobilityOs->open(mkPath("mobility").Get());
}

/** Helper to figure out what something is made of */
void dumpObjectAggregate(Ptr<const Object> obj) {
  auto it = obj->GetAggregateIterator();
  
  while (it.HasNext()) {
    Ptr<const Object> next = it.Next();
    cout << "has " << next->GetInstanceTypeId().GetName() << endl;
  }
}

/** Retrieve the (first) MMWave network device on this node */
Ptr<MmWaveUeNetDevice> GetUeNetDev(Ptr<const Node> node) {
  for (unsigned i = 0; i < node->GetNDevices(); i++) {
    Ptr<MmWaveUeNetDevice> dev = node->GetDevice(i)->GetObject<MmWaveUeNetDevice>();
    if (dev != nullptr) return dev;
  }

  return nullptr;
}

/** Retrieve the (first) MMWave network device on this node */
Ptr<MmWaveEnbNetDevice> GetEnbNetDev(Ptr<const Node> node) {
  for (unsigned i = 0; i < node->GetNDevices(); i++) {
    Ptr<MmWaveEnbNetDevice> dev = node->GetDevice(i)->GetObject<MmWaveEnbNetDevice>();
    if (dev != nullptr) return dev;
  }

  return nullptr;
}

// Log out locations
void LogUeInfo(ofstream *os, string context, Ptr<const MobilityModel> mobility) {
  // From vanet-routing-compare.cc
  Vector pos = mobility->GetPosition (); // Get position
  Vector vel = mobility->GetVelocity (); // Get velocity

  Ptr<const Node> node = mobility->GetObject<Node> ();
  int nodeId = node->GetId ();
  auto netDev = GetUeNetDev(node);

  auto phy = netDev->GetPhy();
  auto enb = netDev->GetTargetEnb();
  
  // Prints connection info, position, and velocities
  *os << "ue\t"
      << Simulator::Now() << "\t"
      // Connection info
      << nodeId << "\t"
      << netDev->GetImsi() << "\t"
      << phy->GetRnti() << "\t"
      << enb->GetCellId() << "\t"
      // Position info
      << pos.x << "\t"
      << pos.y << "\t"
      << pos.z << "\t"
      << vel.x << "\t"
      << vel.y << "\t"
      << vel.z << endl;
}

// Log out drone locations
void LogEnbInfo(ofstream *os, string context, Ptr<const MobilityModel> mobility) {
  // From vanet-routing-compare.cc
  Vector pos = mobility->GetPosition (); // Get position
  Vector vel = mobility->GetVelocity (); // Get velocity

  Ptr<const Node> node = mobility->GetObject<Node> ();
  int nodeId = node->GetId ();

  auto enb = GetEnbNetDev(node);
  
  // Prints connection info, position, and velocities
  *os << "enb\t"
      << Simulator::Now() << "\t"
      // Connection info
      << nodeId << "\t"
      << "\t" // imsi
      << "\t" // rnti
      << enb->GetCellId() << "\t"
      // Position info
      << pos.x << "\t"
      << pos.y << "\t"
      << pos.z << "\t"
      << vel.x << "\t"
      << vel.y << "\t"
      << vel.z << endl;
}

/** Task to log UE locations and connection info, repeats on an interval */
void LogAllUeLocations(ofstream *os, NodeContainer *ueNodes, NodeContainer *enbNodes) {
  for (uint32_t i = 0; i < ueNodes->GetN(); i++) {
    auto node = ueNodes->Get(i);
    auto mobility = node->GetObject<MobilityModel>();
    if (mobility == nullptr) {
      *os << node->GetId() << " had no mobility" << endl;
      continue;
    }
    LogUeInfo(os, "sched", mobility);
  }

  for (uint32_t i = 0; i < enbNodes->GetN(); i++) {
    auto node = enbNodes->Get(i);
    auto mobility = node->GetObject<MobilityModel>();
    if (mobility == nullptr) {
      *os << node->GetId() << " had no mobility" << endl;
      continue;
    }
    LogEnbInfo(os, "sched", mobility);
  }

  Simulator::Schedule(MilliSeconds(100), &LogAllUeLocations, os, ueNodes, enbNodes);
}


/** Generate some traffic */
void useTrafficGenerator(const NodeContainer &ueNodes, const Ipv4InterfaceContainer &ueIpIface, Ptr<Node> remoteHost) {
  // From mmwave-epc-tdma.cc
  
  // In microseconds
  double interPacketInterval = 1;

  // Install and start applications on UEs and remote host
  uint16_t dlPort = 1234;
  uint16_t ulPort = 2000;
  uint16_t otherPort = 3000;
  ApplicationContainer clientApps;
  ApplicationContainer serverApps;
  uint32_t packetSize = 1400;

  double dataRateMB = 250e6;        // 250 MBps
  double onTimeSec = packetSize / dataRateMB;
  std::stringstream ss;
  ss << "ns3::ConstantRandomVariable[Constant=" << onTimeSec << "]";
  std::cout << "OnTime == " << ss.str () << std::endl;
  OnOffHelper dlClient ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
  OnOffHelper ulClient ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
  dlClient.SetAttribute ("PacketSize", UintegerValue (packetSize));
  ulClient.SetAttribute ("PacketSize", UintegerValue (packetSize));
  dlClient.SetAttribute ("DataRate", DataRateValue (8 * dataRateMB));
  ulClient.SetAttribute ("DataRate", DataRateValue (8 * dataRateMB));
  dlClient.SetAttribute ("OnTime", StringValue (ss.str ()));
  ulClient.SetAttribute ("OnTime", StringValue (ss.str ()));
  ss.str ("");
  ss << "ns3::ExponentialRandomVariable[Mean=" << interPacketInterval * 1e-6 << "]";
  std::cout << "OffTime == " << ss.str () << std::endl;
  dlClient.SetAttribute ("OffTime", StringValue (ss.str ()));
  ulClient.SetAttribute ("OffTime", StringValue (ss.str ()));

  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      ++ulPort;
      ++otherPort;
      PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
      PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
      serverApps.Add (dlPacketSinkHelper.Install (ueNodes.Get (u)));
      serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

//		UdpClientHelper dlClient (ueIpIface.GetAddress (u), dlPort);
//		dlClient.SetAttribute ("Interval", TimeValue (MicroSeconds(interPacketInterval)));
//		dlClient.SetAttribute ("MaxPackets", UintegerValue(1000000));
//		dlClient.SetAttribute ("PacketSize", UintegerValue(packetSize));
//
//		UdpClientHelper ulClient (remoteHostAddr, ulPort);
//		ulClient.SetAttribute ("Interval", TimeValue (MicroSeconds(interPacketInterval)));
//		ulClient.SetAttribute ("MaxPackets", UintegerValue(1000000));
//		ulClient.SetAttribute ("PacketSize", UintegerValue(packetSize));

      //UdpClientHelper client (ueIpIface.GetAddress (u), otherPort);
      //client.SetAttribute ("Interval", TimeValue (MicroSeconds(interPacketInterval)));
      //client.SetAttribute ("MaxPackets", UintegerValue(1000000));
      dlClient.SetAttribute ("Remote", AddressValue (InetSocketAddress (ueIpIface.GetAddress (u), dlPort)));
      //ulClient.SetAttribute ("Remote", AddressValue (InetSocketAddress (remoteHostAddr, ulPort)));

      clientApps.Add (dlClient.Install (remoteHost));
//		clientApps.Add (ulClient.Install (ueNodes.Get(u)));
//		if (u+1 < ueNodes.GetN ())
//		{
//			clientApps.Add (client.Install (ueNodes.Get(u+1)));
//		}
//		else
//		{
//			clientApps.Add (client.Install (ueNodes.Get(0)));
//		}
    }
  serverApps.Start (Seconds (0.003));
  clientApps.Start (Seconds (0.003));
}

class InternetComponent : public RefCountBase {
public:
  InternetStackHelper stack;
  Ptr<Node> packetGateway;
  Ipv4StaticRoutingHelper routingHelper;
  Ptr<Node> remoteHost;
  Ptr<MmWavePointToPointEpcHelper> epcHelper;
};

/** Sets up the internet and packet gateway, with a single simulated host */
Ptr<InternetComponent> useInternet(Ptr<MmWaveHelper> mmWave) {
  // From mmwave-epc-tdma.cc
  Ptr<InternetComponent> internet = new InternetComponent();

  internet->epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
  mmWave->SetEpcHelper(internet->epcHelper);
  internet->packetGateway = internet->epcHelper->GetPgwNode ();



  // Create a single RemoteHost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  internet->remoteHost = remoteHostContainer.Get (0);
  internet->stack.Install (remoteHostContainer);

  // Create the Internet
  PointToPointHelper p2ph;
  // Table 1: Input data rate=100 Gbps
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  // Table 1: MTU size = 1500 bytes
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  // Table 1: PGW and remote host link delay=10ms
  p2ph.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (10.0)));
  NetDeviceContainer internetDevices = p2ph.Install (internet->packetGateway, internet->remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);

  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = internet->routingHelper.GetStaticRouting (internet->remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  return internet;
}

Ptr<MmWaveHelper> useMmWave() {
  Ptr<MmWaveHelper> mmWave = CreateObject<MmWaveHelper>();
  // From II.B.1: "All simulations utilize the 3GPP pathloss model [10] to calculate penetration loss in buildings"
  mmWave->SetAttribute("PathlossModel", StringValue("ns3::MmWave3gppBuildingsPropagationLossModel"));
  // From II.A: "The channel model used for the simulations in this paper is
  // based on 3GPP mmWave channel model described in [10]"
  mmWave->SetAttribute("ChannelModel", StringValue("ns3::MmWave3gppChannel"));

  // II.B.1: Do handover when there is a 3 dB SINR advantage (every 256 ms)
  mmWave->SetLteHandoverAlgorithmType ("ns3::A3RsrpHandoverAlgorithm");
  Config::SetDefault ("ns3::A3RsrpHandoverAlgorithm::Hysteresis", DoubleValue (3.0));
  Config::SetDefault ("ns3::A3RsrpHandoverAlgorithm::TimeToTrigger", TimeValue (MilliSeconds (256)));

  mmWave->Initialize();
  return mmWave;
}

NodeContainer usePoleMountedENB() {
  // Set up base stations (ENB/"Evolved Node B" in 3GPP terminology)
  NodeContainer enbNodes;
  enbNodes.Create(4);

  // Assign the ENB a constant position
  auto enbPositionAlloc = CreateObject<ListPositionAllocator>();
  // Table 1: Infrastructure mmWave AP height = 3m
  double enbLoc1 = 75 / 2;
  double enbLoc2 = enbLoc1 + 75 + 15;
  double enbLocMid = 75 + 15 / 2;
  enbPositionAlloc->Add(Vector(enbLocMid, enbLoc1, 3.0));
  enbPositionAlloc->Add(Vector(enbLocMid, enbLoc2, 3.0));
  enbPositionAlloc->Add(Vector(enbLoc1, enbLocMid, 3.0));
  enbPositionAlloc->Add(Vector(enbLoc2, enbLocMid, 3.0));
  MobilityHelper enbMobility;
  enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  enbMobility.SetPositionAllocator(enbPositionAlloc);
  enbMobility.Install(enbNodes);

  return enbNodes;
}

/**
 * @brief Generates UE for scenario 1
 * Generates mobile UEs, 10 per street (Table 1),
 * moving to random locations on the same street
 * @return NodeContainer 
 */
NodeContainer useDistributedUE() {
  NodeContainer xStreetUE, yStreetUE;
  MobilityHelper xStreetMob, yStreetMob;

  StringValue
    rngGridWidth("ns3::UniformRandomVariable[Min=0.0|Max=165.0]"),
    rngInnerWidth("ns3::UniformRandomVariable[Min=75.0|Max=90.0]"),
    // Table 1: Users' height = 1.6m
    rngHeight("ns3::ConstantRandomVariable[Constant=1.6]");

  Ptr<OutdoorPositionAllocator>
    xAllocator = CreateObjectWithAttributes<OutdoorPositionAllocator>(
      "X", rngGridWidth,
      "Y", rngInnerWidth,
      "Z", rngHeight
    ),
    yAllocator = CreateObjectWithAttributes<OutdoorPositionAllocator>(
      "X", rngInnerWidth,
      "Y", rngGridWidth,
      "Z", rngHeight
    );
  

  xStreetUE.Create(UEPerStreet);
  yStreetUE.Create(UEPerStreet);

  xStreetMob.SetMobilityModel("ns3::RandomWaypointMobilityModel",
    // How long to wait initially and at waypoints
    "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"),
    // Table I: Speed = 5 km/h ~= 1.4 m/s
    "Speed", StringValue("ns3::ConstantRandomVariable[Constant=1.4]"),
    // Hopefully stay on the streets?
    "PositionAllocator", PointerValue(xAllocator)
  );
  xStreetMob.SetPositionAllocator(xAllocator);
  xStreetMob.Install(xStreetUE);

  yStreetMob.SetMobilityModel("ns3::RandomWaypointMobilityModel",
    // How long to wait initially and at waypoints
    "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"),
    // Table I: Speed = 5 km/h ~= 1.4 m/s
    "Speed", StringValue("ns3::ConstantRandomVariable[Constant=1.4]"),
    // Hopefully stay on the streets?
    "PositionAllocator", PointerValue(yAllocator)
  );
  yStreetMob.SetPositionAllocator(yAllocator);
  yStreetMob.Install(yStreetUE);

  return NodeContainer(xStreetUE, yStreetUE);
}

Ptr<RandomWaypointMobilityModel>
  xClusterMobility = nullptr,
  yClusterMobility = nullptr;

// Some info for the clusters
double
  // Stay in the middle of either street
  pathMid = 82.5,
  // Start a street radius from the end
  startInline = 7.5,
  // End the distance travelled at 5 km/h in 85 seconds (see Table 1)
  endInline = 7.5 + 5000 / 3600 * 85;

/** 
 * Initialize the cluster mobility model globals,
 * Shared between clusters (2,3) and drones (3)
 **/
void useClusterMobility() {
  if (xClusterMobility == nullptr) {
    Ptr<ListPositionAllocator> xGroupAllocator = CreateObject<ListPositionAllocator>();

    //xGroupAllocator->Add(Vector(startInline, pathMid, 0));
    xGroupAllocator->Add(Vector(endInline, pathMid, 0));

    // Group reference mobility, follows a point moving down the street
    xClusterMobility =
      CreateObjectWithAttributes<RandomWaypointMobilityModel>(
        // How long to wait initially and at waypoints
        "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"),
        // Table I: Speed = 5 km/h ~= 1.4 m/s
        "Speed", StringValue("ns3::ConstantRandomVariable[Constant=1.4]"),
        // Proceed along the street
        "PositionAllocator", PointerValue(xGroupAllocator)
      );
    xClusterMobility->SetPosition(Vector(startInline, pathMid, 0));
  }

  if (yClusterMobility == nullptr) {
    Ptr<ListPositionAllocator> yGroupAllocator = CreateObject<ListPositionAllocator>();

    //yGroupAllocator->Add(Vector(pathMid, startInline, 0));
    yGroupAllocator->Add(Vector(pathMid, endInline, 0));

    // Group reference mobility, follows a point moving down the street
    yClusterMobility =
      CreateObjectWithAttributes<RandomWaypointMobilityModel>(
        // How long to wait initially and at waypoints
        "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"),
        // Table I: Speed = 5 km/h ~= 1.4 m/s
        "Speed", StringValue("ns3::ConstantRandomVariable[Constant=1.4]"),
        // Proceed along the street
        "PositionAllocator", PointerValue(yGroupAllocator)
      );
    yClusterMobility->SetPosition(Vector(pathMid, startInline, 0));
  }
}

/**
 * @brief Generates UE for scenario 2 and 3
 * Generates mobile UEs, 10 per street (Table 1), starting in clusters on
 * one end of the street and moving to the other
 * @return NodeContainer 
 */
NodeContainer useClusteredUE() {
  NodeContainer xStreetUE, yStreetUE;
  MobilityHelper xStreetMob, yStreetMob;

  useClusterMobility();

  // Wander around in a circle around the group center
  Ptr<PositionAllocator> inGroupAllocator = CreateObjectWithAttributes<UniformDiscPositionAllocator>(
    "rho", DoubleValue(7),
    "X", DoubleValue(0),
    "Y", DoubleValue(0),
    "Z", DoubleValue(1.6)
  );

  xStreetUE.Create(UEPerStreet);
  yStreetUE.Create(UEPerStreet);

  
  xStreetMob.PushReferenceMobilityModel(xClusterMobility);
  // Intra-group mobility, wanders around the center (stays out of buildings)
  xStreetMob.SetMobilityModel("ns3::RandomWaypointMobilityModel",
    // How long to wait initially and at waypoints
    "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"),
    // Table I: Speed = 5 km/h ~= 1.4 m/s
    "Speed", StringValue("ns3::ConstantRandomVariable[Constant=0.2]")
  );
  // Initial positions, must take cluster centers into account
  xStreetMob.SetPositionAllocator("ns3::UniformDiscPositionAllocator",
    "rho", DoubleValue(7),
    "X", DoubleValue(0),
    "Y", DoubleValue(0),
    "Z", DoubleValue(1.6)
  );
  xStreetMob.Install(xStreetUE);

  yStreetMob.PushReferenceMobilityModel(yClusterMobility);
  yStreetMob.SetMobilityModel("ns3::RandomWaypointMobilityModel",
    // How long to wait initially and at waypoints
    "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"),
    // Table I: Speed = 5 km/h ~= 1.4 m/s
    "Speed", StringValue("ns3::ConstantRandomVariable[Constant=0.2]")
  );
  yStreetMob.SetPositionAllocator("ns3::UniformDiscPositionAllocator",
    "rho", DoubleValue(7),
    "X", DoubleValue(0),
    "Y", DoubleValue(0),
    "Z", DoubleValue(1.6)
  );
  yStreetMob.Install(yStreetUE);

  cout << " X is " << endl;
  for (unsigned i = 0; i < xStreetUE.GetN(); i++) {
    cout << i << " " << xStreetUE.Get(i)->GetObject<MobilityModel>()->GetPosition() << endl;
  }

  cout << " Y is " << endl;
  for (unsigned i = 0; i < yStreetUE.GetN(); i++) {
    cout << i << " " << yStreetUE.Get(i)->GetObject<MobilityModel>()->GetPosition() << endl;
  }

  return NodeContainer(xStreetUE, yStreetUE);
}

/** Sets up two drone nodes that will follow the clusters */
NodeContainer useDroneENB() {
  useClusterMobility();

  NodeContainer drones;
  drones.Create(2);

  Ptr<MobilityModel> droneHeight = CreateObject<ConstantPositionMobilityModel>();

  

  Ptr<HierarchicalMobilityModel> xMobility = CreateObjectWithAttributes<HierarchicalMobilityModel>(
    "Parent", PointerValue(xClusterMobility),
    "Child", PointerValue(droneHeight)
  );

  Ptr<HierarchicalMobilityModel> yMobility = CreateObjectWithAttributes<HierarchicalMobilityModel>(
    "Parent", PointerValue(yClusterMobility),
    "Child", PointerValue(droneHeight)
  );

  // Paper doesn't really specify the drone height
  // We'll take it to be the same as the infrastructure height
  xMobility->SetPosition(xClusterMobility->GetPosition() + Vector(0, 0, 3.0));
  yMobility->SetPosition(yClusterMobility->GetPosition() + Vector(0, 0, 3.0));

  // Add in mobility models following cluster centers
  drones.Get(0)->AggregateObject(xMobility);
  drones.Get(1)->AggregateObject(yMobility);

  std::cout << "squirrelled " << xClusterMobility->GetPosition() << " + " << droneHeight->GetPosition() << " -> " << xMobility->GetPosition() << endl;

  return drones;
}

int main(int argc, char **argv) {
  try {
   
    CommandLine cmd;
    cmd.Parse(argc, argv);

    // Table 1: Propagation Scenario = UMi-StreetCanyon
    Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::Scenario", StringValue ("UMi-StreetCanyon"));


    // From mc-twoenbs.cc
    // Argument parsing placeholders
    UintegerValue uintegerValue;
    BooleanValue booleanValue;
    StringValue stringValue;
    DoubleValue doubleValue;

    GlobalValue::GetValueByName ("outPath", stringValue);
    std::string path = stringValue.Get();
    GlobalValue::GetValueByName ("runNumber", uintegerValue);
    uint32_t runSet = uintegerValue.Get ();
    GlobalValue::GetValueByName ("scenario", uintegerValue);
    uint32_t scenario = uintegerValue.Get ();
    SetupTraceConfig(path, runSet);

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults ();

    // Link in our simulation components
    Ptr<MmWaveHelper> mmWave = useMmWave();
    Ptr<InternetComponent> internet = useInternet(mmWave);

    // Create the four blocks
    BuildingContainer buildings = generateManhattanGrid(2, 2);
    NodeContainer enbNodes = usePoleMountedENB();
    

    // Set up scenario specific objects
    NodeContainer ueNodes;
    if (scenario == 1) {
      ueNodes.Add(useDistributedUE());
    } else if (scenario == 2) {
      ueNodes.Add(useClusteredUE());
    } else if (scenario == 3) {
      ueNodes.Add(useClusteredUE());
      enbNodes.Add(useDroneENB());
    } else {
      throw std::invalid_argument("Invalid scenario #");
    }

    // Associate with our buildings
    BuildingsHelper::Install(ueNodes);
    BuildingsHelper::Install(enbNodes);

    // Add mmWave model
    NetDeviceContainer enbNetDev = mmWave->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueNetDev = mmWave->InstallUeDevice(ueNodes);

    // Install the IP stack on the UEs
    internet->stack.Install (ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = internet->epcHelper->AssignUeIpv4Address (ueNetDev);
    // Assign IP address to UEs, and install applications
    for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = internet->routingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (internet->epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

    // Log some things
    mmWave->EnableTraces();

    // Associate UE to ENB
    mmWave->AttachToClosestEnb(ueNetDev, enbNetDev);

    // Schedule some traffic
    useTrafficGenerator(ueNodes, ueIpIface, internet->remoteHost);

    // Set QoS
    EpsBearer bearer(EpsBearer::GBR_CONV_VOICE);
    mmWave->ActivateDataRadioBearer(ueNetDev, bearer);

    // Ensures everyone stays outside?
    BuildingsHelper::MakeMobilityModelConsistent();

    // Schedule connection/location logging
    LogAllUeLocations(mobilityOs, &ueNodes, &enbNodes);

    Simulator::Stop(Seconds(85));
    Simulator::Run();
    Simulator::Destroy();
    return 0;
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    throw;
  }
}
