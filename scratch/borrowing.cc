/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/* *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Michele Polese <michele.polese@gmail.com>
 */

#include "ns3/mmwave-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
//#include "ns3/gtk-config-store.h"
#include <ns3/buildings-helper.h>
#include <ns3/buildings-module.h>
#include <ns3/random-variable-stream.h>
#include <ns3/lte-ue-net-device.h>

#include <iostream>
#include <ctime>
#include <stdlib.h>
#include <list>


using namespace ns3;
using namespace mmwave;
using namespace std;

/**
 * Sample simulation script for MC device. It instantiates a LTE and two MmWave eNodeB,
 * attaches one MC UE to both and starts a flow for the UE to and from a remote host.
 */

NS_LOG_COMPONENT_DEFINE ("Borrowing");

/* Start of JM stuff */

static ofstream *mobilityOs;
#define UEPerStreet 10

static ns3::GlobalValue g_scenario ("scenario", "Scenario to use",
                                     ns3::UintegerValue (1), ns3::MakeUintegerChecker<uint32_t> ());


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

/** Retrieve the (first) MMWave network device on this node */
Ptr<McUeNetDevice> GetUeNetDev(Ptr<const Node> node) {
  for (unsigned i = 0; i < node->GetNDevices(); i++) {
    Ptr<McUeNetDevice> dev = node->GetDevice(i)->GetObject<McUeNetDevice>();
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

  auto mmphy = netDev->GetMmWavePhy();
  auto mmenb = netDev->GetMmWaveTargetEnb();

  auto lteenb = netDev->GetLteTargetEnb();
  
  // Prints connection info, position, and velocities
  *os << "ue\t"
      << Simulator::Now() << "\t"
      // Connection info
      << nodeId << "\t"
      << netDev->GetImsi() << "\t"
      << (mmphy ? mmphy->GetRnti() : -1) << "\t"
      << (mmenb ? mmenb->GetCellId() : -1) << "\t"
      << (lteenb ? lteenb->GetCellId() : -1) << "\t"
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
      << "\t" // lte cell id
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

NodeContainer usePoleMountedENB() {
  // Set up base stations (ENB/"Evolved Node B" in 3GPP terminology)
  NodeContainer mmWaveEnb;
  mmWaveEnb.Create(4);

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
  enbMobility.Install(mmWaveEnb);

  return mmWaveEnb;
}

NodeContainer useLteNodes() {
  NodeContainer lteEnb;
  lteEnb.Create(1);

  // Assign the ENB a constant position
  auto enbPositionAlloc = CreateObject<ListPositionAllocator>();

  // For the LTE tower (a single tower on the edge of the map,
  // seems to be needed to coordinate handovers)
  enbPositionAlloc->Add(Vector(0, 0, 16.0));

  MobilityHelper enbMobility;
  enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  enbMobility.SetPositionAllocator(enbPositionAlloc);
  enbMobility.Install(lteEnb);

  return lteEnb;
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

  Ptr<PositionAllocator>
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
    "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.001]"),
    // Table I: Speed = 5 km/h ~= 1.4 m/s
    "Speed", StringValue("ns3::ConstantRandomVariable[Constant=1.4]"),
    // Hopefully stay on the streets?
    "PositionAllocator", PointerValue(xAllocator)
  );
  xStreetMob.SetPositionAllocator(xAllocator);
  xStreetMob.Install(xStreetUE);

  yStreetMob.SetMobilityModel("ns3::RandomWaypointMobilityModel",
    // How long to wait initially and at waypoints
    "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.002]"),
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
  //endInline = 7.5 + 5000 / 3600 * 85;
  // Actually just keep going, it'll stop after the sim is over
  endInline = 165 - 7.5;

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
        "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.003]"),
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
        "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.004]"),
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
    "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.005]"),
    // Table I: Speed = 5 km/h ~= 1.4 m/s
    "Speed", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"),
    "PositionAllocator", PointerValue(inGroupAllocator)
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
    "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.006]"),
    // Table I: Speed = 5 km/h ~= 1.4 m/s
    "Speed", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"),
    "PositionAllocator", PointerValue(inGroupAllocator)
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
  xMobility->SetPosition(xClusterMobility->GetPosition() + Vector(0, 0, 13.0));
  yMobility->SetPosition(yClusterMobility->GetPosition() + Vector(0, 0, 13.0));

  // Add in mobility models following cluster centers
  drones.Get(0)->AggregateObject(xMobility);
  drones.Get(1)->AggregateObject(yMobility);

  std::cout << "squirrelled " << xClusterMobility->GetPosition() << " + " << droneHeight->GetPosition() << " -> " << xMobility->GetPosition() << endl;

  return drones;
}

/* End of JM stuff */


static ns3::GlobalValue g_mmw1DistFromMainStreet ("mmw1Dist", "Distance from the main street of the first MmWaveEnb",
                                                  ns3::UintegerValue (50), ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_mmw2DistFromMainStreet ("mmw2Dist", "Distance from the main street of the second MmWaveEnb",
                                                  ns3::UintegerValue (50), ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_mmw3DistFromMainStreet ("mmw3Dist", "Distance from the main street of the third MmWaveEnb",
                                                  ns3::UintegerValue (110), ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_mmWaveDistance ("mmWaveDist", "Distance between MmWave eNB 1 and 2",
                                          ns3::UintegerValue (200), ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_numBuildingsBetweenMmWaveEnb ("numBlocks", "Number of buildings between MmWave eNB 1 and 2",
                                                        ns3::UintegerValue (8), ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_interPckInterval ("interPckInterval", "Interarrival time of UDP packets (us)",
                                            ns3::UintegerValue (1), ns3::MakeUintegerChecker<uint32_t> ());   // TB 1: Packet Interval=1us
static ns3::GlobalValue g_bufferSize ("bufferSize", "RLC tx buffer size (MB)",
                                      ns3::UintegerValue (20), ns3::MakeUintegerChecker<uint32_t> ());
// Taking to match the Table 1: PGW and remote host link delay = 10ms
static ns3::GlobalValue g_x2Latency ("x2Latency", "Latency on X2 interface (us)",
                                     ns3::DoubleValue (500), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_mmeLatency ("mmeLatency", "Latency on MME interface (us)",
                                      ns3::DoubleValue (10000), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_rlcAmEnabled ("rlcAmEnabled", "If true, use RLC AM, else use RLC UM",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());
static ns3::GlobalValue g_runNumber ("runNumber", "Run number for rng",
                                     ns3::UintegerValue (10), ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_outPath ("outPath",
                                   "The path of output log files",
                                   ns3::StringValue ("./project-stats/"), ns3::MakeStringChecker ());
static ns3::GlobalValue g_noiseAndFilter ("noiseAndFilter", "If true, use noisy SINR samples, filtered. If false, just use the SINR measure",
                                          ns3::BooleanValue (false), ns3::MakeBooleanChecker ());
static ns3::GlobalValue g_handoverMode ("handoverMode",
                                        "Handover mode",
                                        ns3::UintegerValue (3), ns3::MakeUintegerChecker<uint8_t> ());
static ns3::GlobalValue g_reportTablePeriodicity ("reportTablePeriodicity", "Periodicity of RTs",
                                                  ns3::UintegerValue (1600), ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_outageThreshold ("outageTh", "Outage threshold",
                                           ns3::DoubleValue (-5), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_lteUplink ("lteUplink", "If true, always use LTE for uplink signalling",
                                     ns3::BooleanValue (false), ns3::MakeBooleanChecker ());

int
main (int argc, char *argv[])
{
  // Debugging things
  Packet::EnablePrinting();
//  Packet::EnableChecking();

  bool harqEnabled = true;
  bool fixedTti = false;
  unsigned symPerSf = 24;
  double sfPeriod = 100.0;

  std::list<Box>  m_previousBlocks;

  // Command line arguments
  CommandLine cmd;
  cmd.Parse (argc, argv);

  UintegerValue uintegerValue;
  BooleanValue booleanValue;
  StringValue stringValue;
  DoubleValue doubleValue;
  //EnumValue enumValue;

  // Variables for the RT
  int windowForTransient = 150; // number of samples for the vector to use in the filter
  GlobalValue::GetValueByName ("reportTablePeriodicity", uintegerValue);
  int ReportTablePeriodicity = (int)uintegerValue.Get (); // in microseconds
  if (ReportTablePeriodicity == 1600)
    {
      windowForTransient = 150;
    }
  else if (ReportTablePeriodicity == 25600)
    {
      windowForTransient = 50;
    }
  else if (ReportTablePeriodicity == 12800)
    {
      windowForTransient = 100;
    }
  else
    {
      NS_ASSERT_MSG (false, "Unrecognized");
    }

  int vectorTransient = windowForTransient * ReportTablePeriodicity;

  // params for RT, filter, HO mode
  GlobalValue::GetValueByName ("noiseAndFilter", booleanValue);
  bool noiseAndFilter = booleanValue.Get ();
  GlobalValue::GetValueByName ("outageTh", doubleValue);
  double outageTh = doubleValue.Get ();

  GlobalValue::GetValueByName ("rlcAmEnabled", booleanValue);
  bool rlcAmEnabled = booleanValue.Get ();
  GlobalValue::GetValueByName ("bufferSize", uintegerValue);
  uint32_t bufferSize = uintegerValue.Get ();
  GlobalValue::GetValueByName ("interPckInterval", uintegerValue);
  uint32_t interPacketInterval = uintegerValue.Get ();
  GlobalValue::GetValueByName ("x2Latency", doubleValue);
  double x2Latency = doubleValue.Get ();
  GlobalValue::GetValueByName ("mmeLatency", doubleValue);
  double mmeLatency = doubleValue.Get ();

  std::cout << "rlcAmEnabled " << rlcAmEnabled << " bufferSize " << bufferSize << " interPacketInterval " <<
                 interPacketInterval << " x2Latency " << x2Latency << " mmeLatency " << mmeLatency << std::endl;

  // rng things
  GlobalValue::GetValueByName ("runNumber", uintegerValue);
  uint32_t runSet = uintegerValue.Get ();
  uint32_t seedSet = 5;
  RngSeedManager::SetSeed (seedSet);
  RngSeedManager::SetRun (runSet);
  char seedSetStr[21];
  char runSetStr[21];
  sprintf (seedSetStr, "%d", seedSet);
  sprintf (runSetStr, "%d", runSet);

  GlobalValue::GetValueByName ("outPath", stringValue);
  std::string path = stringValue.Get ();
  std::string extension = ".txt";
  Config::SetDefault ("ns3::MmWaveUeMac::UpdateUeSinrEstimatePeriod", DoubleValue (0));

  //get current time
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];
  time (&rawtime);
  timeinfo = localtime (&rawtime);
  strftime (buffer,80,"%Y%m%d_%I-%M-%S",timeinfo);
  std::string time_str (buffer);

  UintegerValue scenarioValue;
  GlobalValue::GetValueByName ("scenario", scenarioValue);
  uint32_t scenario = scenarioValue.Get ();
  char scenarioStr[4];
  sprintf (scenarioStr, "%d", scenario);

  auto mkPath = [=](std::string subName) {
    return StringValue(path + time_str + "_scene" + scenarioStr + "_" + subName + "_seed" + seedSetStr + "_run" + runSetStr + extension);
  };

  Config::SetDefault ("ns3::MmWaveHelper::RlcAmEnabled", BooleanValue (rlcAmEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveFlexTtiMaxWeightMacScheduler::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveFlexTtiMaxWeightMacScheduler::FixedTti", BooleanValue (fixedTti));
  Config::SetDefault ("ns3::MmWaveFlexTtiMaxWeightMacScheduler::SymPerSlot", UintegerValue (6));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::ResourceBlockNum", UintegerValue (1));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::ChunkPerRB", UintegerValue (72));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::SymbolsPerSubframe", UintegerValue (symPerSf));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::SubframePeriod", DoubleValue (sfPeriod));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::TbDecodeLatency", UintegerValue (200.0));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::NumHarqProcess", UintegerValue (100));
  Config::SetDefault ("ns3::MmWaveBeamforming::LongTermUpdatePeriod", TimeValue (MilliSeconds (100.0)));
  Config::SetDefault ("ns3::LteEnbRrc::SystemInformationPeriodicity", TimeValue (MilliSeconds (5.0)));
  Config::SetDefault ("ns3::LteRlcAm::ReportBufferStatusTimer", TimeValue (MicroSeconds (100.0)));
  Config::SetDefault ("ns3::LteRlcUmLowLat::ReportBufferStatusTimer", TimeValue (MicroSeconds (100.0)));
  Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue (320));
  Config::SetDefault ("ns3::LteEnbRrc::FirstSibTime", UintegerValue (2));
  Config::SetDefault ("ns3::MmWavePointToPointEpcHelper::X2LinkDelay", TimeValue (MicroSeconds (x2Latency)));
  Config::SetDefault ("ns3::MmWavePointToPointEpcHelper::X2LinkDataRate", DataRateValue (DataRate ("1000Gb/s")));
  Config::SetDefault ("ns3::MmWavePointToPointEpcHelper::X2LinkMtu",  UintegerValue (10000));
  Config::SetDefault ("ns3::MmWavePointToPointEpcHelper::S1uLinkDelay", TimeValue (MicroSeconds (1000)));
  Config::SetDefault ("ns3::MmWavePointToPointEpcHelper::S1apLinkDelay", TimeValue (MicroSeconds (mmeLatency)));
  
  Config::SetDefault ("ns3::McStatsCalculator::MmWaveOutputFilename", mkPath("mmwave"));
  Config::SetDefault ("ns3::McStatsCalculator::LteOutputFilename", mkPath("lte"));
  Config::SetDefault ("ns3::McStatsCalculator::CellIdInTimeOutputFilename", mkPath("cellIdInTime"));
  Config::SetDefault ("ns3::MmWaveBearerStatsCalculator::DlRlcOutputFilename", mkPath("DlRlc"));
  Config::SetDefault ("ns3::MmWaveBearerStatsCalculator::UlRlcOutputFilename", mkPath("UlRlc"));
  Config::SetDefault ("ns3::MmWaveBearerStatsCalculator::DlPdcpOutputFilename", mkPath("DlPdcp"));
  Config::SetDefault ("ns3::MmWaveBearerStatsCalculator::UlPdcpOutputFilename", mkPath("UlPdcp"));
  Config::SetDefault ("ns3::MmWaveBearerStatsConnector::UeHandoverStartOutputFilename", mkPath("UeHandoverStart"));
  Config::SetDefault ("ns3::MmWaveBearerStatsConnector::EnbHandoverStartOutputFilename", mkPath("enbHandoverStart"));
  Config::SetDefault ("ns3::MmWaveBearerStatsConnector::UeHandoverEndOutputFilename", mkPath("ueHandoverEnd"));
  Config::SetDefault ("ns3::MmWaveBearerStatsConnector::EnbHandoverEndOutputFilename", mkPath("enbHandoverEnd"));
  Config::SetDefault ("ns3::MmWaveBearerStatsConnector::CellIdStatsHandoverOutputFilename", mkPath("cellIdStatsHandover"));
  Config::SetDefault ("ns3::MmWaveBearerStatsConnector::MmWaveSinrOutputFilename", mkPath("mmwaveSinr"));
  //Config::SetDefault ("ns3::CoreNetworkStatsCalculator::X2FileName", mkPath("x2"));
  //std::string lostFilename = path + version + "LostUdpPackets" +  "_" + seedSetStr + "_" + runSetStr + "_" + time_str + extension;
  //Config::SetDefault ("ns3::UdpServer::ReceivedPacketsFilename", StringValue(path + version + "ReceivedUdp" +  "_" + seedSetStr + "_" + runSetStr + "_" + time_str + extension));
  //Config::SetDefault ("ns3::UdpClient::SentPacketsFilename", StringValue(path + version + "SentUdp" +  "_" + seedSetStr + "_" + runSetStr + "_" + time_str + extension));
  //Config::SetDefault ("ns3::UdpServer::ReceivedSnFilename", StringValue(path + version + "ReceivedSn" +  "_" + seedSetStr + "_" + runSetStr + "_" + time_str + extension));
  Config::SetDefault ("ns3::LteRlcAm::BufferSizeFilename", mkPath("bufferSize"));
  Config::SetDefault ("ns3::MmWavePhyRxTrace::OutputFilename", mkPath("PacketTrace"));

  mobilityOs = new ofstream();
  mobilityOs->open(mkPath("mobility").Get());

  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (bufferSize * 1024 * 1024));
  Config::SetDefault ("ns3::LteRlcUmLowLat::MaxTxBufferSize", UintegerValue (bufferSize * 1024 * 1024));
  Config::SetDefault ("ns3::LteRlcAm::StatusProhibitTimer", TimeValue (MilliSeconds (10.0)));
  Config::SetDefault ("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue (bufferSize * 1024 * 1024));

  // handover and RT related params
  // Hard handover on 3 dB advantage
  Config::SetDefault ("ns3::LteEnbRrc::SecondaryCellHandoverMode", EnumValue (LteEnbRrc::THRESHOLD));
  Config::SetDefault ("ns3::LteEnbRrc::HoSinrDifference", DoubleValue(3.0));

  Config::SetDefault ("ns3::LteEnbRrc::FixedTttValue", UintegerValue (250));
  Config::SetDefault ("ns3::LteEnbRrc::CrtPeriod", IntegerValue (ReportTablePeriodicity));
  Config::SetDefault ("ns3::LteEnbRrc::OutageThreshold", DoubleValue (outageTh));
  Config::SetDefault ("ns3::MmWaveEnbPhy::UpdateSinrEstimatePeriod", IntegerValue (ReportTablePeriodicity));
  Config::SetDefault ("ns3::MmWaveEnbPhy::Transient", IntegerValue (vectorTransient));
  Config::SetDefault ("ns3::MmWaveEnbPhy::NoiseAndFilter", BooleanValue (noiseAndFilter));

  GlobalValue::GetValueByName ("lteUplink", booleanValue);
  bool lteUplink = booleanValue.Get ();

  Config::SetDefault ("ns3::McUePdcp::LteUplink", BooleanValue (lteUplink));
  std::cout << "Lte uplink " << lteUplink << "\n";

  // settings for the 3GPP the channel
  //Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::ChannelCondition", StringValue ("a"));
  Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::Scenario", StringValue ("UMi-StreetCanyon"));
  Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::OptionalNlos", BooleanValue (true));
  //Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::Shadowing", BooleanValue (true)); // enable or disable the shadowing effect
  //Config::SetDefault ("ns3::MmWave3gppBuildingsPropagationLossModel::UpdateCondition", BooleanValue (true)); // enable or disable the LOS/NLOS update when the UE moves
  //Config::SetDefault ("ns3::AntennaArrayModel::AntennaHorizontalSpacing", DoubleValue (0.5));
  //Config::SetDefault ("ns3::AntennaArrayModel::AntennaVerticalSpacing", DoubleValue (0.5));
  Config::SetDefault ("ns3::MmWave3gppChannel::UpdatePeriod", TimeValue (MilliSeconds (100))); // interval after which the channel for a moving user is updated,
  // with spatial consistency procedure. If 0, spatial consistency is not used
  Config::SetDefault ("ns3::MmWave3gppChannel::DirectBeam", BooleanValue (true)); // Set true to perform the beam in the exact direction of receiver node.
  Config::SetDefault ("ns3::MmWave3gppChannel::Blockage", BooleanValue (true)); // use blockage or not
  //Config::SetDefault ("ns3::MmWave3gppChannel::PortraitMode", BooleanValue (true)); // use blockage model with UT in portrait mode
  //Config::SetDefault ("ns3::MmWave3gppChannel::NumNonselfBlocking", IntegerValue (4)); // number of non-self blocking obstacles

  // set the number of antennas in the devices
  //Config::SetDefault ("ns3::McUeNetDevice::AntennaNum", UintegerValue(16));
  //Config::SetDefault ("ns3::MmWaveEnbNetDevice::AntennaNum", UintegerValue(64));

  Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper> ();


  // From II.B.1: "All simulations utilize the 3GPP pathloss model [10] to calculate penetration loss in buildings"
  mmwaveHelper->SetAttribute("PathlossModel", StringValue("ns3::MmWave3gppBuildingsPropagationLossModel"));
  // From II.A: "The channel model used for the simulations in this paper is
  // based on 3GPP mmWave channel model described in [10]"
  mmwaveHelper->SetAttribute("ChannelModel", StringValue("ns3::MmWave3gppChannel"));

  //Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper> ();
  //mmwaveHelper->SetSchedulerType ("ns3::MmWaveFlexTtiMaxWeightMacScheduler");
  Ptr<MmWavePointToPointEpcHelper> epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
  mmwaveHelper->SetEpcHelper (epcHelper);
  mmwaveHelper->SetHarqEnabled (harqEnabled);
//  mmwaveHelper->SetAttribute ("PathlossModel", StringValue ("ns3::BuildingsObstaclePropagationLossModel"));
  mmwaveHelper->Initialize ();

  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults ();

  // parse again so you can override default values from the command line
  cmd.Parse (argc, argv);

  // Get SGW/PGW and create a single RemoteHost
  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  // Create the Internet by connecting remoteHost to pgw. Setup routing too
  PointToPointHelper p2ph;
  // Table 1: Input data rate=100 Gbps
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  // Table 1: MTU size = 1500 bytes
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  // Table 1: PGW and remote host link delay=10ms
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  // interface 0 is localhost, 1 is the p2p device
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  // Create buildings
  generateManhattanGrid(2, 2);

  // create LTE, mmWave eNB nodes and UE node
  NodeContainer ueNodes;
  NodeContainer mmWaveEnbNodes;
  NodeContainer lteEnbNodes;
  NodeContainer allEnbNodes;
  mmWaveEnbNodes.Add (usePoleMountedENB());
  lteEnbNodes.Add (useLteNodes());

  // Set up scenario specific objects
  if (scenario == 1) {
    ueNodes.Add(useDistributedUE());
  } else if (scenario == 2) {
    ueNodes.Add(useClusteredUE());
  } else if (scenario == 3) {
    ueNodes.Add(useClusteredUE());
    mmWaveEnbNodes.Add(useDroneENB());
  } else {
    throw std::invalid_argument("Invalid scenario #");
  }

  allEnbNodes.Add (mmWaveEnbNodes);
  allEnbNodes.Add (lteEnbNodes);

  
  
  // Associate with our buildings
  BuildingsHelper::Install(ueNodes);
  BuildingsHelper::Install(mmWaveEnbNodes);
  BuildingsHelper::Install(lteEnbNodes);

  // Install mmWave, lte, mc Devices to the nodes
  NetDeviceContainer lteEnbDevs = mmwaveHelper->InstallLteEnbDevice (lteEnbNodes);
  NetDeviceContainer mmWaveEnbDevs = mmwaveHelper->InstallEnbDevice (mmWaveEnbNodes);
  NetDeviceContainer mcUeDevs = mmwaveHelper->InstallMcUeDevice (ueNodes);

  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (mcUeDevs));
  // Assign IP address to UEs, and install applications
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

  // Add X2 interfaces
  mmwaveHelper->AddX2Interface (lteEnbNodes, mmWaveEnbNodes);

  // Manual attachment
  mmwaveHelper->AttachToClosestEnb (mcUeDevs, mmWaveEnbDevs, lteEnbDevs);

  // Install and start applications on UEs and remote host
  uint16_t dlPort = 1234;
  uint16_t ulPort = 2000;
  ApplicationContainer clientApps;
  ApplicationContainer serverApps;
  bool dl = 1;
  bool ul = 0;

  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      if (dl)
        {
          UdpServerHelper dlPacketSinkHelper (dlPort);
          dlPacketSinkHelper.SetAttribute ("PacketWindowSize", UintegerValue (256));
          serverApps.Add (dlPacketSinkHelper.Install (ueNodes.Get (u)));

          // Simulator::Schedule(MilliSeconds(20), &PrintLostUdpPackets, DynamicCast<UdpServer>(serverApps.Get(serverApps.GetN()-1)), lostFilename);

          UdpClientHelper dlClient (ueIpIface.GetAddress (u), dlPort);
          dlClient.SetAttribute ("Interval", TimeValue (MicroSeconds (interPacketInterval)));
          dlClient.SetAttribute ("MaxPackets", UintegerValue (0xFFFFFFFF));
          clientApps.Add (dlClient.Install (remoteHost));

        }
      if (ul)
        {
          ++ulPort;
          PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
          ulPacketSinkHelper.SetAttribute ("PacketWindowSize", UintegerValue (256));
          serverApps.Add (ulPacketSinkHelper.Install (remoteHost));
          UdpClientHelper ulClient (remoteHostAddr, ulPort);
          ulClient.SetAttribute ("Interval", TimeValue (MicroSeconds (interPacketInterval)));
          ulClient.SetAttribute ("MaxPackets", UintegerValue (0xFFFFFFFF));
          clientApps.Add (ulClient.Install (ueNodes.Get (u)));
        }
    }

  // Start applications
  serverApps.Start (Seconds (0.005));
  clientApps.Start (Seconds (0.005));

  LogAllUeLocations(mobilityOs, &ueNodes, &mmWaveEnbNodes);

  BuildingsHelper::MakeMobilityModelConsistent ();

  mmwaveHelper->EnableTraces ();

  Simulator::Stop (Seconds (85.0));
  Simulator::Run ();

  Simulator::Destroy ();
  return 0;
}

