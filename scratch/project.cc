#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store.h"
#include "ns3/mmwave-helper.h"
#include <ns3/buildings-helper.h>
#include "ns3/log.h"
#include <ns3/buildings-module.h>
#include <vector>
#include <stdexcept>

using namespace std;
using namespace ns3;
using namespace mmwave;

/* NOTES

Some good terminoligy at
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

/** A bunch of configs from mc-twoenbs.cc example */
void stolenConfigs() {
  bool rlcAmEnabled = true;
  bool harqEnabled = true;
  bool fixedTti = false;
  unsigned symPerSf = 24;
  double sfPeriod = 100.0;
  double x2Latency = 500.0;
  double mmeLatency = 10000.0;

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

}

static ns3::GlobalValue g_outPath ("outPath",
                                   "The path of output log files",
                                   ns3::StringValue ("./project-stats/"), ns3::MakeStringChecker ());
static ns3::GlobalValue g_runNumber ("runNumber", "Run number for rng",
                                     ns3::UintegerValue (10), ns3::MakeUintegerChecker<uint32_t> ());

int main(int argc, char **argv) {
  try {
    LogComponentEnable ("MmWavePhyRxTrace", LOG_LEVEL_DEBUG);
    //LogComponentEnable ("MmWaveBearerStatsConnector", LOG_LEVEL_ALL);



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
    std::string mmWaveOutName = "MmWaveSwitchStats";
    std::string lteOutName = "LteSwitchStats";
    std::string dlRlcOutName = "DlRlcStats";
    std::string dlPdcpOutName = "DlPdcpStats";
    std::string ulRlcOutName = "UlRlcStats";
    std::string ulPdcpOutName = "UlPdcpStats";
    std::string  ueHandoverStartOutName =  "UeHandoverStartStats";
    std::string enbHandoverStartOutName = "EnbHandoverStartStats";
    std::string  ueHandoverEndOutName =  "UeHandoverEndStats";
    std::string enbHandoverEndOutName = "EnbHandoverEndStats";
    std::string cellIdInTimeOutName = "CellIdStats";
    std::string cellIdInTimeHandoverOutName = "CellIdStatsHandover";
    std::string mmWaveSinrOutputFilename = "MmWaveSinrTime";
    std::string x2statOutputFilename = "X2Stats";
    std::string udpSentFilename = "UdpSent";
    std::string udpReceivedFilename = "UdpReceived";
    std::string extension = ".txt";
    std::string version = "project";

    

    //get current time
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time (&rawtime);
    timeinfo = localtime (&rawtime);
    strftime (buffer,80,"%d_%m_%Y_%I_%M_%S",timeinfo);
    std::string time_str (buffer);

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

    stolenConfigs();

    auto mkPath = [=](string subName) {
      return StringValue(path + version + subName + "_" + seedSetStr + "_" + runSetStr + "_" + time_str + extension);
    };

    Config::SetDefault ("ns3::McStatsCalculator::MmWaveOutputFilename",                       mkPath(mmWaveOutName));
    Config::SetDefault ("ns3::McStatsCalculator::LteOutputFilename",                          mkPath(lteOutName));
    Config::SetDefault ("ns3::McStatsCalculator::CellIdInTimeOutputFilename",                 mkPath(cellIdInTimeOutName));
    Config::SetDefault ("ns3::MmWaveBearerStatsCalculator::DlRlcOutputFilename",              mkPath(dlRlcOutName));
    Config::SetDefault ("ns3::MmWaveBearerStatsCalculator::UlRlcOutputFilename",              mkPath(ulRlcOutName));
    Config::SetDefault ("ns3::MmWaveBearerStatsCalculator::DlPdcpOutputFilename",             mkPath(dlPdcpOutName));
    Config::SetDefault ("ns3::MmWaveBearerStatsCalculator::UlPdcpOutputFilename",             mkPath(ulPdcpOutName));
    Config::SetDefault ("ns3::MmWaveBearerStatsConnector::UeHandoverStartOutputFilename",     mkPath(ueHandoverStartOutName));
    Config::SetDefault ("ns3::MmWaveBearerStatsConnector::EnbHandoverStartOutputFilename",    mkPath(enbHandoverStartOutName));
    Config::SetDefault ("ns3::MmWaveBearerStatsConnector::UeHandoverEndOutputFilename",       mkPath(ueHandoverEndOutName));
    Config::SetDefault ("ns3::MmWaveBearerStatsConnector::EnbHandoverEndOutputFilename",      mkPath(enbHandoverEndOutName));
    Config::SetDefault ("ns3::MmWaveBearerStatsConnector::CellIdStatsHandoverOutputFilename", mkPath(cellIdInTimeHandoverOutName));
    Config::SetDefault ("ns3::MmWaveBearerStatsConnector::MmWaveSinrOutputFilename",          mkPath(mmWaveSinrOutputFilename));
    Config::SetDefault ("ns3::CoreNetworkStatsCalculator::X2FileName",                        mkPath(x2statOutputFilename));

    // Enable SINR reporting all the time
    //Config::SetDefault ("ns3::MmWaveUeMac::UpdateUeSinrEstimatePeriod", DoubleValue (0));


    Ptr<MmWaveHelper> mmWave = CreateObject<MmWaveHelper>();
    // From II.B.1: "All simulations utilize the 3GPP pathloss model [10] to calculate penetration loss in buildings"
    //mmWave->SetAttribute("PathlossModel", StringValue("ns3::MmWave3gppBuildingsPropagationLossModel"));
    // From II.A: "The channel model used for the simulations in this paper is
    // based on 3GPP mmWave channel model described in [10]"
    //mmWave->SetAttribute("ChannelModel", StringValue("ns3::MmWave3gppChannel"));
    mmWave->Initialize();

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults ();

    // Create the four blocks
    BuildingContainer buildings = generateManhattanGrid(2, 2);
    

    // Set up base stations (ENB/"Evolved Node B" in 3GPP terminology)
    NodeContainer enbNodes;
    enbNodes.Create(1);

    // Assign the ENB a constant position
    auto enbPositionAlloc = CreateObject<ListPositionAllocator>();
    // TODO update locations
    // Table 1: Infrastructure mmWave AP height = 3m
    enbPositionAlloc->Add(Vector(87.5, 0.0, 3.0));
    MobilityHelper enbMobility;
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.SetPositionAllocator(enbPositionAlloc);
    enbMobility.Install(enbNodes);

    // Associate with buildings
    BuildingsHelper::Install(enbNodes);

    // Set up users (UEs/"User Equipments")
    NodeContainer ueNodes;
    ueNodes.Create(1);

    // Assignt the UE a constant position for now
    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ueMobility.Install(ueNodes);
    ueNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(87.5, 70, 2.0));
    ueNodes.Get (0)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (1, 0, 0));



    // Associate with our buildings
    BuildingsHelper::Install(ueNodes);

    // Add mmWave model
    NetDeviceContainer enbNetDev = mmWave->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueNetDev = mmWave->InstallUeDevice(ueNodes);

    // Associate UE to ENB
    mmWave->AttachToClosestEnb(ueNetDev, enbNetDev);
    mmWave->EnableTraces();

    // Set QoS
    EpsBearer bearer(EpsBearer::GBR_CONV_VOICE);
    mmWave->ActivateDataRadioBearer(ueNetDev, bearer);
    
    

    // Figure out some things? Move people out of buildings? Who knows? (the
    // source does!)
    BuildingsHelper::MakeMobilityModelConsistent();


    

    Simulator::Stop(Seconds(2));
    Simulator::Run();
    Simulator::Destroy();
    return 0;
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    throw;
  }
}