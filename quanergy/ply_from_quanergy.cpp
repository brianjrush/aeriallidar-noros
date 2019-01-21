#include <signal.h>

#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

#include <quanergy/client/sensor_client.h>
#include <quanergy/parsers/data_packet_parser_00.h>
#include <quanergy/parsers/data_packet_parser_01.h>
#include <quanergy/parsers/data_packet_parser_04.h>
#include <quanergy/parsers/variadic_packet_parser.h>
#include <quanergy/modules/encoder_angle_calibration.h>
#include <quanergy/modules/polar_to_cart_converter.h>

// define some strings that will be used on command line
namespace
{
  static const std::string MANUAL_CORRECT{"--manual-correct"};
  static const std::string CALIBRATE_STR{"--calibrate"};
  static const std::string AMPLITUDE_STR{"--amplitude"};
  static const std::string PHASE_STR{"--phase"};
}

// output usage message
void usage(char** argv)
{
  std::cout << "usage: " << argv[0]
            << " --host <host> -o <output directory> [-h | --help] [" << CALIBRATE_STR << "][" << MANUAL_CORRECT << " " << AMPLITUDE_STR << " <value> " << PHASE_STR << " <value>]" << std::endl << std::endl

            << "    --host        hostname or IP address of the sensor" << std::endl
            << "    " << CALIBRATE_STR << "   calibrate the host sensor and apply calibration to outgoing points" << std::endl
            << "    " << MANUAL_CORRECT << " --amplitude <amplitude> --phase <phase>    Manually correct encoder error specifying amplitude and phase correction, in radians" << std::endl
            << "-h, --help        show this help and exit" << std::endl;
  return;
}

// convenient typedefs
typedef quanergy::client::SensorClient ClientType;
typedef quanergy::client::VariadicPacketParser<quanergy::PointCloudHVDIRPtr,                      // return type
                                               quanergy::client::DataPacketParser00,              // PARSER_00_INDEX
                                               quanergy::client::DataPacketParser01,              // PARSER_01_INDEX
                                               quanergy::client::DataPacketParser04> ParserType;  // PARSER_04_INDEX

// enum to make indexing into the VariadicPacketParser easier
enum
{
  PARSER_00_INDEX = 0,
  PARSER_01_INDEX = 1,
  PARSER_04_INDEX = 2
};

// PacketParserModule wraps the parser with signal/slot functionality
typedef quanergy::client::PacketParserModule<ParserType> ParserModuleType;
// EncoderAngleCalibration provides some calibration functionality for M8
typedef quanergy::calibration::EncoderAngleCalibration CalibrationType;
// PolarToCartConverter converts the polar point cloud to Cartesian
typedef quanergy::client::PolarToCartConverter ConverterType;

ClientType* client;
// store connections for cleaner shutdown
std::vector<boost::signals2::connection> connections;

void quit(int s) {
  client->stop();
  delete client;
  connections.clear();
  exit(0);
}

int main(int argc, char** argv)
{
  int max_num_args = 10;
  // get host
  if (argc < 2 || argc > max_num_args || pcl::console::find_switch(argc, argv, "-h") ||
      pcl::console::find_switch(argc, argv, "--help") || !pcl::console::find_switch(argc, argv, "--host") ||
      !pcl::console::find_switch(argc, argv, "-o"))
  {
    usage (argv);
    return (0);
  }

  std::string output_dir;
  pcl::console::parse_argument(argc, argv, "-o", output_dir);

  std::string host;
  std::string port = "4141";

  pcl::console::parse_argument(argc, argv, "--host", host);

  // create modules
  client = new ClientType(host, port, 100);
  ParserModuleType parser;
  ConverterType converter;

  CalibrationType calibrator;

  // setup modules
  // setFrameId sets the frame id in the PCL point clouds
  parser.get<PARSER_00_INDEX>().setFrameId("quanergy");
  // setReturnSelection allows choosing between the 3 returns (Packet00, only)
  parser.get<PARSER_00_INDEX>().setReturnSelection(0);
  // setDegreesOfSweepPerCloud allows breaking the point clouds into smaller pieces (M8 only)
  parser.get<PARSER_00_INDEX>().setDegreesOfSweepPerCloud(360.0);
  parser.get<PARSER_01_INDEX>().setFrameId("quanergy");
  parser.get<PARSER_04_INDEX>().setFrameId("quanergy");


  // connect the packets from the client to the parser
  connections.push_back(client->connect([&parser](const ClientType::ResultType& pc){ parser.slot(pc); }));
  
  // if we're doing automatic calibration or if we're setting the calibration
  // manually, include the calibrator in the chain
  if (pcl::console::find_switch(argc, argv, CALIBRATE_STR.c_str()) ||
      pcl::console::find_switch(argc, argv, MANUAL_CORRECT.c_str()))
  {
    // connect the parser to the calibrator
    connections.push_back(parser.connect([&calibrator](const ParserModuleType::ResultType& pc){ calibrator.slot(pc); }));
    // connect the calibrator to the converter
    connections.push_back(calibrator.connect([&converter](const CalibrationType::ResultType& pc){ converter.slot(pc); }));

    // set calibrator parameters
    if (pcl::console::find_switch(argc, argv, MANUAL_CORRECT.c_str()))
    {
      if (!pcl::console::find_switch(argc, argv, AMPLITUDE_STR.c_str()) ||
          !pcl::console::find_switch(argc, argv, PHASE_STR.c_str()))
      {
        usage(argv);
        return(0);
      }
      double amplitude = 0.;
      double phase = 0.;
      pcl::console::parse_argument(argc, argv, AMPLITUDE_STR.c_str(), amplitude);
      pcl::console::parse_argument(argc, argv, PHASE_STR.c_str(), phase);

      calibrator.setParams(amplitude, phase);
    }
  }
  else
  {
    // connect the parser to the converter
    connections.push_back(parser.connect([&converter](const ParserModuleType::ResultType& pc){ converter.slot(pc); }));
  }

  ////////////////////////////////////////////
  /// connect application specific logic here to consume the point cloud
  ////////////////////////////////////////////
  unsigned int cloud_count = 0;
  connections.push_back(converter.connect([&cloud_count, &output_dir](const ConverterType::ResultType& pc) {
    ++cloud_count;
    std::cout << pc->header.stamp << " " << pc->size() << " " << std::setfill('0') << std::setw(5) << cloud_count << std::endl;

    std::ostringstream os;
    os << output_dir << "/" << std::setfill('0') << std::setw(5) << cloud_count << ".ply";
    pcl::io::savePLYFileBinary(os.str(), *pc);
  }));

  signal(SIGINT, quit);

  client->run();
  return (0);
}
