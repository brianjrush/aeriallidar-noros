#include <quanergy/parsers/failover_client.h>
#include <quanergy/common/pointcloud_types.h>
#include <quanergy/parsers/deserialize_00.h>
#include <quanergy/parsers/pointcloud_generator_00.h>
#include <iostream>
#include <iomanip>
#include <quanergy/parsers/deserialize_01.h>
#include <quanergy/parsers/pointcloud_generator_01.h>
#include <quanergy/modules/polar_to_cart_converter.h>

struct Client {
  typedef quanergy::client::FailoverClient<quanergy::client::DataPacket01, quanergy::client::DataPacket00> ClientType;
  typedef quanergy::client::PolarToCartConverter Converter;

  Client(std::string const &host, std::string const &port, std::string const outdir) 
    : client(host, port, "lidar", 100) {
    output_dir = outdir;
    std::cout << "Setting up client..." << std::endl;
    
    cloud_connection = client.connect([this](const ClientType::Result& pc)
                                     { this->converter.slot(pc);});
                                     //{ this->print(pc); });
    std::cout << "Connected to LiDAR" << std::endl;
    converter_connection = converter.connect([this](const Converter::ResultType& pc) {
  std::unique_lock<std::mutex> lock(this->pc_mutex);
  new_point_cloud = pc;
 });
    std::cout << "Connected to converter slot" << std::endl;
  }

  bool newCloud() {
    if (new_point_cloud != point_cloud) {
      std::unique_lock<std::mutex> lock(pc_mutex);
      point_cloud = new_point_cloud;
      return true;
    }
    return false;
  }

  void writePLY(std::string path, const quanergy::PointCloudXYZIRPtr &cloud) {
    FILE* fp = fopen(path.c_str(), "wb");
    fprintf(fp, "ply\n");
    fprintf(fp, "format binary_little_endian 1.0\n");
    fprintf(fp, "element vertex %lu\n", cloud->size());
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "property uchar red\n");
    fprintf(fp, "property uchar green\n");
    fprintf(fp, "property uchar blue\n");
    fprintf(fp, "element timestamp 1\n");
    fprintf(fp, "property double stamp\n");
    fprintf(fp, "end_header\n");

    for(quanergy::PointXYZIR p : cloud->points) {
      float position[3];
      unsigned char color[3];
      
      position[0] = p.x;
      position[1] = p.y;
      position[2] = p.z;

      color[0] = p.intensity;
      color[1] = p.intensity;
      color[2] = p.intensity;

      fwrite(position, sizeof(position), 1, fp);
      fwrite(color, sizeof(color), 1, fp);
    }
    double timestamp[] = {cloud->header.stamp/1E6};
    fwrite(timestamp, sizeof(timestamp), 1, fp);
    fclose(fp);
  }

  void run() {
    std::cout << "RUN" << std::endl;
    std::thread client_thread([this] {
                                       try {
                                         this->client.run();
                                       } catch (std::exception& e) {
                                         std::cerr << "Terminating after catching exception: " << e.what() << std::endl;
                                         this->kill_prog = true;
                                       }
                                     });

    std::cout << "Starting loop" << std::endl;
    count = 0;
    while (!kill_prog) {
      if (newCloud()) {
        count++;
        //std::cout << "got " << count << " pointclouds" << std::endl;
        std::cout << point_cloud->header.stamp << " " << point_cloud->size() << " " << std::setfill('0') << std::setw(5) << count << std::endl;

        std::ostringstream os;
        os << output_dir << "/" << std::setfill('0') << std::setw(5) << count << ".ply";

        writePLY(os.str(), point_cloud);
      }
    }

    converter_connection.disconnect();
    cloud_connection.disconnect();
    client.stop();
    client_thread.join();
  }

  private:
    ClientType client;
    boost::signals2::connection cloud_connection;
    boost::signals2::connection converter_connection;
    Converter::ResultType new_point_cloud;
    Converter::ResultType point_cloud;
    std::mutex pc_mutex;
    Converter converter;
    
    std::string output_dir;
    int count;
    bool kill_prog = false;
};

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "Please supply an output directory." << std::endl;
    return 1;
  }
  Client client("10.0.0.56", "4141", argv[1]);
  client.run();
  return 0;
}
