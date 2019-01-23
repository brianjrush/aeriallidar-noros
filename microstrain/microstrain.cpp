extern "C" {
  #include "mip_sdk.h"
}
#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <fstream>

#define DEFAULT_PACKET_TIMEOUT_MS 1000
#define BAUDRATE 115200

typedef struct {
  float qx;
  float qy;
  float qz;
  float qw;
  double tow;
  u16 gwk;
  u16 time_valid_flags;
} ahrs_output_t;

typedef struct {
  double lat;
  double lng;
  double ellipsoid_height;
  double msl_height;
  float horiz_accuracy;
  float vert_accuracy;
  double tow;
  u16 gwk;
  u16 time_valid_flags;
} gps_output_t;

FILE* ahrs_outfile;
FILE* gps_outfile;
bool should_exit = false;
int count = 0;

void ahrs_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type) {
//  static ros::Publisher att_pub = nptr->advertise<sensor_msgs::Imu>("/microstrain/att", 1000);

  mip_field_header *field_header;
  u8 *field_data;
  u16 field_offset = 0;
  ahrs_output_t ahrs_output_message;

  switch(callback_type) {
    case MIP_INTERFACE_CALLBACK_VALID_PACKET:
    {
//      sensor_msgs::Imu att_msg;
//      att_msg.header.frame_id = "Sensor";
      while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK) {

        switch(field_header->descriptor) {
          case MIP_AHRS_DATA_QUATERNION:
          {
            mip_ahrs_quaternion qtn;
            memcpy(&qtn, field_data, sizeof(mip_ahrs_quaternion));
            mip_ahrs_quaternion_byteswap(&qtn);
//
//            att_msg.orientation.x = qtn.q[1];
//            att_msg.orientation.y = qtn.q[2];
//            att_msg.orientation.z = qtn.q[3];
//            att_msg.orientation.w = qtn.q[0];
            ahrs_output_message.qw = qtn.q[0];
            ahrs_output_message.qx = qtn.q[1];
            ahrs_output_message.qy = qtn.q[2];
            ahrs_output_message.qz = qtn.q[3];
            std::cout << "Qw: " << qtn.q[0] << " ";
            std::cout << "Qx: " << qtn.q[1] << " ";
            std::cout << "Qy: " << qtn.q[2] << " ";
            std::cout << "Qz: " << qtn.q[3] << " ";
          } break;
          case MIP_AHRS_DATA_TIME_STAMP_GPS:
          {
            mip_ahrs_gps_timestamp gpstime;
            memcpy(&gpstime, field_data, sizeof(mip_ahrs_gps_timestamp));
            mip_ahrs_gps_timestamp_byteswap(&gpstime);
//            ros::Time stamp;
//            // Check if we have signal
//            if( (gpstime.valid_flags & 1) == 1) {
//              // we do. Use GPS time
//              stamp = ros::Time(gps2unixtime(gpstime.tow, gpstime.week_number));
//            } else {
//              // we do not. Use the current unix time for now
//              stamp = ros::Time::now();
//            }
//            att_msg.header.stamp = stamp;
            ahrs_output_message.tow = gpstime.tow;
            ahrs_output_message.gwk = gpstime.week_number;
            ahrs_output_message.time_valid_flags = gpstime.valid_flags;
            std::cout << "TOW: " << gpstime.tow << " ";
            std::cout << "GWk: " << gpstime.week_number << " ";
          } break;
//          case MIP_AHRS_DATA_ACCEL_SCALED:
//          {
//            mip_ahrs_scaled_accel accel;
//            memcpy(&accel, field_data, sizeof(mip_ahrs_scaled_accel));
//            mip_ahrs_scaled_accel_byteswap(&accel);
//            
//            att_msg.linear_acceleration.x = accel.scaled_accel[0];
//            att_msg.linear_acceleration.y = accel.scaled_accel[1];
//            att_msg.linear_acceleration.z = accel.scaled_accel[2];
//          } break;
//          case MIP_AHRS_DATA_EULER_ANGLES:
//          {
//            mip_ahrs_euler_angles angle;
//            memcpy(&angle, field_data, sizeof(mip_ahrs_euler_angles));
//            mip_ahrs_euler_angles_byteswap(&angle);
//            
//            // THIS IS FOR TESTING ONLY.
//            // This is roll-pitch-yaw. It DOES NOT belong in att_msg.angular_velocity
//            att_msg.angular_velocity.x = angle.roll * 180. / 3.14;
//            att_msg.angular_velocity.y = angle.pitch * 180. / 3.14;
//            att_msg.angular_velocity.z = angle.yaw * 180. / 3.14;
//            std::cout << "Roll " << angle.roll * 180. / 3.14 << " ";
//            std::cout << "Pitch: " << angle.pitch * 180./3.14 << " ";
//            std::cout << "Yaw: " << angle.yaw * 180./3.14 << " ";
//          } break;
          default:
          {
            std::cerr << "Unknown AHRS packet type received: " << field_header->descriptor << std::endl;
          } break;
        }
      }
//      att_pub.publish(att_msg);
    } break;
    case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
    {
      std::cerr << "Microstrain AHRS packet checksum error" << std::endl;
    } break;
    case MIP_INTERFACE_CALLBACK_TIMEOUT:
    {
      std::cerr << "Microstrain AHRS packet timeout" << std::endl;
    } break;
    default:
    {
      std::cerr << "Unknown AHRS callback type from microstrain INS: " << callback_type << std::endl;
    } break;
  }
  fwrite(&ahrs_output_message.qw, sizeof(float), 1, ahrs_outfile);
  fwrite(&ahrs_output_message.qx, sizeof(float), 1, ahrs_outfile);
  fwrite(&ahrs_output_message.qy, sizeof(float), 1, ahrs_outfile);
  fwrite(&ahrs_output_message.qz, sizeof(float), 1, ahrs_outfile);
  fwrite(&ahrs_output_message.tow, sizeof(double), 1, ahrs_outfile);
  fwrite(&ahrs_output_message.gwk, sizeof(u16), 1, ahrs_outfile);
  fwrite(&ahrs_output_message.time_valid_flags, sizeof(u16), 1, ahrs_outfile);
  std::cout << " Count " << count << std::endl;
  count++;
}

void gps_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type) {
//  static ros::Publisher pos_pub = nptr->advertise<sensor_msgs::NavSatFix>("/microstrain/pos", 1000);

  mip_field_header *field_header;
  u8 *field_data;
  u16 field_offset = 0;
  gps_output_t gps_output_message;

  switch(callback_type) {
    case MIP_INTERFACE_CALLBACK_VALID_PACKET:
    {
//      sensor_msgs::NavSatFix pos_msg;
//      pos_msg.header.frame_id = "Sensor";
      while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK) {

        switch(field_header->descriptor) {
          case MIP_GPS_DATA_LLH_POS:
          {
            mip_gps_llh_pos pos;
            memcpy(&pos, field_data, sizeof(mip_gps_llh_pos));
            mip_gps_llh_pos_byteswap(&pos);

            gps_output_message.lat = pos.latitude;
            gps_output_message.lng = pos.longitude;
            gps_output_message.ellipsoid_height = pos.ellipsoid_height;
            gps_output_message.msl_height = pos.msl_height;
            gps_output_message.horiz_accuracy = pos.horizontal_accuracy;
            gps_output_message.vert_accuracy = pos.vertical_accuracy;
            
//            pos_msg.latitude = pos.latitude;
//            pos_msg.longitude = pos.longitude;
//            pos_msg.altitude = pos.msl_height;
          } break;
          case MIP_GPS_DATA_GPS_TIME:
          {
            mip_gps_time gpstime;
            memcpy(&gpstime, field_data, sizeof(mip_gps_time));
            mip_gps_time_byteswap(&gpstime);

            gps_output_message.tow = gpstime.tow;
            gps_output_message.gwk = gpstime.week_number;
            gps_output_message.time_valid_flags = gpstime.valid_flags;

//            ros::Time stamp;
//            // Check if we have signal
//            if( (gpstime.valid_flags & 1) == 1) {
//              // we do. Use GPS time
//              stamp = ros::Time(gps2unixtime(gpstime.tow, gpstime.week_number));
//            } else {
//              // we do not. Use the current unix time for now (might be better to swap this out for invalid values to be clear we do not have a lock)
//              stamp = ros::Time::now();
//            }
//            pos_msg.header.stamp = stamp;
          } break;
//          case MIP_GPS_DATA_FIX_INFO:
//          { 
//            mip_gps_fix_info fix_info;
//            memcpy(&fix_info, field_data, sizeof(mip_gps_fix_info));
//            mip_gps_fix_info_byteswap(&fix_info);
            
            // status is the type of fix:
            // 0x00 - 3D fix
            // 0x01 - 2D fix
            // 0x02 - Time only
            // 0x03 - None
            // 0x04 - Invalid
//            pos_msg.status.status = fix_info.fix_type;

            // service is the number of sv's we have
//            pos_msg.status.service = fix_info.num_sv;
//          } break;
          default:
          {
            std::cerr << "Unknown GPS packet type received: " << field_header->descriptor << std::endl;
          } break;
        }
      }
//      pos_pub.publish(pos_msg);
    } break;
    case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
    {
      std::cerr << "Microstrain GPS packet checksum error" << std::endl;
    } break;
    case MIP_INTERFACE_CALLBACK_TIMEOUT:
    {
      std::cerr << "Microstrain GPS packet timeout" << std::endl;
    } break;
    default:
    {
      std::cerr << "Unknown GPS callback type from microstrain INS: " << callback_type << std::endl;
    } break;
  }
  fwrite(&gps_output_message.lat, sizeof(double), 1, gps_outfile);
  fwrite(&gps_output_message.lng, sizeof(double), 1, gps_outfile);
  fwrite(&gps_output_message.ellipsoid_height, sizeof(double), 1, gps_outfile);
  fwrite(&gps_output_message.msl_height, sizeof(double), 1, gps_outfile);
  fwrite(&gps_output_message.horiz_accuracy, sizeof(float), 1, gps_outfile);
  fwrite(&gps_output_message.vert_accuracy, sizeof(float), 1, gps_outfile);
  fwrite(&gps_output_message.tow, sizeof(double), 1, gps_outfile);
  fwrite(&gps_output_message.gwk, sizeof(u16), 1, gps_outfile);
  fwrite(&gps_output_message.time_valid_flags, sizeof(u16), 1, gps_outfile);
}

u16 init_microstrain(std::string device, mip_interface *device_interface) {
  // connect to sensor
  if (mip_interface_init(const_cast<char*>(device.c_str()), BAUDRATE, device_interface, DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK) {
    std::cerr << "Failed to connect to Microstrain sensor" << std::endl;
    return MIP_INTERFACE_ERROR;
  }
  
  // define AHRS message
  u8 ahrs_num_fields =  2;
  u8 ahrs_data_descriptors[ahrs_num_fields];
  ahrs_data_descriptors[0] = MIP_AHRS_DATA_QUATERNION;
  ahrs_data_descriptors[1] = MIP_AHRS_DATA_TIME_STAMP_GPS;
  //ahrs_data_descriptors[2] = MIP_AHRS_DATA_EULER_ANGLES;
  //ahrs_data_descriptors[3] = MIP_AHRS_DATA_ACCEL_SCALED;
  
  u16 ahrs_data_decimation[ahrs_num_fields];
  ahrs_data_decimation[0] = 0x01;
  ahrs_data_decimation[1] = 0x01;
  //ahrs_data_decimation[2] = 0x01;
  //ahrs_data_decimation[3] = 0x01;

  // define GPS message
  u8 gps_num_fields = 2;
  u8 gps_data_descriptors[gps_num_fields];
  gps_data_descriptors[0] = MIP_GPS_DATA_LLH_POS;
  gps_data_descriptors[1] = MIP_GPS_DATA_GPS_TIME;
  //gps_data_descriptors[2] = MIP_GPS_DATA_FIX_INFO;

  u16 gps_data_decimation[gps_num_fields];
  gps_data_decimation[0] = 0x01;
  gps_data_decimation[1] = 0x01;
  //gps_data_decimation[2] = 0x01;
  
  // set AHRS and GPS messages
  while (mip_3dm_cmd_ahrs_message_format(device_interface, MIP_FUNCTION_SELECTOR_WRITE, &ahrs_num_fields, ahrs_data_descriptors, ahrs_data_decimation) != MIP_INTERFACE_OK) {}
  while (mip_3dm_cmd_gps_message_format(device_interface, MIP_FUNCTION_SELECTOR_WRITE, &gps_num_fields, gps_data_descriptors, gps_data_decimation) != MIP_INTERFACE_OK) {}

  // register AHRS and GPS message callbacks
  mip_interface_add_descriptor_set_callback(device_interface, MIP_AHRS_DATA_SET, NULL, &ahrs_packet_callback);
  mip_interface_add_descriptor_set_callback(device_interface, MIP_GPS_DATA_SET, NULL, &gps_packet_callback);
  return MIP_INTERFACE_OK;
}

void clean_exit(int s) {
  should_exit = true;
}

int main(int argc, char* argv[]) {
  mip_interface device_interface;
  std::string device = "/dev/microstrain";
  if (init_microstrain(device, &device_interface) == MIP_INTERFACE_ERROR) { return 1; }
  ahrs_outfile = fopen("ahrs.bin", "wb");
  gps_outfile = fopen("gps.bin", "wb");
//  ros::init(argc, argv, "microstrain");
  signal(SIGINT, clean_exit);
  while(!should_exit) {
    mip_interface_update(&device_interface);
    usleep(1000);
  }
  fclose(ahrs_outfile);
  fclose(gps_outfile);
}
