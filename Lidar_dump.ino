#define PKG_HEADER 0x54
#define DATA_PKG_INFO 0x2C
#define POINT_PER_PACK 12
#define HEALTH_PKG_INFO 0xE0
#define MANUFACT_PKG_INF 0x0F

#define GET_PKG_PCD 1
#define GET_PKG_HEALTH 2
#define GET_PKG_MANUFACT 3
#define GET_PKG_ERROR 0

struct PointData {
  // Polar coordinate representation
  float angle;        // Angle ranges from 0 to 359 degrees
  uint16_t distance;  // Distance is measured in millimeters
  uint8_t intensity;  // Intensity is 0 to 255
  //! System time when first range was measured in nanoseconds
  uint32_t stamp;
  // Cartesian coordinate representation
  double x;
  double y;
  PointData(float angle, uint16_t distance, uint8_t intensity, uint64_t stamp = 0, double x = 0, double y = 0) {
    this->angle = angle;
    this->distance = distance;
    this->intensity = intensity;
    this->stamp = stamp;
    this->x = x;
    this->y = y;
  }
  PointData() {}
};

typedef std::vector<PointData> Points2D;

enum class LDType {
  LD_14P,
};

class SlTransform {
private:
  bool to_right_hand_ = true;
  double offset_x_;
  double offset_y_;
  LDType version_;

public:
  SlTransform(LDType version, bool to_right_hand = false);
  Points2D Transform(const Points2D &data);
  ~SlTransform();
};

static const uint8_t CrcTable[256] = {
  0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
  0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
  0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
  0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
  0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
  0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
  0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
  0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
  0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
  0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
  0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
  0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
  0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
  0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
  0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
  0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
  0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
  0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
  0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
  0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
  0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
  0x7f, 0x32, 0xe5, 0xa8
};

#pragma pack(1)
typedef struct {
  uint8_t header;
  uint8_t information;
  uint16_t speed;
  uint16_t product_version;
  uint32_t sn_high;
  uint32_t sn_low;
  uint32_t hardware_version;
  uint32_t firmware_version;
  uint8_t crc8;
} LiDARManufactureInfoType;

typedef struct {
  uint16_t distance;
  uint8_t intensity;
} LidarPointStructType;

typedef struct {
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructType point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARMeasureDataType;

typedef struct {
  uint8_t header;
  uint8_t information;
  uint8_t error_code;
  uint8_t crc8;
} LiDARHealthInfoType;
#pragma pack()

enum class LidarStatus {
  NORMAL,
  ERROR,
  DATA_TIME_OUT,
  DATA_WAIT,
  STOP,
};

LiDARMeasureDataType pcdpkg_data_;
LiDARHealthInfoType healthpkg_data_;
LiDARManufactureInfoType manufacinfpkg_data_;

int lidar_measure_freq_ = 4000;
LDType typenumber_ = LDType::LD_14P;
LidarStatus lidarstatus_;
uint8_t lidarerrorcode_;
bool is_frame_ready_;
bool is_noise_filter_;
uint16_t timestamp_;
double speed_;
bool is_poweron_comm_normal_;
uint64_t last_pkg_timestamp_;

// Set up a new SoftwareSerial object
int incomingByte = 0;
Points2D tmp_lidar_scan_data_vec_;
Points2D lidar_scan_data_vec_;
Points2D sdata;

byte buf1[2048];
byte buf2[2048];
int counter = 0;
int counter2;

void setup() {
  // Set the baud rate for the SoftwareSerial object
  Serial.begin(115200);
}

void loop() {
  Parse();
  if (AssemblePacket()) {
    sdata = GetLaserScanData();
    for (double ii = 0; ii < sdata.size(); ii++) {
      if(abs(sdata[ii].angle-180)<1){
        Serial.println(sdata[ii].angle);
        Serial.println(sdata[ii].distance);
      }
      counter = 0;
    }
  }
}

void setup1() {
  Serial2.setRX(5);
  Serial2.setTX(4);
  Serial2.setFIFOSize(256);
  Serial2.begin(230400);
}

void loop1() {
  if (Serial2.available() > 0) {
    buf1[counter] = Serial2.read();
    counter++;
  }
}

void Parse(void) {
  counter2 = counter;
  memcpy(buf2, buf1, counter2+1);
  counter = 0;

  //if (Serial2.available() > 0) {
  for (int iii = 0; iii < counter2; iii++) {
    // read the incoming byte:
    //uint8_t ret = AnalysisDataPacket(Serial2.read());
    uint8_t ret = AnalysisDataPacket(buf2[iii]);
    if (ret == GET_PKG_PCD) {
      LiDARMeasureDataType datapkg = GetPCDPacketData();
      is_poweron_comm_normal_ = true;
      speed_ = datapkg.speed;
      timestamp_ = datapkg.timestamp;
      // parse a package is success
      double diff = (datapkg.end_angle / 100 - datapkg.start_angle / 100 + 360) % 360;
      if (diff <= ((double)datapkg.speed * POINT_PER_PACK / lidar_measure_freq_ * 1.5)) {
        if (0 == last_pkg_timestamp_) {
          last_pkg_timestamp_ = get_timestamp_();
        } else {
          uint32_t current_pack_stamp = get_timestamp_();
          int pkg_point_number = POINT_PER_PACK;
          double pack_stamp_point_step = static_cast<double>(current_pack_stamp - last_pkg_timestamp_) / static_cast<double>(pkg_point_number - 1);
          uint32_t diff = ((uint32_t)datapkg.end_angle + 36000 - (uint32_t)datapkg.start_angle) % 36000;
          float step = diff / (POINT_PER_PACK - 1) / 100.0;
          float start = (double)datapkg.start_angle / 100.0;
          PointData data;
          for (int i = 0; i < POINT_PER_PACK; i++) {
            data.distance = datapkg.point[i].distance;
            data.angle = start + i * step;
            if (data.angle >= 360.0) {
              data.angle -= 360.0;
            }
            data.intensity = datapkg.point[i].intensity;
            data.stamp = static_cast<uint32_t>(last_pkg_timestamp_ + (pack_stamp_point_step * i));
            tmp_lidar_scan_data_vec_.push_back(PointData(data.angle, data.distance, data.intensity, data.stamp));
          }
          last_pkg_timestamp_ = current_pack_stamp;  //// update last pkg timestamp
        }
      }
    }
  }
}

uint8_t AnalysisDataPacket(uint8_t byte) {
  static enum {
    HEADER,
    VER_LEN,
    DATA,
    DATA_HEALTH,
    DATA_MANUFACTURE,
  } state = HEADER;
  static uint16_t count = 0;
  static uint8_t tmp[128] = { 0 };
  static uint16_t pkg_count = sizeof(LiDARMeasureDataType);
  static uint16_t pkghealth_count = sizeof(LiDARHealthInfoType);
  static uint16_t pkgmanufac_count = sizeof(LiDARManufactureInfoType);
  static bool check_healthinf_flag = false;

  switch (state) {
    case HEADER:
      {
        if (byte == PKG_HEADER) {
          tmp[count++] = byte;
          state = VER_LEN;
        } else {
          if (check_healthinf_flag) {
            check_healthinf_flag = false;
          }
        }
        break;
      }
    case VER_LEN:
      {
        if (byte == DATA_PKG_INFO) {
          tmp[count++] = byte;
          state = DATA;
          if (check_healthinf_flag) {
            check_healthinf_flag = false;
            return GET_PKG_HEALTH;
          }
        } else if (byte == HEALTH_PKG_INFO) {
          tmp[count++] = byte;
          state = DATA_HEALTH;
          if (check_healthinf_flag) {
            check_healthinf_flag = false;
            return GET_PKG_HEALTH;
          }
        } else if (byte == MANUFACT_PKG_INF) {
          tmp[count++] = byte;
          state = DATA_MANUFACTURE;
        } else {
          state = HEADER;
          count = 0;
          if (check_healthinf_flag) {
            check_healthinf_flag = false;
          }
          return GET_PKG_ERROR;
        }
        break;
      }
    case DATA:
      {
        tmp[count++] = byte;
        if (count >= pkg_count) {
          memcpy((uint8_t *)&pcdpkg_data_, tmp, pkg_count);
          uint8_t crc = CalCRC8((uint8_t *)&pcdpkg_data_, pkg_count - 1);
          state = HEADER;
          count = 0;
          if (crc == pcdpkg_data_.crc8) {
            return GET_PKG_PCD;
          } else {
            return GET_PKG_ERROR;
          }
        }
        break;
      }
    case DATA_HEALTH:
      {
        tmp[count++] = byte;
        if (count >= pkghealth_count) {
          memcpy((uint8_t *)&healthpkg_data_, tmp, pkghealth_count);
          uint8_t crc = CalCRC8((uint8_t *)&healthpkg_data_, pkghealth_count - 1);
          state = HEADER;
          count = 0;
          if (crc == healthpkg_data_.crc8) {
            check_healthinf_flag = true;
          } else {
            check_healthinf_flag = false;
          }
          return GET_PKG_ERROR;
        }
        break;
      }
    case DATA_MANUFACTURE:
      {
        tmp[count++] = byte;
        if (count >= pkgmanufac_count) {
          memcpy((uint8_t *)&manufacinfpkg_data_, tmp, pkgmanufac_count);
          uint8_t crc = CalCRC8((uint8_t *)&manufacinfpkg_data_, pkgmanufac_count - 1);
          state = HEADER;
          count = 0;
          if (crc == manufacinfpkg_data_.crc8) {
            return GET_PKG_MANUFACT;
          } else {
            return GET_PKG_ERROR;
          }
        }
        break;
      }
    default:
      {
        break;
      }
  }

  return GET_PKG_ERROR;
}

uint8_t CalCRC8(const uint8_t *data, uint16_t data_len) {
  uint8_t crc = 0;
  while (data_len--) {
    crc = CrcTable[(crc ^ *data) & 0xff];
    data++;
  }
  return crc;
}

LiDARMeasureDataType &GetPCDPacketData(void) {
  return pcdpkg_data_;
}

LiDARHealthInfoType &GetHealthPacketData(void) {
  return healthpkg_data_;
}

LiDARManufactureInfoType &GetManufactureInfoPacketData(void) {
  return manufacinfpkg_data_;
}

double get_timestamp_(void) {
  return millis();
}

double GetSpeed(void) {
  return (speed_ / 360.0);  // unit  is Hz
}

bool AssemblePacket() {
  float last_angle = 0;
  Points2D tmp, data;
  int count = 0;

  if (speed_ <= 0) {
    tmp_lidar_scan_data_vec_.erase(tmp_lidar_scan_data_vec_.begin(), tmp_lidar_scan_data_vec_.end());
    return false;
  }

  for (auto n : tmp_lidar_scan_data_vec_) {
    // wait for enough data, need enough data to show a circle
    // enough data has been obtained
    if ((n.angle < 20.0) && (last_angle > 340.0)) {
      if ((count * GetSpeed()) > (lidar_measure_freq_ * 1.4)) {
        if (count >= (int)tmp_lidar_scan_data_vec_.size()) {
          tmp_lidar_scan_data_vec_.clear();
        } else {
          tmp_lidar_scan_data_vec_.erase(tmp_lidar_scan_data_vec_.begin(), tmp_lidar_scan_data_vec_.begin() + count);
        }
        return false;
      }
      data.insert(data.begin(), tmp_lidar_scan_data_vec_.begin(), tmp_lidar_scan_data_vec_.begin() + count);


      SlTransform trans(typenumber_);
      data = trans.Transform(data);  // transform raw data to stantard data
      tmp = data;

      if (tmp.size() > 0) {
        lidar_scan_data_vec_ = tmp;
        SetFrameReady();

        if (count >= (int)tmp_lidar_scan_data_vec_.size()) {
          tmp_lidar_scan_data_vec_.clear();
        } else {
          tmp_lidar_scan_data_vec_.erase(tmp_lidar_scan_data_vec_.begin(), tmp_lidar_scan_data_vec_.begin() + count);
        }
        return true;
      }
    }
    count++;

    if ((count * GetSpeed()) > (lidar_measure_freq_ * 2)) {
      if (count >= (int)tmp_lidar_scan_data_vec_.size()) {
        tmp_lidar_scan_data_vec_.clear();
      } else {
        tmp_lidar_scan_data_vec_.erase(tmp_lidar_scan_data_vec_.begin(), tmp_lidar_scan_data_vec_.begin() + count);
      }
      return false;
    }

    last_angle = n.angle;
  }

  return false;
}

SlTransform::SlTransform(LDType version, bool to_right_hand) {
  offset_x_ = 5.9;
  offset_y_ = -18.975571;
  to_right_hand_ = to_right_hand;
  version_ = version;
}

Points2D SlTransform::Transform(const Points2D &data) {
  Points2D tmp2;
  static double last_shift_delta = 0;
  for (auto n : data) {
    // transfer the origin to the center of lidar circle
    // The default direction of radar rotation is clockwise
    // transfer to the right-hand coordinate system
    double angle;
    if (n.distance > 0) {
      double x = n.distance + offset_x_;
      double y = n.distance * 0.11923 + offset_y_;
      double shift = atan(y / x) * 180.f / 3.14159;
      // Choose whether to use the right-hand system according to the flag
      if (to_right_hand_) {
        float right_hand = (360.f - n.angle);
        angle = right_hand + shift;
      } else {
        angle = n.angle - shift;
      }
      last_shift_delta = shift;
    } else {
      if (to_right_hand_) {
        float right_hand = (360.f - n.angle);
        angle = right_hand + last_shift_delta;
      } else {
        angle = n.angle - last_shift_delta;
      }
    }

    if (angle > 360) {
      angle -= 360;
    }
    if (angle < 0) {
      angle += 360;
    }
    if (n.distance == 0) {
      tmp2.push_back(PointData(angle, n.distance, 0, n.stamp));
    } else {
      tmp2.push_back(PointData(angle, n.distance, n.intensity, n.stamp));
    }
  }

  return tmp2;
}

SlTransform::~SlTransform() {}

void SetLidarStatus(LidarStatus status) {
  lidarstatus_ = status;
}

void SetFrameReady(void) {
  is_frame_ready_ = true;
}

void SetLaserScanData(Points2D &src) {
  lidar_scan_data_vec_ = src;
}

Points2D GetLaserScanData(void) {
  return lidar_scan_data_vec_;
}