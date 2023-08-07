#include "Arduino.h"
#include "ld14p.h"

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

Points2D tmp_lidar_scan_data_vec_;
Points2D lidar_scan_data_vec_;

void Parse(byte b[], int le) {
  // read the incoming byte:
  for (int iii = 0; iii < le; iii++) {

    uint8_t ret = AnalysisDataPacket(b[iii]);
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
      //tmp2.push_back(PointData(angle, n.distance, 0, n.stamp));
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
