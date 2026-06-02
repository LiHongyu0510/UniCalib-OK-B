#include <filesystem>
#include <regex>
#include <pcl_conversions/pcl_conversions.h>
#include <arpa/inet.h>
#include <vector>
#include <thread>
#include <cmath>
#include <type_traits>
//nt frame_count = 0;
std::unordered_map<std::string, int> framecounts; 
std::vector<std::thread> threads;
namespace fs = std::filesystem;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;
#pragma pack(push, 1)
#define DEGREE_TO_RADIAN(deg)  ((deg) * M_PI / 180)


struct RSPointXYZIT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(RSPointXYZIT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (double, timestamp, timestamp))

struct RSChannel {
    uint16_t distance;
    uint8_t intensity;
};

struct RSTimestampUTC {
    uint8_t sec[6];
    uint8_t ss[4];
};

struct RSTemperature {
    uint8_t tt[2];
};

struct RSHELIOSMsopHeader {
    uint8_t header[4];
    uint32_t packet_count;
    RSTimestampUTC timestamp;
    uint8_t lidar_type;
    uint8_t lidar_model;
    uint8_t others[22];
};
static_assert(sizeof(RSHELIOSMsopHeader) == 42, "RSHELIOSMsopHeader must be 42 bytes");

struct RSHELIOSMsopBlock {
    uint8_t id[2];
    uint16_t azimuth;
    RSChannel channels[32];
};

struct RSHELIOSMsopPkt {
    RSHELIOSMsopHeader header;
    RSHELIOSMsopBlock blocks[12];
    unsigned int index;
    uint16_t tail;
};

struct RSEthNetV2 {
    uint8_t lidar_ip[4];
    uint8_t dest_ip[4];
    uint8_t mac_addr[6];
    uint16_t msop_port;
    uint16_t reserve_1;
    uint16_t difop_port;
    uint16_t reserve_2;
};

struct RSFOV {
    uint16_t start_angle;
    uint16_t end_angle;
};

struct RSVersionV2 {
    uint8_t top_ver[5];
    uint8_t bottom_ver[5];
    uint8_t bot_soft_ver[5];
    uint8_t motor_firmware_ver[5];
    uint8_t hw_ver[3];
};

struct RSSN {
    uint8_t num[6];
};

struct RSTimeInfo {
    uint8_t sync_mode;
    uint8_t sync_sts;
    RSTimestampUTC timestamp;
};

struct RSStatusV2 {
    uint16_t device_current;
    uint16_t vol_fpga;
    uint16_t vol_12v;
    uint16_t vol_dig_5v4;
    uint16_t vol_sim_5v;
    uint16_t vol_apd;
    uint8_t reserved[12];
};

struct RSDiagnoV2 {
    uint16_t bot_fpga_temperature;
    uint16_t recv_A_temperature;
    uint16_t recv_B_temperature;
    uint16_t main_fpga_temperature;
    uint16_t main_fpga_core_temperature;
    uint16_t real_rpm;
    uint8_t lane_up;
    uint16_t lane_up_cnt;
    uint16_t main_status;
    uint8_t gps_status;
    uint8_t reserved[22];
};

struct RSCalibrationAngle {
    uint8_t sign;
    uint16_t value;
};

struct RSHELIOSDifopPkt {
    uint8_t id[8];
    uint16_t rpm;
    RSEthNetV2 eth;
    RSFOV fov;
    uint8_t reserved1[2];
    uint16_t phase_lock_angle;
    RSVersionV2 version;
    uint8_t reserved2[229];
    RSSN sn;
    uint16_t zero_cali;
    uint8_t return_mode;
    RSTimeInfo time_info;
    RSStatusV2 status;
    uint8_t reserved3[5];
    RSDiagnoV2 diagno;
    uint8_t gprmc[86];
    RSCalibrationAngle vert_angle_cali[32];
    RSCalibrationAngle horiz_angle_cali[32];
    uint8_t reserved4[586];
    uint16_t tail;
};

#pragma pack(pop)
class Trigon
{
public:

  constexpr static int32_t ANGLE_MIN = -9000;
  constexpr static int32_t ANGLE_MAX = 45000;

  Trigon()
  {
    int32_t range = ANGLE_MAX - ANGLE_MIN;
#ifdef DBG
    o_angles_ = (int32_t*)malloc(range * sizeof(int32_t));
#endif
    o_sins_ = (float*)malloc(range * sizeof(float));
    o_coss_ = (float*)malloc(range * sizeof(float));

    for (int32_t i = ANGLE_MIN, j = 0; i < ANGLE_MAX; i++, j++)
    {
      double rad = DEGREE_TO_RADIAN(static_cast<double>(i) * 0.01);

#ifdef DBG
      o_angles_[j] = i;
#endif
      o_sins_[j] = (float)std::sin(rad);
      o_coss_[j] = (float)std::cos(rad);
    }

#ifdef DBG
    angles_ = o_angles_ - ANGLE_MIN;
#endif
    sins_ = o_sins_ - ANGLE_MIN;
    coss_ = o_coss_ - ANGLE_MIN;
  }

  ~Trigon()
  {
    free(o_coss_);
    free(o_sins_);
#ifdef DBG
    free(o_angles_);
#endif
  }

  float sin(int32_t angle)
  {
    if (angle < ANGLE_MIN || angle >= ANGLE_MAX)
    {
      angle = 0;
    }

    return sins_[angle];
  }

  float cos(int32_t angle)
  {
    if (angle < ANGLE_MIN || angle >= ANGLE_MAX)
    {
      angle = 0;
    }

    return coss_[angle];
  }

  void print()
  {
    for (int32_t i = -10; i < 10; i++)
    {
      std::cout << 
#ifdef DBG 
        angles_[i] << "\t" << 
#endif
        sins_[i] << "\t" << coss_[i] << std::endl;
    }
  }

private:
#ifdef DBG 
  int32_t* o_angles_;
  int32_t* angles_;
#endif
  float* o_sins_;
  float* o_coss_;
  float* sins_;
  float* coss_;
};

class SplitStrategy
{
public:
  virtual bool newBlock(int32_t angle) = 0;
  virtual ~SplitStrategy() = default;
};

class SplitStrategyByAngle : public SplitStrategy
{
public:
  SplitStrategyByAngle (int32_t split_angle)
   : split_angle_(split_angle), prev_angle_(split_angle)
  {
  }

  virtual ~SplitStrategyByAngle() = default;

  virtual bool newBlock(int32_t angle)
  {
    if (angle < prev_angle_)
    {
      prev_angle_ -= 36000;
    }

    bool v = ((prev_angle_ < split_angle_) && (split_angle_ <= angle));
#if 0
    if (v) 
    {
      std::cout << prev_angle_ << "\t" << angle << std::endl;
    }
#endif
    prev_angle_ = angle;
    return v;
  }


#ifndef UNIT_TEST
private:
#endif
    const int32_t split_angle_;
    int32_t prev_angle_;
};


class BlockIterator
{
public:

  static const int MAX_BLOCKS_PER_PKT = 12;

  void get(uint16_t blk, int32_t& az_diff, double& ts)
  {
    az_diff = az_diffs[blk];
    ts = tss[blk];
  }

  BlockIterator(const RSHELIOSMsopPkt& pkt, uint16_t blocks_per_pkt, double block_duration, 
      uint16_t block_az_duration, double fov_blind_duration)
    : pkt_(pkt), BLOCKS_PER_PKT(blocks_per_pkt), BLOCK_DURATION(block_duration), 
    BLOCK_AZ_DURATION(block_az_duration), FOV_BLIND_DURATION(fov_blind_duration) 
  {
    uint16_t blk = 0;
    double tss = 0;
    for (; blk < (this->BLOCKS_PER_PKT - 1); blk++)
    {
      double ts_diff = this->BLOCK_DURATION;
      int32_t az_diff = ntohs(this->pkt_.blocks[blk+1].azimuth) - ntohs(this->pkt_.blocks[blk].azimuth);
      if (az_diff < 0) { az_diff += 36000; }

      // Skip FOV blind zone 
      if (az_diff > 100)
      {
        az_diff = this->BLOCK_AZ_DURATION;
        ts_diff = this->FOV_BLIND_DURATION;
      }
      this->az_diffs[blk] = az_diff;
      this->tss[blk] = tss;
      //std::cout << "this->tss["<< blk << "]:" << this->tss[blk] << std::endl;
      tss += ts_diff;
    }

    this->az_diffs[blk] = this->BLOCK_AZ_DURATION;
    // assert(BLOCKS_PER_PKT <= MAX_BLOCKS_PER_PKT);
  }

protected:

  const RSHELIOSMsopPkt& pkt_;
  const uint16_t BLOCKS_PER_PKT;
  const double BLOCK_DURATION;
  const uint16_t BLOCK_AZ_DURATION;
  const double FOV_BLIND_DURATION;
  int32_t az_diffs[MAX_BLOCKS_PER_PKT];
  double tss[MAX_BLOCKS_PER_PKT];
};


class RslidarDecode {
public:
    RslidarDecode(const std::string& msop_file, const std::string& difop_file,fs::path directory_path,fs::path save_path,std::string pcd, const std::string& mode)
        :pkt_count(0), mode(mode), first_pts_time_ms(-1)
        {
        save_pcd = !pcd.empty();
        // 创建文件夹
        if(!save_path.empty()){
            directory_path = save_path;
        }else
        {
            directory_path = directory_path.parent_path().parent_path();
        }
        std::filesystem::create_directories(directory_path);
        name = getParentDirectory(msop_file);

        std::string filename = fs::path(msop_file).filename().string();
        std::string msop_time = filename.substr(11, 13);
        if(save_pcd){
            if (mode == "bev_mode") {
                pcd_directory = directory_path.string() + "/pcd/" + name + "/" + msop_time + "/";
            } else {
                pcd_directory = directory_path.string() + "/pcd/" + name + "/";
            }
            std::filesystem::create_directories(pcd_directory);

        }else
        {
            if (mode == "bev_mode") {
                bin_directory = directory_path.string() + "/bin/" + name + "/" + msop_time + "/";
            } else {
                bin_directory = directory_path.string() + "/bin/" + name + "/";
            }
            std::filesystem::create_directories(bin_directory);
        }
        
        // 判断文件打开是否正常
        infile_msop.open(msop_file, std::ios::binary);
        if (!infile_msop) {
            throw std::runtime_error("Failed to open MSOP file: " + msop_file);
        }
        infile_difop.open(difop_file, std::ios::binary);
        if (!infile_difop) {
            throw std::runtime_error("Failed to open DIFOP file: " + difop_file);
        }
        
        RSHELIOSDifopPkt pkt_difop;
        infile_difop.read(reinterpret_cast<char*>(&pkt_difop), sizeof(RSHELIOSDifopPkt));
        // 解析雷达数据
        parseAngleCalibration(pkt_difop, vert_angles, horiz_angles);
    }

    void start() {
        //parsePointCloud2_thread = std::thread(&RslidarDecode::parsePointCloud2, this);
        //publisher = node->create_publisher<PointCloud2>("rslidar_decode", 10);
        // 发布频率，可根据需求更改
        //timer = node->create_wall_timer(std::chrono::milliseconds(100), [this]() { timer_callback(); });
        // rclcpp::spin(node);
        while(start_falg) {timer_callback();}
    }

private:
    double first_pts_time_ms;
    std::string mode;
    std::ifstream infile_msop;
    std::ifstream infile_difop;
    int pkt_count;
    bool save_pcd;
    std::string pcd_directory;
    std::string bin_directory;
    std::string name;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud;
    pcl::PointCloud<RSPointXYZIT>::Ptr cloud_bev;
    double vert_angles[32];
    double horiz_angles[32];
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    bool firstRunFlag = false;
    bool start_falg = true;
    uint64_t pointNum = 0;
    bool first_packet = false;
    //builtin_interfaces::msg::Time last_timestamp;
    std::shared_ptr<SplitStrategy> split_strategy_;
    /** 低于该径向距离(米)的笛卡尔坐标点丢弃（车体近距）；与原始测距 distanceSection 下限无关。 */
    static constexpr float kFilterMinRadialDistanceM = 0.05f;
    /** 原始通道距离(米)下限；原 0.2m 在标定/比例异常时易全场无点。 */
    static constexpr float kDistanceSectionMinM = 0.05f;
    /** 车尾楔形盲区过滤（基于点反算方位角）；车顶前向/解析调试建议关。 */
    static constexpr bool kApplyEgoWedgeFilter = false;
    Trigon trigon_;
    //std::thread parsePointCloud2_thread;
    #define SIN(angle) this->trigon_.sin(angle)
    #define COS(angle) this->trigon_.cos(angle)
    std::string getParentDirectory(const std::string& filePath) {
        fs::path p(filePath);
        return p.parent_path().filename().string();
    }

    // 用于解析dfop，按照速腾文档中的方式计算
    void parseAngleCalibration(const RSHELIOSDifopPkt& pkt_difop, double vert_angles[], double horiz_angles[]) {
        for (int i = 0; i < 32; ++i) {
            uint8_t sign = pkt_difop.horiz_angle_cali[i].sign;
            uint16_t value = ntohs(pkt_difop.horiz_angle_cali[i].value);
            int16_t angle = (sign == 0x00) ? value : -value;
            horiz_angles[i] = static_cast<double>(angle);//* 0.01;
        }

        for (int i = 0; i < 32; ++i) {
            uint8_t sign = pkt_difop.vert_angle_cali[i].sign;
            uint16_t value = ntohs(pkt_difop.vert_angle_cali[i].value);
            int16_t angle = (sign == 0x00) ? value : -value;
            vert_angles[i] = static_cast<double>(angle);//* 0.01;
        }
    }
    // 获取雷达时间戳
    uint64_t getTimestamp(const RSTimestampUTC& tsUtc) {
        uint64_t sec = 0;
        for (int i = 0; i < 6; i++) {
            sec <<= 8;
            sec += tsUtc.sec[i];
        }

        uint64_t us = 0;
        for (int i = 0; i < 4; i++) {
            us <<= 8;
            us += tsUtc.ss[i];
        }

        return (sec * 1000000 + us);
    }
    // 距离范围
    bool distanceSection(float distance){
        return (kDistanceSectionMinM <= distance) && (distance <= 200.0f);
    }
    // 角度范围
    bool azimuthSection(int angle)
    {
        int start = 0 * 100;
        int end = 360 * 100;

        int full_round = (start == 0) && (end == 36000);
        int start_ = start % 36000;
        int end_ = end % 36000;
        int cross_zero = (start_ > end_);
        if (full_round)
        {
            return true;
        }
        if (cross_zero)
        {
            return (angle >= start_) || (angle < end_);
        }else{
            return(angle >= start_) && (angle < end_);
        }
    }
    // 
    bool parsePointCloud2() {
        RSHELIOSMsopPkt pkt;
        uint64_t pointNum_value = 0;
        std::array<uint8_t, 2> expected_id = {0xFF, 0xEE};
        first_packet = true;
        float CHAN_AZIS[128];
        double CHAN_TSS[128];
        double pkt_ts;
        double chan_ts;
        constexpr float blk_ts = 55.56f;
        double BLOCK_DURATION = blk_ts / 1000000;
        bool debug_= false;
        static builtin_interfaces::msg::Time first_timestamp;
        split_strategy_ = std::make_shared<SplitStrategyByAngle>(0);
        constexpr float firing_tss[] = {
            0.00f,  1.57f,  3.15f,  4.72f,  6.30f,  7.87f,  9.45f, 11.36f,
            13.26f, 15.17f, 17.08f, 18.99f, 20.56f, 22.14f, 23.71f, 25.29f,
            26.53f, 29.01f, 27.77f, 30.25f, 31.49f, 33.98f, 32.73f, 35.22f,
            36.46f, 37.70f, 38.94f, 40.18f, 41.42f, 42.67f, 43.91f, 45.15f
        };

        for (uint16_t i = 0; i < sizeof(firing_tss) / sizeof(firing_tss[0]); i++) {
            CHAN_AZIS[i] = firing_tss[i] / blk_ts;
            CHAN_TSS[i] = (double)firing_tss[i] / 1000000;
        }

        uint64_t timestamp_ms;
        while (infile_msop.read(reinterpret_cast<char*>(&pkt), sizeof(RSHELIOSMsopPkt))) {
            if(firstRunFlag)
                pkt_count++;
            // 遍历12个块
            BlockIterator iter(pkt, 12, BLOCK_DURATION, 20, 0.0);
            for (size_t blocks_i = 0; blocks_i < std::size(pkt.blocks); ++blocks_i) {
                //auto& block = pkt.blocks[blocks_i];
                const RSHELIOSMsopBlock& block = pkt.blocks[blocks_i];
                // 判断block——id是否正确
                if (memcmp(expected_id.data(), block.id, 2) != 0) {
                    RCLCPP_INFO(rclcpp::get_logger("rslidar"), "block error");
                    break;
                }
                // 角度值
                float azimuth = static_cast<float>(ntohs(block.azimuth)) / 100.0f;
                int32_t block_az = ntohs(block.azimuth);
                int azimuth_int = std::floor(azimuth);
                int32_t az_diff = 0;
                double block_ts_off;
                double block_ts;
                iter.get(blocks_i, az_diff, block_ts_off);
                // 取第一个点的时间戳
                if(azimuth_int == 0){
                    firstRunFlag = true;
                }

                if(firstRunFlag)
                {
                    if (first_packet) {
                    first_packet = false;
                    }
                    // 处理数据
                    pkt_ts = getTimestamp(pkt.header.timestamp) * 1e-6;
                    //std::cout << std::fixed << std::setprecision(std::numeric_limits<double>::digits10) << std::endl;
                    block_ts = pkt_ts + block_ts_off;
                    if (split_strategy_->newBlock(block_az)){
                        if (mode == "bev_mode") {
                            cloud_bev->width = cloud_bev->points.size();
                            cloud_bev->height = 1;
                            cloud_bev->is_dense = false;
                        } else {
                            pcl_cloud->width = pcl_cloud->points.size();
                            pcl_cloud->height = 1;
                            pcl_cloud->is_dense = false;
                        }
                        sendPointCloud2();
                        first_timestamp.sec = (uint32_t)floor(block_ts);
                        first_timestamp.nanosec = (uint32_t)round((block_ts - first_timestamp.sec) * 1e9);
                        pointNum_value = 0;
                        first_pts_time_ms = -1;
                    }
                    // 遍历32个通道
                    for (size_t chan = 0; chan < std::size(block.channels); ++chan) {
                        // 计算水平角度、垂直角度、距离值，可参考速腾源码
                        double point_timestamp = block_ts + CHAN_TSS[chan];
                        //std::cout << "point_timestamp:" << point_timestamp << " block_ts:" << block_ts << "CHAN_TSS[chan]:" << CHAN_TSS[chan] << std::endl;
                        const RSChannel& channel = block.channels[chan]; 
                        float distance = ntohs(channel.distance) * 0.0025f;
                        double angle_horiz = (block_az + (int32_t)((float)az_diff * CHAN_AZIS[chan]));
                        double angle_vert = vert_angles[chan];
                        double angle_horiz_final = (angle_horiz + horiz_angles[chan]);
                        pointNum ++;
                        // 处理无效点
                        if (distanceSection(distance) &&
                            azimuthSection(static_cast<int>(std::lround(angle_horiz_final)))) {
                            
                            float x = distance * COS(angle_vert) * COS(angle_horiz_final) + 0.03498f * COS(angle_horiz);
                            float y = -distance * COS(angle_vert) * SIN(angle_horiz_final) - 0.03498f * SIN(angle_horiz);
                            float z = distance * SIN(angle_vert);
                            float intensity = block.channels[chan].intensity;
                            if (std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(intensity)){
                                    continue;
                            }
                            double azimuth_filter = atan2(y, x) * 57.2958;
                            double pitch = atan2(z, sqrt(x * x + y * y)) * 57.2958;
                            double distance_filter = sqrt(x * x + y * y + z * z);

                            bool wedge_drop = false;
                            if (kApplyEgoWedgeFilter) {
                                bool in_angle_range_first =
                                    (azimuth_filter >= 145.0 && azimuth_filter <= 180.0) &&
                                    (pitch >= -90.0 && pitch <= -0.1) &&
                                    (distance_filter >= 0.0 && distance_filter <= 3.5);
                                bool in_angle_range_second =
                                    (azimuth_filter >= -180 && azimuth_filter <= -145.0) &&
                                    (pitch >= -90.0 && pitch <= -0.1) &&
                                    (distance_filter >= 0.0 && distance_filter <= 3.5);
                                wedge_drop = in_angle_range_first || in_angle_range_second;
                            }
                            if (wedge_drop || distance_filter < kFilterMinRadialDistanceM) {
                                continue;
                            }
                            if (first_pts_time_ms == -1) first_pts_time_ms = point_timestamp * 1000; //ms
                            if (mode == "bev_mode") {
                                RSPointXYZIT pt;
                                pt.x = x;
                                pt.y = y;
                                pt.z = z;
                                pt.intensity = intensity;
                                pt.timestamp = point_timestamp * 1000; //ms
                                cloud_bev->points.push_back(pt);
                            } else {
                                
                                pcl::PointXYZI point;
                                point.x = x;
                                point.y = y;
                                point.z = z;
                                point.intensity = intensity;
                                pcl_cloud->points.push_back(point);
                            }
                            pointNum_value ++;                  
                                                       
                        }
                    }
                
                }
            }

        }

        return false;
    }
    int32_t horizAdjust(uint16_t chan, int32_t horiz)
  {
    return (horiz + horiz_angles[chan]);
  }
    void iterGet(uint16_t blk, int32_t& az_diff, double& ts)
    {
        double tss[12];
        int32_t az_diffs[12];
        az_diff = az_diffs[blk];
        ts = tss[blk];
    }
    // 保存bin格式
    template<typename PointT>
    int saveBINfile(const pcl::PointCloud<PointT>& cloud, const std::string& file_path)
    {
        FILE* stream = fopen(file_path.c_str(), "wb");
        if (stream == nullptr) {
            return -1; // 文件打开失败
        }
        for (int i = 0; i < cloud.points.size(); i++)
        {
            fwrite(&cloud.points[i].x, sizeof(float), 1, stream);
            fwrite(&cloud.points[i].y, sizeof(float), 1, stream);
            fwrite(&cloud.points[i].z, sizeof(float), 1, stream);
            fwrite(&cloud.points[i].intensity, sizeof(float), 1, stream);

            if constexpr (std::is_same_v<PointT, RSPointXYZIT>) {
                fwrite(&cloud.points[i].timestamp, sizeof(double), 1, stream);
            }
        }
        fclose(stream);
        return 0;
    }
    void sendPointCloud2()
    {
        auto cloud = std::make_shared<PointCloud2>();
        const bool bev = (mode == "bev_mode");
        if (bev) {
            pcl::toROSMsg(*cloud_bev, *cloud);
        } else {
            pcl::toROSMsg(*pcl_cloud, *cloud);
        }
        const uint64_t stamp_ms =
            (first_pts_time_ms >= 0.0) ? static_cast<uint64_t>(first_pts_time_ms + 0.5) : 0ULL;

        //publisher->publish(*cloud);
        if (pointNum == 57600) {
            const size_t n = bev ? cloud_bev->points.size() : pcl_cloud->points.size();
            if (n == 0) {
                RCLCPP_WARN(
                    rclcpp::get_logger("rslidar"),
                    "Skip save: pointNum=%zu but point cloud empty (bev=%d). Check filters / MSOP.",
                    static_cast<size_t>(pointNum), bev ? 1 : 0);
            } else if (save_pcd) {
                std::string pcd_filename =
                    pcd_directory + std::to_string(framecounts[name]) + "_" + std::to_string(stamp_ms) + ".pcd";
                if (bev) {
                    pcl::io::savePCDFileBinary(pcd_filename, *cloud_bev);
                } else {
                    pcl::io::savePCDFileASCII(pcd_filename, *pcl_cloud);
                }
            } else {
                std::string bin_filename =
                    bin_directory + std::to_string(framecounts[name]) + "_" + std::to_string(stamp_ms) + ".bin";
                if (bev) {
                    if (saveBINfile(*cloud_bev, bin_filename) != 0) {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("rslidar"), "Failed to save Bin file in %s", bin_filename.c_str());
                    }
                } else {
                    if (saveBINfile(*pcl_cloud, bin_filename) != 0) {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("rslidar"), "Failed to save Bin file in %s", bin_filename.c_str());
                    }
                }
            }
        }
        framecounts[name]++;
        pkt_count = 0;
        pcl_cloud->clear();
        cloud_bev->clear();
        pointNum = 0;
        first_packet = true;
        firstRunFlag = false;
    }

    void timer_callback() {
        pcl_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        cloud_bev = pcl::make_shared<pcl::PointCloud<RSPointXYZIT>>();
        if (parsePointCloud2() == false){
            start_falg = false;
        }
    }

};

#pragma pack(push, 1)
struct LivoxPointXyzrtlt {
  float x;            /**< X axis, Unit:m */
  float y;            /**< Y axis, Unit:m */
  float z;            /**< Z axis, Unit:m */
  float intensity; /**< Reflectivity   */
  uint8_t tag;        /**< Livox point tag   */
  uint8_t line;       /**< Laser line id     */
  double timestamp;   /**< Timestamp of point*/
};
#pragma pack(pop)


POINT_CLOUD_REGISTER_POINT_STRUCT(LivoxPointXyzrtlt,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint8_t, tag, tag)
    (uint8_t, line, line)
    (double, timestamp, timestamp)
)

#pragma pack(1)

struct PointXYZIT {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t intensity;
    uint8_t tag;
};

struct PointXYZITO {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t intensity;
    uint8_t tag;
    uint16_t offset;
};

#pragma pack()

struct LivoxPacketXYZIT {
    uint64_t timestamp;
    uint16_t points_num;
    std::vector<PointXYZITO> points;
};

struct HesaiPacketXYZIT {
    uint64_t timestamp;
    uint32_t points_num;
    std::vector<PointXYZITO> points;
};

class LivoxlidarDecode {
public:
    LivoxlidarDecode(const std::string& filename,fs::path directory_path,fs::path save_path,std::string pcd, const std::string& mode)
    : mode(mode)
    {
        save_pcd = !pcd.empty();
        if(!save_path.empty()){
            directory_path = save_path;
        }else
        {
            directory_path = directory_path.parent_path();
        }
        
        std::filesystem::create_directories(directory_path);
        name = getParentDirectory(filename);
        
        std::string output_file_name = name;

        size_t pos = output_file_name.find("_airy"); //erase "_airy"
        if (pos != std::string::npos) {
            output_file_name.erase(pos, 5); 
        }

        std::string basename = fs::path(filename).filename().string();
        std::string package_time = basename.substr(7, 13);
        if(save_pcd){
            if (mode == "bev_mode") {
                pcd_directory = directory_path.string() + "/pcd/" + output_file_name + "/" + package_time + "/";
            } else {
                pcd_directory = directory_path.string() + "/pcd/" + output_file_name + "/";
            }
            std::filesystem::create_directories(pcd_directory);
        }else
        {
            if (mode == "bev_mode") {
                bin_directory = directory_path.string() + "/bin/" + output_file_name + "/" + package_time + "/";
            } else {
                bin_directory = directory_path.string() + "/bin/" + output_file_name + "/";
            }
            std::filesystem::create_directories(bin_directory);
        }
        

        infile.open(filename, std::ios::binary);
        if (!infile) {
            throw std::runtime_error("Failed to open file: " + filename);
        }
    }

    void start() {
        //publisher = node->create_publisher<PointCloud2>("livoxLidar_decode", 10);
        while (start_falg) {timer_callback();}
    }

private:
    std::string mode;
    std::ifstream infile;
    bool save_pcd;
    std::string pcd_directory;
    std::string bin_directory;
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    pcl::PCLPointCloud2 pcl_cloud;
    bool start_falg = true;
    std::string name;
    std::shared_ptr<PointCloud2> cloud = std::make_shared<PointCloud2>();
    std::string getParentDirectory(const std::string& filePath) {
        fs::path p(filePath);
        return p.parent_path().filename().string();
    }
    void InitPointcloud2MsgHeader(std::shared_ptr<PointCloud2>& cloud) {
        cloud->header.frame_id = "";
        cloud->height = 1;
        cloud->width = 0;
        cloud->fields.resize(7);

        cloud->fields[0].name = "x";
        cloud->fields[0].offset = 0;
        cloud->fields[0].datatype = PointField::FLOAT32;
        cloud->fields[0].count = 1;

        cloud->fields[1].name = "y";
        cloud->fields[1].offset = 4;
        cloud->fields[1].datatype = PointField::FLOAT32;
        cloud->fields[1].count = 1;

        cloud->fields[2].name = "z";
        cloud->fields[2].offset = 8;
        cloud->fields[2].datatype = PointField::FLOAT32;
        cloud->fields[2].count = 1;

        cloud->fields[3].name = "intensity";
        cloud->fields[3].offset = 12;
        cloud->fields[3].datatype = PointField::FLOAT32;
        cloud->fields[3].count = 1;

        cloud->fields[4].name = "tag";
        cloud->fields[4].offset = 16;
        cloud->fields[4].datatype = PointField::UINT8;
        cloud->fields[4].count = 1;

        cloud->fields[5].name = "line";
        cloud->fields[5].offset = 17;
        cloud->fields[5].datatype = PointField::UINT8;
        cloud->fields[5].count = 1;

        cloud->fields[6].name = "timestamp";
        cloud->fields[6].offset = 18;
        cloud->fields[6].datatype = PointField::FLOAT64;
        cloud->fields[6].count = 1;

        cloud->point_step = sizeof(LivoxPointXyzrtlt);
        cloud->is_bigendian = false;
        cloud->is_dense = true;
    }


    bool livox_parsePointCloud2(std::ifstream& file, std::shared_ptr<PointCloud2>& cloud, uint64_t& timestamp) {
        if (!file.is_open()) {
            std::cerr << "File not open!" << std::endl;
            return false;
        }

        auto pkt_prt = std::make_shared<LivoxPacketXYZIT>();
        uint16_t data_id = 0;

        uint64_t curr_point_timstamp_us = 0;
        while (file.read(reinterpret_cast<char*>(&data_id), sizeof(data_id))) {
            InitPointcloud2MsgHeader(cloud);
            //std::cout << "data_id: 0x" << std::hex << data_id << std::endl;
            file.read(reinterpret_cast<char*>(&pkt_prt->timestamp), sizeof(LivoxPacketXYZIT::timestamp));
            file.read(reinterpret_cast<char*>(&pkt_prt->points_num), sizeof(LivoxPacketXYZIT::points_num));
            pkt_prt->points.resize(pkt_prt->points_num);
            std::vector<LivoxPointXyzrtlt> livox_points(pkt_prt->points_num);
            uint64_t last_point_timestamp_us = pkt_prt->timestamp / 1000;

            switch (data_id)
            {
            case 0xddbb:
                //std::cout << "data_id:" << data_id << std::endl;
                for (size_t i = 0; i < pkt_prt->points_num; ++i) {
                    file.read(reinterpret_cast<char*>(&pkt_prt->points[i]), sizeof(PointXYZITO));

                    LivoxPointXyzrtlt& point = livox_points[i];
                    curr_point_timstamp_us = last_point_timestamp_us + (pkt_prt->points[i].offset);
                    point.x = static_cast<float>(pkt_prt->points[i].x) / 100.0;
                    point.y = static_cast<float>(pkt_prt->points[i].y) / 100.0;
                    point.z = static_cast<float>(pkt_prt->points[i].z) / 100.0;
                    point.intensity = static_cast<float>(pkt_prt->points[i].intensity);
                    point.tag = pkt_prt->points[i].tag;
                    point.line = 1;
                    point.timestamp = static_cast<double>(curr_point_timstamp_us) / 1000.0; // ms
                    //std::cout << std::dec << "curr_point_timstamp_us:" << curr_point_timstamp_us  <<   std::endl;  
                    last_point_timestamp_us = curr_point_timstamp_us;            
                }
                break;

            case 0xccaa:
                //std::cout << "data_id:" << data_id << std::endl;
                for (size_t i = 0; i < pkt_prt->points_num; ++i) {
                    file.read(reinterpret_cast<char*>(&pkt_prt->points[i]), sizeof(PointXYZIT));

                    LivoxPointXyzrtlt& point = livox_points[i];
                    point.x = static_cast<float>(pkt_prt->points[i].x) / 100.0;
                    point.y = static_cast<float>(pkt_prt->points[i].y) / 100.0;
                    point.z = static_cast<float>(pkt_prt->points[i].z) / 100.0;
                    point.intensity = static_cast<float>(pkt_prt->points[i].intensity);
                    point.tag = pkt_prt->points[i].tag;
                    point.line = 1;
                    point.timestamp = 0;           
                }
                break;
            
            default:
                std::cout << "invalid data_id:" << data_id << std::endl;
                break;
            }
            

            if (!pkt_prt->points.empty()) {
                timestamp = pkt_prt->timestamp;
            }
            cloud->width = pkt_prt->points_num;
            cloud->row_step = cloud->width * cloud->point_step;
            cloud->header.stamp = rclcpp::Time(timestamp);
            cloud->data.resize(pkt_prt->points_num * sizeof(LivoxPointXyzrtlt));
            memcpy(cloud->data.data(), livox_points.data(), pkt_prt->points_num * sizeof(LivoxPointXyzrtlt));
            //std::cout << "timestamp:" << pkt_prt->timestamp << std::endl;
            return true;
        }

        return false;
    }

    bool hesai_parsePointCloud2(std::ifstream& file, std::shared_ptr<PointCloud2>& cloud, uint64_t& timestamp) {
        if (!file.is_open()) {
            std::cerr << "File not open!" << std::endl;
            return false;
        }

        auto pkt_prt = std::make_shared<HesaiPacketXYZIT>();
        //std::cout << "sizeof(HesaiPacketXYZIT):" << sizeof(HesaiPacketXYZIT) << "sizeof(PointXYZITO):" << sizeof(PointXYZITO) << std::endl;
        uint16_t data_id = 0;
        while (file.read(reinterpret_cast<char*>(&data_id), sizeof(data_id))) {
            InitPointcloud2MsgHeader(cloud);
            file.read(reinterpret_cast<char*>(&pkt_prt->timestamp), sizeof(HesaiPacketXYZIT::timestamp));
            file.read(reinterpret_cast<char*>(&pkt_prt->points_num), sizeof(HesaiPacketXYZIT::points_num));
            pkt_prt->points.resize(pkt_prt->points_num);
            std::vector<LivoxPointXyzrtlt> livox_points(pkt_prt->points_num);
            //std::cout << "pkt_prt->timestamp:" << pkt_prt->timestamp << "pkt_prt->points_num:" << pkt_prt->points_num << std::endl;
            uint64_t last_point_timestamp_us = pkt_prt->timestamp / 1000;
            //std::cout << std::dec <<  "pkt_prt->timestamp :"  << pkt_prt->timestamp << std::endl;
            uint64_t curr_point_timstamp_us = 0;

            switch (data_id)
            {
            case 0xddbb:
                //std::cout << "data_id:0x"  << std::hex  << data_id << std::endl;
                for (size_t i = 0; i < pkt_prt->points_num; ++i) {
                    file.read(reinterpret_cast<char*>(&pkt_prt->points[i]), sizeof(PointXYZITO));

                    LivoxPointXyzrtlt& point = livox_points[i];
                    curr_point_timstamp_us = last_point_timestamp_us + (pkt_prt->points[i].offset);
                    point.x = static_cast<float>(pkt_prt->points[i].x) / 100.0;
                    point.y = static_cast<float>(pkt_prt->points[i].y) / 100.0;
                    point.z = static_cast<float>(pkt_prt->points[i].z) / 100.0;
                    point.intensity = static_cast<float>(pkt_prt->points[i].intensity);
                    point.tag = pkt_prt->points[i].tag;
                    point.line = 1;
                    uint64_t seconds = curr_point_timstamp_us / 1000000;
                    uint64_t us_remainder = curr_point_timstamp_us % 1000000;
                    point.timestamp = (static_cast<double>(seconds) + static_cast<double>(us_remainder) / 1000000.0) * 1000.0; // ms
                    //std::cout << "point.timestamp: " << std::fixed << std::setprecision(6) << point.timestamp << std::endl;
                    last_point_timestamp_us = curr_point_timstamp_us;            
                }
                break;

            case 0xccaa:
                //std::cout << "data_id:0x"  << std::hex  << data_id << std::endl;
                for (size_t i = 0; i < pkt_prt->points_num; ++i) {
                    file.read(reinterpret_cast<char*>(&pkt_prt->points[i]), sizeof(PointXYZIT));

                    LivoxPointXyzrtlt& point = livox_points[i];
                    point.x = static_cast<float>(pkt_prt->points[i].x) / 100.0;
                    point.y = static_cast<float>(pkt_prt->points[i].y) / 100.0;
                    point.z = static_cast<float>(pkt_prt->points[i].z) / 100.0;
                    point.intensity = static_cast<float>(pkt_prt->points[i].intensity);
                    point.tag = pkt_prt->points[i].tag;
                    point.line = 1;
                    point.timestamp = 0;                
                }
                break;
            
            default:
                std::cout << "invalid data_id:" << data_id << std::endl;
                break;
            }

            if (!pkt_prt->points.empty()) {
                timestamp = pkt_prt->timestamp;
            }
            cloud->width = pkt_prt->points_num;
            cloud->row_step = cloud->width * cloud->point_step;
            cloud->header.stamp = rclcpp::Time(timestamp);
            cloud->data.resize(pkt_prt->points_num * sizeof(LivoxPointXyzrtlt));
            memcpy(cloud->data.data(), livox_points.data(), pkt_prt->points_num * sizeof(LivoxPointXyzrtlt));
            return true;
        }

        return false;
    }
    int saveBINfile(pcl::PointCloud<pcl::PointXYZI>& cloud, std::string file_path)
    {
        FILE* stream = fopen(file_path.c_str(), "wb");
        for (int i = 0; i < cloud.points.size(); i++)
        {
            fwrite((char*)(&(cloud.points[i].x)), sizeof(float), 1, stream);
            fwrite((char*)(&(cloud.points[i].y)), sizeof(float), 1, stream);
            fwrite((char*)(&(cloud.points[i].z)), sizeof(float), 1, stream);
            fwrite((char*)(&(cloud.points[i].intensity)), sizeof(float), 1, stream);
        }
        fclose(stream);
        return 0;
    }

    int SavePCDfile(const std::string& file_path, 
                 const pcl::PointCloud<LivoxPointXyzrtlt>& cloud_specific) {
    if (cloud_specific.empty()) {
        std::cerr << "错误：点云 cloud_specific 为空！" << std::endl;
        return -1;
    }

    // 打开文件流
    std::ofstream ofs(file_path, std::ios::out);
    if (!ofs.is_open()) {
        std::cerr << "错误：无法打开文件 " << file_path << std::endl;
        return -1;
    }

    // 1. 写入 PCD 文件头（严格遵循格式规范）
    ofs << "# .PCD v0.7 - Point Cloud Data file format\n";
    ofs << "VERSION 0.7\n";
    ofs << "FIELDS x y z intensity tag line timestamp\n";
    ofs << "SIZE 4 4 4 4 1 1 8\n";
    ofs << "TYPE F F F F U U F\n";
    ofs << "COUNT 1 1 1 1 1 1 1\n";
    ofs << "WIDTH " << cloud_specific.width << "\n";
    ofs << "HEIGHT " << cloud_specific.height << "\n";
    ofs << "VIEWPOINT 0 0 0 1 0 0 0\n";
    ofs << "POINTS " << cloud_specific.size() << "\n";
    ofs << "DATA ascii\n";

    ofs << std::fixed << std::setprecision(6);  // 固定小数位，确保微秒级显示
    for (const auto& point : cloud_specific) {
        ofs << point.x << " "
            << point.y << " "
            << point.z << " "
            << point.intensity  << " "
            << static_cast<int>(point.tag) << " "
            << static_cast<int>(point.line) << " "
            << point.timestamp << "\n";
    }

    ofs.close();
    return 0;
}

    void timer_callback() {
        uint64_t frame_timestamp = 0;
        bool parseRet;
        if((name.find("hesai") != std::string::npos) || name.find("airy") != std::string::npos)
        {
            parseRet = hesai_parsePointCloud2(infile, cloud, frame_timestamp);
            
        }
        
        else{
            parseRet = livox_parsePointCloud2(infile, cloud, frame_timestamp);
        } 
        if (parseRet) {
            pcl_conversions::toPCL(*cloud, pcl_cloud);
            pcl::PointCloud<pcl::PointXYZI> point_cloud;
            pcl::PointCloud<LivoxPointXyzrtlt> cloud_specific;
            pcl::fromPCLPointCloud2(pcl_cloud, cloud_specific);
            pcl::fromPCLPointCloud2(pcl_cloud, point_cloud);
            cloud->header.frame_id = "blind_filling_lidar";
            //publisher->publish(*cloud);
            //verifyPclCloudTimestamps(pcl_cloud);
            if (save_pcd) {
                std::string pcd_filename = pcd_directory + std::to_string({framecounts[name]}) + "_" + std::to_string(frame_timestamp / 1000000) + ".pcd";
                if (mode == "bev_mode") {
                    pcl::io::savePCDFile(pcd_filename, pcl_cloud, Eigen::Vector4f::Zero(),
                             Eigen::Quaternionf::Identity(), true);
                } else {
                    SavePCDfile(pcd_filename,cloud_specific);
                }
                //pcl::io::savePCDFile(pcd_filename, pcl_cloud);
                //RCLCPP_INFO(rclcpp::get_logger("LivoxlidarDecode"), "Pcd saved in %s", pcd_filename.c_str());
            }else
            {
                std::string bin_filename = bin_directory + std::to_string(framecounts[name]) + "_" + std::to_string(frame_timestamp / 1000000) + ".bin";
                if (saveBINfile(point_cloud, bin_filename) == 0) {
                //    RCLCPP_INFO(rclcpp::get_logger("LivoxlidarDecode"), "Bin saved in %s", bin_filename.c_str());
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("LivoxlidarDecode"), "Failed to save Bin file in %s", bin_filename.c_str());
                }
            }
            
            framecounts[name]++; 
            // RCLCPP_INFO(rclcpp::get_logger("LivoxlidarDecode"), "timestamp_ms: %s", std::to_string(frame_timestamp).c_str());
        } else {
            start_falg = false;
            rclcpp::shutdown();
        }
    }
};

void classifyFiles(const fs::path& directory_path, 
                   std::map<std::string, std::vector<std::pair<std::string, std::string>>>& rslidar_files,
                   std::map<std::string, std::vector<std::string>>& livox_files) {
    
    // 使用正则表达式匹配文件
    std::regex msop_pattern(R"(lidarMSOP(\d+)_(\d{6,8}_\d{6}))");
    std::regex difop_pattern(R"(lidarDIFOP(\d+)_(\d{6,8}_\d{6}))");
    std::regex livox_pattern(R"(lidar([0-3]))");
    std::smatch match;

    std::vector<fs::path> files;

    for (const auto& entry : fs::recursive_directory_iterator(directory_path)) {
        if (entry.is_regular_file()) {
            files.push_back(entry.path());
        }
    }
    // 按升序排序
    std::sort(files.begin(), files.end());

    for (const auto& file_path : files) {
        std::string file_name = file_path.filename().string();
        std::string rslidar_key;

        if (std::regex_search(file_name, match, livox_pattern)) {
            std::string livox_key = "lidar" + match[1].str();
            livox_files[livox_key].push_back(file_path.string());
        } else if (std::regex_search(file_name, match, msop_pattern)) {
            rslidar_key = match[1].str() + "_" + match[2].str();
            rslidar_files[rslidar_key].emplace_back(file_path.string(), "");
        } else if (std::regex_search(file_name, match, difop_pattern)) {
            rslidar_key = match[1].str() + "_" + match[2].str();
            rslidar_files[rslidar_key].emplace_back("", file_path.string());
        }
    }
}

void signalHandler(int signum) {
    rclcpp::shutdown();
    exit(signum);
}

void process_rslidar(const std::map<std::string, std::vector<std::pair<std::string, std::string>>>& rslidar_files, 
                    const fs::path& directory_path, const fs::path& save_path, const fs::path& pcd, const std::string& mode)
{
// 遍历分类后的速腾文件
    for (const auto& [rslidar_key, files] : rslidar_files) {
        std::string msop_files, difop_files;
        for (const auto& [msop_file, difop_file] : files) {
            if (!msop_file.empty()) msop_files += msop_file;
            if (!difop_file.empty()) difop_files += difop_file;
        }
        std::cout << "rslidar_key: " << rslidar_key << ", msop_files: " << msop_files << ", difop_files: " << difop_files << std::endl;

        try {
            // 初始化解释速腾雷达类
            RslidarDecode rslidar_decoder(msop_files, difop_files,directory_path,save_path,pcd, mode);
            rslidar_decoder.start();
        } catch (const std::exception& e) {
            std::cerr << "Error processing RSLiDAR files (" << msop_files << ", " << difop_files << "): " << e.what() << std::endl;
        }
    }
}

void decodeThraed(const std::pair<std::string, std::vector<std::string>>& pair, const fs::path& directory_path, 
                    const fs::path& save_path, const fs::path& pcd, const std::string& mode)
{
    std::cout << pair.first << " file:" << std::endl;
    for(const auto& livox_file : pair.second)
    {
        std::cout << livox_file << std::endl;
        std::ifstream file(livox_file); 
        if (!file) {
            std::cerr << "错误: 文件 " << livox_file << " 无法打开!" << std::endl;
            continue; // 文件无法打开，跳过当前文件
        }

        // 检查文件是否为空
        file.seekg(0, std::ios::end); 
        if (file.tellg() == 0) { 
            std::cout << "提示: 文件 " << livox_file << " 是空的!" << std::endl;
        } 
        file.close(); 
        try {
            LivoxlidarDecode livox_decoder(livox_file, directory_path, save_path, pcd, mode);
            livox_decoder.start();
        } catch (const std::exception& e) {
            std::cerr << "Error processing Livox file " << livox_file << ": " << e.what() << std::endl;
        }            
    }
}

void process_livox(const std::map<std::string, std::vector<std::string>>& livox_files, const fs::path& directory_path, 
                    const fs::path& save_path, const fs::path& pcd, const std::string& mode)
{
    for(const auto& pair : livox_files)
    {
        threads.push_back(std::thread(decodeThraed, pair, directory_path, save_path, pcd, mode));
    }

    for (auto& t : threads) {
    if (t.joinable()) {
        t.join();  // 确保每个线程执行完后才继续
        }
    }
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage:\n";
        std::cerr << "\tDecode lidar : ros2 run lidar_decode lidar_decode <directory_path>\n";
        std::cerr << "\tSave pcd : ros2 run lidar_decode lidar_decode <directory_path> [pcd]\n";
        return -1;
    }
    // 捕获ctrl+c信号
    std::signal(SIGINT, signalHandler);
    // 接收命令行参数
    /*
    存在问题：第三个参数为保存pcd，第二个参数省略时会有问题，需要优化
    例：ros2 run lidar_decode lidar_decode ../data/record/lidar/ pcd
    */
    fs::path directory_path = argv[1];
    fs::path save_path = (argc > 2) ? argv[2] : "";
    fs::path pcd = (argc > 3) ? argv[3] : "";
    std::string mode = (argc > 4) ? argv[4] : "";

    // 存放分类后的速腾和大疆雷达文件底子
    std::map<std::string, std::vector<std::pair<std::string, std::string>>> rslidar_files;
    std::map<std::string, std::vector<std::string>> livox_files;

    auto sTime = std::chrono::system_clock::now();
    // 转换为时间戳（秒为单位）
    auto start_time = std::chrono::system_clock::to_time_t(sTime);
    // 文件分类
    classifyFiles(directory_path, rslidar_files, livox_files);
    
    std::thread rslidar_thread(process_rslidar, rslidar_files, directory_path,
                               save_path, pcd, mode);

    process_livox(livox_files, directory_path, save_path, pcd, mode);
    //std::thread livox_thread(process_livox, livox_files, directory_path,
    //                         save_path, pcd);

    rslidar_thread.join();
    auto cTime = std::chrono::system_clock::now();
    auto end_time = std::chrono::system_clock::to_time_t(cTime);
    auto final_time = end_time - start_time;
    std::cout << "done!\n解析时长:" << final_time << "s" << std::endl;
    rclcpp::shutdown();
    return 0;
}

