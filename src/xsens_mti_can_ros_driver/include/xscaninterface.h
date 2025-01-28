#ifndef XSCANINTERFACE_H
#define XSCANINTERFACE_H

#include <ros/ros.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <map>
#include "xsens_parser.h"
#include "xsens_time_handler.h"

class PacketCallback;

class XsCanInterface
{
public:
    XsCanInterface();
    ~XsCanInterface();
    bool initialize();
    void spinFor();
	void registerPublishers(ros::NodeHandle &node);
    void closeInterface();

private:
    int socket_;
    XsDataPacket packet_;
    XsensTimeHandler m_time_handler_;
    uint32_t m_start_frame_id_ = 0xFFFFFFFF; // initialize with an invalid frame ID.
    uint32_t last_frame_id_ = 0xFFFFFFFF; // initialize with an invalid frame ID.
    std::string can_interface_ = "can0";
    std::list<PacketCallback *> m_callbacks_;

    void updateMinFrameId(uint32_t frame_id);
    void registerCallback(PacketCallback *cb);
    void processCANMessages();
    void saveToPacket(const can_frame frame);
     
    // default frame ids
    static constexpr uint32_t error_default_frame_id_ = 0x01u;
    static constexpr uint32_t warning_default_frame_id_ = 0x02u;
    static constexpr uint32_t sample_time_default_frame_id_ = 0x05u;
    static constexpr uint32_t group_counter_default_frame_id_ = 0x06u;
    static constexpr uint32_t status_word_default_frame_id_ = 0x11u;
    static constexpr uint32_t quaternion_default_frame_id_ = 0x21u;
    static constexpr uint32_t delta_v_default_frame_id_ = 0x31u;
    static constexpr uint32_t rate_of_turn_default_frame_id_ = 0x32u;
    static constexpr uint32_t delta_q_default_frame_id_ = 0x33u;
    static constexpr uint32_t acceleration_default_frame_id_ = 0x34u;
    static constexpr uint32_t free_acceleration_default_frame_id_ = 0x35u;
    static constexpr uint32_t rate_of_turn_hr_default_frame_id_ = 0x61u;
    static constexpr uint32_t acceleration_hr_default_frame_id_ = 0x62u;
    static constexpr uint32_t magnetic_field_default_frame_id_ = 0x41u;
    static constexpr uint32_t temperature_default_frame_id_ = 0x51u;
    static constexpr uint32_t barometric_pressure_default_frame_id_ = 0x52u;
    static constexpr uint32_t utc_default_frame_id_ = 0x07u;
    static constexpr uint32_t euler_angles_default_frame_id_ = 0x22u;
    static constexpr uint32_t lat_lon_default_frame_id_ = 0x71u;
    static constexpr uint32_t altitude_ellipsoid_default_frame_id_ = 0x72u;
    static constexpr uint32_t position_ecef_x_default_frame_id_ = 0x73u;
    static constexpr uint32_t position_ecef_y_default_frame_id_ = 0x74u;
    static constexpr uint32_t position_ecef_z_default_frame_id_ = 0x75u;
    static constexpr uint32_t velocity_default_frame_id_ = 0x76u;
    static constexpr uint32_t gnss_receiver_status_default_frame_id_ = 0x79u;
    static constexpr uint32_t gnss_receiver_dop_default_frame_id_ = 0x7au;

    //default param frame id mapping
    const std::map<uint32_t, std::string> frame_id_default_map_ = {
        {uint32_t(error_default_frame_id_), "error_frame_id"},
        {uint32_t(warning_default_frame_id_), "warning_frame_id"},
        {uint32_t(sample_time_default_frame_id_), "sample_time_frame_id"},
        {uint32_t(group_counter_default_frame_id_), "group_counter_frame_id"},
        {uint32_t(status_word_default_frame_id_), "status_word_frame_id"},
        {uint32_t(quaternion_default_frame_id_), "quaternion_frame_id"},
        {uint32_t(delta_v_default_frame_id_), "delta_v_frame_id"},
        {uint32_t(rate_of_turn_default_frame_id_), "rate_of_turn_frame_id"},
        {uint32_t(delta_q_default_frame_id_), "delta_q_frame_id"},
        {uint32_t(acceleration_default_frame_id_), "acceleration_frame_id"},
        {uint32_t(free_acceleration_default_frame_id_), "free_acceleration_frame_id"},
        {uint32_t(rate_of_turn_hr_default_frame_id_), "rate_of_turn_hr_frame_id"},
        {uint32_t(acceleration_hr_default_frame_id_), "acceleration_hr_frame_id"},
        {uint32_t(magnetic_field_default_frame_id_), "magnetic_field_frame_id"},
        {uint32_t(temperature_default_frame_id_), "temperature_frame_id"},
        {uint32_t(barometric_pressure_default_frame_id_), "barometric_pressure_frame_id"},
        {uint32_t(utc_default_frame_id_), "utc_frame_id"},
        {uint32_t(euler_angles_default_frame_id_), "euler_angles_frame_id"},
        {uint32_t(lat_lon_default_frame_id_), "lat_lon_frame_id"},
        {uint32_t(altitude_ellipsoid_default_frame_id_), "altitude_ellipsoid_frame_id"},
        {uint32_t(position_ecef_x_default_frame_id_), "position_ecef_x_frame_id"},
        {uint32_t(position_ecef_y_default_frame_id_), "position_ecef_y_frame_id"},
        {uint32_t(position_ecef_z_default_frame_id_), "position_ecef_z_frame_id"},
        {uint32_t(velocity_default_frame_id_), "velocity_frame_id"},
        {uint32_t(gnss_receiver_status_default_frame_id_), "gnss_receiver_status_frame_id"},
        {uint32_t(gnss_receiver_dop_default_frame_id_), "gnss_receiver_dop_frame_id"},
    };

    //frame ids to remap to the defaults 
    std::map< uint32_t, uint32_t > frame_id_remap_;

};

#endif // XSCANINTERFACE_H
