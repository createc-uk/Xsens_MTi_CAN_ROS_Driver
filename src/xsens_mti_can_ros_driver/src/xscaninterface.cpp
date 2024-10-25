#include "xscaninterface.h"
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>

#include "messagepublishers/accelerationpublisher.h"
#include "messagepublishers/angularvelocitypublisher.h"
#include "messagepublishers/freeaccelerationpublisher.h"
#include "messagepublishers/gnssposepublisher.h"
#include "messagepublishers/gnsspublisher.h"
#include "messagepublishers/imupublisher.h"
#include "messagepublishers/magneticfieldpublisher.h"
#include "messagepublishers/orientationincrementspublisher.h"
#include "messagepublishers/orientationpublisher.h"
#include "messagepublishers/packetcallback.h"
#include "messagepublishers/positionllapublisher.h"
#include "messagepublishers/pressurepublisher.h"
#include "messagepublishers/statuspublisher.h"
#include "messagepublishers/temperaturepublisher.h"
#include "messagepublishers/timereferencepublisher.h"
#include "messagepublishers/transformpublisher.h"
#include "messagepublishers/twistpublisher.h"
#include "messagepublishers/utctimepublisher.h"
#include "messagepublishers/velocityincrementpublisher.h"
#include "messagepublishers/velocitypublisher.h"
// Include other publishers as needed

XsCanInterface::XsCanInterface()
    : socket_(-1)
{
    // Initialize
    ros::param::get("~can_interface_", can_interface_);
    ROS_INFO("XsCanInterface::constructor, can_interface_: %s", can_interface_.c_str());

    std::string time_option = "mti_utc";
    ros::param::get("~time_option", time_option);
    ROS_INFO("XsCanInterface::constructor, time_option: %s", time_option.c_str());
  	m_time_handler_.setTimeOption(time_option);

    // Fill the remapping frame id map with only the frames of interest 
    for (auto& f : frame_id_default_map_)
    {
        uint32_t default_value = f.first;
        std::string default_string = f.second;        
        
        int temp_param;
        if (ros::param::get("~" + default_string, temp_param)) 
        {
            uint32_t frame_id = static_cast<uint32_t>(temp_param);
            if (frame_id_remap_.count(frame_id))
            {
                ROS_ERROR_STREAM("XsCanInterface::constructor, remapping frame_id:" << unsigned(frame_id) << " has already been remapped!\n"
                << "was mapped to:" << unsigned(frame_id_remap_[frame_id]) << " (name:" << frame_id_default_map_.at(frame_id_remap_[frame_id]) <<")\n"
                << "overriding to:" << unsigned(default_value) << " (name:" << default_string << ")" );
            }
            frame_id_remap_[frame_id] = default_value;
            updateMinFrameId(frame_id);
        }       
    }
    ROS_INFO("XsCanInterface::constructor, packet start frame_id: %u", m_start_frame_id_);

    std::stringstream ss;
    ss << "XsCanInterface::constructor, remapping frame_ids:\n";
    ss << "Remapped | Default";
    for(const auto ids:frame_id_remap_)
    {
        ss << "\n" << std::setw(8) << ids.first << " | " << ids.second;
    }
    ROS_DEBUG_STREAM(ss.str());
}

XsCanInterface::~XsCanInterface()
{
    if (socket_ != -1)
    {
        close(socket_);
    }
}

void XsCanInterface::updateMinFrameId(uint32_t frame_id)
{
    if(frame_id < m_start_frame_id_)
    {
        m_start_frame_id_ = frame_id;
    }
}

bool XsCanInterface::initialize()
{
    // Check we have some frames defined
    if (frame_id_remap_.empty()) {
        ROS_ERROR("Error no frames are defined not creating a CAN socket");
		return false;
	}

    // Initialize SocketCAN socket
    ROS_DEBUG("XsCanInterface::initialize, Setting up socket");
    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0) {
        ROS_ERROR("Error setting up the CAN socket");
		return false;
	}

    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_interface_.c_str());
    ioctl(socket_, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;


    ROS_DEBUG("XsCanInterface::initialize, Setting socketcan frame id filter");
    // Filter for the frames we need
    struct can_filter rfilter[frame_id_remap_.size()];
    int index = 0;
    for(const auto f: frame_id_remap_)
    {
       rfilter[index].can_id   = f.first;
       rfilter[index].can_mask = CAN_SFF_MASK;
       index++; 
    }
    int ret = setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
    if (ret < 0) {
        ROS_WARN("Unable to setup socketcan frame id filters");
    } 

    ROS_DEBUG("XsCanInterface::initialize, Binding to socket");
    ret = bind(socket_, (struct sockaddr *)&addr, sizeof(addr));
	if (ret < 0) {
        ROS_ERROR("Error bind the CAN socket");
		close(socket_);
		return false;
	}

    return true;
}

void XsCanInterface::spinFor()
{
    processCANMessages();
}

void XsCanInterface::registerPublishers(ros::NodeHandle &node)
{
	bool should_publish;

	if (ros::param::get("~pub_imu", should_publish) && should_publish)
	{
		registerCallback(new ImuPublisher(node));
	}
	if (ros::param::get("~pub_quaternion", should_publish) && should_publish)
	{
		registerCallback(new OrientationPublisher(node));
	}
	if (ros::param::get("~pub_acceleration", should_publish) && should_publish)
	{
		registerCallback(new AccelerationPublisher(node));
	}
	if (ros::param::get("~pub_angular_velocity", should_publish) && should_publish)
	{
		registerCallback(new AngularVelocityPublisher(node));
	}
	if (ros::param::get("~pub_mag", should_publish) && should_publish)
	{
		registerCallback(new MagneticFieldPublisher(node));
	}
	if (ros::param::get("~pub_dq", should_publish) && should_publish)
	{
		registerCallback(new OrientationIncrementsPublisher(node));
	}
	if (ros::param::get("~pub_dv", should_publish) && should_publish)
	{
		registerCallback(new VelocityIncrementPublisher(node));
	}
	if (ros::param::get("~pub_sampletime", should_publish) && should_publish)
	{
		registerCallback(new TimeReferencePublisher(node));
	}
	if (ros::param::get("~pub_temperature", should_publish) && should_publish)
	{
		registerCallback(new TemperaturePublisher(node));
	}
	if (ros::param::get("~pub_pressure", should_publish) && should_publish)
	{
		registerCallback(new PressurePublisher(node));
	}
	// if (ros::param::get("~pub_gnss", should_publish) && should_publish)
	// {
	// 	registerCallback(new GnssPublisher(node));
	// }
	if (ros::param::get("~pub_twist", should_publish) && should_publish)
	{
		registerCallback(new TwistPublisher(node));
	}
	if (ros::param::get("~pub_free_acceleration", should_publish) && should_publish)
	{
		registerCallback(new FreeAccelerationPublisher(node));
	}
	if (ros::param::get("~pub_transform", should_publish) && should_publish)
	{
		registerCallback(new TransformPublisher(node));
	}
	if (ros::param::get("~pub_positionLLA", should_publish) && should_publish)
	{
		registerCallback(new PositionLLAPublisher(node));
	}
	if (ros::param::get("~pub_velocity", should_publish) && should_publish)
	{
		registerCallback(new VelocityPublisher(node));
	}
	if (ros::param::get("~pub_status", should_publish) && should_publish)
	{
		// ROS_INFO("registerCallback StatusPublisher....");
		registerCallback(new StatusPublisher(node));
	}
	if (ros::param::get("~pub_gnsspose", should_publish) && should_publish)
	{
		// ROS_INFO("registerCallback GNSSPOSEPublisher....");
		registerCallback(new GNSSPOSEPublisher(node));
	}
	if (ros::param::get("~pub_utctime", should_publish) && should_publish)
	{
		// ROS_INFO("registerCallback UTCTimePublisher....");
		registerCallback(new UTCTimePublisher(node));
	}
}

void XsCanInterface::processCANMessages()
{
    struct can_frame frame;
    int nbytes = read(socket_, &frame, sizeof(struct can_frame));
    
    if (nbytes < 0)
    {
        ROS_ERROR("Error while reading CAN frame");
        return;
    }

    if (nbytes < sizeof(struct can_frame))
    {
        ROS_ERROR("Incomplete CAN frame read");
        return;
    }

    uint32_t currentFrameId = frame.can_id;
    if (currentFrameId == m_start_frame_id_)
    {
        // We have reached the start of a new packet, publish the previous packet
        if (last_frame_id_ != 0xFFFFFFFF) // Check if this is not the first frame ever received
        {
            ros::Time now = m_time_handler_.convertUtcTimeToRosTime(packet_);
            for (auto &cb : m_callbacks_)
            {
                cb->operator()(packet_, now);
            }
        }

        // Clear the packet and start filling it with new data
        packet_ = XsDataPacket();
        saveToPacket(frame);
    }
    else
    {
        // Still filling the current packet
        saveToPacket(frame);
    }

    // Update the last_frame_id_ regardless of whether it's a new packet or not
    last_frame_id_ = currentFrameId;
}


void XsCanInterface::saveToPacket(const can_frame frame)
{
    // Checking if the remap contains the frame id
    auto it = frame_id_remap_.find(frame.can_id);
    if (it != frame_id_remap_.end()) 
    {      
        uint32_t frame_id = it->first;
        uint32_t frame_id_default = it->second;
        std::string frame_id_name = frame_id_default_map_.at(it->second); //note it has to be in default map

        // if frame is in the mapping
        ROS_DEBUG_STREAM("XsCanInterface::saveToPacket got frame id " << unsigned(frame.can_id)
             << " with remapping match [remap, default]: [" << frame_id << ", " << frame_id_default << "]"
             << " (default frame_id as string: " << frame_id_name << ")" );
        
        switch(frame_id_default)
        {
            case error_default_frame_id_:
            {
                if (xserror_unpack(packet_.error, frame))
                {
                    packet_.containsXsError = true;
                }
                break;
            }
            case warning_default_frame_id_:
                if (xswarning_unpack(packet_.warning, frame))
                {
                    packet_.containsXsWarning = true;
                }
                break;
            case sample_time_default_frame_id_:
                if (xssampletime_unpack(packet_.sample_time_fine, frame))
                {
                    packet_.containsXsSampleTimeFine = true;
                    // ROS_INFO("xscaninterface.cpp, get mti_sampletime: %u", packet_.sample_time_fine);
                }
                break;

            case group_counter_default_frame_id_:
                if (xsgroupcounter_unpack(packet_.counter, frame))
                {
                    packet_.containsXsGroupCounter = true;
                }
                break;

            case utc_default_frame_id_:
                if (xsutctime_unpack(packet_.utc_time, frame))
                {
                    packet_.containsXsUtcTime = true;
                    // ROS_INFO("xscaninterface.cpp, get utc time");
                }
                break;

            case status_word_default_frame_id_:
                if (xsstatusword_unpack(packet_.status_word, frame))
                {
                    packet_.containsXsStatusWord = true;
                }
                break;

            case quaternion_default_frame_id_:

                if (xsquaternion_unpack(packet_.quaternion, frame))
                {
                    packet_.containsXsQuaternion = true;
                }
                break;

            case euler_angles_default_frame_id_:
                if (xseuler_unpack(packet_.euler, frame))
                {
                    packet_.containsXsEuler = true;
                }
                break;

            case delta_v_default_frame_id_:
                if (xsdeltavelocity_unpack(packet_.delta_velocity, frame))
                {
                    packet_.containsXsDeltaVelocity = true;
                }
                break;

            case rate_of_turn_default_frame_id_:
                if (xsrateofturn_unpack(packet_.rate_of_turn, frame))
                {
                    packet_.containsXsRateOfTurn = true;
                }
                break;

            case delta_q_default_frame_id_:
                if (xsquaternion_unpack(packet_.delta_q, frame))
                {
                    packet_.containsXsDeltaQ = true;
                }
                break;

            case acceleration_default_frame_id_:
                if (xsacceleration_unpack(packet_.acceleration, frame))
                {
                    packet_.containsXsAcceleration = true;
                }
                break;

            case free_acceleration_default_frame_id_:
                if (xsacceleration_unpack(packet_.free_acceleration, frame))
                {
                    packet_.containsXsFreeAcceleration = true;
                }
                break;

            case magnetic_field_default_frame_id_:
                if (xsmagneticfield_unpack(packet_.magnetic_field, frame))
                {
                    packet_.containsXsMagneticField = true;
                }
                break;

            case temperature_default_frame_id_:
                if (xstemperature_unpack(packet_.temperature, frame))
                {
                    packet_.containsXsTemperature = true;
                }
                break;

            case barometric_pressure_default_frame_id_:
                if (xsbaropressure_unpack(packet_.baro_pressure, frame))
                {
                    packet_.containsXsBarometricPressure = true;
                }
                break;

            case rate_of_turn_hr_default_frame_id_:
                if (xsrateofturn_unpack(packet_.rate_of_turn_hr, frame))
                {
                    packet_.containsXsRateOfTurnHR = true;
                }
                break;

            case acceleration_hr_default_frame_id_:
                if (xsacceleration_unpack(packet_.acceleration_hr, frame))
                {
                    packet_.containsXsAccelerationHR = true;
                }
                break;

            case lat_lon_default_frame_id_:
                if (xslatlon_unpack(packet_.lat_lon, frame))
                {
                    packet_.containsXsLatLon = true;
                }
                break;

            case altitude_ellipsoid_default_frame_id_:
                if (xsaltellipsoid_unpack(packet_.altitude, frame))
                {
                    packet_.containsXsAltitudeEllipsoid = true;
                }
                break;

            case position_ecef_x_default_frame_id_:
                if (xspositionecefX_unpack(packet_.position_ecef.x, frame))
                {
                    packet_.containsXsPositionEcefX = true;
                }
                break;

            case position_ecef_y_default_frame_id_:
                if (xspositionecefX_unpack(packet_.position_ecef.y, frame))
                {
                    packet_.containsXsPositionEcefY = true;
                }
                break;

            case position_ecef_z_default_frame_id_:
                if (xspositionecefX_unpack(packet_.position_ecef.z, frame))
                {
                    packet_.containsXsPositionEcefZ = true;
                }
                break;

            case velocity_default_frame_id_:
                if (xsvelocity_unpack(packet_.velocity, frame))
                {
                    packet_.containsXsVelocity = true;
                }
                break;

            case gnss_receiver_status_default_frame_id_:
                if (xsgnsssreceiverstatus_unpack(packet_.gnss_receiver_status, frame))
                {
                    packet_.containsXsGnssReceiverStatus = true;
                }
                break;

            case gnss_receiver_dop_default_frame_id_:
                if (xsgnsssreceiverdop_unpack(packet_.gnss_receiver_dop, frame))
                {
                    packet_.containsXsGnssReceiverDop = true;
                }
                break;
            
            //TODO: positionECEF



            default:
                ROS_WARN("Frame ID look up unpacker not defined!");
                break;
            }
    }
    else 
    {
        // if frame_id is not in our frame map
        ROS_WARN("Unknown CAN ID");
    }



    
}

void XsCanInterface::registerCallback(PacketCallback *cb)
{
    m_callbacks_.push_back(cb);

}

void XsCanInterface::closeInterface()
{
    if (socket_ != -1)
    {
        close(socket_);
    }
}
