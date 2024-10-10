#include "pch.h"

#include "qdevice_api.h"
#include "qdevice_utilities.h"
#include "RTPacket.h"
#include "RTProtocol.h"

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <iostream>
#include <thread>
#include <atomic>
#include <vector>
#include <mutex>
#include <algorithm>

// Undefine the min/max macros to prevent conflicts with std::min()
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

namespace
{
    // Mutex to protect frames_data access between threads
    std::mutex frames_data_mutex;

    const std::string UNKNOWN = "<unknown>";
    const std::string INTEGRATION_NAME = "Force Input";
    const std::string INTEGRATION_DESCRIPTION = "Test live force input device";
    const std::string INTEGRATION_URL = "trinoma.fr";
    const std::string DEVICE_MANUFACTURER = "TRINOMA";
    const std::string DEVICE_PRODUCT = "Test Live Force Input Product";
    const std::string DEVICE_MODEL = "Test Live Force Input Model";
    constexpr auto DEVICE_FREQUENCY = 1000;

    struct force_plate_channel
    {
        std::string name;
        qd_channel_unit unit;
    };

    struct force_device
    {
        std::string serial_number;
        float position[3];
        float rotation[3];
        float width, length, height;
        std::vector<force_plate_channel> channels;
    };

    std::vector<force_device> force_device_definitions
    {
        {
            "Front",
            { 0.0f, 0.0f, 0.0f },
            { 0.0f, 180.0f, 0.0f },//rotate 180 degrees so the z-axis of the plate is down
            1.0f, 1.0f, 1.0f,
            {
                { "Force X", qd_channel_unit_newtons },
                { "Force Y", qd_channel_unit_newtons },
                { "Force Z", qd_channel_unit_newtons },
                { "Moment X", qd_channel_unit_newton_meters },
                { "Moment Y", qd_channel_unit_newton_meters },
                { "Moment Z", qd_channel_unit_newton_meters },
                { "Application X", qd_channel_unit_meters },
                { "Application Y", qd_channel_unit_meters },
                { "Application Z", qd_channel_unit_meters }
            }
        },
        {
            "Rear",
            { 0.0f, 0.0f, 0.0f },
            { 0.0f, 0.0f, 0.0f },//rotate 180 degrees so the z-axis of the plate is down
            1.0f, 1.0f, 1.0f,
            {
                { "Force X", qd_channel_unit_newtons },
                { "Force Y", qd_channel_unit_newtons },
                { "Force Z", qd_channel_unit_newtons },
                { "Moment X", qd_channel_unit_newton_meters },
                { "Moment Y", qd_channel_unit_newton_meters },
                { "Moment Z", qd_channel_unit_newton_meters },
                { "Application X", qd_channel_unit_meters },
                { "Application Y", qd_channel_unit_meters },
                { "Application Z", qd_channel_unit_meters }
            }
        },
        {
            "Left",
            { 0.0f, 0.0f, 0.0f },
            { 0.0f, 0.0f, 0.0f },//rotate 180 degrees so the z-axis of the plate is down
            1.0f, 1.0f, 1.0f,
            {
                { "Force X", qd_channel_unit_newtons },
                { "Force Y", qd_channel_unit_newtons },
                { "Force Z", qd_channel_unit_newtons },
                { "Moment X", qd_channel_unit_newton_meters },
                { "Moment Y", qd_channel_unit_newton_meters },
                { "Moment Z", qd_channel_unit_newton_meters },
                { "Application X", qd_channel_unit_meters },
                { "Application Y", qd_channel_unit_meters },
                { "Application Z", qd_channel_unit_meters }
            }
        },
        {
			"Right",
			{ 0.0f, 0.0f, 0.0f },
			{ 0.0f, 0.0f, 0.0f },//rotate 180 degrees so the z-axis of the plate is down
			1.0f, 1.0f, 1.0f,
			{
				{ "Force X", qd_channel_unit_newtons },
				{ "Force Y", qd_channel_unit_newtons },
				{ "Force Z", qd_channel_unit_newtons },
				{ "Moment X", qd_channel_unit_newton_meters },
				{ "Moment Y", qd_channel_unit_newton_meters },
				{ "Moment Z", qd_channel_unit_newton_meters },
				{ "Application X", qd_channel_unit_meters },
				{ "Application Y", qd_channel_unit_meters },
				{ "Application Z", qd_channel_unit_meters }
			}
		}
    };

    std::thread rt_thread;
    std::atomic<bool> rt_running{ false };

    /*std::vector< std::array<float, 36> > frames_data;*/
    std::vector< std::array<float, 9> > frames_data;

    using std_clock_t = std::chrono::high_resolution_clock;
    std_clock_t::time_point g_streaming_start{};
    using sample_duration_ratio_t = std::ratio<1, DEVICE_FREQUENCY>;
    using sample_duration_t = std::chrono::duration<long long, sample_duration_ratio_t>;

    qd_state g_state = qd_state_disconnected;

    std::size_t g_pushed_samples = 0;
    int frame_number = 0;

    qd_device create_device_from_definition(const force_device& fdevice)
    {
        qd_device ret = QDevice::Utilities::create_device(qd_device_category_force_plate, fdevice.serial_number, fdevice.channels.size(), 13);
        ret.properties.data[0] = QDevice::Utilities::create_property("Name", true, qd_property_type_string_utf8, fdevice.serial_number, UNKNOWN, "");
        ret.properties.data[1] = QDevice::Utilities::create_property("Manufacturer", true, qd_property_type_string_utf8, DEVICE_MANUFACTURER, UNKNOWN, "");
        ret.properties.data[2] = QDevice::Utilities::create_property("Product", true, qd_property_type_string_utf8, DEVICE_PRODUCT, UNKNOWN, "");
        ret.properties.data[3] = QDevice::Utilities::create_property("Model", true, qd_property_type_string_utf8, DEVICE_MODEL, UNKNOWN, "");
        ret.properties.data[4] = QDevice::Utilities::create_property("Width", false, qd_property_type_float32, std::to_string(fdevice.width), "0", "");
        ret.properties.data[5] = QDevice::Utilities::create_property("Length", false, qd_property_type_float32, std::to_string(fdevice.length), "0", "");
        ret.properties.data[6] = QDevice::Utilities::create_property("Height", false, qd_property_type_float32, std::to_string(fdevice.height), "0", "");
        ret.properties.data[7] = QDevice::Utilities::create_property("Rotation X", false, qd_property_type_float32, std::to_string(fdevice.rotation[0]), "0", "");
        ret.properties.data[8] = QDevice::Utilities::create_property("Rotation Y", false, qd_property_type_float32, std::to_string(fdevice.rotation[1]), "0", "");
        ret.properties.data[9] = QDevice::Utilities::create_property("Rotation Z", false, qd_property_type_float32, std::to_string(fdevice.rotation[2]), "0", "");
        ret.properties.data[10] = QDevice::Utilities::create_property("Position X", false, qd_property_type_float32, std::to_string(fdevice.position[0]), "0", "");
        ret.properties.data[11] = QDevice::Utilities::create_property("Position Y", false, qd_property_type_float32, std::to_string(fdevice.position[1]), "0", "");
        ret.properties.data[12] = QDevice::Utilities::create_property("Position Z", false, qd_property_type_float32, std::to_string(fdevice.position[2]), "0", "");

        for (std::size_t c = 0; c < fdevice.channels.size(); c++)
        {
            ret.channels.data[c] = QDevice::Utilities::create_channel(fdevice.channels[c].name, qd_sample_encoding_float32, fdevice.channels[c].unit, DEVICE_FREQUENCY);
        }

        return ret;
    }
}


void update(std::atomic<bool>& running, std::vector< std::array<float, 9> >& frames_data)
{
    try {

        CRTProtocol rtProtocol;


        const char           serverAddr[] = "127.0.0.1";
        const unsigned short basePort = 22222;
        const int            majorVersion = 1;
        const int            minorVersion = 19;
        const bool           bigEndian = false;

        bool dataAvailable = false;
        bool streamFrames = false;
        unsigned short udpPort = 6734;

        //std::array<float, 36> fdata{ {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
        //    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        //    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        //    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f} };

        std::array<float, 9> fdata{ {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f} };
        

        while (running) {

            if (!rtProtocol.Connected())
            {
                if (!rtProtocol.Connect(serverAddr, basePort, &udpPort, majorVersion, minorVersion, bigEndian))
                {
                    printf("rtProtocol.Connect: %s\n\n", rtProtocol.GetErrorString());
                    //rtProtocol.GetErrorString();
                }
            }

            if (!dataAvailable)
            {
                if (!rtProtocol.ReadForceSettings(dataAvailable))
                {
                    printf("rtProtocol.ReadForceSettings: %s\n\n", rtProtocol.GetErrorString());
                    //rtProtocol.GetErrorString();
                }
            }

            if (!streamFrames)
            {
                if (!rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, udpPort, NULL, CRTProtocol::cComponentForce))
                {
                    printf("rtProtocol.StreamFrames: %s\n\n", rtProtocol.GetErrorString());
                    //rtProtocol.GetErrorString();
                }
                streamFrames = true;
            }

            CRTPacket::EPacketType packetType;

            if (rtProtocol.Receive(packetType) == CNetwork::ResponseType::success)
            {
                if (packetType == CRTPacket::PacketData)
                {
                    CRTPacket* rtPacket = rtProtocol.GetRTPacket();

                    frame_number = rtPacket->GetFrameNumber();

                    unsigned int nCount = rtPacket->GetForcePlateCount();

                    if (nCount > 0) //if force plate count is not null
                    {
                        CRTPacket::SForce sForce;
                        unsigned int        nForceNumber;

                        //for (unsigned int iPlate = 0; iPlate < 1; iPlate++) //loop on force plates
                        //{
                        //    if (iPlate < 4)
                        //    {
                                unsigned int nForceCount = rtPacket->GetForceCount(0);

                                if (nForceCount > 0)
                                {
                                    for (unsigned int iForce = 0; iForce < nForceCount; iForce++) //loop over force frames
                                    {
                                        if (rtPacket->GetForceData(0, iForce, sForce)) //plate 0
                                        {

                                            unsigned int iPlate = 0;

                                            fdata[iPlate * 9] = sForce.fForceX;
                                            fdata[iPlate * 9 + 1] = sForce.fForceY;
                                            fdata[iPlate * 9 + 2] = sForce.fForceZ;
                                            fdata[iPlate * 9 + 3] = sForce.fMomentX;
                                            fdata[iPlate * 9 + 4] = sForce.fMomentY;
                                            fdata[iPlate * 9 + 5] = sForce.fMomentZ;
                                            fdata[iPlate * 9 + 6] = sForce.fApplicationPointX;
                                            fdata[iPlate * 9 + 7] = sForce.fApplicationPointY;
                                            fdata[iPlate * 9 + 8] = sForce.fApplicationPointZ;

                                            frames_data.push_back(fdata); //TO DO solve push_back issue
                                        }

                                        /*std::array<float, 9> fdata{ {(float)nForceCount, 0, 0,
                                           0, 0, 0,
                                           0, 0, 0} };

                                        frames_data.push_back(fdata);*/
                                    }

                                }
                                /*else {
                                    std::array<float, 9> fdata{ {123.456f, 0, 0,
                                                       0, 0, 0,
                                                       0, 0, 0} };

                                    frames_data.push_back(fdata);
                                }*/
                        //    }
                        //}

                        
                    }

                    

                    
                }
            }
            
        }
    }
    catch (std::exception& e)
    {
        printf("%s\n", e.what());
    }
}

qd_call_status qd_get_integration_version(struct qd_static_chars* version)
{
    static const char str[] = "0.1.1";
    *version = { str, sizeof(str) / sizeof(char) - 1 };
    return {};
}

qd_call_status qd_get_name(qd_static_chars* name)
{
    *name = { INTEGRATION_NAME.c_str(), INTEGRATION_NAME.length() };
    return { qd_call_status_type_ok, nullptr };
}

qd_call_status qd_get_description(qd_static_chars* desc)
{
    *desc = { INTEGRATION_DESCRIPTION.c_str(), INTEGRATION_DESCRIPTION.length() };
    return { qd_call_status_type_ok, nullptr };
}

qd_call_status qd_get_manufacturer_url(qd_static_chars* url)
{
    *url = { INTEGRATION_URL.c_str(), INTEGRATION_URL.length() };
    return { qd_call_status_type_ok, nullptr };
}

qd_call_status qd_get_contracts(qd_contracts* contracts)
{
    *contracts = { nullptr, 0 };
    return { qd_call_status_type_ok, nullptr };
}

qd_call_status qd_get_category(qd_device_category* category)
{
    *category = qd_device_category_force_plate;
    return { qd_call_status_type_ok, nullptr };
}

qd_call_status qd_get_configuration(qd_configuration* config)
{
    const auto deviceCount = force_device_definitions.size();
    *config = QDevice::Utilities::create_configuration(0, deviceCount);

    for (std::size_t i = 0; i < deviceCount; i++)
    {
        config->devices.data[i] = create_device_from_definition(force_device_definitions[i]);
    }

    return { qd_call_status_type_ok, nullptr };
}

qd_call_status qd_free_configuration(qd_configuration* config)
{
    QDevice::Utilities::free_configuration(*config);
    return { qd_call_status_type_ok, nullptr };
}

qd_call_status qd_get_actions(qd_actions* actions)
{
    *actions = { nullptr, 0 };
    return { qd_call_status_type_ok, nullptr };
}

qd_call_status qd_connect(const qd_configuration* config)
{
    if (g_state != qd_state_disconnected)
    {
        return { qd_call_status_type_error, "Device already connected" };
    }

    //no writable properties

    g_state = qd_state_connected;
    return { qd_call_status_type_ok, nullptr };
}

qd_call_status qd_disconnect()
{
    if (g_state == qd_state_disconnected)
    {
        return { qd_call_status_type_error, "Device not connected" };
    }

    if (g_state == qd_state_streaming)
    {
        qd_stop_streaming();
    }

    frames_data.clear();
    g_state = qd_state_disconnected;
    return { qd_call_status_type_ok, nullptr };
}

qd_call_status qd_start_streaming()
{
    if (g_state == qd_state_streaming)
    {
        return { qd_call_status_type_error, "Can't start stream, streaming already started" };
    }

    if (g_state == qd_state_disconnected)
    {
        return { qd_call_status_type_error, "Can't start stream, device not connected" };
    }

    g_state = qd_state_streaming;
    g_streaming_start = std_clock_t::now();
    g_pushed_samples = 0;
    rt_running = true;
    rt_thread = std::thread(update, std::ref(rt_running), std::ref(frames_data));
    return { qd_call_status_type_ok, nullptr };
}

qd_call_status qd_stop_streaming()
{
    if (g_state != qd_state_streaming)
    {
        return { qd_call_status_type_error, "Can't stop stream, streaming not started" };
    }

    rt_running = false;
    rt_thread.join();
    g_state = qd_state_connected;
    return { qd_call_status_type_ok, nullptr };
}

qd_call_status qd_get_state(qd_state* state)
{
    *state = g_state;
    return { qd_call_status_type_ok, nullptr };
}


namespace
{
    template<typename T>
    void push_samples(qd_sample_push_fn callback,
        const force_device& fdevice,
        //const std::size_t fdevice_index,
        const std::size_t sample,
        const std::size_t count)
    {
        for (std::size_t c = 0; c < fdevice.channels.size(); c++)
        {
            std::vector<T> samples{};
            samples.reserve(count);

            // Process only available frames
            for (std::size_t sample_index = 0; sample_index < count; sample_index++)
            {
                try {
                    const auto& frame = frames_data.at(sample_index);  // Access frame data safely
                    //samples.push_back(frame[fdevice_index * 9 + c]);  // Get channel data
                    samples.push_back(frame[c]);
                }
                catch (const std::out_of_range& e) {
                    // Handle case where not enough frames are available
                    printf("\n No data, error: %s\n", e.what());
                    samples.push_back(0.0);  // Add placeholder data
                }
            }

            // Push the collected samples for this channel to the callback
            callback(fdevice.serial_number.c_str(), fdevice.channels[c].name.c_str(), qd_samples{
                sample,
                qd_sample_encoding_float32,
                reinterpret_cast<uint8_t*>(samples.data()),
                count });
        }
    }
}

qd_call_status qd_read_samples(qd_sample_push_fn callback)
{
    // Calculate how many samples should be processed
    const auto samples_since_start = std::chrono::duration_cast<sample_duration_t>(std_clock_t::now() - g_streaming_start);
    const auto sample_start = g_pushed_samples;
    const auto sample_count = static_cast<std::size_t>(samples_since_start.count() - sample_start);

    // Make sure there are enough new frames to process
    if (sample_count > 0 && !frames_data.empty())
    {
        // Cast both values to std::size_t for compatibility
        const auto frames_to_process = std::min(static_cast<std::size_t>(sample_count), static_cast<std::size_t>(frames_data.size()));

        // Push samples for all devices
        for (size_t fdeviceIndex = 0; fdeviceIndex < force_device_definitions.size(); ++fdeviceIndex) {
            const auto& fdevice = force_device_definitions[fdeviceIndex];
            //push_samples<std::float_t>(callback, fdevice, fdeviceIndex, sample_start, frames_to_process);
            push_samples<std::float_t>(callback, fdevice, sample_start, frames_to_process);
        }

        // Update the number of pushed samples
        g_pushed_samples += frames_to_process;

        // Remove processed frames from frames_data
        frames_data_mutex.lock();  // Manually lock before modifying shared data
        frames_data.erase(frames_data.begin(), frames_data.begin() + frames_to_process);
        frames_data_mutex.unlock();  // Unlock after modifying shared data
    }

    return { qd_call_status_type_ok, nullptr };
}


