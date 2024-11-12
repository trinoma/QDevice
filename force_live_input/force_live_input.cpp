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
            { 0.0f, 0.0f, 0.0f },
            0.635f, 0.6223f, 0.0f,
            {
                { "Force X", qd_channel_unit_newtons },
                { "Force Y", qd_channel_unit_newtons },
                { "Force Z", qd_channel_unit_newtons },
                { "Moment X", qd_channel_unit_newton_meters },
                { "Moment Y", qd_channel_unit_newton_meters },
                { "Moment Z", qd_channel_unit_newton_meters },
                { "Application X", qd_channel_unit_millimeters },
                { "Application Y", qd_channel_unit_millimeters },
                { "Application Z", qd_channel_unit_millimeters }
            }
        },
        {
            "Rear",
            { 0.0f, 0.0f, 0.0f },
            { 0.0f, 0.0f, 0.0f },
            0.635f, 0.6223f, 0.0f,
            {
                { "Force X", qd_channel_unit_newtons },
                { "Force Y", qd_channel_unit_newtons },
                { "Force Z", qd_channel_unit_newtons },
                { "Moment X", qd_channel_unit_newton_meters },
                { "Moment Y", qd_channel_unit_newton_meters },
                { "Moment Z", qd_channel_unit_newton_meters },
                { "Application X", qd_channel_unit_millimeters },
                { "Application Y", qd_channel_unit_millimeters },
                { "Application Z", qd_channel_unit_millimeters }
            }
        },
        {
            "Right",
            { 0.0f, 0.0f, 0.0f },
            { 0.0f, 0.0f, 0.0f },
            0.635f, 1.2466f, 0.0f,
            {
                { "Force X", qd_channel_unit_newtons },
                { "Force Y", qd_channel_unit_newtons },
                { "Force Z", qd_channel_unit_newtons },
                { "Moment X", qd_channel_unit_newton_meters },
                { "Moment Y", qd_channel_unit_newton_meters },
                { "Moment Z", qd_channel_unit_newton_meters },
                { "Application X", qd_channel_unit_millimeters },
                { "Application Y", qd_channel_unit_millimeters },
                { "Application Z", qd_channel_unit_millimeters }
            }
        },
        {
			"Left",
			{ 0.0f, 0.0f, 0.0f },
			{ 0.0f, 0.0f, 0.0f },
            0.635f, 1.2466f, 0.0f,
			{
				{ "Force X", qd_channel_unit_newtons },
				{ "Force Y", qd_channel_unit_newtons },
				{ "Force Z", qd_channel_unit_newtons },
				{ "Moment X", qd_channel_unit_newton_meters },
				{ "Moment Y", qd_channel_unit_newton_meters },
				{ "Moment Z", qd_channel_unit_newton_meters },
				{ "Application X", qd_channel_unit_millimeters },
				{ "Application Y", qd_channel_unit_millimeters },
				{ "Application Z", qd_channel_unit_millimeters }
			}
		}
    };

    std::thread rt_thread;
    std::atomic<bool> rt_running{ false };

    std::vector< std::array<float, 9> > frames_data_front;
    std::vector< std::array<float, 9> > frames_data_rear;
    std::vector< std::array<float, 9> > frames_data_right;
    std::vector< std::array<float, 9> > frames_data_left;

    using std_clock_t = std::chrono::high_resolution_clock;
    std_clock_t::time_point g_streaming_start{};
    using sample_duration_ratio_t = std::ratio<1, DEVICE_FREQUENCY>;
    using sample_duration_t = std::chrono::duration<long long, sample_duration_ratio_t>;

    qd_state g_state = qd_state_disconnected;

    std::size_t g_pushed_samples_front = 0;
    std::size_t g_pushed_samples_rear = 0;
    std::size_t g_pushed_samples_right = 0;
    std::size_t g_pushed_samples_left = 0;

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


void update(std::atomic<bool>& running, std::vector< std::array<float, 9> >& frames_data_front, std::vector< std::array<float, 9> >& frames_data_rear, std::vector< std::array<float, 9> >& frames_data_right, std::vector< std::array<float, 9> >& frames_data_left)
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

        const char components[256] = "Force 6deuler ";
        const char* Treadmill6DOF = "Treadmill";

        //force plate parameters (TODO READ FROM CODE)
        const float front_a = 0.0002794f; //x in mm
        const float front_b = 0.0006096f;
        const float front_c = -0.0432054f;
        const float rear_a = 0.0005842f;
        const float rear_b = -0.0014478f;
        const float rear_c = -0.0415036f;

        //forceplate positions in mm
        const float front_plate_position[3] = {0.0f, 359.55f, 0.0f};
        const float rear_plate_position[3] = { 0.0f, -359.55f, 0.0f };


        const float FP_params[2][3] = {
            {front_a, front_b, front_c},
            {rear_a, rear_b, rear_c}
        };


        float prevForceMean[4][3] = {
            {0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f}
        };

        float prevMomentMean[4][3] = {
            {0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f}
        };


        bool plateON[4] = { false, false, false, false };

        float treadmillAngle = 0.0f;
        float lastTreadmillAngle = 0.0f;

        bool assignFrontToRight = true;
        bool assignRearToRight = true;

        unsigned int nForceCountFront;
        unsigned int nForceCountRear;

        //force and moment angle offset correction factor
        float inclineForceFactor[2][3] = {
            {0.0f, -8.0f, 0.0f},
            {0.0f, -8.0f, 0.0f},
        };

        float inclineMomentFactor[2][3] = {
            {0.0f, -8.0f, 0.0f},
            {0.0f, -8.0f, 0.0f},
        };

        //force and moment baseline for front and rear plates
        float forceBaseline[2][3] = {
            {0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f},
        };

        float momentBaseline[2][3] = {
            {0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f},
        };

        float unloadedForceBaseline[2][3] = {
            {0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f},
        };

        float unloadedMomentBaseline[2][3] = {
            {0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f},
        };
        

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
                if (!rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, udpPort, NULL, components))
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

                    //Get 6DOF Euler data
                    float fX, fY, fZ, fAng1, fAng2, fAng3;
                    
                    unsigned int RBCount = rtPacket->Get6DOFEulerBodyCount();

                    if (RBCount > 0)
                    {
                        for (unsigned int iRB = 0; iRB < RBCount; iRB++)
                        {
                            if (rtPacket->Get6DOFEulerBody(iRB, fX, fY, fZ, fAng1, fAng2, fAng3))
                            {
                                const char* pTmpStr = rtProtocol.Get6DOFBodyName(iRB);

                                if (pTmpStr != nullptr && strcmp(pTmpStr, Treadmill6DOF) == 0)
                                {
                                    treadmillAngle = -fAng2;
                                    break;
                                }
                            }
                        }
                    }

                    

                    unsigned int nCount = rtPacket->GetForcePlateCount();

                    if (nCount > 0) //if force plate count is not null
                    {
                        CRTPacket::SForce sForce;

                        float forceMean[3] = { 0.0f, 0.0f , 0.0f };
                        float momentMean[3] = { 0.0f, 0.0f , 0.0f };

                        std::vector< std::array<float, 9> > temp_frames_data_front;
                        std::vector< std::array<float, 9> > temp_frames_data_rear;

                        for (unsigned int iPlate = 0; iPlate < nCount; iPlate++) //loop on force plates
                        {
                            if (iPlate < 4)
                            {
                                unsigned int nForceCount = rtPacket->GetForceCount(iPlate);

                                if (nForceCount > 0)
                                {
                                    std::array<std::unique_ptr<float[]>, 3> forces = {
                                        std::make_unique<float[]>(nForceCount),  // xForce
                                        std::make_unique<float[]>(nForceCount),  // yForce
                                        std::make_unique<float[]>(nForceCount)   // zForce
                                    };

                                    std::array<std::unique_ptr<float[]>, 3> moments = {
                                        std::make_unique<float[]>(nForceCount),  // xMoment
                                        std::make_unique<float[]>(nForceCount),  // yMoment
                                        std::make_unique<float[]>(nForceCount)   // zMoment
                                    };

                                    std::array<std::unique_ptr<float[]>, 3> cop = {
                                        std::make_unique<float[]>(nForceCount),  // xCOP
                                        std::make_unique<float[]>(nForceCount),  // yCOP
                                        std::make_unique<float[]>(nForceCount)   // zCOP
                                    };

                                    float forceSum[3] = { 0.0f, 0.0f , 0.0f };
                                    float momentSum[3] = { 0.0f, 0.0f , 0.0f };

                                    for (unsigned int iForce = 0; iForce < nForceCount; iForce++) //loop over force frames
                                    {
                                        if (rtPacket->GetForceData(iPlate, iForce, sForce)) //plate 0
                                        {
                                            forces[0][iForce] = sForce.fForceX;
                                            forces[1][iForce] = sForce.fForceY;
                                            forces[2][iForce] = sForce.fForceZ;

                                            moments[0][iForce] = sForce.fMomentX;
                                            moments[1][iForce] = sForce.fMomentY;
                                            moments[2][iForce] = sForce.fMomentZ;

                                            cop[0][iForce] = sForce.fApplicationPointX;
                                            cop[1][iForce] = sForce.fApplicationPointY;
                                            cop[2][iForce] = sForce.fApplicationPointZ;

                                            //compute mean force and moment over camera frame (as force frame rate is higher than camera rate, there are several force frames per camera frame)
                                            //forceMean[0] = (forceMean[0] + forces[0][iForce]) / 2; //X
                                            //forceMean[1] = (forceMean[1] + forces[1][iForce]) / 2; //Y
                                            //forceMean[2] = (forceMean[2] + forces[2][iForce]) / 2; //Z

                                            //momentMean[0] = (momentMean[0] + moments[0][iForce]) / 2; //X
                                            //momentMean[1] = (momentMean[1] + moments[1][iForce]) / 2; //Y
                                            //momentMean[2] = (momentMean[2] + moments[2][iForce]) / 2; //Z

                                            forceSum[0] += forces[0][iForce];
                                            forceSum[1] += forces[1][iForce];
                                            forceSum[2] += forces[2][iForce];

                                            momentSum[0] += moments[0][iForce];
                                            momentSum[1] += moments[1][iForce];
                                            momentSum[2] += moments[2][iForce];
                                        }
                                    }

                                    //compute mean force and moment over camera frame (as force frame rate is higher than camera rate, there are several force frames per camera frame)
                                    forceMean[0] = forceSum[0] / nForceCount; //X
                                    forceMean[1] = forceSum[1] / nForceCount; //Y
                                    forceMean[2] = forceSum[2] / nForceCount; //Z

                                    momentMean[0] = momentSum[0] / nForceCount; //X
                                    momentMean[1] = momentSum[1] / nForceCount; //Y
                                    momentMean[2] = momentSum[2] / nForceCount; //Z

                                    if (forceMean[2] < 20) { //if mean Z force on is below 20N
                                        if (prevForceMean[iPlate][2] > 20) { //if previous frame mean Z force was above 20N (offset threshold)
                                            plateON[iPlate] = false;

                                            // reinitialize factors and baseline at foot off
                                            if (iPlate == 0 || iPlate == 1) // front/rear plates
                                            {
                                                inclineForceFactor[iPlate][0] = 0.0f;
                                                inclineForceFactor[iPlate][1] = -8.0f;
                                                inclineForceFactor[iPlate][2] = 0.0f;

                                                inclineMomentFactor[iPlate][0] = 0.0f;
                                                inclineMomentFactor[iPlate][1] = 0.0f;
                                                inclineMomentFactor[iPlate][2] = 0.0f;

                                                //reinitialize unloaded force baseline to force baseline to unsure previous force baseline saved value does not affect next value badly in the average computation
                                                unloadedForceBaseline[iPlate][0] = forceBaseline[iPlate][0];
                                                unloadedForceBaseline[iPlate][1] = forceBaseline[iPlate][1];
                                                unloadedForceBaseline[iPlate][2] = forceBaseline[iPlate][2];

                                                unloadedMomentBaseline[iPlate][0] = momentBaseline[iPlate][0];
                                                unloadedMomentBaseline[iPlate][1] = momentBaseline[iPlate][1];
                                                unloadedMomentBaseline[iPlate][2] = momentBaseline[iPlate][2];
                                            }

                                            if (iPlate == 2) {
                                                assignRearToRight = false;
                                            }
                                            else if (iPlate == 3) {
                                                assignRearToRight = true;
                                            }
                                        }
                                        //TODO VERIFIER ASSIGNATION QUAND FORCE sur LEFT

                                    }
                                    else if (forceMean[2] > 20) { //if mean Z force on is above 20N
                                        if (prevForceMean[iPlate][2] < 20) { //if previous frame mean Z force was below 20N (onset threshold)
                                            plateON[iPlate] = true;
                                            if (iPlate == 2) {
                                                assignFrontToRight = true;
                                            }
                                            else if (iPlate == 3) {
                                                assignFrontToRight = false;
                                            }
                                        }
                                    }

                                    if (iPlate == 0 || iPlate == 1) // front/rear plates
                                    {
                                        // when plate is off and treadmill inclined (0.5 to avoid division by O), adjustment factors are computed
                                        // adjustement factors serve to adjust force offset when foot is on while moving

                                        if (plateON[iPlate] == false)
                                        {
                                            unloadedForceBaseline[iPlate][0] = (forceMean[0] + unloadedForceBaseline[iPlate][0]) / 2;
                                            unloadedForceBaseline[iPlate][1] = (forceMean[1] + unloadedForceBaseline[iPlate][1]) / 2;
                                            unloadedForceBaseline[iPlate][2] = (forceMean[2] + unloadedForceBaseline[iPlate][2]) / 2;


                                            forceBaseline[iPlate][0] = unloadedForceBaseline[iPlate][0];
                                            forceBaseline[iPlate][1] = unloadedForceBaseline[iPlate][1];
                                            forceBaseline[iPlate][2] = unloadedForceBaseline[iPlate][2];

                                            unloadedMomentBaseline[iPlate][0] = (momentMean[0] + unloadedMomentBaseline[iPlate][0]) / 2;
                                            unloadedMomentBaseline[iPlate][1] = (momentMean[1] + unloadedMomentBaseline[iPlate][1]) / 2;
                                            unloadedMomentBaseline[iPlate][2] = (momentMean[2] + unloadedMomentBaseline[iPlate][2]) / 2;

                                            momentBaseline[iPlate][0] = unloadedMomentBaseline[iPlate][0];
                                            momentBaseline[iPlate][1] = unloadedMomentBaseline[iPlate][1];
                                            momentBaseline[iPlate][2] = unloadedMomentBaseline[iPlate][2];

                                            if (abs(treadmillAngle) > 0.5f)
                                            {
                                                inclineForceFactor[iPlate][0] = ((forceMean[0] / treadmillAngle) + inclineForceFactor[iPlate][0]) / 2;
                                                inclineForceFactor[iPlate][1] = ((forceMean[1] / treadmillAngle) + inclineForceFactor[iPlate][1]) / 2;
                                                inclineForceFactor[iPlate][2] = ((forceMean[2] / treadmillAngle)) + inclineForceFactor[iPlate][2] / 2;
                                                inclineMomentFactor[iPlate][0] = ((momentMean[0] / treadmillAngle) + inclineMomentFactor[iPlate][0]) / 2;
                                                inclineMomentFactor[iPlate][1] = ((momentMean[1] / treadmillAngle) + inclineMomentFactor[iPlate][1]) / 2;
                                                inclineMomentFactor[iPlate][2] = ((momentMean[2] / treadmillAngle)) + inclineMomentFactor[iPlate][2] / 2;
                                            }
                                            lastTreadmillAngle = treadmillAngle;
                                        }
                                        else {
                                            forceBaseline[iPlate][0] = unloadedForceBaseline[iPlate][0] + inclineForceFactor[iPlate][0] * (treadmillAngle - lastTreadmillAngle);
                                            forceBaseline[iPlate][1] = unloadedForceBaseline[iPlate][1] + inclineForceFactor[iPlate][1] * (treadmillAngle - lastTreadmillAngle);
                                            forceBaseline[iPlate][2] = unloadedForceBaseline[iPlate][2] + inclineForceFactor[iPlate][2] * (treadmillAngle - lastTreadmillAngle);

                                            momentBaseline[iPlate][0] = unloadedMomentBaseline[iPlate][0] + inclineMomentFactor[iPlate][0] * (treadmillAngle - lastTreadmillAngle);
                                            momentBaseline[iPlate][1] = unloadedMomentBaseline[iPlate][1] + inclineMomentFactor[iPlate][1] * (treadmillAngle - lastTreadmillAngle);
                                            momentBaseline[iPlate][2] = unloadedMomentBaseline[iPlate][2] + inclineMomentFactor[iPlate][2] * (treadmillAngle - lastTreadmillAngle);
                                        }

                                        for (unsigned int iForce = 0; iForce < nForceCount; iForce++) { //loop over force frames
                                            if (forces[0] && forces[1] && forces[2])
                                            {
                                                forces[0][iForce] -= forceBaseline[iPlate][0];
                                                forces[1][iForce] -= forceBaseline[iPlate][1];
                                                forces[2][iForce] -= forceBaseline[iPlate][2];

                                                moments[0][iForce] -= momentBaseline[iPlate][0];
                                                moments[1][iForce] -= momentBaseline[iPlate][1];
                                                moments[2][iForce] -= momentBaseline[iPlate][2];


                                                if (plateON[iPlate] == true)
                                                {
                                                    cop[0][iForce] = (((FP_params[iPlate][2] * forces[0][iForce] - moments[1][iForce]) / forces[2][iForce]) + FP_params[iPlate][0]) * 1000; //((ORIGIN[Z] * Force[X] - M[Y]) / Force[Z]) + ORIGIN[X]
                                                    cop[1][iForce] = (((FP_params[iPlate][2] * forces[1][iForce] + moments[0][iForce]) / forces[2][iForce]) + FP_params[iPlate][1]) * 1000; //((ORIGIN[Z] * Force[Y] + M[X]) / Force[Z]) + ORIGIN[Y]
                                                    cop[2][iForce] = 0.0f; //cop Z stays O
                                                }
                                            }
                                        }
                                        printf("\n");

                                    }


                                    if (iPlate == 0 || iPlate == 1) //Front/Rear Plate
                                    {
                                        for (unsigned int iForce = 0; iForce < nForceCount; iForce++) //loop over force frames
                                        {
                                            std::array<float, 9> fdata{ {forces[0][iForce], forces[1][iForce], forces[2][iForce],
                                                moments[0][iForce], moments[1][iForce], moments[2][iForce],
                                                cop[0][iForce], cop[1][iForce], cop[2][iForce]} };

                                            if (iPlate == 0) {
                                                frames_data_front.push_back(fdata);
                                                temp_frames_data_front.push_back(fdata);
                                                nForceCountFront = nForceCount;
                                            }
                                            else if (iPlate == 1) {
                                                frames_data_rear.push_back(fdata);
                                                temp_frames_data_rear.push_back(fdata);
                                                nForceCountRear = nForceCount;
                                            }
                                        }
                                    }
                                    
                                    //TODO CORRECTIONS UNITES COP ET MOMENTS

                                    if (iPlate == 2) //Right plate
                                    {
                                        if (assignFrontToRight == true && plateON[0] == true && assignRearToRight == true && plateON[1] == true)
                                        {
                                            if (temp_frames_data_front.size() >= std::min(nForceCountFront,nForceCountRear) && temp_frames_data_rear.size() >= std::min(nForceCountFront, nForceCountRear))
                                            {
                                                for (unsigned int iForce = 0; iForce < std::min(nForceCountFront, nForceCountRear); iForce++) //loop over force frames
                                                {
                                                    float front_force_norm = sqrt(
                                                        temp_frames_data_front[iForce][0] * temp_frames_data_front[iForce][0]
                                                        + temp_frames_data_front[iForce][1] * temp_frames_data_front[iForce][1]
                                                        + temp_frames_data_front[iForce][2] * temp_frames_data_front[iForce][2]);

                                                    float rear_force_norm = sqrt(
                                                        temp_frames_data_front[iForce][0] * temp_frames_data_front[iForce][0]
                                                        + temp_frames_data_front[iForce][1] * temp_frames_data_front[iForce][1]
                                                        + temp_frames_data_front[iForce][2] * temp_frames_data_front[iForce][2]);

                                                    float cop_x = ((temp_frames_data_front[iForce][6] + front_plate_position[0]) * front_force_norm + (temp_frames_data_rear[iForce][6] + rear_plate_position[0]) * rear_force_norm) / (front_force_norm + rear_force_norm);
                                                    float cop_y = ((temp_frames_data_front[iForce][7] + front_plate_position[1]) * front_force_norm + (temp_frames_data_rear[iForce][7] + rear_plate_position[1]) * rear_force_norm) / (front_force_norm + rear_force_norm);
                                                    float cop_z = ((temp_frames_data_front[iForce][8] + front_plate_position[2]) * front_force_norm + (temp_frames_data_rear[iForce][8] + rear_plate_position[2]) * rear_force_norm) / (front_force_norm + rear_force_norm);


                                                    std::array<float, 9> fdata{ {
                                                        temp_frames_data_front[iForce][0] + temp_frames_data_rear[iForce][0],
                                                        temp_frames_data_front[iForce][1] + temp_frames_data_rear[iForce][1],
                                                        temp_frames_data_front[iForce][2] + temp_frames_data_rear[iForce][2],
                                                        cop_y * (temp_frames_data_front[iForce][2] + temp_frames_data_rear[iForce][2]),
                                                        - cop_x * temp_frames_data_front[iForce][2] + temp_frames_data_rear[iForce][2],
                                                        (temp_frames_data_front[iForce][1] + temp_frames_data_rear[iForce][1]) * cop_x - (temp_frames_data_front[iForce][0] + temp_frames_data_rear[iForce][0]) * cop_y,
                                                        cop_x,
                                                        cop_y,
                                                        cop_z,
                                                        } };

                                                    frames_data_right.push_back(fdata);
                                                }
                                            }  
                                        }
                                        else if(assignFrontToRight == true && plateON[0] == true)
                                        {
                                            if (temp_frames_data_front.size() >= nForceCountFront)
                                            {
                                                for (unsigned int iForce = 0; iForce < nForceCountFront; iForce++) //loop over force frames
                                                {
                                                    std::array<float, 9> fdata{ {
                                                        temp_frames_data_front[iForce][0],
                                                        temp_frames_data_front[iForce][1],
                                                        temp_frames_data_front[iForce][2],
                                                        temp_frames_data_front[iForce][2] * (temp_frames_data_front[iForce][7] + front_plate_position[1]),
                                                        - temp_frames_data_front[iForce][2] * (temp_frames_data_front[iForce][6] + front_plate_position[0]),
                                                        temp_frames_data_front[iForce][1] * (temp_frames_data_front[iForce][6] + front_plate_position[0]) - temp_frames_data_front[iForce][0] * (temp_frames_data_front[iForce][7] + front_plate_position[1]),
                                                        temp_frames_data_front[iForce][6] + front_plate_position[0],
                                                        temp_frames_data_front[iForce][7] + front_plate_position[1],
                                                        temp_frames_data_front[iForce][8] + front_plate_position[2]} };

                                                    frames_data_right.push_back(fdata);
                                                }
                                            }  
                                        }
                                        else if (assignRearToRight == true && plateON[1] == true)
                                        {
                                            if (temp_frames_data_rear.size() >= nForceCountRear)
                                            {
                                                for (unsigned int iForce = 0; iForce < nForceCountRear; iForce++) //loop over force frames
                                                {
                                                    std::array<float, 9> fdata{ {
                                                        temp_frames_data_rear[iForce][0],
                                                        temp_frames_data_rear[iForce][1],
                                                        temp_frames_data_rear[iForce][2],
                                                        temp_frames_data_rear[iForce][2] * (temp_frames_data_rear[iForce][7] + rear_plate_position[1]),
                                                        -temp_frames_data_rear[iForce][2] * (temp_frames_data_rear[iForce][6] + rear_plate_position[0]),
                                                        temp_frames_data_rear[iForce][1] * (temp_frames_data_rear[iForce][6] + rear_plate_position[0]) - temp_frames_data_rear[iForce][0] * (temp_frames_data_rear[iForce][7] + rear_plate_position[1]),
                                                        temp_frames_data_rear[iForce][6] + rear_plate_position[0],
                                                        temp_frames_data_rear[iForce][7] + rear_plate_position[1],
                                                        temp_frames_data_rear[iForce][8] + rear_plate_position[2]} };

                                                    frames_data_right.push_back(fdata);
                                                }
                                            } 
                                        }
                                        else 
                                        {
                                            for (unsigned int iForce = 0; iForce < std::min(nForceCountFront, nForceCountRear); iForce++) //loop over force frames
                                            {
                                                std::array<float, 9> fdata{ { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f} };

                                                frames_data_right.push_back(fdata);
                                            }
                                        }
                                        
                                    }


                                    if (iPlate == 3) //Left plate
                                    {
                                        if (assignFrontToRight == false && plateON[0] == true && assignRearToRight == false && plateON[1] == true)
                                        {
                                            if (temp_frames_data_front.size() >= std::min(nForceCountFront, nForceCountRear) && temp_frames_data_rear.size() >= std::min(nForceCountFront, nForceCountRear))
                                            {
                                                for (unsigned int iForce = 0; iForce < std::min(nForceCountFront, nForceCountRear); iForce++) //loop over force frames
                                                {

                                                    float front_force_norm = sqrt(
                                                        temp_frames_data_front[iForce][0] * temp_frames_data_front[iForce][0]
                                                        + temp_frames_data_front[iForce][1] * temp_frames_data_front[iForce][1]
                                                        + temp_frames_data_front[iForce][2] * temp_frames_data_front[iForce][2]);

                                                    float rear_force_norm = sqrt(
                                                        temp_frames_data_front[iForce][0] * temp_frames_data_front[iForce][0]
                                                        + temp_frames_data_front[iForce][1] * temp_frames_data_front[iForce][1]
                                                        + temp_frames_data_front[iForce][2] * temp_frames_data_front[iForce][2]);

                                                    float cop_x = ((temp_frames_data_front[iForce][6] + front_plate_position[0]) * front_force_norm + (temp_frames_data_rear[iForce][6] + rear_plate_position[0]) * rear_force_norm) / (front_force_norm + rear_force_norm);
                                                    float cop_y = ((temp_frames_data_front[iForce][7] + front_plate_position[1]) * front_force_norm + (temp_frames_data_rear[iForce][7] + rear_plate_position[1]) * rear_force_norm) / (front_force_norm + rear_force_norm);
                                                    float cop_z = ((temp_frames_data_front[iForce][8] + front_plate_position[2]) * front_force_norm + (temp_frames_data_rear[iForce][8] + rear_plate_position[2]) * rear_force_norm) / (front_force_norm + rear_force_norm);

                                                    std::array<float, 9> fdata{ {
                                                        temp_frames_data_front[iForce][0] + temp_frames_data_rear[iForce][0],
                                                        temp_frames_data_front[iForce][1] + temp_frames_data_rear[iForce][1],
                                                        temp_frames_data_front[iForce][2] + temp_frames_data_rear[iForce][2],
                                                        cop_y * (temp_frames_data_front[iForce][2] + temp_frames_data_rear[iForce][2]),
                                                        -cop_x * temp_frames_data_front[iForce][2] + temp_frames_data_rear[iForce][2],
                                                        (temp_frames_data_front[iForce][1] + temp_frames_data_rear[iForce][1]) * cop_x - (temp_frames_data_front[iForce][0] + temp_frames_data_rear[iForce][0]) * cop_y,
                                                        cop_x,
                                                        cop_y,
                                                        cop_z,
                                                        } };

                                                    frames_data_left.push_back(fdata);
                                                }
                                            } 
                                        }
                                        else if (assignFrontToRight == false && plateON[0] == true)
                                        {
                                            if (temp_frames_data_front.size() >= nForceCountFront)
                                            {
                                                for (unsigned int iForce = 0; iForce < nForceCountFront; iForce++) //loop over force frames
                                                {
                                                    std::array<float, 9> fdata{ {
                                                        temp_frames_data_front[iForce][0],
                                                        temp_frames_data_front[iForce][1],
                                                        temp_frames_data_front[iForce][2],
                                                        temp_frames_data_front[iForce][2] * (temp_frames_data_front[iForce][7] + front_plate_position[1]),
                                                        -temp_frames_data_front[iForce][2] * (temp_frames_data_front[iForce][6] + front_plate_position[0]),
                                                        temp_frames_data_front[iForce][1] * (temp_frames_data_front[iForce][6] + front_plate_position[0]) - temp_frames_data_front[iForce][0] * (temp_frames_data_front[iForce][7] + front_plate_position[1]),
                                                        temp_frames_data_front[iForce][6] + front_plate_position[0],
                                                        temp_frames_data_front[iForce][7] + front_plate_position[1],
                                                        temp_frames_data_front[iForce][8] + front_plate_position[2]} };

                                                    frames_data_left.push_back(fdata);
                                                }
                                            }  
                                        }
                                        else if (assignRearToRight == false && plateON[1] == true)
                                        {
                                            if (temp_frames_data_rear.size() >= nForceCountRear)
                                            {
                                                for (unsigned int iForce = 0; iForce < nForceCountRear; iForce++) //loop over force frames
                                                {
                                                    std::array<float, 9> fdata{ {
                                                        temp_frames_data_rear[iForce][0],
                                                        temp_frames_data_rear[iForce][1],
                                                        temp_frames_data_rear[iForce][2],
                                                        temp_frames_data_rear[iForce][2] * (temp_frames_data_rear[iForce][7] + rear_plate_position[1]),
                                                        -temp_frames_data_rear[iForce][2] * (temp_frames_data_rear[iForce][6] + rear_plate_position[0]),
                                                        temp_frames_data_rear[iForce][1] * (temp_frames_data_rear[iForce][6] + rear_plate_position[0]) - temp_frames_data_rear[iForce][0] * (temp_frames_data_rear[iForce][7] + rear_plate_position[1]),
                                                        temp_frames_data_rear[iForce][6] + rear_plate_position[0],
                                                        temp_frames_data_rear[iForce][7] + rear_plate_position[1],
                                                        temp_frames_data_rear[iForce][8] + rear_plate_position[2]} };

                                                    frames_data_left.push_back(fdata);
                                                }
                                            }  
                                        }
                                        else
                                        {
                                            for (unsigned int iForce = 0; iForce < std::min(nForceCountFront, nForceCountRear); iForce++) //loop over force frames
                                            {
                                                std::array<float, 9> fdata{ { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f} };

                                                frames_data_left.push_back(fdata);
                                            }
                                        }

                                    }


                                }
                                prevForceMean[iPlate][0] = forceMean[0];
                                prevForceMean[iPlate][1] = forceMean[1];
                                prevForceMean[iPlate][2] = forceMean[2];

                                prevMomentMean[iPlate][0] = momentMean[0];
                                prevMomentMean[iPlate][1] = momentMean[1];
                                prevMomentMean[iPlate][2] = momentMean[2];
                            }
                        }
                        temp_frames_data_front.clear();
                        temp_frames_data_rear.clear();
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

    frames_data_front.clear();
    frames_data_rear.clear();
    frames_data_right.clear();
    frames_data_left.clear();

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
    g_pushed_samples_front = 0;
    g_pushed_samples_rear = 0;
    g_pushed_samples_right = 0;
    g_pushed_samples_left = 0;

    rt_running = true;
    rt_thread = std::thread(update, std::ref(rt_running), std::ref(frames_data_front), std::ref(frames_data_rear), std::ref(frames_data_right), std::ref(frames_data_left));
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
        const std::size_t sample,
        const std::size_t count,
        const std::vector<std::array<float, 9>>& frames_data_to_push)
    {
        for (std::size_t c = 0; c < fdevice.channels.size(); c++)
        {
            std::vector<T> samples{};
            samples.reserve(count);

            // Process only available frames
            for (std::size_t sample_index = 0; sample_index < count; sample_index++)
            {
                try {
                    const auto& frame = frames_data_to_push.at(sample_index);  // Access frame data safely
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
    // Process frames_data_front
    if (!frames_data_front.empty()) {
        const auto frames_to_process_front = static_cast<std::size_t>(frames_data_front.size());

        const auto& fdevice = force_device_definitions[0];
        push_samples<std::float_t>(callback, fdevice, g_pushed_samples_front, frames_to_process_front, frames_data_front);

        g_pushed_samples_front += frames_to_process_front;

        frames_data_mutex.lock();  // Lock before modifying shared data
        frames_data_front.erase(frames_data_front.begin(), frames_data_front.begin() + frames_to_process_front);
        frames_data_mutex.unlock();  // Unlock after modifying shared data
    }
    // Process frames_data_rear
    if (!frames_data_rear.empty()) {
        const auto frames_to_process_rear = static_cast<std::size_t>(frames_data_rear.size());

        const auto& fdevice = force_device_definitions[1];
        push_samples<std::float_t>(callback, fdevice, g_pushed_samples_rear, frames_to_process_rear, frames_data_rear);

        g_pushed_samples_rear += frames_to_process_rear;

        frames_data_mutex.lock();  // Lock before modifying shared data
        frames_data_rear.erase(frames_data_rear.begin(), frames_data_rear.begin() + frames_to_process_rear);
        frames_data_mutex.unlock();  // Unlock after modifying shared data
    }
    // Process frames_data_right
    if (!frames_data_right.empty()) {
		const auto frames_to_process_right = static_cast<std::size_t>(frames_data_right.size());

		const auto& fdevice = force_device_definitions[2];
		push_samples<std::float_t>(callback, fdevice, g_pushed_samples_right, frames_to_process_right, frames_data_right);

		g_pushed_samples_right += frames_to_process_right;

		frames_data_mutex.lock();  // Lock before modifying shared data
		frames_data_right.erase(frames_data_right.begin(), frames_data_right.begin() + frames_to_process_right);
		frames_data_mutex.unlock();  // Unlock after modifying shared data
	}
    // Process frames_data_left
    if (!frames_data_left.empty()) {
        const auto frames_to_process_left = static_cast<std::size_t>(frames_data_left.size());

        const auto& fdevice = force_device_definitions[3];
        push_samples<std::float_t>(callback, fdevice, g_pushed_samples_left, frames_to_process_left, frames_data_left);

        g_pushed_samples_left += frames_to_process_left;

        frames_data_mutex.lock();  // Lock before modifying shared data
        frames_data_left.erase(frames_data_left.begin(), frames_data_left.begin() + frames_to_process_left);
        frames_data_mutex.unlock();  // Unlock after modifying shared data
    }



    return { qd_call_status_type_ok, nullptr };
}


