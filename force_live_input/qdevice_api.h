// Copyright 2024 Qualisys AB

#pragma once

/**
 * \file
 */

#if defined(__WINDOWS__) || defined(_WIN32)
#define QD_API __declspec(dllexport)
#define QD_STDCALL __cdecl
#else
#define QD_API
#define QD_STDCALL
#endif

#define QD_API_MAJOR 0
#define QD_API_MINOR 7

#if (QD_API_MAJOR == 1)
#error "1. Comment out this line + 2. Rebuild + 3. Remove all deprecations + 4. Increment RHS above + 5. Uncomment this line"
#define DEPRECATED(date) [[deprecated("QDevice - qdevice_api.h: Deprecated function since (" #date ") and MUST NOW BE REMOVED!")]]
#define DEPRECATED_R(date, replacementFuncName) [[deprecated("QDevice - qdevice_api.h: Deprecated function since " #date " and MUST NOW BE REMOVED! REPLACEMENT: " #replacementFuncName)]]
#else
#if defined(_MSC_VER) || defined(_WIN32)
#pragma warning(3:4996) // Makes "[deprecated]" generate warning instead of error
#endif
#define DEPRECATED(date) [[deprecated("QDevice - qdevice_api.h: Deprecated function since (" #date "), will be removed in next MAJOR version!")]]
#define DEPRECATED_REPLACED(date, replacementFuncName) [[deprecated("QDevice - qdevice_api.h: Deprecated function since (" #date "), will be removed in next MAJOR version! REPLACEMENT: " #replacementFuncName)]]
#endif

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif
    /**
     * \brief Dynamically allocated UTF-8 encoded and `NULL`-terminated string of a
     * fixed size.
     *
     * qd_chars::size does not include the `NULL`-terminator. Hence, actual
     * memory size of qd_chars::data is qd_chars::size + 1.
     */
    struct qd_chars
    {
        /**
         * \brief Pointer to the array of chars.
         */
        const char* data;

        /**
         * \brief Number of bytes in the array, excluding `NULL`-terminator.
         */
        size_t size;
    };

    /**
     * \brief Statically allocated UTF-8 encoded and `NULL`-terminated string of a fixed
     * size.
     *
     * qd_chars::size does not include the `NULL`-terminator. Hence, actual
     * memory size of qd_chars::data is qd_chars::size + 1.
     *
     * See qd_get_name() for example usage.
     */
    struct qd_static_chars
    {
        /**
         * \brief Pointer to the array of chars.
         */
        const char* data;

        /**
         * \brief Number of bytes in the array, excluding `NULL`-terminator.
         */
        size_t size;
    };

    /**
     * \brief QDevice API version.
     */
    struct qd_semver
    {
        /**
         * \brief Major version number.
         */
        uint32_t major;

        /**
         * \brief Minor version number.
         */
        uint32_t minor;
    };

    /**
     * \brief Specifies data type of a qd_property_value.
     *
     * The property values are encoded as qd_chars, this enum specifies how the
     * values are serialized.
     */
    typedef enum
    {
        /**
         * \brief Serialized using UTF-8 for Unicode strings.
         *
         * Note that this is the same representation used in qd_chars which
         * means no parsing/formatting is necessary.
         */
        qd_property_type_string_utf8,

        /**
         * \brief Serialized using ASCII representation for decimal signed
         * integer.
         */
        qd_property_type_int32,

        /**
         * \brief Serialized using ASCII representation for IEEE-754 single
         * precision floating point.
         */
        qd_property_type_float32,

        /**
         * \brief Serialized using ASCII representation of strings "true" and "false".
         */
        qd_property_type_boolean
    } qd_property_type;

    /**
     * \brief Typed property value encoded as qd_chars.
     */
    struct qd_property_value
    {
        /**
         * \brief Type of property value.
         */
        qd_property_type type;

        /**
         * \brief Property value.
         */
        struct qd_chars value;
    };

    /**
     * \brief Typed property for metadata and configuration.
     */
    struct qd_property
    {
        /**
         * \brief Name of the property.
         *
         * Must be non-empty. Names must be title case.
         */
        struct qd_chars name;

        /**
         * \brief Declares whether the property is read only.
         *
         * Designed for use in user interfaces to provide feedback that this
         * property can not be modified by user (by for example graying out the
         * user interface element). Integrations are expected to ignore value
         * changes to properties where this bool is true.
         */
        bool read_only;

        /**
         * \brief Value of the property.
         */
        struct qd_property_value value;

        /**
         * \brief Default value of the property.
         *
         * Designed for use in user interfaces to enable user to restore
         * values to default. For ::read_only properties this is nominally set to
         * the same as ::value.
         */
        struct qd_property_value default_value;

        /**
         * \brief Validation string.
         *
         * Describes what values are valid for this property. Provides a way
         * for user interfaces to validate user input without calling into the
         * integration for verification. Format is described below.
         *
         * A valid range is one of the following:
         *
         * * `"< x"`, less than `x`
         * * `"<= x"`, less than or equal to `x`
         * * `"> x"`, greater than `x`
         * * `">= x"`, greater than or equal to `x`
         * * `"x"`, equal to `x`
         *
         * The value `x` can be an boolean, integer, float or string. For
         * strings, lexicographic comparison is used. Examples:
         *
         * * `"> 0"` would match `1`, `2`, `3`, etc
         * * `"<= 3.14"` would match `0.0`, `1.5`, `3.14`, etc
         * * `"example string"` would match `"example string"`
         *
         * Multiple valid ranges can be combined in this string using comma
         * separation. If there is both comparisons and specific values in the
         * string, the specific values will be matched against first,
         * considering value valid if there is a match. Then the remaining
         * conditions will be combined using logical and, same as if there were
         * no specific values in the string. This allows matching specific
         * special values outside of a specified range. Examples:
         *
         * * `"> 0, <= 4"` would match `1`, `2`, `3` and `4`
         * * `"left, right, up, down"` would match `"left"`, `"right"`, `"up"` and `"down"`
         * * `"0, >= 10, <= 20"` would match `0` and `10` through `20`
         *
         * the comparison values must be on the right-hand side. Place the
         * value to the left must be treated as an error by the calling
         * application.
         *
         * Spaces are optional except for within specific values where they are
         * taken into account.
         */
        struct qd_chars valid_ranges;
    };

    /**
     * \brief Array of qd_property.
     *
     * May be empty. If ::size is 0 then ::data must also be `NULL`.
     */
    struct qd_properties
    {
        /**
         * \brief Pointer to first item in this array.
         */
        struct qd_property* data;

        /**
         * \brief Number of items in this array.
         */
        size_t size;
    };

    /**
     * \brief Unit of measurement of sample data.
     */
    typedef enum
    {
        /**
         * \brief Length in meters (m).
         */
        qd_channel_unit_meters,

        /**
         * \brief Length in millimeters (mm).
         */
        qd_channel_unit_millimeters,

        /**
         * \brief Angle in degrees (deg).
         */
        qd_channel_unit_degrees,

        /**
         * \brief Force in newtons (N).
         */
        qd_channel_unit_newtons,

        /**
         * \brief Acceleration in g-forces (g).
         */
        qd_channel_unit_g_forces,

        /**
         * \brief Magnetic flux density in microteslas (uT).
         */
        qd_channel_unit_microteslas,

        /**
         * \brief Speed in meters per second (m/s).
         */
        qd_channel_unit_meters_per_second,

        /**
         * \brief Torque in newton-meters (Nm).
         */
        qd_channel_unit_newton_meters,

        /**
         * \brief Electric potential difference in volts (V).
         */
        qd_channel_unit_volts,

        /**
         * \brief Electric potential difference in millivolts (mV).
         */
        qd_channel_unit_millivolts,

        /**
         * \brief Electric potential difference in microvolts (uV).
         */
        qd_channel_unit_microvolts,

        /**
         * \brief Tempo in beats per minute (bpm).
         */
        qd_channel_unit_beats_per_minute,

        /**
         * \brief Temperature in degrees celsius (C).
         */
        qd_channel_unit_celsius,

        /**
         * \brief Angular velocity in degrees per second (deg/s).
         */
        qd_channel_unit_degrees_per_second,

        /**
         * \brief Power in watts (W).
         */
        qd_channel_unit_watts,

        /**
         * \brief Magnetic induction in milligauss (mG).
         */
        qd_channel_unit_milligauss,

        /**
         * \brief Unitless measure.
         */
        qd_channel_unit_none
    } qd_channel_unit;

    /**
     * \brief Specifies the binary encoding of sample data.
     */
    typedef enum
    {
        /**
         * \brief Sample data encoded as array of 32-bit signed integers.
         */
        qd_sample_encoding_int32,
        /**
         * \brief Sample data encoded as array of 32-bit single precision IEEE-754 floating point values.
         */
        qd_sample_encoding_float32,
        /**
         * \brief Sample data encoded as array of 8-bit booleans.
         *
         * `0` is false and `1` is true.
         */
        qd_sample_encoding_bool8
    } qd_sample_encoding;

    /**
     * \brief Describes a data sample channel.
     *
     * A channel represents a fixed-frequency and named data stream with
     * specific binary encoding and unit of measurement. The qd_channel also
     * has an possibly empty array of properties for configuration and
     * metadata.
     *
     * Each qd_device has an array of channels.
     */
    struct qd_channel
    {
        /**
         * \brief Name of the channel.
         *
         * Must be non-empty and unique among the channels in the same
         * qd_device. Names must be title case.
         */
        struct qd_chars name;

        /**
         * \brief Binary encoding of the channel's samples.
         */
        qd_sample_encoding sample_encoding;

        /**
         * \brief Unit of measurement of the channel's samples.
         */
        qd_channel_unit unit;

        /**
         * \brief Frequency of the channel's data samples in Hertz.
         *
         * Must be positive.
         */
        float frequency;
    };

    /**
     * \brief Array of qd_channel.
     *
     * May be empty. If ::size is 0 then ::data must also be `NULL`.
     */
    struct qd_channels
    {
        /**
         * \brief Pointer to first item in this array.
         */
        struct qd_channel* data;

        /**
         * \brief Number of items in this array.
         */
        size_t size;
    };

    /**
     * \brief Specifies the category of hardware a device belongs to.
     *
     * Different categories can require specific qd_device::channels and
     * qd_device::properties to enable specific processing in the calling
     * application. These requirements are verified by the QDevice Validator.
     */
    typedef enum
    {
        /**
         * \brief Inertial Measurement Unit (IMU) device.
         */
        qd_device_category_imu,

        /**
         * \brief Electromyography (EMG) device.
         */
        qd_device_category_emg,

        /**
         * \brief Force plate device.
         */
        qd_device_category_force_plate,

        /**
         * \brief Eye tracking device.
         */
        qd_device_category_eye_tracker,

        /**
         * \brief Motion capture finger tracking device.
         */
        qd_device_category_glove,

        /**
         * \brief Uncategorised device.
         */
        qd_device_category_generic
    } qd_device_category;

    /**
     * \brief Describes a device provided by the integration.
     */
    struct qd_device
    {
        /**
         * \brief Category of hardware of the device.
         */
        qd_device_category category;

        /**
         * \brief Unique serial number for the device.
         *
         * Required to be unique among the integration's devices.
         */
        struct qd_chars serial_number;

        /**
         * \brief Array of channels the device provides.
         */
        struct qd_channels channels;

        /**
         * \brief Possibly empty array of properties for the device.
         */
        struct qd_properties properties;
    };

    /**
     * \brief Array of qd_device.
     *
     * May be empty. If ::size is 0 then ::data must also be `NULL`.
     */
    struct qd_devices
    {
        /**
         * \brief Pointer to first item in this array.
         */
        struct qd_device* data;

        /**
         * \brief Number of items in this array.
         */
        size_t size;
    };

    /**
     * \brief Array of binary encoded samples read from a device.
     */
    struct qd_samples
    {
        /**
         * \brief Sample number of the first sample in this array of samples.
         *
         * The sample number is used to relate qd_samples arrays to each other
         * in time using the associated channel's frequency.
         *
         * Sample number does not have to start at 0 after
         * qd_start_streaming(). The calling application must treat the first
         * sample in the first callback from qd_read_samples() after streaming
         * has started as the initial sample.
         */
        uint64_t start_sample_number;

        /**
         * \brief Binary encoding of this array of samples.
         */
        qd_sample_encoding encoding;

        /**
         * \brief Byte pointer to the first sample.
         */
        uint8_t* data;

        /**
         * \brief Number of samples in the array.
         *
         * Note that this is the *sample* count and not the *byte* count.
         */
        size_t size;
    };

    /**
     * \brief Specifies if a function call was successful or returned an error.
     */
    typedef enum
    {
        /**
         * \brief Function returned successfully.
         */
        qd_call_status_type_ok,

        /**
         * \brief Function returned with an error.
         */
        qd_call_status_type_error
    } qd_call_status_type;

    /**
     * \brief Describes API call status, containing error message in case of failure.
     */
    struct qd_call_status
    {
        /**
         * \brief Function call status type.
         */
        qd_call_status_type result;

        /**
         * \brief Error message.
         *
         * Error message can be shown to end user to enable debugging. It is
         * helpful to include information like:
         *
         *  * If error occurred with a specific device, include the device's serial number.
         *  * If network error occurred, include destination IP address.
         *  * If parsing error occurred, include the offending value.
         */
        const char* error_message;
    };

    /**
     * \brief Configuration of the integration.
     *
     * This struct represents the current property values and devices associated with the integration.
     *
     * See qd_properties and qd_devices for more information.
     */
    struct qd_configuration
    {
        /**
         * \brief Properties of the integration.
         *
         * Integration properties are global to the integration and apply to all devices.
         *
         * Examples are "IP Address and "Port Number" properties which can enable
         * user configuration of the service that provides the API for the actual
         * hardware.
         */
        struct qd_properties properties;

        /**
         * \brief Devices currently associated with the integration.
         */
        struct qd_devices devices;
    };

    /**
     * \brief Action function type alias.
     *
     * See qd_action for more information.
     */
    typedef struct qd_call_status(QD_STDCALL* qd_action_fn)(const struct qd_configuration*);

    /**
     * \brief Provides custom function over the QDevice API.
     */
    struct qd_action
    {
        /**
         * \brief Name of the action.
         *
         * Must be non-empty. Names must be title case.
         */
        struct qd_static_chars name;

        /**
         * \brief Pointer to the function.
         *
         * Must be non-null.
         */
        qd_action_fn function;
    };

    /**
     * \brief Array of qd_action.
     *
     * May be empty. If ::size is 0 then ::data must also be `NULL`.
     */
    struct qd_actions
    {
        /**
         * \brief Pointer to first item in this array.
         */
        const struct qd_action* data;

        /**
         * \brief Number of items in this array.
         */
        size_t size;
    };

    /**
     * \brief Describes a contract an end user must accept before the integration can be used.
     */
    struct qd_contract
    {
        /**
         * \brief Title of the contract.
         *
         * Must be non-empty.
         */
        struct qd_static_chars title;

        /**
         * \brief Text of the contract.
         *
         * Must be non-empty.
         */
        struct qd_static_chars text;
    };

    /**
     * \brief Array of qd_contract.
     *
     * May be empty which indicates that there are no contracts the end user
     * must accept. If ::size is 0 then ::contracts must also be `NULL`.
     */
    struct qd_contracts
    {
        /**
         * \brief Pointer to first item in this array.
         */
        const struct qd_contract* contracts;

        /**
         * \brief Number of items in this array.
         */
        size_t size;
    };

    /**
     * \brief Read sample callback function type alias.
     *
     * See qd_read_samples() for more information.
     *
     * \param[out] deviceName device which produced the samples
     * \param[out] channelName channel which the samples belong to
     * \param[out] samples read sample data
     */
    typedef void(QD_STDCALL* qd_sample_push_fn)(const char* deviceName, const char* channelName, struct qd_samples samples);

    /**
     * \name QDevice API functions \{
     */

     /**
      * \brief Get the QDevice API version the integration is implemented for.
      *
      * Implementation provided by qdevice_api.h.
      *
      * \param[out] version pointer to qd_semver where version is written
      * \return Must always return qd_call_status_type::qd_call_status_type_ok.
      */
    QD_API inline struct qd_call_status QD_STDCALL qd_get_api_version(struct qd_semver* version)
    {
        version->major = QD_API_MAJOR;
        version->minor = QD_API_MINOR;
        struct qd_call_status result = { qd_call_status_type_ok, NULL };
        return result;
    }

    /**
     * \brief Enum for specifying the state of the integration.
     */
    typedef enum
    {
        /**
         * \brief The default state. Also the state after qd_disconnect has been successfully called.
         */
        qd_state_disconnected,

        /**
         * \brief qd_connect has been successfully called.
         */
        qd_state_connected,

        /**
         * \brief qd_start_streaming has been successfully called.
         */
        qd_state_streaming
    } qd_state;

    /**
     * \brief Get the library build version information.
     *
     * Must be non-empty.
     * Can include commit id, branch, etc
     * Can be semver.
     * \param[out] version pointer to where the integration build version information is written
     */
    QD_API inline struct qd_call_status QD_STDCALL qd_get_integration_version(struct qd_static_chars* version);

    /**
     * \brief Get the name of the integration.
     *
     * Must be non-empty and unique among all QDevice integrations.
     *
     * \param[out] name pointer to where name is written
     */
    QD_API struct qd_call_status QD_STDCALL qd_get_name(struct qd_static_chars* name);

    /**
     * \brief Get a short description of the integration.
     *
     * \param[out] desc pointer to where description is written
     */
    QD_API struct qd_call_status QD_STDCALL qd_get_description(struct qd_static_chars* desc);

    /**
     * \brief Get an URL to the manufacturer website of the devices being integrated.
     *
     * \param[out] url pointer to where url is written
     */
    QD_API struct qd_call_status QD_STDCALL qd_get_manufacturer_url(struct qd_static_chars* url);

    /**
     * \brief Get an array of end user contracts that end users must accept.
     *
     * \param[out] contracts pointer to where contracts are written
     */
    QD_API struct qd_call_status QD_STDCALL qd_get_contracts(struct qd_contracts* contracts);

    /**
     * \brief Get the category of the integration.
     *
     * \param[out] category pointer to where category is written
     */
    QD_API struct qd_call_status QD_STDCALL qd_get_category(qd_device_category* category);

    /**
     * \brief Perform any load-time initialization that is needed by the integration.
     *
     * This function is optional. If it is implemented, it is called just after loading the integration DLL.
     */
    QD_API struct qd_call_status QD_STDCALL qd_initialize();

    /**
     * \brief Perform any unload-time deinitialization.
     *
     * This function is optional. If it is implemented, it is called just before freeing the integration DLL.
     */
    QD_API struct qd_call_status QD_STDCALL qd_deinitialize();

    /**
     * \brief Get array of actions provided by the integration.
     *
     * May be empty if integration provides no actions.
     *
     * See qd_action for further documentation.
     *
     * \param[out] actions pointer to where the qd_action objects will be written
     */
    QD_API struct qd_call_status QD_STDCALL qd_get_actions(struct qd_actions* actions);


    /**
     * \brief Get the current integration configuration.
     *
     * Either the properties or devices lists may be empty. The memory allocated by this
     * function must not be altered once it is returned to the calling application, with the
     * exception of freeing it via qd_free_configuration.
     *
     * See qd_configuration for further documentation.
     *
     * \param[out] configuration pointer to where the qd_configuration object will be written
     */
    QD_API struct qd_call_status QD_STDCALL qd_get_configuration(struct qd_configuration* configuration);


    /**
     * \brief Free memory allocated by qd_get_configuration
     *
     * \param[in] configuration qd_configuration structure whose resources will be freed
     */
    QD_API struct qd_call_status QD_STDCALL qd_free_configuration(struct qd_configuration* configuration);


    /**
     * \brief Connect to the integration's devices.
     *
     * \param[in] config Integration property and device information to use for connecting.
     *
     * config.properties contains integration properties which should be applied to the hardware.
     * The calling application is expected to provide values that matches the given
     * property definitions. However, integration is required to be resilient
     * against bugs and validate the give input with no assumptions about its
     * correctness.
     *
     * config.devices contains device properties which should be
     * applied to the hardware. For integrations that require each device to be
     * described before a connection can be made, the devices should be described here.
     * For integrations in which the devices available is determined after a connection is
     * made, this can be left as an empty list of qd_devices.
     * The calling application is expected to provide values that matches the given
     * property definitions. However, integration is required to be resilient
     * against bugs and validate the give input with no assumptions about its
     * correctness.
     *
     * Should propagate the relevant properties provided in
     * the function parameters to the device hardware.
     * For example, if the qd_device has a configurable frequency, the setting
     * should be applied by this function after successful connection.
     *
     * * Must return an error if the integration is already connected.
     * * Must return an error if all devices cannot be connected to.
     * * Must return an error if properties cannot be propagated to the devices correctly.
     */
    QD_API struct qd_call_status QD_STDCALL qd_connect(const struct qd_configuration* config);

    /**
     * \brief Disconnect from integration's devices.
     *
     * * Must also perform qd_stop_streaming() if the integration is streaming.
     * * Must return an error if the integration is already disconnected.
     */
    QD_API struct qd_call_status QD_STDCALL qd_disconnect();

    /**
     * \brief Start streaming samples from hardware.
     *
     * All samples generated by the hardware between qd_start_streaming() and
     * qd_stop_streaming() must be readable via calls to qd_read_samples().
     *
     * * Must return an error if streaming could not be started for any connected device.
     * * Must return an error if the integration is already streaming.
     */
    QD_API struct qd_call_status QD_STDCALL qd_start_streaming();

    /**
     * \brief Stop streaming samples from hardware.
     *
     * Note that there can be samples generated by the hardware between the
     * last qd_read_samples() call and the qd_stop_streaming() call. Those
     * samples must be kept by the integration such that the calling
     * application can read them. The calling application must read those
     * samples by making one final qd_read_samples() call after
     * qd_stop_streaming().
     *
     * If the hardware needs additional time to collect the final samples
     * qd_stop_streaming() may block. If blocking, it must block with a timeout
     * of at most 1 second.
     *
     * * Must return an error if the integration is not streaming.
     */
    QD_API struct qd_call_status QD_STDCALL qd_stop_streaming();

    /**
     * \brief Get the current state of the hardware.
     */
    QD_API struct qd_call_status QD_STDCALL qd_get_state(qd_state* state);

    /**
     * \brief Read samples from all devices while streaming.
     *
     * Reads all samples available at the time of the call from all streaming
     * devices. Data is transferred via callback and one callback occurs for
     * each channel in each device, associating the samples with the
     * appropriate device and channel.
     *
     * Must not block. If there are no samples, the callback can be skipped but
     * callback with empty sample array is also allowed.
     *
     * Calling application will generally call this 1-10 times per second but
     * no guarantees are made. Hence, integrations are recommended to support
     * up to 10 seconds of buffering. If buffer limit is reached, integration
     * should stop streaming and subsequent calls to qd_read_samples() should
     * return an error.
     *
     * \param[in] callback the callback function that consumes the samples for the calling application
     */
    QD_API struct qd_call_status QD_STDCALL qd_read_samples(qd_sample_push_fn callback);

    /**
     * /}
     */

#ifdef __cplusplus
}
#endif
