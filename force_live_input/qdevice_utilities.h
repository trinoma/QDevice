// Copyright 2024 Qualisys AB

#pragma once
#include "./qdevice_api.h"

#include <any>
#include <array>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>

namespace
{
    inline qd_chars create_chars(const std::string& src)
    {
        const auto strSize = src.size();
        const auto bufSize = strSize + 1;
        char* copy = new char[bufSize];
        copy[strSize] = '\0';
        std::memcpy(copy, src.data(), strSize);
        return { copy, strSize };
    }

    inline qd_chars create_chars(const qd_chars& src)
    {
        return create_chars(std::string(src.data));
    }

    inline qd_property_value create_property_value(qd_property_type type, const std::string& src)
    {
        return {
            type,
            create_chars(src)
        };
    }

    inline qd_channels create_channels(std::size_t nChannels)
    {
        return { new qd_channel[nChannels], nChannels };
    }

    inline qd_properties create_properties(std::size_t nProperties)
    {
        return { new qd_property[nProperties], nProperties };
    }

    inline qd_devices create_devices(std::size_t nDevices)
    {
        return { new qd_device[nDevices], nDevices };
    }

    inline void free_chars(qd_chars& string)
    {
        if (string.data)
        {
            delete[] string.data;
        }
        string.data = nullptr;
        string.size = 0;
    }

    inline void free_property_value(qd_property_value& value)
    {
        free_chars(value.value);
    }

    inline void free_property(qd_property& property)
    {
        free_chars(property.name);
        free_chars(property.valid_ranges);
        free_property_value(property.value);
        free_property_value(property.default_value);
    }

    inline void free_properties(qd_properties& properties)
    {
        for (std::size_t iProperty = 0; iProperty < properties.size; iProperty++)
        {
            free_property(properties.data[iProperty]);
        }
        delete[] properties.data;
        properties.data = nullptr;
        properties.size = 0;
    }

    inline void free_channel(qd_channel& channel)
    {
        free_chars(channel.name);
    }

    inline void free_channels(qd_channels& channels)
    {
        for (std::size_t iChannel = 0; iChannel < channels.size; iChannel++)
        {
            free_channel(channels.data[iChannel]);
        }
        delete[] channels.data;
        channels.data = nullptr;
        channels.size = 0;
    }

    inline void free_device(qd_device& device)
    {
        free_properties(device.properties);
        free_channels(device.channels);
        free_chars(device.serial_number);
    }

    inline void free_devices(qd_devices& devices)
    {
        for (std::size_t iDevice = 0; iDevice < devices.size; iDevice++)
        {
            free_device(devices.data[iDevice]);
        }
        delete[] devices.data;
        devices.data = nullptr;
        devices.size = 0;
    }

    std::optional<std::string> find_property_value(const std::string& name, const qd_properties& properties)
    {
        for (std::size_t i = 0; i < properties.size; i++)
        {
            const auto& prop = properties.data[i];
            if (std::string(prop.name.data, prop.name.size) == name)
            {
                return std::string{ prop.value.value.data, prop.value.value.size };
            }
        }

        return {};
    }

    inline std::optional<qd_device> find_device(const std::string& serialNumber, const qd_devices& devices)
    {
        for (std::size_t i = 0; i < devices.size; i++)
        {
            const auto& device = devices.data[i];
            if (std::string(device.serial_number.data, device.serial_number.size) == serialNumber)
            {
                return device;
            }
        }

        return {};
    }
}

namespace QDevice::Utilities
{
    inline qd_property create_property(
        const std::string& name,
        bool readOnly,
        qd_property_type type,
        const std::string& value,
        const std::string& defaultValue,
        const std::string& validRanges)
    {
        return {
            create_chars(name),
            readOnly,
            create_property_value(type, value),
            create_property_value(type, defaultValue),
            create_chars(validRanges)
        };
    }

    inline qd_channel create_channel(
        const std::string& name,
        qd_sample_encoding dataType,
        qd_channel_unit unit,
        float frequency)
    {
        return {
            create_chars(name),
            dataType,
            unit,
            frequency
        };
    }

    inline qd_device create_device(
        qd_device_category category,
        const std::string& serialNumber,
        std::size_t nChannels,
        std::size_t nProperties)
    {
        return {
            category,
            create_chars(serialNumber),
            create_channels(nChannels),
            create_properties(nProperties)
        };
    }

    inline qd_configuration create_configuration(std::size_t nProperties, std::size_t nDevices)
    {
        return {
            create_properties(nProperties),
            create_devices(nDevices)
        };
    }

    inline void free_configuration(qd_configuration& config)
    {
        free_properties(config.properties);
        free_devices(config.devices);
    }

    template<typename value_type_t>
    inline void try_assign_property_value(const std::string& name, const qd_properties& properties, value_type_t& assignTo)
    {
        if (const auto value = find_property_value(name, properties))
        {
            assignTo = *value;
        }
    }

    inline void try_assign_property_value(const std::string& name, const qd_properties& properties, std::int32_t& assignTo)
    {
        if (const auto value = find_property_value(name, properties))
        {
            assignTo = std::stoi(*value);
        }
    }

    inline void try_assign_property_value(const std::string& name, const qd_properties& properties, float& assignTo)
    {
        if (const auto value = find_property_value(name, properties))
        {
            assignTo = std::stof(*value);
        }
    }

    inline void try_assign_property_value(const std::string& name, const qd_properties& properties, bool& assignTo)
    {
        if (const auto value = find_property_value(name, properties))
        {
            if (*value == "true")
            {
                assignTo = true;
            }
            else if (*value == "false")
            {
                assignTo = false;
            }
            else
            {
                throw std::runtime_error(std::string("Invalid boolean string \"") + *value + "\".");
            }
        }
    }

    template<typename value_type_t>
    inline void try_assign_device_property_value(const std::string& serialNumber, const std::string& propertyName, const qd_devices& devices, value_type_t& assignTo)
    {
        if (const auto device = find_device(serialNumber, devices))
        {
            try_assign_property_value(propertyName, device->properties, assignTo);
        }
    }

    inline std::string to_string(bool b)
    {
        return b ? "true" : "false";
    }
}
