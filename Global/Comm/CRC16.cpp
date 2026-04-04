/*****************************************************************************
*	Name: CRC16.cpp
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Implementation of the CRC16 class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#include "CRC16.h"

//////////////////////////////////////////////////////////////////////
// Construction
//////////////////////////////////////////////////////////////////////
CRC16::CRC16(uint16_t apolynomial, uint16_t init_value /*= 0*/, bool areflect_input /*= true*/, bool areflect_output /*= true*/, uint16_t axor_output /*= 0*/)
{
    polynomial = apolynomial;
    reflect_input = areflect_input;
    reflect_output = areflect_output;
    xor_output = axor_output;
    create_lookup_table();
}
//////////////////////////////////////////////////////////////////////
uint16_t
CRC16::reflect(uint16_t value)
{
    uint16_t reflected = 0;

    for (uint16_t i = 0; i < 16; i++) {
        if (value & 0x01)
            reflected |= (uint16_t)((1 << ((16 - 1) - i)));
        value = (uint16_t)((value >> 1));
    }

    return reflected;
}
void
CRC16::create_lookup_table()
{
    uint16_t x;

    for (uint16_t i = 0; i < 256; i++) {
        x = (uint16_t)(i << 8);
        for (uint16_t j = 0; j < 8; j++) {
            x = (uint16_t)(x & 0x8000 ? (x << 1) ^ polynomial : x << 1);
        }
        lut_crc[i] = x;
    }
}

void
CRC16::display_table()
{
    int index = 0;

    for (int i = 0; i < 16; ++i)
    {
        for (int j = 0; j < 16; ++j)
        {
        std::cout << "0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(4) << lut_crc[index++] << ", ";
        }

        std::cout << "\n";
    }
}


uint16_t
CRC16::calculate(std::vector<uint8_t> data)
{
    uint16_t crc = init_value;

    for (size_t i = 0; i < data.size(); i++) {
        uint8_t byte = reflect_input ? REFLECT_BIT_ORDER_TABLE[data[i]] : data[i];
        crc = (uint16_t)((crc << 8) ^ lut_crc[((crc >> 8) ^ byte)]);
    }

    if (reflect_output) {
        crc = reflect(crc);
    }

    return (crc ^ xor_output) & 0xFFFF;
}


uint16_t
CRC16::calculate(const char* string)
{
	std::vector<uint8_t> data;
	uint16_t result;

	for (size_t i = 0; i < strlen(string); i++) {
		data.push_back(string[i]);
	}

	try
	{
		result = calculate(data);
	}
	catch (std::bad_array_new_length&)
	{
		result = 0;
	}
	catch (...)
	{
		result = 0;
	}

	return result;
}
uint16_t
CRC16::calculate(const uint8_t* data_in, const int length)
{
	std::vector<uint8_t> data;
	uint16_t result;

	for (int i = 0; i < length; i++) {
		data.push_back(data_in[i]);
	}

	try
	{
		result = calculate(data);
	}
	catch (std::bad_array_new_length&)
	{
		result = 0;
	}

	return result;
}