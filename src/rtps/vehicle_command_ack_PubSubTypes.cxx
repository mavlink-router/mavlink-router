// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*! 
 * @file vehicle_command_ack_PubSubTypes.cpp
 * This header file contains the implementation of the serialization functions.
 *
 * This file was generated by the tool fastcdrgen.
 */


#include <fastcdr/FastBuffer.h>
#include <fastcdr/Cdr.h>

#include "vehicle_command_ack_PubSubTypes.h"

vehicle_command_ack_PubSubType::vehicle_command_ack_PubSubType() {
    setName("vehicle_command_ack_");
    m_typeSize = (uint32_t)vehicle_command_ack_::getMaxCdrSerializedSize() + 4 /*encapsulation*/;
    m_isGetKeyDefined = vehicle_command_ack_::isKeyDefined();
    m_keyBuffer = (unsigned char*)malloc(vehicle_command_ack_::getKeyMaxCdrSerializedSize()>16 ? vehicle_command_ack_::getKeyMaxCdrSerializedSize() : 16);
}

vehicle_command_ack_PubSubType::~vehicle_command_ack_PubSubType() {
    if(m_keyBuffer!=nullptr)
        free(m_keyBuffer);
}

bool vehicle_command_ack_PubSubType::serialize(void *data, SerializedPayload_t *payload) {
    vehicle_command_ack_ *p_type = (vehicle_command_ack_*) data;
    eprosima::fastcdr::FastBuffer fastbuffer((char*) payload->data, payload->max_size); // Object that manages the raw buffer.
    eprosima::fastcdr::Cdr ser(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
            eprosima::fastcdr::Cdr::DDS_CDR); // Object that serializes the data.
    payload->encapsulation = ser.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
    // Serialize encapsulation
    ser.serialize_encapsulation();
    p_type->serialize(ser); // Serialize the object:
    payload->length = (uint32_t)ser.getSerializedDataLength(); //Get the serialized length
    return true;
}

bool vehicle_command_ack_PubSubType::deserialize(SerializedPayload_t* payload, void* data) {
    vehicle_command_ack_* p_type = (vehicle_command_ack_*) data; 	//Convert DATA to pointer of your type
    eprosima::fastcdr::FastBuffer fastbuffer((char*)payload->data, payload->length); // Object that manages the raw buffer.
    eprosima::fastcdr::Cdr deser(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
            eprosima::fastcdr::Cdr::DDS_CDR); // Object that deserializes the data.
    // Deserialize encapsulation.
    deser.read_encapsulation();
    payload->encapsulation = deser.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
    p_type->deserialize(deser); //Deserialize the object:
    return true;
}

std::function<uint32_t()> vehicle_command_ack_PubSubType::getSerializedSizeProvider(void* data) {
    return [data]() -> uint32_t
    {
        return (uint32_t)type::getCdrSerializedSize(*static_cast<vehicle_command_ack_*>(data)) + 4 /*encapsulation*/;
    };
}

void* vehicle_command_ack_PubSubType::createData() {
    return (void*)new vehicle_command_ack_();
}

void vehicle_command_ack_PubSubType::deleteData(void* data) {
    delete((vehicle_command_ack_*)data);
}

bool vehicle_command_ack_PubSubType::getKey(void *data, InstanceHandle_t* handle) {
    if(!m_isGetKeyDefined)
        return false;
    vehicle_command_ack_* p_type = (vehicle_command_ack_*) data;
    eprosima::fastcdr::FastBuffer fastbuffer((char*)m_keyBuffer,vehicle_command_ack_::getKeyMaxCdrSerializedSize()); 	// Object that manages the raw buffer.
    eprosima::fastcdr::Cdr ser(fastbuffer, eprosima::fastcdr::Cdr::BIG_ENDIANNESS); 	// Object that serializes the data.
    p_type->serializeKey(ser);
    if(vehicle_command_ack_::getKeyMaxCdrSerializedSize()>16)	{
        m_md5.init();
        m_md5.update(m_keyBuffer,(unsigned int)ser.getSerializedDataLength());
        m_md5.finalize();
        for(uint8_t i = 0;i<16;++i)    	{
            handle->value[i] = m_md5.digest[i];
        }
    }
    else    {
        for(uint8_t i = 0;i<16;++i)    	{
            handle->value[i] = m_keyBuffer[i];
        }
    }
    return true;
}

