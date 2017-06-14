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
 * @file vehicle_command_ack_.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "vehicle_command_ack_.h"

#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>

vehicle_command_ack_::vehicle_command_ack_()
{
    m_command = 0;
    m_result = 0;
}

vehicle_command_ack_::~vehicle_command_ack_()
{
}

vehicle_command_ack_::vehicle_command_ack_(const vehicle_command_ack_ &x)
{
    m_command = x.m_command;
    m_result = x.m_result;
}

vehicle_command_ack_::vehicle_command_ack_(vehicle_command_ack_ &&x)
{
    m_command = x.m_command;
    m_result = x.m_result;
}

vehicle_command_ack_& vehicle_command_ack_::operator=(const vehicle_command_ack_ &x)
{
    m_command = x.m_command;
    m_result = x.m_result;
    
    return *this;
}

vehicle_command_ack_& vehicle_command_ack_::operator=(vehicle_command_ack_ &&x)
{
    m_command = x.m_command;
    m_result = x.m_result;
    
    return *this;
}

size_t vehicle_command_ack_::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;
            
    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);

    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    return current_alignment - initial_alignment;
}

size_t vehicle_command_ack_::getCdrSerializedSize(const vehicle_command_ack_& data, size_t current_alignment)
{
    size_t initial_alignment = current_alignment;
            
    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);

    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    return current_alignment - initial_alignment;
}

void vehicle_command_ack_::serialize(eprosima::fastcdr::Cdr &scdr) const
{
    scdr << m_command;

    scdr << m_result;

}

void vehicle_command_ack_::deserialize(eprosima::fastcdr::Cdr &dcdr)
{
    dcdr >> m_command;
    dcdr >> m_result;
}

size_t vehicle_command_ack_::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
	size_t current_align = current_alignment;
            



    return current_align;
}

bool vehicle_command_ack_::isKeyDefined()
{
    return false;
}

void vehicle_command_ack_::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
	 
	 
}