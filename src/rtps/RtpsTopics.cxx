#include "RtpsTopics.h"

bool RtpsTopics::init()
{
    // Initialise subscribers
    if (_vehicle_command_sub.init()) {
        std::cout << "vehicle_command subscriber started" << std::endl;
    } else {
        std::cout << "ERROR starting vehicle_command subscriber" << std::endl;
        return false;
    }

    // Initialise publishers
    if (_sensor_combined_pub.init()) {
        std::cout << "sensor_combined publisher started" << std::endl;
    } else {
        std::cout << "ERROR starting sensor_combined publisher" << std::endl;
        return false;
    }

    if (_vehicle_command_ack_pub.init()) {
        std::cout << "vehicle_command_ack publisher started" << std::endl;
    } else {
        std::cout << "ERROR starting vehicle_command_ack publisher" << std::endl;
        return false;
    }

    return true;
}

void RtpsTopics::publish(char topic_ID, char data_buffer[], size_t len)
{
    switch (topic_ID)
    {
        case 58:
        {
//                        printf("sensor-combined\n");
//                        printf("buf %s\n", data_buffer);
            sensor_combined_ st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            _sensor_combined_pub.publish(&st);
            //printf("                >>[%d] %lu\n", <built-in function id>, st.timestamp());
        }
        break;
        case 79:
        {
            vehicle_command_ack_ st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            _vehicle_command_ack_pub.publish(&st);
            //printf("                >>[%d] %lu\n", <built-in function id>, st.timestamp());
        }
        break;
    }
}

bool RtpsTopics::nextMsg(char *topic_ID, eprosima::fastcdr::Cdr &scdr)
{
    while (_next_sub_idx < sizeof(_sub_topics)/sizeof(_sub_topics[0])) {
        switch (_sub_topics[_next_sub_idx]) {
        case 80: // vehicle_command
            if (_vehicle_command_sub.hasMsg()) {
                vehicle_command_ msg = _vehicle_command_sub.getMsg();
                msg.serialize(scdr);
                *topic_ID = 80;
                goto ok;
            }
            break;
        }
        _next_sub_idx++;
    }

    _next_sub_idx = 0;
    return false;

ok:
    _next_sub_idx++;
    return true;
}
