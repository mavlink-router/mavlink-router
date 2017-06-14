#include <fastcdr/Cdr.h>

#include "sensor_combined_Publisher.h"
#include "vehicle_command_ack_Publisher.h"
#include "vehicle_command_Subscriber.h"

class RtpsTopics {
public:
    bool init();
    void publish(char topic_ID, char data_buffer[], size_t len);
    // Get next available topic and put it on scdr
    // return false when all topics were iterated or when no more
    // topics have messages
    // TODO could this be a proper C++ iterator?
    bool nextMsg(char *topic_ID, eprosima::fastcdr::Cdr &scdr);

private:
    // Publishers
    sensor_combined_Publisher _sensor_combined_pub;
    vehicle_command_ack_Publisher _vehicle_command_ack_pub;

    // Subscribers
    vehicle_command_Subscriber _vehicle_command_sub;

    unsigned _next_sub_idx = 0;
    char _sub_topics[1] = {
        80, // vehicle_command_sub
    };
};
