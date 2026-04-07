// ROS2 compatibility stub: rosbag::View - provides minimal API for compilation.
// Actual bag reading requires porting to rosbag2 API.
#ifndef IKALIBR_ROSBAG_VIEW_STUB_H
#define IKALIBR_ROSBAG_VIEW_STUB_H

#include "rosbag/bag.h"
#include "rosbag/message_instance.h"
#include <stdexcept>
#include <vector>
#include <string>

namespace rosbag {

class View {
public:
    View() = default;
    ~View() = default;

    void addQuery(const Bag& bag, const TopicQuery& query) {
        (void)bag; (void)query;
    }

    void addQuery(const Bag& bag, const TopicQuery& query,
                  const ros::Time& startTime, const ros::Time& stopTime) {
        (void)bag; (void)query; (void)startTime; (void)stopTime;
    }

    ros::Time getBeginTime() const { return ros::Time(); }
    ros::Time getEndTime() const { return ros::Time(); }
    std::size_t size() const { return 0; }

    class iterator {
    public:
        using value_type = MessageInstance;
        using reference = const MessageInstance&;
        using pointer = const MessageInstance*;
        using difference_type = std::ptrdiff_t;
        using iterator_category = std::forward_iterator_tag;

        iterator() = default;
        reference operator*() const { return instance_; }
        pointer operator->() const { return &instance_; }
        iterator& operator++() { return *this; }
        iterator operator++(int) { return *this; }
        bool operator==(const iterator&) const { return true; }
        bool operator!=(const iterator&) const { return false; }
    private:
        MessageInstance instance_;
    };

    iterator begin() { return iterator(); }
    iterator end() { return iterator(); }
};

}  // namespace rosbag

#endif  // IKALIBR_ROSBAG_VIEW_STUB_H
