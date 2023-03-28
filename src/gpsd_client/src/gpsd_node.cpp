#include "gpsd_client/gpsd_node.h"
#include "gpsd_client/msg/gps_fix.hpp"

using namespace placeholders;

GpsdNode::GpsdNode()
        : Node("gpsd_node"){

    init_parameters();
    init_interfaces();

    gps_ = new gpsmm("localhost", DEFAULT_GPSD_PORT);

    if(gps_->stream(WATCH_ENABLE | WATCH_JSON) == nullptr){
        RCLCPP_WARN(this->get_logger(), "[gpsd_node] Failed to open GPSd");
        exit(EXIT_FAILURE);
    }
    RCLCPP_INFO(this->get_logger(),"[gpsd_node] GPSd opened");
    gps_->clear_fix();

    timer_ = this->create_wall_timer(
            loop_dt_, std::bind(&GpsdNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "[gpsd_node] Start Ok");
}

GpsdNode::~GpsdNode() {
    if(gps_!=nullptr) {
        gps_->stream(WATCH_DISABLE);
        gps_->~gpsmm();
        free(gps_);
    }
}

void GpsdNode::init_parameters() {
    this->declare_parameter<long>("loop_dt", loop_dt_.count());
    this->declare_parameter<string>("frame_id", frame_id_);
    this->declare_parameter<bool>("publish_when_no_fix", publish_when_no_fix_);

    frame_id_ = this->get_parameter_or("frame_id", frame_id_);
    publish_when_no_fix_ = this->get_parameter_or("publish_when_no_fix", publish_when_no_fix_);
    loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("dt", loop_dt_.count()));
}

void GpsdNode::init_interfaces() {
    publisher_fix_ = this->create_publisher<gpsd_client::msg::GpsFix>("fix", 1);
}

void GpsdNode::timer_callback(){
    struct gps_data_t *p;
    if (!gps_->waiting(2000000)) // us ? => 2s
        return;

    if((p = gps_->read()) == nullptr)
        RCLCPP_WARN(this->get_logger(), "[gpsd_node] Error reading gpsd");
    else
        process_data(p);
}

void GpsdNode::process_data(struct gps_data_t* p) {
    gpsd_client::msg::GpsFix msg;

    msg.header.stamp = this->now();

    msg.header.frame_id = frame_id_;

    msg.status = p->fix.status;
    msg.mode = p->fix.mode;
    msg.time = p->fix.time.tv_sec+p->fix.time.tv_nsec*1e-9;

    if(publish_when_no_fix_ || p->fix.mode >= MODE_2D) {
        msg.latitude = p->fix.latitude;
        msg.longitude = p->fix.longitude;

        msg.altitude = p->fix.altHAE;

        msg.track = p->fix.track;
        msg.speed = p->fix.speed;

        msg.pdop = p->dop.pdop;
        msg.hdop = p->dop.hdop;
        msg.vdop = p->dop.vdop;
        msg.tdop = p->dop.tdop;
        msg.gdop = p->dop.gdop;

        msg.satellites_visible = p->satellites_visible;

        if(!isnan(p->fix.eph))
            msg.err = p->fix.eph;
        if(!isnan(p->fix.epv))
            msg.err_vert = p->fix.epv;
        if(!isnan(p->fix.epd))
            msg.err_track = p->fix.epd;
        if(!isnan(p->fix.eps))
            msg.err_speed = p->fix.eps;
        if(!isnan(p->fix.ept))
            msg.err_time = p->fix.ept;

        publisher_fix_->publish(msg);
        last_msg_no_fix_ = false;
    }

    if(!publish_when_no_fix_ && p->fix.mode < MODE_2D){
        /// Send a last message before stopping sending message when there is no fix
        if(!last_msg_no_fix_)
            publisher_fix_->publish(msg);
        last_msg_no_fix_ = true;
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsdNode>());
    rclcpp::shutdown();
    return 0;
}
