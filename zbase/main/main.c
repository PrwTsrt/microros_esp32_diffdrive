#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/uart.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/string_utilities.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <nav_msgs/msg/odometry.h>

#include "esp32_serial_transport.h"
#include "car_motion.h"
#include "icm42670p.h"


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}



#define ROS_NAMESPACE      CONFIG_MICRO_ROS_NAMESPACE
#define ROS_DOMAIN_ID      CONFIG_MICRO_ROS_DOMAIN_ID
#define ROS_AGENT_IP       CONFIG_MICRO_ROS_AGENT_IP
#define ROS_AGENT_PORT     CONFIG_MICRO_ROS_AGENT_PORT



static const char *TAG = "MAIN";


rcl_subscription_t twist_subscriber;
rcl_publisher_t publisher_imu;
rcl_publisher_t publisher_odom;
sensor_msgs__msg__Imu msg_imu;
geometry_msgs__msg__Twist twist_msg;
nav_msgs__msg__Odometry msg_odom;

rcl_timer_t timer_imu;
rcl_timer_t timer_odom;

unsigned long long time_offset = 0;
unsigned long prev_odom_update = 0;

float x_pos_ = 0.0;
float y_pos_ = 0.0;
float heading_ = 0.0;

car_motion_t car_motion;

// Initializes the ROS topic information for odom
void odom_ros_init(void)
{
    char* content_frame_id = "odom_frame";
    char* content_child_frame_id = "base_footprint";
    int len_namespace = strlen(ROS_NAMESPACE);
    int len_frame_id_max = len_namespace + strlen(content_frame_id) + 2;
    int len_child_frame_id_max = len_namespace + strlen(content_child_frame_id) + 2;
    char* frame_id = malloc(len_frame_id_max);
    char* child_frame_id = malloc(len_child_frame_id_max);
    if (len_namespace == 0)
    {
        // The ROS namespace is empty characters
        sprintf(frame_id, "%s", content_frame_id);
        sprintf(child_frame_id, "%s", content_child_frame_id);
    }
    else
    {
        // Concatenate the namespace and frame id
        sprintf(frame_id, "%s/%s", ROS_NAMESPACE, content_frame_id);
        sprintf(child_frame_id, "%s/%s", ROS_NAMESPACE, content_child_frame_id);
    }
    msg_odom.header.frame_id = micro_ros_string_utilities_set(msg_odom.header.frame_id, frame_id);
    msg_odom.child_frame_id = micro_ros_string_utilities_set(msg_odom.child_frame_id, child_frame_id);
    free(frame_id);
    free(child_frame_id);
}

// Euler's angular revolution quaternion
void odom_euler_to_quat(float roll, float pitch, float yaw, float *q)
{
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    q[0] = cy * cp * cr + sy * sp * sr;
    q[1] = cy * cp * sr - sy * sp * cr;
    q[2] = sy * cp * sr + cy * sp * cr;
    q[3] = sy * cp * cr - cy * sp * sr;
}

// Update odom data
void odom_update(float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z)
{
    float delta_heading = angular_vel_z * vel_dt; // radians
    float cos_h = cos(heading_);
    float sin_h = sin(heading_);
    float delta_x = (linear_vel_x * cos_h - linear_vel_y * sin_h) * vel_dt; // m
    float delta_y = (linear_vel_x * sin_h + linear_vel_y * cos_h) * vel_dt; // m

    // calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    // calculate robot's heading in quaternion angle
    // ROS has a function to calculate yaw in quaternion angle
    float q[4];
    odom_euler_to_quat(0, 0, heading_, q);

    // robot's position in x,y, and z
    msg_odom.pose.pose.position.x = x_pos_;
    msg_odom.pose.pose.position.y = y_pos_;
    msg_odom.pose.pose.position.z = 0.0;

    // robot's heading in quaternion
    msg_odom.pose.pose.orientation.x = (double)q[1];
    msg_odom.pose.pose.orientation.y = (double)q[2];
    msg_odom.pose.pose.orientation.z = (double)q[3];
    msg_odom.pose.pose.orientation.w = (double)q[0];

    msg_odom.pose.covariance[0] = 0.001;
    msg_odom.pose.covariance[7] = 0.001;
    msg_odom.pose.covariance[35] = 0.001;

    // linear speed from encoders
    msg_odom.twist.twist.linear.x = linear_vel_x;
    msg_odom.twist.twist.linear.y = linear_vel_y;
    msg_odom.twist.twist.linear.z = 0.0;

    // angular speed from encoders
    msg_odom.twist.twist.angular.x = 0.0;
    msg_odom.twist.twist.angular.y = 0.0;
    msg_odom.twist.twist.angular.z = angular_vel_z;

    msg_odom.twist.covariance[0] = 0.0001;
    msg_odom.twist.covariance[7] = 0.0001;
    msg_odom.twist.covariance[35] = 0.0001;
}


// Initializes the ROS topic information for IMU
void imu_ros_init(void)
{
    msg_imu.angular_velocity.x = 0;
    msg_imu.angular_velocity.y = 0;
    msg_imu.angular_velocity.z = 0;

    msg_imu.linear_acceleration.x = 0;
    msg_imu.linear_acceleration.y = 0;
    msg_imu.linear_acceleration.z = 0;

    msg_imu.orientation.x = 0;
    msg_imu.orientation.y = 0;
    msg_imu.orientation.z = 0;
    msg_imu.orientation.w = 1;

    char* content_frame_id = "imu_frame";
    int len_namespace = strlen(ROS_NAMESPACE);
    int len_frame_id_max = len_namespace + strlen(content_frame_id) + 2;
    // ESP_LOGI(TAG, "imu frame len:%d", len_frame_id_max);
    char* frame_id = malloc(len_frame_id_max);
    if (len_namespace == 0)
    {
        // The ROS namespace is empty characters
        sprintf(frame_id, "%s", content_frame_id);
    }
    else
    {
        // Concatenate the namespace and frame id
        sprintf(frame_id, "%s/%s", ROS_NAMESPACE, content_frame_id);
    }
    msg_imu.header.frame_id = micro_ros_string_utilities_set(msg_imu.header.frame_id, frame_id);
    free(frame_id);
}

// IMU update data task
void imu_update_data_task(void *arg)
{
    int16_t gyro_raw[3] = {0};
    int16_t accel_raw[3] = {0};
    float imu_accel_g[3] = {0};
    float imu_gyro_dps[3] = {0};

    while (1)
    {
        Icm42670p_Get_Gyro_RawData(gyro_raw);
        Icm42670p_Get_Accel_RawData(accel_raw);
        Icm42670p_Get_Accel_g(imu_accel_g);
        Icm42670p_Get_Gyro_dps(imu_gyro_dps);
        msg_imu.angular_velocity.x = imu_gyro_dps[0];
        msg_imu.angular_velocity.y = imu_gyro_dps[1];
        msg_imu.angular_velocity.z = imu_gyro_dps[2];

        msg_imu.linear_acceleration.x = imu_accel_g[0];
        msg_imu.linear_acceleration.y = imu_accel_g[1];
        msg_imu.linear_acceleration.z = imu_accel_g[2];
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelete(NULL);
}

// Gets the number of seconds since boot
unsigned long get_millisecond(void)
{
    return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

// Calculate the time difference between the microROS agent and the MCU
static void sync_time(void)
{
    unsigned long now = get_millisecond();
    RCSOFTCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
}

// Get timestamp
struct timespec get_timespec(void)
{
    struct timespec tp = {0};
    // deviation of synchronous time
    unsigned long long now = get_millisecond() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

// Timer callback function
void timer_odom_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        struct timespec time_stamp = get_timespec();
        unsigned long now = get_millisecond();
        float vel_dt = (now - prev_odom_update) / 1000.0;
        prev_odom_update = now;
        Motion_Get_Speed(&car_motion);
        odom_update(
            vel_dt,
            car_motion.Vx,
            car_motion.Vy,
            car_motion.Wz);
        msg_odom.header.stamp.sec = time_stamp.tv_sec;
        msg_odom.header.stamp.nanosec = time_stamp.tv_nsec;
        RCSOFTCHECK(rcl_publish(&publisher_odom, &msg_odom, NULL));
    }
}

// Timer callback function
void timer_imu_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        struct timespec time_stamp = get_timespec();
        msg_imu.header.stamp.sec = time_stamp.tv_sec;
        msg_imu.header.stamp.nanosec = time_stamp.tv_nsec;
        RCSOFTCHECK(rcl_publish(&publisher_imu, &msg_imu, NULL));
    }
}

void twist_Callback(const void *msgin)
{
    ESP_LOGI(TAG, "cmd_vel:%.2f, %.2f, %.2f", twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z);
    Motion_Ctrl(twist_msg.linear.x, 0, twist_msg.angular.z);
}

// micro ros processes tasks
void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    
    // create ROS2 node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "base_controller", ROS_NAMESPACE, &support));

    // create publisher_odom
    RCCHECK(rclc_publisher_init_default(
        &publisher_odom,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom_raw"));

    // create publisher_imu
    RCCHECK(rclc_publisher_init_default(
        &publisher_imu,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu"));

    // Create subscriber cmd_vel
    RCCHECK(rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // create timer. Set the publish frequency to 20HZ
    const unsigned int timer_timeout = 50;
    RCCHECK(rclc_timer_init_default(
        &timer_imu,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_imu_callback));
    RCCHECK(rclc_timer_init_default(
        &timer_odom,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_odom_callback));

    // create executor. Three of the parameters are the number of actuators controlled that is greater than or equal to the number of subscribers and publishers added to the executor.
    rclc_executor_t executor;
    int handle_num = 3;
    RCCHECK(rclc_executor_init(&executor, &support.context, handle_num, &allocator));
    
    // Adds the publishers's timer to the executor
    RCCHECK(rclc_executor_add_timer(&executor, &timer_imu));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_odom));

    // Add a subscriber twist to the executor
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &twist_subscriber,
        &twist_msg,
        &twist_Callback,
        ON_NEW_DATA));
    
    sync_time();

    // Loop the microROS task
    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(1000);
    }

    // free resources
    RCCHECK(rcl_subscription_fini(&twist_subscriber, &node));
    RCCHECK(rcl_publisher_fini(&publisher_odom, &node));
    RCCHECK(rcl_publisher_fini(&publisher_imu, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

static size_t uart_port = UART_NUM_0;

void app_main(void)
{
    Motor_Init();

    // Initialize the network and connect the WiFi signal
    // ESP_ERROR_CHECK(uros_network_interface_initialize());

#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
	rmw_uros_set_custom_transport(
		true,
		(void *) &uart_port,
		esp32_serial_open,
		esp32_serial_close,
		esp32_serial_write,
		esp32_serial_read
	);
#else
#error micro-ROS transports misconfigured
#endif  // RMW_UXRCE_TRANSPORT_CUSTOM

    odom_ros_init();

    // Initialize the IMU
    Icm42670p_Init();
    imu_ros_init();

    // Start microROS tasks
    xTaskCreate(micro_ros_task,
                "micro_ros_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);

    // Start IMU tasks
    xTaskCreatePinnedToCore(imu_update_data_task,
                "imu_update_data_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL, 1);
}
