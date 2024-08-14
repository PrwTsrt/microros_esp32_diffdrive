#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "math.h"
#include <stdlib.h>
#include <time.h>

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
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>
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
rcl_subscription_t bumper_state_subscriber;
rcl_publisher_t publisher_imu;
rcl_publisher_t publisher_odom;
//////////////////////////////////////////////////////
// rcl_publisher_t publisher_range_left, publisher_range_center, publisher_range_right, publisher_bump_state;
/////////////////////////////////////////////////////
sensor_msgs__msg__Imu msg_imu;
geometry_msgs__msg__Twist twist_msg;
nav_msgs__msg__Odometry msg_odom;
std_msgs__msg__Bool msg_bumper;
//////////////////////////////////////////////////////
// sensor_msgs__msg__Range msg_range_left, msg_range_center, msg_range_right;
/////////////////////////////////////////////////////

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer_imu;
rcl_timer_t timer_odom;
rcl_timer_t timer_control;
// rcl_timer_t timer_range;

rcl_init_options_t init_options;

unsigned long long time_offset = 0;
unsigned long prev_odom_update = 0;
unsigned long prev_cmd_time = 0;

float x_pos_ = 0.0;
float y_pos_ = 0.0;
float heading_ = 0.0;

car_motion_t car_motion;

// Timeout for each ping attempt
const int timeout_ms = 100;

// Number of ping attempts
const uint8_t attempts = 1;

// Spin period
const unsigned int spin_timeout = RCL_MS_TO_NS(100);

// Enum with connection status
enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
}state ; 

bool bumper_state;
bool flag;

//////////////////////////////////////////////////////
// unsigned long previousMillis = 0;
// float range[3];

// // Initialize msg information for range
// void range_msgs_init(sensor_msgs__msg__Range* range_msg, const char *frame_id_name){
//   range_msg->header.frame_id = micro_ros_string_utilities_set(range_msg->header.frame_id, frame_id_name);
//   range_msg->radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
//   range_msg->field_of_view = 15 * (3.14/180); // rad
//   range_msg->min_range = 0.03; // m
//   range_msg->max_range = 4.0; // m  
// }

// // Range finder update data task
// void range_finder_update_data_task(void *arg)
// {
//   int period = 45;
//   float lower = 0.0;
//   float upper = 1.0;

//   while (1)
//   {
//     // unsigned long currentMillis = millis();
//     // unsigned long currentMillis = esp_timer_get_time()/1000;
    
//     // if(currentMillis - previousMillis > period){
//         // srand(time(NULL));
//         for (int i = 0; i < 3; i++){
//         // range[i] = ((float) rand() / RAND_MAX) * (upper - lower) + lower;
//             range[i] = range[i] + 0.01;
//             if(range[i] > 1.0){
//                 range[i] = 0;
//             }
//         }
//         msg_range_left.range =range[0];
//         msg_range_center.range = range[1];
//         msg_range_right.range = range[2];
//     // }
//     // previousMillis = currentMillis;  
//     vTaskDelay(45 / portTICK_PERIOD_MS);    
//   }
//   vTaskDelete(NULL);
// }
//////////////////////////////////////////////////////

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

    // msg_transform.header.frame_id = micro_ros_string_utilities_set(msg_transform.header.frame_id, frame_id);
    // msg_transform.child_frame_id = micro_ros_string_utilities_set(msg_transform.child_frame_id, child_frame_id);

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

    // msg_transform.transform.translation.x = x_pos_;
    // msg_transform.transform.translation.y = y_pos_;
    // msg_transform.transform.translation.z = 0.0;

    // robot's heading in quaternion
    msg_odom.pose.pose.orientation.x = (double)q[1];
    msg_odom.pose.pose.orientation.y = (double)q[2];
    msg_odom.pose.pose.orientation.z = (double)q[3];
    msg_odom.pose.pose.orientation.w = (double)q[0];

    // msg_transform.transform.rotation.x = (double)q[1];
    // msg_transform.transform.rotation.y = (double)q[2];
    // msg_transform.transform.rotation.z = (double)q[3];
    // msg_transform.transform.rotation.w = (double)q[0];

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

        // msg_transform.header.stamp.sec = time_stamp.tv_sec;
        // msg_transform.header.stamp.nanosec = time_stamp.tv_nsec;

        RCSOFTCHECK(rcl_publish(&publisher_odom, &msg_odom, NULL));
        // RCSOFTCHECK(rcl_publish(&publisher_transform, &msg_transform, NULL));

    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////
// void timer_rangefinder_callback(rcl_timer_t *timer, int64_t last_call_time)
// {
//     RCLC_UNUSED(last_call_time);
//     if (timer != NULL)
//     {
//         RCSOFTCHECK(rcl_publish(&publisher_range_left, &msg_range_left, NULL));
//         RCSOFTCHECK(rcl_publish(&publisher_range_center, &msg_range_center, NULL));
//         RCSOFTCHECK(rcl_publish(&publisher_range_right, &msg_range_right, NULL));
//     }
// }
///////////////////////////////////////////////////////////////////////////////////////////////////////

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

void moveBase()
{   
    float x = 0.0, y = 0.0 , z= 0.0;
    // brake if there's no command received, or when it's only the first command sent
    if(((get_millisecond() - prev_cmd_time) >= 500) || bumper_state) 
    // if(((get_millisecond() - prev_cmd_time) >= 500) || bumper_state || flag == 0) 
    {
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;
        // if(flag == 0)
        // {
        //     flag = 1;
        // }
        // else
        // {
        //     flag = 0;          
        // }
    }
    else{
        x = twist_msg.linear.x;
        y = twist_msg.linear.y;
        z = twist_msg.angular.z;
    }
    Motion_Ctrl(x, y, z);
}

void controlCallback(rcl_timer_t * timer, int64_t last_call_time) 
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
       moveBase();
    }
}

void twist_Callback(const void *msgin)
{
    // ESP_LOGI(TAG, "cmd_vel:%.2f, %.2f, %.2f", twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z);
    // bumper_state ? Motion_Stop(bumper_state) : Motion_Ctrl(twist_msg.linear.x, 0, twist_msg.angular.z);
    // Motion_Ctrl(twist_msg.linear.x, 0, twist_msg.angular.z);
    // if((bumper_state) && (twist_msg.linear.x > 0||twist_msg.angular.z != 0)){twist_msg.linear.x = 0; twist_msg.angular.z = 0;}
    // Motion_Ctrl(twist_msg.linear.x, 0, twist_msg.angular.z);

    prev_cmd_time = get_millisecond();
}

void bumper_Callback(const void *msgin)
{
    bumper_state = msg_bumper.data;
}

bool create_entities(void) {

    init_options = rcl_get_zero_initialized_init_options();
    allocator = rcl_get_default_allocator();

    RCCHECK(rcl_init_options_init(&init_options, allocator));

    size_t domain_id = (size_t)(ROS_DOMAIN_ID);
    RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id));

    // create init_options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

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

    // Create subscriber bumper_state
    RCCHECK(rclc_subscription_init_default(
        &bumper_state_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "bumper_state"));

    ///////////////////////////////////////////////////////////////////////////////////////
    // RCCHECK(rclc_publisher_init_default(
    //     &publisher_range_left,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    //     "range_left"));
    // RCCHECK(rclc_publisher_init_default(
    //     &publisher_range_center,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    //     "range_center"));
    // RCCHECK(rclc_publisher_init_default(
    //     &publisher_range_right,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    //     "range_right"));
    ///////////////////////////////////////////////////////////////////////////////////////

    // create timer. Set the publish frequency to 20HZ
    const unsigned int timer_timeout = 5;
    RCCHECK(rclc_timer_init_default(
        &timer_imu,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_imu_callback));
    RCCHECK(rclc_timer_init_default(
        &timer_control,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        controlCallback));
    RCCHECK(rclc_timer_init_default(
        &timer_odom,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_odom_callback));
    ///////////////////////////////////////////////////////////////////////////////////////
    // RCCHECK(rclc_timer_init_default(
    //     &timer_range,
    //     &support,
    //     RCL_MS_TO_NS(timer_timeout),
    //     timer_rangefinder_callback));
    ///////////////////////////////////////////////////////////////////////////////////////

    // create executor. Three of the parameters are the number of actuators controlled that is greater than or equal to the number of subscribers and publishers added to the executor.
    int handle_num = 5;
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, handle_num, &allocator));
        
    // Adds the publishers's timer to the executor
    RCCHECK(rclc_executor_add_timer(&executor, &timer_imu));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_odom));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_control));
    ///////////////////////////////////////////////////////////////////////////////////////
    // RCCHECK(rclc_executor_add_timer(&executor, &timer_range));
    ///////////////////////////////////////////////////////////////////////////////////////

    // Add a subscriber twist to the executor
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &twist_subscriber,
        &twist_msg,
        &twist_Callback,
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &bumper_state_subscriber,
        &msg_bumper,
        &bumper_Callback,
        ON_NEW_DATA));
        
    sync_time();

    return true;
}

void destroy_entities(void) {

    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // free resources
    RCCHECK(rcl_subscription_fini(&twist_subscriber, &node));
    RCCHECK(rcl_subscription_fini(&bumper_state_subscriber, &node));
    RCCHECK(rcl_publisher_fini(&publisher_odom, &node));
    RCCHECK(rcl_publisher_fini(&publisher_imu, &node));
    ///////////////////////////////////////////////////////////////////////////////////////
    // RCCHECK(rcl_publisher_fini(&publisher_range_left, &node));
    // RCCHECK(rcl_publisher_fini(&publisher_range_center, &node));
    // RCCHECK(rcl_publisher_fini(&publisher_range_right, &node));
    // RCCHECK(rcl_timer_fini(&timer_range));
    ///////////////////////////////////////////////////////////////////////////////////////
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rcl_timer_fini(&timer_imu));
    RCCHECK(rcl_timer_fini(&timer_odom));
    RCCHECK(rcl_timer_fini(&timer_control));
    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rclc_support_fini(&support)); 
    }

// micro ros processes tasks
void micro_ros_task(void *arg)
{
    while (true)
    {
        switch (state)
        {
            case WAITING_AGENT:
                // Check for agent connection
                state = (RMW_RET_OK == rmw_uros_ping_agent(timeout_ms, attempts)) ? AGENT_AVAILABLE : WAITING_AGENT;
                break;

            case AGENT_AVAILABLE:
                // Create micro-ROS entities
                state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;

                if (state == WAITING_AGENT)
                {
                    // Creation failed, release allocated resources
                    destroy_entities();
                };
                break;

            case AGENT_CONNECTED:
                // Check connection and spin on success
                state = (RMW_RET_OK == rmw_uros_ping_agent(timeout_ms, attempts)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
                if (state == AGENT_CONNECTED)
                {
                    RCSOFTCHECK(rclc_executor_spin_some(&executor, spin_timeout));
                }
                break;

            case AGENT_DISCONNECTED:
                // Connection is lost, destroy entities and go back to first step
                destroy_entities();
                state = WAITING_AGENT;
                break;

            default:
                break;
        }
    }

}

static size_t uart_port = UART_NUM_0;

void app_main(void)
{
    Motor_Init();

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

    // range_msgs_init(&msg_range_left, "ranger_left_frame");
    // range_msgs_init(&msg_range_center, "ranger_center_frame");
    // range_msgs_init(&msg_range_right, "ranger_right_frame");

    // Initialize the IMU
    Icm42670p_Init();
    //////////////////////////////////////////////////////
    // imu_ros_init();
    // xTaskCreatePinnedToCore(range_finder_update_data_task,
    //             "range_finder_update_data_task",
    //             CONFIG_MICRO_ROS_APP_STACK,
    //             NULL,
    //             CONFIG_MICRO_ROS_APP_TASK_PRIO,
    //             NULL, 0);
    //////////////////////////////////////////////////////

    xTaskCreatePinnedToCore(micro_ros_task,
                "micro_ros_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL, 1);

    // Start IMU tasks
    xTaskCreatePinnedToCore(imu_update_data_task,
                "imu_update_data_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL, 0);
}
