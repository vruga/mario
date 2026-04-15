/*
MIT License

Copyright (c) 2024 Society of Robotics and Automation

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sra_board.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#ifdef CONFIG_MICRO_ROS_ESP_UART_TRANSPORT
#include "driver/uart.h"
// esp32_serial_transport.h must come after micro-ROS headers (needs uxrCustomTransport)
#include "esp32_serial_transport.h"
#else
#include <uros_network_interfaces.h>
#endif

#define pi 3.141592653589

//defined macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Restarting.\n",__LINE__,(int)temp_rc);esp_restart();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define ARRAY_LEN 200
#define JOINT_DOUBLE_LEN 5

//Declaring variables with their respective data types
rcl_subscription_t subscriber;
sensor_msgs__msg__JointState recv_msg;
char test_array[ARRAY_LEN];

// structs to set up servo configurations
servo_config servo_a = {
	.servo_pin = SERVO_A,
	.min_pulse_width = CONFIG_SERVO_A_MIN_PULSEWIDTH,
	.max_pulse_width = CONFIG_SERVO_A_MAX_PULSEWIDTH,
	.max_degree = CONFIG_SERVO_A_MAX_DEGREE,
	.mcpwm_num = MCPWM_UNIT_0,
	.timer_num = MCPWM_TIMER_0,
	.gen = MCPWM_OPR_A,
};

servo_config servo_b = {
	.servo_pin = SERVO_B,
	.min_pulse_width = CONFIG_SERVO_B_MIN_PULSEWIDTH,
	.max_pulse_width = CONFIG_SERVO_B_MAX_PULSEWIDTH,
	.max_degree = CONFIG_SERVO_B_MAX_DEGREE,
	.mcpwm_num = MCPWM_UNIT_0,
	.timer_num = MCPWM_TIMER_0,
	.gen = MCPWM_OPR_B,
};

servo_config servo_c = {
	.servo_pin = SERVO_C,
	.min_pulse_width = CONFIG_SERVO_C_MIN_PULSEWIDTH,
	.max_pulse_width = CONFIG_SERVO_C_MAX_PULSEWIDTH,
	.max_degree = CONFIG_SERVO_C_MAX_DEGREE,
	.mcpwm_num = MCPWM_UNIT_0,
	.timer_num = MCPWM_TIMER_1,
	.gen = MCPWM_OPR_A,
};

servo_config servo_d = {
	.servo_pin = SERVO_D,
	.min_pulse_width = CONFIG_SERVO_D_MIN_PULSEWIDTH,
	.max_pulse_width = CONFIG_SERVO_D_MAX_PULSEWIDTH,
	.max_degree = CONFIG_SERVO_D_MAX_DEGREE,
	.mcpwm_num = MCPWM_UNIT_0,
	.timer_num = MCPWM_TIMER_1,
	.gen = MCPWM_OPR_B,
};


// Callback function defined by user, automatically called whenever new message arrives on subscribed topic 
void subscription_callback(const void * msgin)
{
    
     const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
    
    //To display data on terminal
    printf("Received: %lf\n",  msg->position.data[0]);
    printf("Received: %lf\n",  msg->position.data[1]);
    printf("Received: %lf\n",  msg->position.data[2]);
    printf("Received: %lf\n",  msg->position.data[3]);

    //To control motors according to the data
    set_angle_servo(&servo_a,msg->position.data[0]*180/pi);
    set_angle_servo(&servo_b,msg->position.data[1]*180/pi);
    set_angle_servo(&servo_c,msg->position.data[2]*180/pi);
    set_angle_servo(&servo_d,msg->position.data[3]*180/pi);
}

void micro_ros_task(void * arg)
{
	memset(test_array,'z',ARRAY_LEN);
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;
	enable_servo();

	// Wait until the micro-ROS agent is reachable.
	printf("Waiting for micro-ROS agent...\n");
	while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
		vTaskDelay(pdMS_TO_TICKS(500));
	}
	printf("Agent found!\n");

	// Init support, node, subscriber, executor — restart ESP32 on any failure
	// so the transport stack starts completely fresh on the next boot.
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
		rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
		RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	#endif
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
#else
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
#endif

	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "joint_state_sub", "", &support));

	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
		"/joint_states"));

	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(1000)));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));

	// Set up message receive buffers.
	rosidl_runtime_c__String string_buffer[JOINT_DOUBLE_LEN];
	recv_msg.name.data = string_buffer;
	recv_msg.name.size = 0;
	recv_msg.name.capacity = JOINT_DOUBLE_LEN;
	for (int i = 0; i < JOINT_DOUBLE_LEN; i++) {
		recv_msg.name.data[i].data = (char*) malloc(ARRAY_LEN);
		recv_msg.name.data[i].size = 0;
		recv_msg.name.data[i].capacity = ARRAY_LEN;
	}
	recv_msg.position.data = (double*) malloc(JOINT_DOUBLE_LEN * sizeof(double));
	recv_msg.position.size = 0;
	recv_msg.position.capacity = JOINT_DOUBLE_LEN;
	recv_msg.velocity.data = (double*) malloc(JOINT_DOUBLE_LEN * sizeof(double));
	recv_msg.velocity.size = 0;
	recv_msg.velocity.capacity = JOINT_DOUBLE_LEN;

	printf("Running.\n");
	rclc_executor_spin(&executor);

	// Executor returned — agent disconnected. Restart ESP32 for a clean reconnect.
	printf("Connection lost. Restarting...\n");
	esp_restart();
}

void app_main(void)
{
#ifdef CONFIG_MICRO_ROS_ESP_UART_TRANSPORT
    // Wired USB serial transport: plug ESP32 into Mac, run MicroXRCEAgent on the same port.
    static size_t uart_port = UART_NUM_0;
    rmw_uros_set_custom_transport(
        true,
        (void *) &uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read
    );
#elif defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}
