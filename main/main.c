#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joint_state.h>

#include <micro_ros_utilities/type_utilities.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


#define DOMAIN_ID 1
uint64_t time_offset = 0;



rcl_publisher_t publisher;
rcl_subscription_t js_subscriber;
sensor_msgs__msg__JointState js_msg;

std_msgs__msg__Int32 recv_msg;
rcl_subscription_t intsubscriber;
std_msgs__msg__Int32 msg;


void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		printf("Publishing: %d\n", (int) msg.data);
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		msg.data++;
	}
}


void intsubscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
        printf("Received: %d\n",  (int)  msg->data);
}

void js_subscription_callback(const void * msgin)
{
	const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;
         //printf("Received: name size %d\n",  msg->name.size);

         printf("\nJoint State: time: %lf\n",
			 msg->header.stamp.sec +
			 (msg->header.stamp.nanosec)/1.0e9 );

	for ( int i = 0; i < msg->name.size ; i++ ){
           printf("%s: %f %f %f \n", msg->name.data[i].data, 
			           msg->position.data[i],
				   msg->velocity.data[i],
				   msg->effort.data[i] );
	}

}


#define NJOINTS 10
#define NAME_LEN   25

/*
rosidl_runtime_c__String string_buffer[NJOINTS];
char dname[NJOINTS][NAME_LEN];
        char h_buffer[HEADER_LEN];
        double p_buffer[NJOINTS];
        double v_buffer[NJOINTS];
        double e_buffer[NJOINTS];
	*/

void   set_js_msg() {

      static micro_ros_utilities_memory_conf_t conf =
            { .max_string_capacity = NAME_LEN,
              .max_ros2_type_sequence_capacity = NJOINTS,
              .max_basic_type_sequence_capacity = NJOINTS, };

      micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            &js_msg,
            conf);

      return ;

      /*
      js_msg.name.data = string_buffer;
      js_msg.name.size = 0;
      js_msg.name.capacity = NJOINTS;


      for(int i = 0; i < NJOINTS; i++){
            js_msg.name.data[i].data = (char*) &dname[i][0];
            js_msg.name.data[i].size = 0;
            js_msg.name.data[i].capacity = NAME_LEN;
      }


        js_msg.header.frame_id.data = h_buffer;
        js_msg.header.frame_id.size = 0;
        js_msg.header.frame_id.capacity = HEADER_LEN;


        js_msg.position.data = p_buffer;
        js_msg.position.size= 0;
        js_msg.position.capacity = NJOINTS;

        js_msg.velocity.data = v_buffer;
        js_msg.velocity.size = 0;
        js_msg.velocity.capacity = NJOINTS;

        js_msg.effort.data = e_buffer;
        js_msg.effort.size = 0;
        js_msg.effort.capacity = NJOINTS;
	*/

}
	 

uint64_t  millis() {
  return (int) ( esp_timer_get_time()/1e3 );
}

void get_time_offset() {
     uint64_t now = millis();
     uint64_t ros_time_ms = rmw_uros_epoch_millis();
     time_offset = ros_time_ms - now;
     printf("time offsest %lld %lld %lld\n", now, ros_time_ms, time_offset ); 
}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

	printf("entering task\n");

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

	//set init_options
	//RCCHECK(rcl_init_options_set_domain_id(&init_options, (size_t)DOMAIN_ID));
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "servoarm_uros", "", &support));

	printf("node ok\n ");

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"servoarm_uros_pub"));

	// Create int subscriber.
        RCCHECK(rclc_subscription_init_default(
                &intsubscriber,
                &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                "/topic"));

	// Create subscriber.
        RCCHECK(rclc_subscription_init_default(
                &js_subscriber,
                &node,
        	ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                "joint_states"));



	printf("created nodes\n");
	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));


	// create executor
	//rclc_executor_t executor;
        rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

        //unsigned int rcl_wait_timeout = 1000;   // in ms
        //RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	RCCHECK(rclc_executor_add_timer(&executor, &timer));
        RCCHECK(rclc_executor_add_subscription(&executor, &js_subscriber, &js_msg, &js_subscription_callback, ON_NEW_DATA));
        RCCHECK(rclc_executor_add_subscription(&executor, &intsubscriber, &recv_msg, &intsubscription_callback, ON_NEW_DATA));

        set_js_msg();

	RCCHECK(rmw_uros_sync_session(10));
	get_time_offset();

	printf("created executor\n");


	printf("created msg\n");

	printf("first spin\n");

	msg.data = 0;
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// free resources
	RCCHECK(rcl_subscription_fini(&js_subscriber, &node));
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

	printf("creating task dd \n");
    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}

