#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "math.h"

#include "driver/ledc.h"
#include "esp_err.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joint_state.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include "led_strip.h"



#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


#define INTERNAL_LED_GPIO 38
static led_strip_handle_t led_strip;

#define HAND_FRAME_ID ""
#define ARM_FRAME_ID  ""

#define NA_JOINTS 5
#define NH_JOINTS 1
#define NAME_LEN   25

#define DOMAIN_ID 1

#define   TOL_POS  1.0e-4
#define   TOL_VEL  1.0e-8
#define   TOL_EFF  1.0e-8

#define NMOTOR (NA_JOINTS + NH_JOINTS)
#define LEDC_MODE      LEDC_LOW_SPEED_MODE

#define PWM_FREQ  50 


const float pi = 3.14156;
const float deg2rad = pi/180.0 ;

//int motgpio[NMOTOR] = {19, 18, 5, 17, 16, 4}; //for esp32
int motgpio[NMOTOR] = {14, 13, 12, 11, 10, 9}; //for esp32s3
					      
//motor properties
//motor-1 min-duty=206, max-duty=975
//motor-2 min-duty=420, max-duty=1079
const float motor_zero =  0*deg2rad;
const float motor_range = (180+45)*deg2rad;
const float motor_range_ms = 3.757 ; //adjusted value
const float motor_ref = 1.006 ; //adjusted value

float ms_per_rad =  motor_range_ms / ( motor_range);
float duty_per_ms  =  4096 * PWM_FREQ * 1.e-3 ;




uint64_t time_offset = 0;


char *JOINT[] = { "base_joint" ,"platform_joint" ,"lower_joint" ,"top_joint" ,"eef_joint" ,"clawl_joint" ,"clawr_joint"};

double motval[NA_JOINTS + NH_JOINTS] = {0};

rcl_publisher_t arm_js_publisher;
rcl_publisher_t hand_js_publisher;

rcl_subscription_t arm_jc_subscriber;
rcl_subscription_t hand_jc_subscriber;

sensor_msgs__msg__JointState arm_jc_msg;
sensor_msgs__msg__JointState hand_jc_msg;
sensor_msgs__msg__JointState arm_js_msg;
sensor_msgs__msg__JointState hand_js_msg;
sensor_msgs__msg__JointState arm_jsaved_msg;
sensor_msgs__msg__JointState hand_jsaved_msg;

rcl_subscription_t intsubscriber;
rcl_publisher_t intpublisher;
std_msgs__msg__Int32 recv_msg;
std_msgs__msg__Int32 pub_msg;


uint64_t  millis() {
  return (int) ( esp_timer_get_time()/1e3 );
}

void get_js_msg() {

  uint64_t now = millis() + time_offset;

  arm_js_msg.header.stamp.sec = (now / 1e3);
  arm_js_msg.header.stamp.nanosec = (now % 1000) * 1e6;

  hand_js_msg.header.stamp.sec = arm_js_msg.header.stamp.sec ;
  hand_js_msg.header.stamp.nanosec = arm_js_msg.header.stamp.nanosec ;

  for ( int i = 0; i < NA_JOINTS; i++ ) {
      arm_js_msg.position.data[i] = motval[i];
  }

  for ( int i = 0; i < NH_JOINTS; i++ ) {
      hand_js_msg.position.data[i] = motval[NA_JOINTS+i];
  }

}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
        RCLC_UNUSED(last_call_time);

        if (timer != NULL) {
                get_js_msg();
                printf("Publishing: JointState\n" );
                printf("   at time: %ld %ld\n",  arm_js_msg.header.stamp.sec, arm_js_msg.header.stamp.nanosec );
                RCSOFTCHECK(rcl_publish(&arm_js_publisher, &arm_js_msg, NULL));
                RCSOFTCHECK(rcl_publish(&hand_js_publisher, &hand_js_msg, NULL));

        }
}


void movemotor(int i, double angle) {

   //this motor does not move
   if ( i == 6 ) {
     motval[i] = angle ;
     return ;
   }

   printf ("mot %d angin %f %f\n", i, angle/deg2rad, motval[i]/deg2rad );

   if  ( angle  <  (motor_zero - motor_range/2) )  angle =  -(motor_zero - motor_range/2) ;
   if  ( angle  >  (motor_range/2 + motor_zero)  )  angle =   (motor_range/2 + motor_zero) ;

   double tangle = angle + (motor_range/2 - motor_zero); 
   double msprad = ms_per_rad ;
   double mref = motor_ref ;

   if ( i == 1 ) mref =  motor_ref + 1.05 ; //adjusted values
   if ( i == 1 ) msprad = msprad / 1.1765 ; //adjusted values

   int duty =  duty_per_ms * msprad * tangle  ;

   printf ("ms_p_rad %f  duty_per_ms %f  tangle %f\n", ms_per_rad, duty_per_ms ,  tangle )  ;
   duty +=  duty_per_ms*mref ;

   printf ("here mspd %f %f  %f\n", ms_per_rad, angle, duty_per_ms * ms_per_rad*tangle  );
   printf("moving motor: %d angle: %.15g duty: %d\n", i, angle/deg2rad, duty );

   ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, i, duty));
   ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, i));
   motval[i] = angle ;

}

void movebot() {

    //arm 
    for ( int i = 0; i < NA_JOINTS; i++ ) {
      if ( fabs(  arm_jsaved_msg.position.data[i] - arm_js_msg.position.data[i] ) > TOL_POS  ) {
        movemotor(i, arm_jsaved_msg.position.data[i] );
      } else {
        motval[i] = arm_jsaved_msg.position.data[i] ;
      }
    }

    //hand  
    for ( int i = 0; i < NH_JOINTS; i++ ) {
      if ( fabs(  hand_jsaved_msg.position.data[i] - hand_js_msg.position.data[i] ) > TOL_POS  ) {
        movemotor(NA_JOINTS + i, hand_jsaved_msg.position.data[i]);
      } else {
        motval[NA_JOINTS + i] = hand_jsaved_msg.position.data[i] ;
      }
    }
 

}

void jc_subscription_callback(const void * msgin)
{
        const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;

        if ( msg->name.size ==  NA_JOINTS ) printf("\nJoint State: ARM  ");
        if ( msg->name.size ==  NH_JOINTS ) printf("\nJoint State: HAND ");
        printf(" time %lf\n", msg->header.stamp.sec + (msg->header.stamp.nanosec)/1.0e9 );


    int new = 0;


    if ( msg->name.size ==  NA_JOINTS ) {

        for ( int i = 0; i < msg->name.size ; i++ ) {
           if ( fabs(msg->position.data[i] - arm_jsaved_msg.position.data[i] ) > TOL_POS ) {
                  new = 1 ; 
           }
           if ( fabs(msg->velocity.data[i] - arm_jsaved_msg.velocity.data[i] ) > TOL_VEL ) {
                  new = 1 ; 
           }

           if ( fabs(msg->effort.data[i] - arm_jsaved_msg.effort.data[i] ) > TOL_EFF ) {
                  new = 1 ; 
           }
        }

        arm_jsaved_msg = *msg ;
    }


    if ( msg->name.size ==  NH_JOINTS ) {

        for ( int i = 0; i < msg->name.size ; i++ ) {
           if ( fabs(msg->position.data[i] - hand_jsaved_msg.position.data[i] ) > TOL_POS ) {
                  new = 1 ; 
           }
           if ( fabs(msg->velocity.data[i] - hand_jsaved_msg.velocity.data[i] ) > TOL_VEL ) {
                  new = 1 ; 
           }

           if ( fabs(msg->effort.data[i] - hand_jsaved_msg.effort.data[i] ) > TOL_EFF ) {
                  new = 1 ; 
           }
        }

         hand_jsaved_msg = *msg ;
    }

    if ( new ) {
     for ( int i = 0; i < msg->name.size ; i++ ){
         printf("%s: %.15g %.15g %.15g \n", msg->name.data[i].data, 
                                   msg->position.data[i],
                                   msg->velocity.data[i],
                                   msg->effort.data[i] );
     
     }
    }

    movebot();
}

void   init_msg() {

      static micro_ros_utilities_memory_conf_t aconf =
            { .max_string_capacity = NAME_LEN,
              .max_ros2_type_sequence_capacity = NA_JOINTS,
              .max_basic_type_sequence_capacity = NA_JOINTS, };

      static micro_ros_utilities_memory_conf_t hconf =
            { .max_string_capacity = NAME_LEN,
              .max_ros2_type_sequence_capacity = NH_JOINTS,
              .max_basic_type_sequence_capacity = NH_JOINTS, };


      micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            &arm_jc_msg,
            aconf);

      micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            &arm_js_msg,
            aconf);

      micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            &arm_jsaved_msg,
            aconf);

      micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            &hand_jc_msg,
            hconf);

      micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            &hand_js_msg,
            hconf);

      micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            &hand_jsaved_msg,
            hconf);


      arm_js_msg.name.size = arm_js_msg.position.size =  NA_JOINTS; 
      arm_js_msg.velocity.size = NA_JOINTS;
      arm_js_msg.header.frame_id = micro_ros_string_utilities_set(arm_js_msg.header.frame_id, ARM_FRAME_ID);
      arm_js_msg.name.data[0] = micro_ros_string_utilities_set( arm_js_msg.name.data[0], JOINT[0]);
      arm_js_msg.name.data[1] = micro_ros_string_utilities_set( arm_js_msg.name.data[1], JOINT[1]);
      arm_js_msg.name.data[2] = micro_ros_string_utilities_set( arm_js_msg.name.data[2], JOINT[2]);
      arm_js_msg.name.data[3] = micro_ros_string_utilities_set( arm_js_msg.name.data[3], JOINT[3]);
      arm_js_msg.name.data[4] = micro_ros_string_utilities_set( arm_js_msg.name.data[4], JOINT[4]);

      hand_js_msg.name.size = hand_js_msg.position.size =  NH_JOINTS; 
      hand_js_msg.velocity.size = NH_JOINTS;
      hand_js_msg.header.frame_id = micro_ros_string_utilities_set(hand_js_msg.header.frame_id, HAND_FRAME_ID);
      hand_js_msg.name.data[0] = micro_ros_string_utilities_set( hand_js_msg.name.data[0], JOINT[5]);
      if ( NH_JOINTS > 1 ) hand_js_msg.name.data[1] = micro_ros_string_utilities_set( hand_js_msg.name.data[1], JOINT[6]);


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

        printf("created node\n");

        // Create subscriber.
        RCCHECK(rclc_subscription_init_default(
                &arm_jc_subscriber,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                "/arm_joint_commands"));

        RCCHECK(rclc_subscription_init_default(
                &hand_jc_subscriber,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                "/hand_joint_commands"));


        printf("created jc subscriber\n");


        // create publisher
        RCCHECK(rclc_publisher_init_default(
                &arm_js_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                "/arm_joint_states"));

        RCCHECK(rclc_publisher_init_default(
                &hand_js_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                "/hand_joint_states"));

        printf("created js publisher\n");



        // create timer,
        rcl_timer_t timer;
        const unsigned int timer_timeout = 1000;
        RCCHECK(rclc_timer_init_default(
                &timer,
                &support,
                RCL_MS_TO_NS(timer_timeout),
                timer_callback));

        printf("created timer\n");

        // create executor
        rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
        RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

        //unsigned int rcl_wait_timeout = 1000;   // in ms
        //RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
        printf("created executor\n");

        RCCHECK(rclc_executor_add_timer(&executor, &timer));

        RCCHECK(rclc_executor_add_subscription(&executor, &arm_jc_subscriber, &arm_jc_msg, &jc_subscription_callback, ON_NEW_DATA));
        RCCHECK(rclc_executor_add_subscription(&executor, &hand_jc_subscriber, &hand_jc_msg, &jc_subscription_callback, ON_NEW_DATA));


        printf("ready to spin\n");

        init_msg();
        printf("created msg\n");

        RCCHECK( rmw_uros_sync_session(20) );
        get_time_offset();
	led_strip_set_pixel(led_strip, 0, 0, 8, 0);
        led_strip_refresh(led_strip);

        printf("Start spinning\n");
        while(1){
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
                usleep(10000);
        }

        // free resources
        printf("Free resources\n");
        RCCHECK(rcl_subscription_fini(&arm_jc_subscriber, &node));
        RCCHECK(rcl_subscription_fini(&hand_jc_subscriber, &node));
        RCCHECK(rcl_publisher_fini(&arm_js_publisher, &node));
        RCCHECK(rcl_publisher_fini(&hand_js_publisher, &node));

        RCCHECK(rcl_node_fini(&node));

        vTaskDelete(NULL);
}

void ledc_init(void)
{

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = PWM_FREQ,  // Hz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));


    // common channel config 
    ledc_channel_config_t _lchan = { 
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = 0,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config_t ledc_channel[NMOTOR] = { _lchan } ;

    for ( int i = 0; i < NMOTOR; i++ ) {
      ledc_channel[i]  = _lchan ;
      ledc_channel[i].channel  = i;
      ledc_channel[i].gpio_num = motgpio[i];
      ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[i] ));
    }

}



void test_motor() {
    int duty = 0;
    int imot = 5;

    int dd = 0; //duty/angle
		
    duty = 950;
    duty =  duty_per_ms*motor_ref ;
    duty = 420;
    printf("duty start %d\n", duty );
    float angle = -112.5;
    angle = 25;

    vTaskDelay(pdMS_TO_TICKS(3000));

    while (1) {

	if ( dd ) {
	   printf("duty now %d\n",duty);
           ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, imot, duty));
           ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, imot));
           //if( duty == 310 ){ duty = 221;}
	   duty -= 5;
	} else {
	   printf("angle now %f\n", angle);
           movemotor(imot, angle*deg2rad);
           //if( angle == -112.5 ){ angle = -106;}
	   angle += 2 ;
	}

	vTaskDelay(pdMS_TO_TICKS(5000));

	
    }

}

static void led_init(void)
{

    led_strip_config_t strip_config = {
        .strip_gpio_num = INTERNAL_LED_GPIO,
        .max_leds = 1, 
	//.led_pixel_format = LED_PIXEL_FORMAT_GRB, 
        //.led_model = LED_MODEL_WS2812, 
        //.flags.invert_out = false, 
    };


    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        //.clk_src = RMT_CLK_SRC_DEFAULT,
        //.flags.with_dma = false, 
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
}

void app_main(void)
{
    led_init();
    led_strip_set_pixel(led_strip, 0, 8, 0, 0);
    led_strip_refresh(led_strip);

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif
    ledc_init();

    //test_motor();

    xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}

