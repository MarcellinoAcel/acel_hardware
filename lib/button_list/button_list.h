#ifndef BUTTON_LIST_H
#define BUTTON_LIST_H
#include <Arduino.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

/*
button A = 0
button B = 1
button X = 3
button Y = 4
button RT = 9
button LT = 8
button LB = 6
button RB = 7
*/

class Button_list
{
private:
    rcl_subscription_t button_sub;
    std_msgs__msg__Int32MultiArray button_msg;
    unsigned long prev_cmd_time = 0;
    static void button_callback(const void *msg_in)
    {

        digitalWrite(13, !digitalRead(13));
        // prev_cmd_time = millis();
        // const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msg_in;
        // Button_list *instance = (Button_list *)arg;
        // for (size_t i = 0; i < msg->data.size; i++)
        // {

        // printf("Button[%zu]: %ld\n", i, msg->data.data[i]);
        // }
        // for (size_t i = 0; i < msg->data.size; ++i)
        // {
        //     switch (i)
        //     {
        //     case 0:
        //         instance->button.A = msg->data.data[0];
        //         break;
        //     case 1:
        //         instance->button.B = msg->data.data[1];
        //         break;
        //     case 3:
        //         instance->button.X = msg->data.data[3];
        //         break;
        //     case 4:
        //         instance->button.Y = msg->data.data[4];
        //         break;
        //     case 10:
        //         instance->button.RT = msg->data.data[9];
        //         break;
        //     case 9:
        //         instance->button.LT = msg->data.data[8];
        //         break;
        //     case 7:
        //         instance->button.LB = msg->data.data[6];
        //         break;
        //     case 8:
        //         instance->button.RB = msg->data.data[7];
        //         break;
        //     default:
        //         break;
        //     }
        // }
    }

public:
    struct but
    {
        int A;
        int B;
        int X;
        int Y;
        int RT;
        int LT;
        int LB;
        int RB;
    } button;
    Button_list()
    {
        button_msg.data.data = NULL;
        button_msg.data.size = 0;
        button_msg.data.capacity = 0;
    }

    bool init(rcl_node_t *node, rclc_executor_t *executor, rcl_allocator_t *allocator,rcl_timer_t *control_timer)
    {
        // Initialize the message
        button_msg.data.data = (int32_t *)malloc(10 * sizeof(int32_t)); // Allocate space for the array
        button_msg.data.capacity = 10;
        button_msg.data.size = 0;

        // Initialize the subscription
        if (rclc_subscription_init_default(
                &button_sub,
                node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
                "button") != RCL_RET_OK)
        {
            // printf("Failed to create button subscription\n");
            return false;
        }

        // Add the subscription to the executor
        if (rclc_executor_add_subscription(
                executor,
                &button_sub,
                &button_msg,
                &button_callback,
                ON_NEW_DATA) != RCL_RET_OK)
        {
            // printf("Failed to add button subscription to executor\n");
            return false;
        }

        if(rclc_executor_add_timer(executor, control_timer) != RCL_RET_OK){
            return false;
        }

        return true;
    }

    void spin_once()
    {
        rcl_ret_t ret = rcl_take(&button_sub, &button_msg, NULL, NULL);
        if (ret == RCL_RET_OK)
        {
            button_callback(&button_msg);
        }
    }
};

#endif