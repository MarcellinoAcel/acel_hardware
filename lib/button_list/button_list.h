#ifndef BUTTON_LIST_H
#define BUTTON_LIST_H
#include <Arduino.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <rclc/rclc.h>

/*
button A = 0
button B = 1
button X = 3
button Y = 4
button RT = 10
button LT = 9
button LB = 7
button RB = 8
*/

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }

class Button_list
{
private:
    rcl_subscription_t button_sub;
    std_msgs__msg__Int32MultiArray button_msg;

    static void button_callback(const void *msg_in, void *arg)
    {
        const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msg_in;
        Button_list *instance = (Button_list *)arg;
        Serial.println("Button states:");
        for (size_t i = 0; i < msg->data.size; ++i)
        {
            switch (i)
            {
            case 0:
                instance->button.A = msg->data.data[0];
                break;
            case 1:
                instance->button.B = msg->data.data[1];
                break;
            case 3:
                instance->button.X = msg->data.data[3];
                break;
            case 4:
                instance->button.Y = msg->data.data[4];
                break;
            case 10:
                instance->button.RT = msg->data.data[10];
                break;
            case 9:
                instance->button.LT = msg->data.data[9];
                break;
            case 7:
                instance->button.LB = msg->data.data[7];
                break;
            case 8:
                instance->button.RB = msg->data.data[8];
                break;
            default:
                break;
            }
        }
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

    void init(rcl_node_t *node)
    {
        button_sub = rcl_get_zero_initialized_subscription();

        const rosidl_message_type_support_t *type_support =
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray);

        rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();

        RCCHECK(
            rcl_subscription_init(
                &button_sub,
                node,
                type_support,
                "button",
                &subscription_ops);)
    }

    void spin_once()
    {
        rcl_ret_t ret = rcl_take(&button_sub, &button_msg, NULL, NULL);
        if (ret == RCL_RET_OK)
        {
            button_callback(&button_msg, this);
        }
    }

    // ~Button_list()
    // {
    //     rcl_ret_t ret = rcl_subscription_fini(&button_sub, NULL);
    //     if (ret != RCL_RET_OK)
    //     {
    //         Serial.print("Error finalizing subscription: ");
    //         Serial.println(ret);
    //     }
    // }
};

#endif