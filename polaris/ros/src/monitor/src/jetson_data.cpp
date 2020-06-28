#include <ros/ros.h>
#include "monitor/jetson_data_msg.h"

#include <stdio.h>
#include <memory>
#include <array>
#include <math.h>

/**
 * @brief   Formats input to take out all numbers and store them in msg.
 * 
 * @param input Cout from tegrastats.
 * @param msg   Pass by reference variable storing all data to be published.
 */
void update_values(std::string input, monitor::jetson_data_msg &msg)
{
    float values[50];
    int index = 0;
    float temp = 0.0;
    bool found_num = false;
    int decimal = 0;
    
    /**
     * Iterates through input locating all numbers and storing each in the array values.
     * A possible state for each CPU_usage is "off". In this case, that CPU is given a
     * value of 0. 
     */
    for(int i=0; i< input.length(); i++) {
        char c = input[i];

        if(isdigit(c)) {
            if(decimal > 0) {
                temp += (float)(c - '0') / pow(10.0, decimal);
                decimal++;
            } else {
                temp *= 10.0;
                temp += (float)(c - '0');
            }
            found_num = true;
        } else if(c == '.') {
            decimal++;
        } else if(found_num) {
            values[index++] = temp;
            temp = 0.0;
            found_num = false;
            decimal = 0;
        } else {
            temp = 0.0;
            found_num = false;
            decimal = 0;
        }
        if(i + 2 <= input.length()) {
            if((c == 'o') && (input[i+1] == 'f') && (input[i+2] == 'f')) {
                values[index++] = 0;
                values[index++] = 0;
            }
        }
    }

    msg.RAM_N = (int)values[0];
    msg.RAM_D = (int)values[1];
    msg.CPU_usage_1 = (int)values[4];
    msg.CPU_usage_2 = (int)values[6];
    msg.CPU_usage_3 = (int)values[8];
    msg.CPU_usage_4 = (int)values[10];
    msg.CPU_usage_5 = (int)values[12];
    msg.CPU_usage_6 = (int)values[14];
    msg.BCPU_temp = values[16];
    msg.MCPU_temp = values[17];
    msg.GPU_temp = values[18];
    msg.PLL_temp = values[19];
    msg.Tboard_temp = values[20];
    msg.Tdiode_temp = values[21];
    msg.PMIC_temp = values[22];
    msg.thermal = (int)values[23];
    msg.VDD_IN_N = (int)values[24];
    msg.VDD_IN_D = (int)values[25];
    msg.VDD_CPU_N = (int)values[26];
    msg.VDD_CPU_D = (int)values[27];
    msg.VDD_GPU_N = (int)values[28];
    msg.VDD_GPU_D = (int)values[29];
    msg.VDD_SOC_N = (int)values[30];
    msg.VDD_SOC_D = (int)values[31];
    msg.VDD_WIFI_N = (int)values[32];
    msg.VDD_WIFI_D = (int)values[33];
    msg.VDD_DDR_N = (int)values[34];
    msg.VDD_DDR_D = (int)values[35];
}

/**
 * @brief   Publishes information about the Jetson board for use in the web GUI.
 *          All data in taken from the cout of the executable file tegrastats 
 *          located on the submarine at ~/tegrastats.  
 */
int main(int argc, char **argv)
{
    // Standard ros publisher initialization
    ros::init(argc, argv, "jetson_data_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<monitor::jetson_data_msg>("jetson_data_msg", 10);
    ros::Rate loop_rate(10);

    // Reads cout from tegrastats and stores it in pipe
    std::array<char, 1024> buffer;
    std::string input = "";
    std::string prev_input = "";
    FILE* pipe = popen("~/tegrastats", "r");

    while(ros::ok()) {

        monitor::jetson_data_msg msg;

        if(fgets(buffer.data(), 1024, pipe) != NULL) {
            input = buffer.data();
        }
        if(prev_input != input) {
            update_values(input, msg);
        }
        prev_input = input;
        
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    pclose(pipe);
    return 0;   
}
