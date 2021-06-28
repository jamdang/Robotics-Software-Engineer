#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot

    // the request to be sent to the command_robot service
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the command_robot service
    if (!client.call(srv))
        ROS_ERROR("Failed to call service safe_move");
    
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    bool ball_in_view = false;    
    float coeff_to_center = 0.f; // relative position to the center: if far left, then this is 1.0, far right, -1.0, center, 0.
    
    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    
    for (int i = 0; i < img.height; ++i) {
        for (int j = 0; j < img.step; ++j) {
            // if we found a white pixel
            if (img.data[i*img.step + j] == white_pixel &&
                img.data[i*img.step + j + 1] == white_pixel &&
                img.data[i*img.step + j + 2] == white_pixel) {
                ball_in_view = true;
                // get its relative position to center                
                int half_num_step = img.step/2;
                coeff_to_center = static_cast<float>(half_num_step - j)/static_cast<float>(half_num_step);            
                //ROS_INFO_STREAM("half_num_step: " + std::to_string(half_num_step) + "j: " + std::to_string(j)); // for debug
                break;              
            }
        }        
    }

    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    if (ball_in_view) {
        drive_robot((1.f - std::abs(coeff_to_center)) * 1.f, coeff_to_center * .5f);
    }
    // Request a stop when there's no white ball seen by the camera
    else {
        drive_robot(0.f, 0.f); // request stop
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
