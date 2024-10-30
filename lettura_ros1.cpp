#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

class Reader {
public:
    Reader(ros::NodeHandle& nh)
    {
        // Sottoscrizione al topic "example_topic"
        image_subscription_ = nh.subscribe("/image_raw", 10, &Reader::imageCallback, this);
        depth_image_subscription_ = nh.subscribe("/depth_registered/image_rect", 10, &Reader::imageDepthCallback, this);
        keyPressThread=std::thread(&Reader::waitForKeyPress,this); 
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
            // Conversione dell'immagine letta (ROS) in un'immagine OpenCV
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image=cv_ptr->image;
            cv::imshow("Maschera Verde",image);
            if(save_image)
            {
                try {
                        ROS_INFO("ciao");

                        //Salva l'immagine
                        std::string filename= "image_"+std::to_string(image_count)+".png";
                        cv::imwrite(filename,image);
                        std::string filename2= "depth_"+std::to_string(image_count)+".png";
                        cv::imwrite(filename2,depth_glob);

                        image_count++;
                        save_image=false;
                        
       
                    } catch (cv_bridge::Exception& e) {
                        ROS_ERROR("Errore: %s", e.what());
                        return;
                    }
            }
            cv::waitKey(1); 
    }

    void imageDepthCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
            // Conversione dell'immagine letta (ROS) in un'immagine OpenCV
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            cv::Mat image=cv_ptr->image;

            cv::Mat depth_image_normalized;
            cv::normalize(image,depth_image_normalized,0,255, cv::NORM_MINMAX, CV_8U);
            depth_glob=depth_image_normalized;
            cv::imshow(" Verde",depth_image_normalized);
            /*if(save_image){
                try {
                    ROS_INFO("ciao");
                    //Salva l'immagine

                    std::string filename= "depth_"+std::to_string(image_count)+".png";
                    cv::imwrite(filename,image);
                    image_count++;
                    save_image=false;
                    
       
                } catch (cv_bridge::Exception& e) {
                    ROS_ERROR("Errore: %s", e.what());
                    return;
                }


            }*/
            cv::waitKey(1); 
    }


    void waitForKeyPress()
    {
            while(ros::ok())
            {
                char key= std::cin.get();
                if(key=='s')
                {
                    save_image=true;
                    ROS_INFO("PRONTO A SALVARE");
                }
            }
    }

    ~Calibrazione()
    {
         if(keyPressThread.joinable())
         {
            keyPressThread.join();
         }
    }

private:
     ros::Subscriber image_subscription_;
     ros::Subscriber depth_image_subscription_;
     std::thread keyPressThread;
     std::atomic<bool> save_image;
     int image_count;
     cv::Mat depth_glob;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "reader_node");
    ros::NodeHandle nh;

    Reader listener(nh);

    // Loop principale
    ros::spin();

    return 0;
}
