#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

class Calibrazione: public rclcpp::Node{

    private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_subscription_;

    std::thread keyPressThread;
    std::atomic<bool> save_image;
    int image_count;
    cv::Mat depth_glob;

    public:
        Calibrazione():Node("calibrazione"),image_count(0){
            
            image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            std::bind(&Calibrazione::imageCallback, this, std::placeholders::_1));

            depth_image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/depth_registered/image_rect", 10,
            std::bind(&Calibrazione::imageDepthCallback, this, std::placeholders::_1));

            keyPressThread=std::thread(&Calibrazione::waitForKeyPress,this);        }

        ~Calibrazione(){
            if(keyPressThread.joinable()){
                keyPressThread.join();
            }
        }
    
        void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg){

            // Conversione dell'immagine letta (ROS) in un'immagine OpenCV
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image=cv_ptr->image;
            cv::imshow("Maschera Verde",image);
            if(save_image){
                try {
                    RCLCPP_INFO(this->get_logger(), "ciao");
                    //Salva l'immagine

                    std::string filename= "image_"+std::to_string(image_count)+".png";
                    cv::imwrite(filename,image);
                    std::string filename2= "depth_"+std::to_string(image_count)+".png";
                    cv::imwrite(filename2,depth_glob);

                    image_count++;
                    save_image=false;
                    
       
                } catch (cv_bridge::Exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "errore");
                    return;
                }

            
            }
            cv::waitKey(1); 
        }

        void imageDepthCallback(const sensor_msgs::msg::Image::SharedPtr msg){

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
                    RCLCPP_INFO(this->get_logger(), "ciao");
                    //Salva l'immagine

                    std::string filename= "depth_"+std::to_string(image_count)+".png";
                    cv::imwrite(filename,image);
                    image_count++;
                    save_image=false;
                    
       
                } catch (cv_bridge::Exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "errore");
                    return;
                }


            }*/
            cv::waitKey(1); 
        }

        void waitForKeyPress(){
            while(rclcpp::ok()){
                char key= std::cin.get();
                if(key=='s')
                {
                    save_image=true;
                    RCLCPP_INFO(this->get_logger(),"PRONTO A SALVARE");
                }
            }
        }
};


int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Calibrazione>());
    rclcpp::shutdown();
    return 0;
}