/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/system.h>
#include <glog/logging.h>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

#include <dirent.h>
#include <time.h>
#include <signal.h>

using namespace aditof;

cv::VideoWriter outvideo;

std::string getTimeString()
{
    time_t unixTime = time(NULL);
    struct tm *locTime = localtime(&unixTime);

    std::string str;

    str = std::to_string(locTime->tm_year - 100);

    if ((locTime->tm_mon+1) < 10) str += "0";
    str += std::to_string(locTime->tm_mon + 1);

    if (locTime->tm_mday < 10) str += "0";
    str += std::to_string(locTime->tm_mday);

    str += "_";

    if (locTime->tm_hour < 10) str += "0";
    str += std::to_string(locTime->tm_hour);

    if (locTime->tm_min < 10) str += "0";
    str += std::to_string(locTime->tm_min);

    if (locTime->tm_sec < 10) str += "0";
    str += std::to_string(locTime->tm_sec);

    return str;
}

static void exit_handler(int s)
{
    LOG(ERROR) << "Caught signal " << s;

	//outvideo.release();

    exit(1);
}

int main(int argc, char *argv[]) {

    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;


	struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    Status status = Status::OK;
    
    System system;
    status = system.initialize();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize system!";
        return 0;
    }

    std::vector<std::shared_ptr<Camera>> cameras;
    system.getCameraList(cameras);
    if (cameras.empty()) {
        LOG(WARNING) << "No cameras found";
        return 0;
    }

    auto camera = cameras.front();
    status = camera->initialize();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return 0;
    }

    std::vector<std::string> frameTypes;
    camera->getAvailableFrameTypes(frameTypes);
    if (frameTypes.empty()) {
        std::cout << "no frame type avaialble!";
        return 0;
    }
    status = camera->setFrameType(frameTypes.front());
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera frame type!";
        return 0;
    }

    std::vector<std::string> modes;
    camera->getAvailableModes(modes);
    if (modes.empty()) {
        LOG(ERROR) << "no camera modes available!";
        return 0;
    }
    status = camera->setMode(modes.front());
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera mode!";
        return 0;
    }

    aditof::Frame frame;
    
    status = camera->requestFrame(&frame);
	if (status != Status::OK) {
		LOG(ERROR) << "Could not request frame!";
		return 0;
	} else {
		//LOG(INFO) << "succesfully requested frame!"; 
	}

	FrameDetails fDetails;
	frame.getDetails(fDetails);
	
	int frameHeight = static_cast<int>(fDetails.height) / 2;
	int frameWidth = static_cast<int>(fDetails.width);
	int currentFrame;
	int rangeMax = 1200;
	int rangeMin = 500;
    
    std::string timestamp_str = getTimeString();
    
    outvideo.open(timestamp_str + "_orig.avi", 
					CV_FOURCC('X','V','I','D'), 
					15.0, 
					cv::Size(frameWidth, frameHeight), 
					true);
					
    if (!outvideo.isOpened())
    {
        LOG(ERROR) << "Error opening outvideo video";
        return 0;
    }
    
    currentFrame = 0;
    
    while(1)
    {

		status = camera->requestFrame(&frame);
		if (status != Status::OK) {
			LOG(ERROR) << "Could not request frame!";
			return 0;
		} else {
			//LOG(INFO) << "succesfully requested frame!"; 
		}

		uint16_t *data;
		status = frame.getData(FrameDataType::RAW, &data);

		if (status != Status::OK) {
			LOG(ERROR) << "Could not get frame data!";
			return 0;
		}

		if (!data) {
			LOG(ERROR) << "no memory allocated in frame";
			return 0;
		}
	
		std::ofstream outbin (timestamp_str + "_" + std::to_string(currentFrame) + ".bin");
		
		cv::Mat m_depthImage = cv::Mat(cv::Size(frameWidth, frameHeight), CV_16UC1, data);
			 
		outbin.write( (char*) m_depthImage.data, 
					m_depthImage.elemSize() * m_depthImage.total());
		
		m_depthImage.convertTo(
				m_depthImage, CV_8U,
				(255.0 / (rangeMax - rangeMin)),
				(-(255.0 / (rangeMax - rangeMin)) * rangeMin));
		
		cv::applyColorMap(m_depthImage, m_depthImage, cv::COLORMAP_RAINBOW);
		cv::flip(m_depthImage, m_depthImage, 1);

		//cv::imshow( "TEST", m_depthImage);
		outvideo << m_depthImage;
		
		currentFrame++;
		outbin.close();
		
		//cv::waitKey(1);
	}

    return 0;
}
