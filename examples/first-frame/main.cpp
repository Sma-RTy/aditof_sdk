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

#include <sys/stat.h> 
#include <sys/types.h> 

#include "aditof_utils.h"

//#define DEBUG_VIDEO

int halt_flag;

static void exit_handler(int s)
{
    LOG(ERROR) << "Caught signal " << s;
    
    halt_flag = 1;

}

int main(int argc, char *argv[]) 
{
    
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    aditof::Status status = aditof::Status::OK;
    halt_flag = 0;
#ifdef DEBUG_VIDEO
    cv::VideoWriter outvideo;
#endif
    std::shared_ptr<aditof::Camera> camera = initCamera(argc, argv);
    
    /*
     * ADI ToF board revision
     * RevB, RevC
     * */
    setCameraRevision(camera, "RevB");
    
    /*
     * Available frameTypes
     * depth_ir, depth_only, ir_only, raw
     * If depth_ir is choosen, frameHeight will be doubled!
     * */
    setFrameType(camera, "depth_only");
    
    /*
     * Available modes
     * near, medium, far, custom
     * */
    std::string mode = "medium";
    setMode(camera, mode);

    /*
     * Retrieve FrameDetails
     * */
    aditof::Frame frame;
    getNewFrame(camera, &frame);
    
    aditof::FrameDetails fDetails;
    frame.getDetails(fDetails);

    int frameHeight = static_cast<int>(fDetails.height);
    int frameWidth = static_cast<int>(fDetails.width);
    int currentFrame;
    int rangeMax = getRangeMax(camera);
    int rangeMin = getRangeMin(camera);
    
    std::string dir = getTimeString() + "_" + mode;
    
    if (0 > mkdir(dir.c_str(), 0777) ) 
    {
        LOG(ERROR) << "Error :  " << strerror(errno) << std::endl; 
    }
    else
    {
        LOG(INFO) << "Directory " + dir +" created"; 
    }

#ifdef DEBUG_VIDEO
    outvideo.open(  dir + "/video_orig.avi", 
                    CV_FOURCC('X','V','I','D'), 
                    15.0, 
                    cv::Size(frameWidth, frameHeight), 
                    true);

    if (!outvideo.isOpened())
    {
        LOG(ERROR) << "Error opening outvideo video";
        return 0;
    }
#endif  
    currentFrame = 0;
    
    time_t start, end;
    time(&start);
    
    while(0 == halt_flag)
    {
        
        getNewFrame(camera, &frame);
        
        uint16_t *frameData = getFrameData(&frame, aditof::FrameDataType::DEPTH);
        if (!frameData) 
        {
            LOG(ERROR) << "getFrameData call failed";
            return (-1);
        }

        std::ofstream outbin (dir + "/" + std::to_string(currentFrame) + ".bin");
        
        cv::Mat m_depthImage = cv::Mat(cv::Size(frameWidth, frameHeight), CV_16UC1, frameData);
             
        outbin.write( (char*) m_depthImage.data, 
                    m_depthImage.elemSize() * m_depthImage.total());
#ifdef DEBUG_VIDEO
        m_depthImage.convertTo(
                m_depthImage, CV_8U,
                (255.0 / (rangeMax - rangeMin)),
                (-(255.0 / (rangeMax - rangeMin)) * rangeMin));
        
        cv::applyColorMap(m_depthImage, m_depthImage, cv::COLORMAP_JET);

        outvideo << m_depthImage;
#endif
        currentFrame++;
        
        if ((currentFrame % 50) == 0)
        {
            time(&end);
            double seconds = difftime (end, start);
            LOG(INFO) << "FPS : " << 50.0 / seconds;
            time(&start);
        }
        
        outbin.close();
    }

    return 0;
}
