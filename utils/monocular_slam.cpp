/**
* This file is part of  UCOSLAM
*
* Copyright (C) 2018 Rafael Munoz Salinas <rmsalinas at uco dot es> (University of Cordoba)
*
* UCOSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* UCOSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with UCOSLAM. If not, see <http://wwmap->gnu.org/licenses/>.
*/
#include "ucoslam.h"
#include "basictypes/debug.h"
#include "mapviewer.h"
#include "basictypes/timers.h"
#include "map.h"
#include "inputreader.h"
#include "basictypes/cvversioning.h"
#include <sstream>
#include "geometry_msgs/Point32.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <thread>

#include "std_msgs/String.h"

cv::Mat in_image;
int ready = 0;
bool finish = false;
int vspeed;
int currentFrameIndex = -1;
cv::Point3f coordxyz;
geometry_msgs::Point32 publ;

InputReader vcap;
ucoslam::UcoSlam Slam;
ucoslam::ImageParams image_params;
ucoslam::Params params;

ucoslam::TimerAvrg Fps;
cv::Mat camPose_c2g;
ucoslam::MapViewer TheViewer;
ucoslam::Frame coord;

ros::Publisher chatter_pub;

auto TheMap=std::make_shared<ucoslam::Map>();

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
     {
      try
      {
           in_image = cv_bridge::toCvShare(msg, "bgr8")->image;
           
         // int currentFrameIndex = vcap.getCurrentFrameIndex();
            currentFrameIndex++;
            Fps.start();
            camPose_c2g=Slam.process(in_image, image_params,currentFrameIndex);
            Fps.stop();
            
            coordxyz = coord.getCameraCenter();
            publ.x = coordxyz.x;
            publ.y = coordxyz.y;
            publ.z = coordxyz.z; 
            chatter_pub.publish(publ);

            cout << "Image " << currentFrameIndex << " fps=" << 1./Fps.getAvrg()<< endl;
         // Slam.drawMatches(in_image);
         // char k = TheViewer.show(&Slam, in_image,"#" + std::to_string(currentFrameIndex) + " fps=" + to_string(1./Fps.getAvrg()) );
            char k = TheViewer.show(TheMap,   in_image, camPose_c2g,"#" + std::to_string(currentFrameIndex)/* + " fps=" + to_string(1./Fps.getAvrg())*/ ,Slam.getCurrentKeyFrameIndex());
            
            if (int(k) == 27){finish = true;}//pressed ESC

            //save to output video?
            // if (!TheOutputVideo.empty()){
            //     auto image=TheViewer.getImage();
            //     if(!videoout.isOpened())
            //         videoout.open(TheOutputVideo, CV_FOURCC('X', '2', '6', '4'), stof(cml("-fps","30")),image.size()  , image.channels()!=1);
            //     if(videoout.isOpened())  videoout.write(image);
            // }

            //reset?
            if (k=='r') {Slam.clear();}
            
            //write the current map
            if (k=='e'){
                string number = std::to_string(currentFrameIndex);
                while (number.size() < 5) number = "0" + number;
                TheMap->saveToFile("world-"+number+".map");
            }

            if (k=='v'){
                Slam.saveToFile("slam.slm");
            }
           // cout<<"Number of KeyFrames= "<< TheMap->keyframes.getNextFrameIndex()<<endl;
            
            //ros::spinOnce();

       cv::waitKey(5);
     }
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
     }
     catch (const std::exception &ex) 
     {
        cerr << ex.what() << endl; 
     }
   }

cv::Mat getImage(cv::VideoCapture &vcap,int frameIdx){
    cv::Mat im;
    ucoslam::Frame frame;
    vcap.set(CV_CAP_PROP_POS_FRAMES,frameIdx);
 // vcap.grab();
    vcap.set(CV_CAP_PROP_POS_FRAMES,frameIdx);
    vcap.retrieve(im);
    return im;
}

class CmdLineParser{int argc; char **argv;
                public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue=""){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
                    std::vector<std::string> getAllInstances(string str){
                        std::vector<std::string> ret;
                        for(int i=0;i<argc-1;i++){
                            if (string(argv[i])==str)
                                ret.push_back(argv[i+1]);
                        }
                        return ret;
                    }
                   };

cv::Size readInpuSize(string s){
    for(auto &c:s)if(c==':')c =' ';
    stringstream sstr(s.c_str());
    cv::Size size;
    if ( sstr>>size.width>>size.height) return size;
    else return cv::Size(0,0);
}

cv::Mat resize(cv::Mat &in,cv::Size size){
    if (size.area()<=0)return in;
    cv::Mat ret,ret2;
    cv::resize(in,ret,size);  return ret;
}


int cIndexLive=0;
int getCurrentFrameIndex(cv::VideoCapture &vcap,bool isLive){

    if (isLive)return cIndexLive++;
    else return  int(vcap.get(CV_CAP_PROP_POS_FRAMES));
}


void overwriteParamsByCommandLine(CmdLineParser &cml,ucoslam::Params &params){
    if ( cml["-aruco-markerSize"])      params.aruco_markerSize = stof(cml("-aruco-markerSize", "1"));
    if ( cml["-marker_minsize"])    params.aruco_minMarkerSize= stod(cml("-marker_minsize", "0.025"));
    if (cml["-nokeypoints"])params.detectKeyPoints=false;
    if (cml["-nomarkers"])  params.detectMarkers =false;
    if (cml["-sequential"]) params.runSequential=true;
    if (cml["-maxFeatures"])    params.maxFeatures = stoi(cml("-maxFeatures","4000"));
    if (cml["-nOct"])       params.nOctaveLevels = stoi(cml("-nOct","8"));
    if (cml["-fdt"])        params.nthreads_feature_detector = stoi(cml("-fdt", "2"));
     if (cml["-desc"])       params.kpDescriptorType = ucoslam::DescriptorTypes::fromString(cml("-desc", "orb"));
    if (cml["-dict"])       params.aruco_Dictionary = cml("-dict");
    if (cml["-tfocus"])  params.targetFocus =stof(cml("-tfocus","-1"));
    if (cml["-KFMinConfidence"])  params.KFMinConfidence =stof(cml("-KFMinConfidence"));
    if(cml["-nonmax"])    params.KPNonMaximaSuppresion=true;

    if(cml["-autoAdjustKpSensitivity"])    params.autoAdjustKpSensitivity=true;
    if(cml["-extra_params"])    params.extraParams=cml("-extra_params");

    if(cml["-scale"]) params.kptImageScaleFactor=stof(cml("-scale"));

    if(cml["-nokploopclosure"]) params.reLocalizationWithKeyPoints=false;
    if(cml["-inplanemarkers"]) params.inPlaneMarkers=true;
    params.aruco_CornerRefimentMethod=cml("-aruco-cornerRefinementM","CORNER_SUBPIX");

    if (cml["-dbg_str"])
        ucoslam::debug::Debug::addString(cml("-dbg_str"),"");
}


int main(int argc,char **argv){
 
 CmdLineParser cml(argc, argv);

 ros::init(argc, argv, "image_listener");
 ros::NodeHandle nh;
 chatter_pub = nh.advertise<geometry_msgs::Point32>("coordinates", 1000);
 //ros::Publisher test;
// test = nh.advertise<std_msgs::String>("coordinates/xyz", 1000);
 image_transport::ImageTransport it(nh);
 image_transport::Subscriber sub = it.subscribe("csi_cam_0/image_raw", 1, imageCallback);

 

	try {
        cout<<"testing the test";
		
        if (argc < 3 || cml["-h"]) {
            cerr << "Usage: (video|live[:cameraIndex(0,1...)])  camera_params.yml [-params ucoslam_params.yml] [-map world]  [-out name] [-scale <float>:video resize factor]"
                    "[-loc_only do not update map, only do localization. Requires -in]"
                    "\n"
                    "[-desc descriptor orb,akaze,brisk,freak] "
                    "[-aruco-markerSize markers_size] [-dict <dictionary>:. By default ARUCO_MIP_36h12]  "
                    "[-nomarkers]  [-debug level] [-voc bow_volcabulary_file] "
                    "[-t_fe n:number of threads of the feature detector] [-st starts the processing stopped ] "
                    "[-nokeypoints] [-marker_minsize <val_[0,1]>] [-em . Uses enclosed markers] [-noX disabled windows] "
                    "[-fps X: set video sequence frames per second] [-outvideo filename]"
                    "[-featDensity <float>:features density]"
                    "[-nOct <int>:number of octave layers]"
                    "[-noMapUpdate]"
                    "[-tfocus <float>: target focus employed to create the map. Replaces the one of the camera] "
                    "[-undistort] will undistort image before processing it"
                    "[-extra_params \"param1=value1 param2=value2...\"]"
                    "[-vspeed <int:def 1> video analysis speed ]"
                 << endl; return -1;
		}

		bool liveVideo = false;
        
        cv::VideoWriter videoout;
		string TheInputVideo = string(argv[1]);
     // string TheOutputVideo=cml("-outvideo");
      
		if (TheInputVideo.find("live") != std::string::npos)
		{
              
            liveVideo = true;

		}
        else vcap.open(argv[1]);
        
		int debugLevel = stoi(cml("-debug", "0"));
        Slam.setDebugLevel(debugLevel);
        Slam.showTimers(true);
       
        image_params.readFromXMLFile(argv[2]);

        if( cml["-params"])        params.readFromYMLFile(cml("-params"));
        overwriteParamsByCommandLine(cml,params);

        //read the map from file?
        if ( cml["-map"]) TheMap->readFromFile(cml("-map"));

        Slam.setParams(TheMap, params,cml("-voc"));

        if(!cml["-voc"]  && !cml["-map"])
        {
            cerr<<"Warning!! No VOCABULARY INDICATED. RELOCALIZATION IMPOSSIBLE WITHOUT VOCABULARY FILE!!!!!"<<endl;
        }

        if (cml["-loc_only"]) Slam.setMode(ucoslam::MODE_LOCALIZATION);

        //need to skip frames?
        if (cml["-skip"]) {
            int n=stoi(cml("-skip","0"));
            vcap.set(CV_CAP_PROP_POS_FRAMES,n);
            cerr<<endl;
        }
       
        //need undistortion
        bool undistort=cml["-undistort"];
        vector<cv::Mat > undistMap;
        if(undistort ){
            if( undistMap.size()==0){
                undistMap.resize(2);
                cv::initUndistortRectifyMap(image_params.CameraMatrix,image_params.Distorsion,cv::Mat(),cv::Mat(),image_params.CamSize,CV_32FC1,undistMap[0],undistMap[1]);
            }
            image_params.Distorsion.setTo(cv::Scalar::all(0));
        }
        

        if (cml["-slam"]){
            Slam.readFromFile(cml("-slam"));
            vcap.set(CV_CAP_PROP_POS_FRAMES,Slam.getLastProcessedFrame());
            //vcap.retrieve(in_image);
            vcap.set(CV_CAP_PROP_POS_FRAMES,Slam.getLastProcessedFrame());
           // vcap.retrieve(in_image);
            TheMap=Slam.getMap();
            overwriteParamsByCommandLine(cml,params);
            Slam.updateParams(params);

        }

        if (cml["-noMapUpdate"])
            Slam.setMode(ucoslam::MODE_LOCALIZATION);
        
        cv::Mat auxImage;
         
        vspeed=stoi(cml("-vspeed","1"));
        cout<<"we made it further!";
         
        //release the video output if required
        if(videoout.isOpened()) videoout.release();

        //save the output

        TheMap->saveToFile(cml("-out","world") +".map");
        //save also the parameters finally employed
        params.saveToYMLFile("ucoslam_params_"+cml("-out","world") +".yml");
        if (debugLevel >=10){
            Slam.saveToFile("slam.slm");
        }
        TheMap->saveToMarkerMap("markermap.yml");

          cout<<"even til here!";
    }
    catch (const std::exception &ex) {
        cerr << ex.what() << endl; 
    }
     ros::spin();
}
