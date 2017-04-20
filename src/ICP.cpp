#include <icpcuda/ICPOdometry.h>

#include <iomanip>
#include <fstream>
#include <chrono>
#include <pangolin/image/image_io.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

std::ifstream asFile;
std::string directory;

void tokenize(const std::string & str, std::vector<std::string> & tokens, std::string delimiters = " ")
{
        tokens.clear();

        std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
        std::string::size_type pos = str.find_first_of(delimiters, lastPos);

        while (std::string::npos != pos || std::string::npos != lastPos)
        {
                tokens.push_back(str.substr(lastPos, pos - lastPos));
                lastPos = str.find_first_not_of(delimiters, pos);
                pos = str.find_first_of(delimiters, lastPos);
        }
}

uint64_t loadNextDepth(pangolin::Image<unsigned short> & depth, std::string& depthPath)
{
        std::string currentLine;
        std::vector<std::string> tokens;
        std::vector<std::string> timeTokens;

        do
        {
                getline(asFile, currentLine);
                tokenize(currentLine, tokens);
        } while(tokens.size() > 2);

        if(tokens.size() == 0)
                return 0;

        std::string depthLoc = directory;
        depthLoc.append(tokens[1]);

        depthPath = depthLoc;

        pangolin::TypedImage depthRaw = pangolin::LoadImage(depthLoc, pangolin::ImageFileTypePng);

        pangolin::Image<unsigned short> depthRaw16((unsigned short*)depthRaw.ptr, depthRaw.w, depthRaw.h, depthRaw.w * sizeof(unsigned short));

        tokenize(tokens[0], timeTokens, ".");

        std::string timeString = timeTokens[0];
        timeString.append(timeTokens[1]);

        uint64_t time;
        std::istringstream(timeString) >> time;

        for(unsigned int i = 0; i < 480; i++)
        {
                for(unsigned int j = 0; j < 640; j++)
                {
                        depth.RowPtr(i)[j] = depthRaw16(j, i)  / 5;
                }
        }

        depthRaw.Dealloc();

        return time;
}

void outputTransformedDepthMap(pangolin::Image<unsigned short> & depth, std::string& depthPath, const Eigen::Matrix4f & transformation)
{
        pangolin::Image<unsigned char> transformedDepthMap((unsigned char*)depth.ptr, depth.w, depth.h, depth.w * sizeof(unsigned char));

        std::pair<short, short> minmax = depth.MinMax();
        short min = minmax.first;
        short max = minmax.second;
        short diff = max - min;
        for(unsigned int i = 0; i < 480; i++)
        {
                for(unsigned int j = 0; j < 640; j++)
                {
                        transformedDepthMap.RowPtr(i)[j] = (depth(j, i)-min)*255 / diff;
                }
        }
        for(unsigned int i = 0; i < 480; i++)
        {
                for(unsigned int j = 0; j < 640; j++)
                {
                        Eigen::Vector4f position(i,j,transformedDepthMap(j, i), 1);
                        transformedDepthMap.RowPtr(i)[j] = 255;
                        position = transformation * position;
                        if((int)position[0] >= 0 && (int)position[0] < 480
                           && (int)position[1] >= 0 && (int)position[1] < 640)
                        {
                                transformedDepthMap.RowPtr((short)position[0])[(short)position[1]] = (short)position[2];
                        }
                }
        }
        minmax = transformedDepthMap.MinMax();
        min = minmax.first;
        max = minmax.second;
        diff = max - min;
        for(unsigned int i = 0; i < 480; i++)
        {
                for(unsigned int j = 0; j < 640; j++)
                {
                        if(transformedDepthMap(j, i) != 255) {
                                transformedDepthMap.RowPtr(i)[j] = (transformedDepthMap(j, i)-min) * 51 / diff;
                        }
                }
        }
        pangolin::PixelFormat fmt = pangolin::PixelFormatFromString("GRAY8");
        int pngPos = depthPath.find(".png");
        std::string path = depthPath.substr(0,pngPos);
        pangolin::SaveImage(transformedDepthMap, fmt, path+"_t.png", pangolin::ImageFileTypePng);
        //transformedDepthMap.Dealloc();
}

void outputFreiburg(const std::string filename, const uint64_t & timestamp, const Eigen::Matrix4f & currentPose)
{
        std::ofstream file;
        file.open(filename.c_str(), std::fstream::app);

        std::stringstream strs;

        strs << std::setprecision(6) << std::fixed << (double)timestamp / 1000000.0 << " ";

        Eigen::Vector3f trans = currentPose.topRightCorner(3, 1);
        Eigen::Matrix3f rot = currentPose.topLeftCorner(3, 3);

        file << strs.str() << trans(0) << " " << trans(1) << " " << trans(2) << " ";

        Eigen::Quaternionf currentCameraRotation(rot);

        file << currentCameraRotation.x() << " " << currentCameraRotation.y() << " " << currentCameraRotation.z() << " " << currentCameraRotation.w() << "\n";

        file.close();
}

uint64_t getCurrTime()
{
        return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
}

int main(int argc, char * argv[])
{
        assert((argc == 2 || argc == 3) && "Please supply the depth.txt dir as the first argument");

        directory.append(argv[1]);

        if(directory.at(directory.size() - 1) != '/')
        {
                directory.append("/");
        }

        std::string associationFile = directory;
        associationFile.append("depth.txt");

        asFile.open(associationFile.c_str());

        pangolin::ManagedImage<unsigned short> firstData(640, 480);
        pangolin::ManagedImage<unsigned short> secondData(640, 480);

        pangolin::Image<unsigned short> firstRaw(firstData.w, firstData.h, firstData.pitch, (unsigned short*)firstData.ptr);
        pangolin::Image<unsigned short> secondRaw(secondData.w, secondData.h, secondData.pitch, (unsigned short*)secondData.ptr);

        ICPOdometry icpOdom(640, 480, 319.5, 239.5, 528, 528);

        assert(!asFile.eof() && asFile.is_open());

        std::string firstDepthPath, secondDepthPath;
        loadNextDepth(firstRaw, firstDepthPath);
        uint64_t timestamp = loadNextDepth(secondRaw, secondDepthPath);

        Sophus::SE3d T_wc_prev;
        Sophus::SE3d T_wc_curr;

        std::ofstream file;
        file.open("output.poses", std::fstream::out);
        file.close();

        cudaDeviceProp prop;

        cudaGetDeviceProperties(&prop, 0);

        std::string dev(prop.name);

        std::cout << "Device : " << dev << std::endl;

        float mean = std::numeric_limits<float>::max();
        int count = 0;

        int threads = 224;
        int blocks = 96;

        int bestThreads = threads;
        int bestBlocks = blocks;
        float best = mean;

        bool stepByStep = false;

        if(argc == 3)
        {
                std::string searchArg(argv[2]);

                stepByStep = (searchArg.compare("-s") == 0);

                if(searchArg.compare("-v") == 0)
                {
                        std::cout << "Searching for the best thread/block configuration for your GPU..." << std::endl;
                        std::cout << "Best: " << bestThreads << " threads, " << bestBlocks << " blocks (" << best << "ms)"; std::cout.flush();

                        float counter = 0;

                        for(threads = 16; threads <= 512; threads += 16)
                        {
                                for(blocks = 16; blocks <= 512; blocks += 16)
                                {
                                        mean = 0.0f;
                                        count = 0;

                                        for(int i = 0; i < 5; i++)
                                        {
                                                icpOdom.initICPModel(firstRaw.ptr);
                                                icpOdom.initICP(secondRaw.ptr);

                                                uint64_t tick = getCurrTime();

                                                T_wc_prev = T_wc_curr;

                                                Sophus::SE3d T_prev_curr = T_wc_prev.inverse() * T_wc_curr;

                                                icpOdom.getIncrementalTransformation(T_prev_curr, threads, blocks);

                                                T_wc_curr = T_wc_prev * T_prev_curr;

                                                uint64_t tock = getCurrTime();

                                                mean = (float(count) * mean + (tock - tick) / 1000.0f) / float(count + 1);
                                                count++;
                                        }

                                        counter++;

                                        if(mean < best)
                                        {
                                                best = mean;
                                                bestThreads = threads;
                                                bestBlocks = blocks;
                                        }

                                        std::cout << "\rBest: " << bestThreads << " threads, " << bestBlocks << " blocks (" << best << "ms), " << int((counter / 1024.f) * 100.f) << "%    "; std::cout.flush();
                                }
                        }

                        std::cout << std::endl;
                }
        }

        threads = bestThreads;
        blocks = bestBlocks;

        mean = 0.0f;
        count = 0;

        T_wc_prev = Sophus::SE3d();
        T_wc_curr = Sophus::SE3d();

        while(!asFile.eof())
        {

                icpOdom.initICPModel(firstRaw.ptr);
                icpOdom.initICP(secondRaw.ptr);

                uint64_t tick = getCurrTime();

                T_wc_prev = T_wc_curr;

                Sophus::SE3d T_prev_curr = T_wc_prev.inverse() * T_wc_curr;

                icpOdom.getIncrementalTransformation(T_prev_curr, threads, blocks);

                T_wc_curr = T_wc_prev * T_prev_curr;
                // T_wc_curr is now the transformation to get
                // from very first frame to current frame

                uint64_t tock = getCurrTime();

                mean = (float(count) * mean + (tock - tick) / 1000.0f) / float(count + 1);
                count++;

                std::cout << std::setprecision(4) << std::fixed
                          << "\rICP : "
                          << mean << "ms";
                std::cout.flush();

                Eigen::Vector3f trans = T_wc_curr.cast<float>().matrix().topRightCorner(3, 1);
                Eigen::Matrix3f rot = T_wc_curr.cast<float>().matrix().topLeftCorner(3, 3);
                if(stepByStep)
                {
                        std::cout << std::endl << "ICP between " << firstDepthPath << " and " << secondDepthPath << " : " << std::endl;
                        std::cout << "Found a rotation of : " << std::endl;
                        std::cout << rot(0) << " " << "0" << " " << "0" << " " << std::endl;
                        std::cout << "0" << " " << rot(1) << " " << "0" << " " << std::endl;
                        std::cout << "0" << " " << "0" << " " << rot(2) << " " << std::endl;
                        std::cout << "Found a translation of : " << std::endl;
                        std::cout << trans(0) << " " << trans(1) << " " << trans(2) << " " << std::endl;
                        std::cout << std::endl << "Press enter to continue..." << std::endl;
                        std::cin.ignore();
                }

                outputTransformedDepthMap(firstRaw, firstDepthPath, T_prev_curr.cast<float>().matrix());

                std::swap(firstRaw, secondRaw);
                std::swap(firstDepthPath, secondDepthPath);

                outputFreiburg("output.poses", timestamp, T_wc_curr.cast<float>().matrix());

                timestamp = loadNextDepth(secondRaw, secondDepthPath);
        }

        std::cout << std::endl;

        std::cout << "ICP speed: " << int(1000.f / mean) << "Hz" << std::endl;

        // Kinect stuff
        libfreenect2::Freenect2 freenect2;
        libfreenect2::Freenect2Device *kinect = 0;
        libfreenect2::PacketPipeline *pipeline = 0;

        pipeline = new libfreenect2::CpuPacketPipeline();

        if(freenect2.enumerateDevices() == 0)
        {
                std::cout << "no device connected!" << std::endl;
                return -1;
        }
        std::string serial = "";
        if (serial == "")
        {
                serial = freenect2.getDefaultDeviceSerialNumber();
        }

        kinect = freenect2.openDevice(serial, pipeline);

        int types = 0;
        bool enable_rgb, enable_depth;
        enable_rgb = true;
        enable_depth = false;
        if (enable_rgb)
                types |= libfreenect2::Frame::Color;
        if (enable_depth)
                types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
        libfreenect2::SyncMultiFrameListener listener(types);
        libfreenect2::FrameMap frames;
        kinect->setColorFrameListener(&listener);
        kinect->setIrAndDepthFrameListener(&listener);

        if (enable_rgb && enable_depth)
        {
                if (!kinect->start())
                        return -1;
        }
        else
        {
                if (!kinect->startStreams(enable_rgb, enable_depth))
                        return -1;
        }
        std::cout << "device serial: " << kinect->getSerialNumber() << std::endl;
        std::cout << "device firmware: " << kinect->getFirmwareVersion() << std::endl;

        //libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
        libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

        while(!protonect_shutdown && (framemax == (size_t)-1 || framecount < framemax))
        {
                if (!listener.waitForNewFrame(frames, 10*1000)) // 10 sconds
                {
                        std::cout << "timeout!" << std::endl;
                        return -1;
                }
                libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
                libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
                libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

                listener.release(frames);
        }

        kinect->stop();
        kinect->close();

        kinect->stop();

        return 0;
}
