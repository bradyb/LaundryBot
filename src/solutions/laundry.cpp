#include <string>

#include "ImageManager.hpp"

#include "VxGui.hpp"

using namespace std;

int main(int argc, char** argv)
{
    string camUrl, savePath = "./data/testImage.pnm";
    int camFormat = 0;
    // parse command line opts
    opterr = 1;
    int opt = 0;
    while ((opt = getopt(argc, argv, "hc:f:ls:")) != -1)
    {
        switch (opt)
        {
            case 'h': 
                ImageManager::help();
                exit(0);
            case 'l':
                ImageManager::listCameras();
                exit(0);
            case 'c':
                camUrl = string(optarg);
            case 'f':
                camFormat = atoi(optarg);
            case 's':
            	savePath = string(optarg);
        }
    }

    ImageManager imgMgr(camUrl, camFormat);
    VxGui gui(argc, argv, savePath);
    imgMgr.setGui(&gui);
    thread imgThread (ImageManager::run, &imgMgr);
    imgThread.join();

    return 0;
}
