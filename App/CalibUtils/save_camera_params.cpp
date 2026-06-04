#include <visp3/sensor/vpRealSense2.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    const std::string output  = (argc > 1) ? argv[1] : "camera.xml";
    const std::string camName = (argc > 2) ? argv[2] : "RealSense_color";
    const int width           = (argc > 3) ? std::stoi(argv[3]) : 640;
    const int height          = (argc > 4) ? std::stoi(argv[4]) : 480;

    // 지정한 해상도로 color 스트림을 열어야 그 해상도의 내부 파라미터를 얻을 수 있음
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, 30);

    vpRealSense2 rs;
    rs.open(cfg);

    // open() 후 실제 초기화된 해상도를 장치에서 직접 읽어 저장에 사용
    const rs2_intrinsics intr = rs.getIntrinsics(RS2_STREAM_COLOR);
    const int actualWidth  = intr.width;
    const int actualHeight = intr.height;

    vpCameraParameters cam = rs.getCameraParameters(
        RS2_STREAM_COLOR,
        vpCameraParameters::perspectiveProjWithDistortion);

    std::cout << "[save_camera_params] Camera: " << camName
              << "  " << actualWidth << "x" << actualHeight << "\n"
              << cam << "\n";

    vpXmlParserCamera parser;
    if (parser.save(cam, output, camName, actualWidth, actualHeight) != vpXmlParserCamera::SEQUENCE_OK)
    {
        std::cerr << "ERROR: failed to save " << output << "\n";
        return 1;
    }

    std::cout << "Saved → " << output << "\n";
    rs.close();
    return 0;
}
