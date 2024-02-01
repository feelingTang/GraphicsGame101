#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

// 生成视图矩阵，将世界左边系变换到观察者坐标系
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

// 生成模型矩阵，用于对模型进行变换，绕Z轴进行旋转
// Eigen::Matrix4f get_model_matrix(float rotation_angle)
// {
//     Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

//     // TODO: Implement this function
//     // Create the model matrix for rotating the triangle around the Z axis.
//     float rotation_angle_radian = rotation_angle*MY_PI/180;
//     model(0,0) = cos(rotation_angle_radian); 
//     model(0,1) = -sin(rotation_angle_radian);
//     model(1,0) = sin(rotation_angle_radian);
//     model(1,1) = cos(rotation_angle_radian);
//     // Then return it.

//     return model;
// }
Eigen::Matrix4f get_model_matrix_rotateanyaxis(Eigen::Vector3f axis, float rotation_angle)
{
    float rotation_angle_radian = rotation_angle*MY_PI/180; 
    Eigen::Matrix3f I3f = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f Maxis_product; // axis product matrix
    Maxis_product << 0, -axis(2) , axis(1), axis(2), 0, -axis(0), -axis(1), axis(0), 0;
    Eigen::Matrix3f model3 = I3f + (1-cos(rotation_angle_radian))*(Maxis_product*Maxis_product) + sin(rotation_angle_radian)*Maxis_product;

    Eigen::Matrix4f model4 = Eigen::Matrix4f::Identity();
    model4.block(0,0,3,3) << model3;
    // std::cout<<model4<<std::endl;
    return model4;
}


// 生成投影矩阵、用于将三维场景投影到二维屏幕上
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
        // Compute l, r, b, t
    float t = tan((eye_fov*MY_PI/180)/2) * fabs(zNear);
    float r = aspect_ratio * t;
    float l = -r;
    float b = -t;

    // Orthographic projection
    // Translate to origin
    Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
    translate(0,3) = -(r+l)/2;
    translate(1,3) = -(t+b)/2;
    translate(2,3) = -(zNear+zFar)/2;

    // Sclae to [-1,1]^3
    Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
    scale(0,0) = 2/(r-l);
    scale(1,1) = 2/(t-b);
    scale(2,2) = 2/(zNear-zFar);

    // get Orthographic projection
    Eigen::Matrix4f ortho = scale * translate;
    // std::cout<<"Orthographic:"<<std::endl<<ortho<<std::endl;

    // Perspective projection
    // get Matrix_persp2ortho
    Eigen::Matrix4f persp2ortho = Eigen::Matrix4f::Zero();
    persp2ortho(0,0) = zNear;
    persp2ortho(1,1) = zNear;
    persp2ortho(2,2) = zNear+zFar;
    persp2ortho(2,3) = -zNear*zFar;
    persp2ortho(3,2) = 1;

    // get Perspective projection
    projection = ortho * persp2ortho;
    // Then return it.

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0; // 绕Z轴逆时针旋转角度
    bool command_line = false;  // 判断是否有-r参数，true有-r参数
    std::string filename = "output.png";

    // 不同入参对应不同模式：那都有那些参数形式？
    if (argc >= 3) {
        command_line = true;
        // 将命令行参数转换为浮点数、并赋值 angle 度数
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }
    // 定义一个光栅化对象
    rst::rasterizer r(700, 700);

    // eye相机位置
    Eigen::Vector3f eye_pos = {0, 0, 5};
    
    // 三角形三点坐标
    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    // 坐标索引
    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    // 返回pos_buf的索引，并存值
    auto pos_id = r.load_positions(pos);
    // 返回ind_buf的索引，并存值
    auto ind_id = r.load_indices(ind);

    // 输入ASCII码对应键值
    int key = 0;
    // 生成帧数
    int frame_count = 0;

    // 命令行模式则存图像
    if (command_line) {
        // 初始化 frame_buf-设置为黑色，depth_buf-设置为正无穷大
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // 载入模型变换矩阵、视图变换矩阵、投影变换矩阵进光栅化类中
        // r.set_model(get_model_matrix(angle));
        r.set_model(get_model_matrix_rotateanyaxis(Vector3f(0,0,1), angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));

        // 光栅化，传入Triangle三角形
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        // 使用OpenCV库，保存渲染的图像：创建一个cv::Mat 图像矩阵（700x700）CV_32FC3图像类型为32位浮点数、 r.frame_buffer().data()r对象的渲染结果
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        // 将图像数据从浮点“CV_32FC3”转换为8位无符号整数类型“CV_8UC3”
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    // runtime模式
    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        r.set_model(get_model_matrix_rotateanyaxis(Vector3f(0,0,1), angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
