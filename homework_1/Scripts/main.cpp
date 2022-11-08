#include "Triangle.h"
#include "rasterizer.h"
#include<Eigen/Core>
#include<Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float angle = rotation_angle * MY_PI / 180.0f;
    model <<
            cos(angle), -sin(angle), 0, 0,
            sin(angle),  cos(angle), 0, 0,
                     0,           0, 1, 0,
                     0,           0, 0, 1;

    return model;
}

//任意轴我们用一个3维向量n来表示，这里默认是过原点的向量，直接用罗格里德斯公式：
Eigen::Matrix4f get_random_model_matrix(Eigen::Vector3f n, float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float angle = rotation_angle * MY_PI / 180.0f;
    float nx = n[0];
    float ny = n[1];
    float nz = n[2];
    Eigen::Matrix3f N;
    N <<
        0, -nz, ny,
        nz, 0, -nx,
        -ny, nx, 0;
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f R = std::cos(angle) * I + (1 - std::cos(angle)) * n * n.transpose() + std::sin(angle) * N;
    model <<
        R(0, 0), R(0, 1), R(0, 2), 0,
        R(1, 0), R(1, 1), R(1, 2), 0,
        R(2, 0), R(2, 1), R(2, 2), 0,
        0, 0, 0, 1;

    return model;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
    float zNear, float zFar)
{
    // Students will implement this function
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f M_trans;
    Eigen::Matrix4f M_persp;
    Eigen::Matrix4f M_ortho;
    M_persp <<
        zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zFar * zNear,
        0, 0, 1, 0;

    float alpha = 0.5 * eye_fov * MY_PI / 180.0f;
    float yTop = -zNear * std::tan(alpha); //因为这里的z给的是负数，所以加负号之后再转换
    float yBottom = -yTop;
    float xRight = yTop * aspect_ratio;
    float xLeft = -xRight;

    M_trans <<
        1, 0, 0, -(xLeft + xRight) / 2,
        0, 1, 0, -(yTop + yBottom) / 2,
        0, 0, 1, -(zNear + zFar) / 2,
        0, 0, 0, 1;
    M_ortho <<
        2 / (xRight - xLeft), 0, 0, 0,
        0, 2 / (yTop - yBottom), 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;

    M_ortho = M_ortho * M_trans;
    projection = M_ortho * M_persp * projection;
    return projection;
}

void test() {
    Eigen::Matrix2f I = Eigen::Matrix2f::Identity(); // 默认初始化矩阵为单位矩阵
    std::cout << "I:\n" << I << std::endl;
    Eigen::Matrix2f model = Eigen::Matrix2f::Identity();
    model <<
        1.0, 2.0,
        3.0, 4.0;
    std::cout << "matrix model:\n" << model << std::endl;
    Eigen::Matrix2f m1 = model + I;
    std::cout << "model+E:\n" << m1 << std::endl;
    std::cout << "model*m1:\n" << model * m1 << std::endl; //矩阵相乘
    Eigen::Vector3f v(1.0f, 2.0f, 3.0f); //虽然横着写，但默认是列向量
    Eigen::Vector3f w(1.0f, 2.0f, 3.0f);
    std::cout << "vector v:\n" << v << std::endl;
    std::cout << "v · w:\n" << v.dot(w) << std::endl; //向量点乘
    std::cout << "v × w:\n" << v.cross(w) << std::endl; // 向量叉乘，这里可以看到相同向量叉乘结果为零向量
    Eigen::Matrix3f m2 = v * w.transpose(); //向量相乘，和矩阵相乘一样内定乘外定型
    std::cout << "m2:\n" << m2 << std::endl;
    std::cout << "m2*w:\n" << m2 * w << std::endl;
    std::cout << "m2*m2:\n" << m2.array() * m2.array() << std::endl;//矩阵点乘，即对应位置相乘
}

int main(int argc, const char** argv)
{
    float angle = 45;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);//创建一个用来光栅化的对象

    Eigen::Vector3f eye_pos = { 0, 0, 5 };

    std::vector<Eigen::Vector3f> pos{ {2, 0, -2}, {0, 2, -2}, {-2, 0, -2} };

    std::vector<Eigen::Vector3i> ind{ {0, 1, 2} };

    auto pos_id = r.load_positions(pos);//把顶点坐标数据存到r里面，然后获取到一个id，一会用来从r里面取出。
    auto ind_id = r.load_indices(ind);//把索引数据存到r里面，然后获取到一个id，一会用来从r里面取出。

    int key = 0;
    int frame_count = 0;
    //俩分支都差不多，说一个。
    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);//先把我们的深度缓冲区和存放结果的framebuffer清空
        //下面三句是把我们的的矩阵计算出来，然后设置到r里面去
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        //上边你会发现，好多东西都给r了，那么下面r就开始画三角形了。
        //画三角形，我们需要给他刚刚的id，因为有id才知道我们顶点和索引存的位置。然后第三个参数就表示画三角形
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        //下面就是我们最后的窗口显示相关的，先不管。核心是上边的怎么根据数据来画三角形的。
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

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


