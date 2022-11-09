// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.h"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle_Barycentric(int x, int y, const Vector3f* v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by v[0], v[1], v[2]

    float alpha = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
    float beta = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
    float gamma = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
    if (alpha < 1 && alpha > 0 && beta < 1 && beta>0 && gamma < 1 && gamma > 0) return true;
    else return false;
}

static bool insideTriangle_Cross(float x, float y, const Vector3f* _v)
{
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    // get the vectices of the triangle
    const Vector3f& A = _v[0];
    const Vector3f& B = _v[1];
    const Vector3f& C = _v[2];
    // get the line of the triangle
    Vector3f AB = B - A;
    Vector3f BC = C - B;
    Vector3f CA = A - C;
    // use cross multiply to decide whether it is inside
    Vector3f P(x, y, 1.0);
    Vector3f AP = P - A;
    Vector3f BP = P - B;
    Vector3f CP = P - C;

    Vector3f res_1 = AB.cross(AP);
    Vector3f res_2 = BC.cross(BP);
    Vector3f res_3 = CA.cross(CP);

    // if res_1,2,3 point to the same direction in z, it is inside
    return (res_1.z() > 0 && res_2.z() > 0 && res_3.z() > 0) || (res_1.z() < 0 && res_2.z() < 0 && res_3.z() < 0);
}


//利用重心坐标插值的方法求三角形内任意一点的深度值（URL:https://blog.csdn.net/Q_pril/article/details/123598746）
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float alpha = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float beta = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float gamma = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return { alpha, beta, gamma };
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        //rasterize_triangle(t);
        rasterize_triangle_MSAA(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    int min_x = INT_MAX;
    int max_x = INT_MIN;
    int min_y = INT_MAX;
    int max_y = INT_MIN;
    for (auto point : v) //获取包围盒边界
    {
        if (point[0] < min_x) min_x = point[0];
        if (point[0] > max_x) max_x = point[0];
        if (point[1] < min_y) min_y = point[1];
        if (point[1] > max_y) max_y = point[1];
    }

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
    for (int y = min_y; y <= max_y; y++)
    {
        for (int x = min_x; x <= max_x; x++)
        {
            if (insideTriangle_Cross((float)x + 0.5, (float)y + 0.5, t.v)) //用像素中心点当像素坐标
            {
                auto abg = computeBarycentric2D((float)x + 0.5, (float)y + 0.5, t.v);
                float alpha = std::get<0>(abg);
                float beta = std::get<1>(abg);
                float gamma = std::get<2>(abg);
                float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                //上面均为作业已经给出的插值方法直接用即可
                if (z_interpolated < depth_buf[get_index(x, y)]) //判断当前z值是否小于原来z表此位置的z值
                {
                    Eigen::Vector3f p = { (float)x,(float)y, z_interpolated }; //当前坐标
                    set_pixel(p, t.getColor());
                    depth_buf[get_index(x, y)] = z_interpolated; //更新z值
                }
            }
        }
    }
}

void rst::rasterizer::rasterize_triangle_MSAA(const Triangle& t) {
    auto v = t.toVector4();
    int min_x = INT_MAX;
    int max_x = INT_MIN;
    int min_y = INT_MAX;
    int max_y = INT_MIN;
    for (auto point : v)
    {
        if (point[0] < min_x) min_x = point[0];
        if (point[0] > max_x) max_x = point[0];
        if (point[1] < min_y) min_y = point[1];
        if (point[1] > max_y) max_y = point[1];
    }

    //MSAA(比例法，还有另一种SSAA：思路就是创建两个新的buffer：frame_buf_ssaa 和 depth_buf_ssaa，对三角形进行光栅化的时候更新它们的值。
    //最后在绘画之前做downsampling，即把四个小像素的值加起来求平均，然后更新frame_buf  URL:https://zhuanlan.zhihu.com/p/454001952):
    for (int y = min_y; y <= max_y; y++)
    {
        for (int x = min_x; x <= max_x; x++)
        {
            float fineness = 0; //创建一个纯度值
            if (insideTriangle_Cross((float)x + 0.25, (float)y + 0.25, t.v)) fineness += 0.25; //改为判断四个采样块的中心点在不在三角形内
            if (insideTriangle_Cross((float)x + 0.25, (float)y + 0.75, t.v)) fineness += 0.25;
            if (insideTriangle_Cross((float)x + 0.75, (float)y + 0.25, t.v)) fineness += 0.25;
            if (insideTriangle_Cross((float)x + 0.75, (float)y + 0.75, t.v)) fineness += 0.25;
            if (fineness != 0)
            {
                auto abg = computeBarycentric2D((float)x + 0.5, (float)y + 0.5, t.v);
                float alpha = std::get<0>(abg);
                float beta = std::get<1>(abg);
                float gamma = std::get<2>(abg);
                // 这里的 v[i].w 本来应该是深度值，这个框架写死了= 1。又因为 alpha + beta + gamma = 1，所以 w_reciprocal = 1
                float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                if (z_interpolated < depth_buf[get_index(x, y)])
                {
                    Eigen::Vector3f p = { (float)x,(float)y, z_interpolated };
                    set_pixel(p, fineness * t.getColor()); //将比例乘给颜色
                    depth_buf[get_index(x, y)] = z_interpolated;
                }
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::cout << "ClearColor\n";
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::cout << "ClearDepth\n";
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on