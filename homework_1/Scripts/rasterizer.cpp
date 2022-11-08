//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdexcept>
#include "rasterizer.h"


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return { id };
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return { id };
}

// Bresenham's line drawing algorithm
// Code taken from a stack overflow answer: https://stackoverflow.com/a/16405254
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = { 255, 255, 255 };

    int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;

    dx = x2 - x1;
    dy = y2 - y1;
    dx1 = fabs(dx);
    dy1 = fabs(dy);
    px = 2 * dy1 - dx1;
    py = 2 * dx1 - dy1;

    if (dy1 <= dx1)
    {
        if (dx >= 0)
        {
            x = x1;
            y = y1;
            xe = x2;
        }
        else
        {
            x = x2;
            y = y2;
            xe = x1;
        }
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point, line_color);
        for (i = 0; x < xe; i++)
        {
            x = x + 1;
            if (px < 0)
            {
                px = px + 2 * dy1;
            }
            else
            {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                {
                    y = y + 1;
                }
                else
                {
                    y = y - 1;
                }
                px = px + 2 * (dy1 - dx1);
            }
            //            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point, line_color);
        }
    }
    else
    {
        if (dy >= 0)
        {
            x = x1;
            y = y1;
            ye = y2;
        }
        else
        {
            x = x2;
            y = y2;
            ye = y1;
        }
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point, line_color);
        for (i = 0; y < ye; i++)
        {
            y = y + 1;
            if (py <= 0)
            {
                py = py + 2 * dx1;
            }
            else
            {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                {
                    x = x + 1;
                }
                else
                {
                    x = x - 1;
                }
                py = py + 2 * (dx1 - dy1);
            }
            //            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point, line_color);
        }
    }
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

void rst::rasterizer::draw(rst::pos_buf_id pos_buffer, rst::ind_buf_id ind_buffer, rst::Primitive type)
{
    if (type != rst::Primitive::Triangle)
    {
        throw std::runtime_error("Drawing primitives other than triangle is not implemented yet!");
    }
    //下面两句，先把参数中的id给拿出来，这里我们说的id其实外面包了一层结构体，但是实际上里面只是一个int类型的id
    //把id取出来，然后从容器里面给拿出来顶点和索引数据。
    //注意这里的容器是一个哈希的类似结构，[]中写的是key值，不代表下标。
    auto& buf = pos_buf[pos_buffer.pos_id];

    auto& ind = ind_buf[ind_buffer.ind_id];

    float f1 = (100 - 0.1) / 2.0;
    float f2 = (100 + 0.1) / 2.0;
    //根据之前传递给r的三个矩阵，算出mvp矩阵，表示一下子进行仨变换。
    Eigen::Matrix4f mvp = projection * view * model;
    //这里循环遍历ind，注意ind是一个vector，里面装有n个Vector3f对象。而一个对象的xyz分量就表示三个索引，就是一个三角形的
    //三个索引，所以一圈循环就会画一个三角形。
    for (auto& i : ind)
    {

        Triangle t;
        //通过索引取出对应的顶点，这时候顶点是一个三维向量，我们进行矩阵变换需要是齐次坐标
        //所以扩充成4维的，然后进行mvp变换。变换结果放到一个数组里
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //把齐次坐标变换回到普通坐标，我们需要同时除以第四个分量，也就是下面操作
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //下面就是对这三个顶点进行一个viewport变换了，也就是把他映射到屏幕上，这里并没有采用矩阵的形式。
        //看一下x，先对x+1，也就是-1到1，变成了0到2，然后乘以width*0.5，这样x就变成了0~width范围。
        for (auto& vert : v)
        {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }
        //下面就是把我们变换完成的坐标点，放到一个新的三角形中
        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }
        //我们可以给三角形三个顶点设置颜色，这块没啥用，因为画的是线框
        t.setColor(0, 255.0, 0.0, 0.0);
        t.setColor(1, 0.0, 255.0, 0.0);
        t.setColor(2, 0.0, 0.0, 255.0);
        //给三角形画三条线框，把三角形传进去，线框就是三角内不不填充任何颜色，只有三边
        rasterize_wireframe(t);
    }
}

void rst::rasterizer::rasterize_wireframe(const Triangle& t)
{
    //画三遍也就是画三条线
    draw_line(t.c(), t.a());
    draw_line(t.c(), t.b());
    draw_line(t.b(), t.a());
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
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    if (point.x() < 0 || point.x() >= width ||
        point.y() < 0 || point.y() >= height) return;
    auto ind = (height - point.y()) * width + point.x();
    frame_buf[ind] = color;
}

