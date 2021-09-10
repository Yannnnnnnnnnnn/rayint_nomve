/*
 * Copyright (C) 2015, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */
#define TINYOBJLOADER_IMPLEMENTATION

#include <iostream>

#include <opencv2/opencv.hpp>

#include "obj_loader.h"

#include <acc/bvh_tree.h>
typedef acc::BVHTree<unsigned int, math::Vec3f> BVHTree;


int main(int argc,char *argv[])
{

    // loading a obj file
    std::string obj_file = "../../../data/sphere.obj";

    // using tinyobj to load the file
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string warn;
    std::string err;
    bool ret = tinyobj::LoadObj(
        &attrib, 
        &shapes, 
        &materials, 
        &warn, 
        &err,
        obj_file.c_str(),
        NULL,
        false);

    if (!warn.empty()) 
    {
        std::cout << "WARN: " << warn << std::endl;
    }

    if (!err.empty()) 
    {
        std::cerr << "ERR: " << err << std::endl;
    }

    if (!ret) 
    {
        printf("Failed to load/parse .obj.\n");
        return false;
    }

    // re-organise the data to meet the demand of rayint
    std::vector<unsigned int> faces;
    std::vector<math::Vec3f> vertices;
    std::vector<math::Vec3f> normals;

    vertices.reserve(attrib.vertices.size()/3);
    for (size_t v = 0; v < attrib.vertices.size() / 3; v++) 
    {
        vertices.emplace_back(
            static_cast<const double>(attrib.vertices[3 * v + 0]),
            static_cast<const double>(attrib.vertices[3 * v + 1]),
            static_cast<const double>(attrib.vertices[3 * v + 2]));
    }
    normals.reserve(attrib.normals.size() / 3);
    for (size_t v = 0; v < attrib.normals.size() / 3; v++) 
    {
        normals.emplace_back(
            static_cast<const double>(attrib.normals[3 * v + 0]),
            static_cast<const double>(attrib.normals[3 * v + 1]),
            static_cast<const double>(attrib.normals[3 * v + 2]));
    }

    size_t face_cnt = 0;
    for (size_t i = 0; i < shapes.size(); i++)
    {
        face_cnt += shapes[i].mesh.indices.size();
    }
    faces.reserve(face_cnt);
    for (size_t i = 0; i < shapes.size(); i++)
    {
        for (size_t j = 0; j < shapes[i].mesh.indices.size(); j++)
            faces.push_back(shapes[i].mesh.indices[j].vertex_index);
    }

    // obj info
    std::cout<<"obj loaded"<<std::endl;
    std::cout<<"face num: "<<faces.size()<<std::endl;
    std::cout<<"vert num: "<<vertices.size()<<std::endl;

    // building bvh tree to speed up
    BVHTree bvhtree(faces, vertices);
    std::cout << "building tree" << std::endl;

    // setting a camera
    float pixel_space_resolution = 0.002;
    int width = 1000;
    int height = 1000;
    float view_height = 4;
    math::Vec3f view_direction(0,0,-1);
    cv::Mat mat = cv::Mat::zeros(cv::Size(width,height),CV_32FC1);

    // check the depth
    for(int y=0;y<height;y++)
    {
        for(int x=0;x<width;x++)
        {
            float view_x = (x-500)*pixel_space_resolution;
            float view_y = (y-500)*pixel_space_resolution;
            float view_z = view_height;

            BVHTree::Ray ray;
            ray.origin = math::Vec3f(view_x,view_y,view_z);
            ray.dir = view_direction;
            ray.tmin = 0.0f;
            ray.tmax = 10;

            BVHTree::Hit hit;
            if (bvhtree.intersect(ray, &hit)) 
            {
                // math::Vec3f const & n1 = normals[faces[hit.idx * 3 + 0]];
                // math::Vec3f const & n2 = normals[faces[hit.idx * 3 + 1]];
                // math::Vec3f const & n3 = normals[faces[hit.idx * 3 + 2]];
                // math::Vec4f const & c1 = colors[faces[hit.idx * 3 + 0]];
                // math::Vec4f const & c2 = colors[faces[hit.idx * 3 + 1]];
                // math::Vec4f const & c3 = colors[faces[hit.idx * 3 + 2]];
                // math::Vec3f const & w = hit.bcoords;
                // math::Vec4f color = math::interpolate(c1, c2, c3, w[0], w[1], w[2]);
                // math::Vec3f normal = math::interpolate(n1, n2, n3, w[0], w[1], w[2]).normalize();
                // for (std::size_t c = 0; c < 3; ++c) 
                // {
                //     uvmap->at(x, y, c) = 255.0f * color[c];
                //     normalmap->at(x, y, c) = 255.0f * (0.5f + normal[c] / 2.0f);
                // }
                mat.at<float>(y,x) = hit.t;
            }
        }
    }

    // viz the depth map
    mat = 255*(mat-2)/2;
    cv::Mat save_depth;
    mat.convertTo(save_depth,CV_8UC1);
    cv::applyColorMap(save_depth,save_depth,cv::COLORMAP_JET);
    cv::imwrite("depth.png",save_depth);

    return 0;
}