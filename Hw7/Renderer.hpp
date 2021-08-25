//
// Created by goksu on 2/25/20.
//
#include "Scene.hpp"

#pragma once
struct hit_payload
{
    float tNear;
    uint32_t index;
    Vector2f uv;
    Object* hit_obj;
};

class Renderer
{
public:
    //void multithread_run(const Scene& scene, const Ray &ray, const int m, std::vector<Vector3f> *framebuffer);
    void Render(const Scene& scene);

private:
};
