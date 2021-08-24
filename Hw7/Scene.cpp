//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject) const
{
    *hitObject = nullptr;    
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    /*
    float tNear;
    uint32_t index;
    Object *hitObject;
    trace(ray, objects, tNear, index, &hitObject);
    */

    // TO DO Implement Path Tracing Algorithm here
    Intersection viewpoint = intersect(ray);
    if (!viewpoint.happened)
        return Vector3f();

    for (uint32_t k = 0; k < objects.size(); ++k) 
    {
        if (objects[k] == viewpoint.obj)
            std::cout << k << "\n";
    }
    Vector3f p = viewpoint.coords;
    Vector3f N = viewpoint.normal;

    if (viewpoint.m->Kd.x == 0.63)
        std::cout << "exm\n";

    Intersection inter;
    float pdf_light;
    sampleLight(inter, pdf_light);

    Vector3f x = inter.coords;
    Vector3f NN = inter.normal;
    Vector3f emit = inter.emit;

    Vector3f wo = ray.direction;
    Vector3f ws = (p-x).normalized();

    Vector3f L_dir(0.0);
    Intersection anyobj = intersect(Ray(x, ws));
    if (anyobj.distance - (p-x).norm() > -0.001)
    {
        L_dir = emit * viewpoint.m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(ws, NN) / (p - x).norm2() / pdf_light;
    }

    return L_dir;
}