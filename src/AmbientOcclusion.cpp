#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class AmbientOcclusion: public Integrator {
public:
    AmbientOcclusion(const PropertyList& props) {
    }
/*
    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            return Color3f(0.0f);
        }
        const auto& frame = its.shFrame;
        auto rng = sampler->next2D();//单位正方形内均匀分布的点
        auto h = Warp::squareToCosineHemisphere(rng);//变换到单位半球余弦上
        auto p = frame.toWorld(h);//因为是切线空间中的点，所以要变换到世界空间
        auto pN = p.normalized();
        int visiblity = 0;
        if (!scene->rayIntersect(Ray3f(its.p + pN * 0.00001f, pN))) {
            visiblity = 1;
        }
        return Color3f(float(visiblity));
    }*/

    
    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);
        const auto& frame = its.shFrame;
        auto sample =  sampler->next2D();
        Vector3f dir = frame.toWorld(Warp::squareToCosineHemisphere(sample));
        bool v = !scene->rayIntersect(Ray3f(its.p, dir));
        if (!v) return Color3f(0.f);
        float cosTheta = std::max(0.f, frame.cosTheta(frame.toLocal(dir)));

        return 1;
    }
    std::string toString() const {
        return "AmbientOcclusion[]";
    }
};

NORI_REGISTER_CLASS(AmbientOcclusion, "ao");

NORI_NAMESPACE_END