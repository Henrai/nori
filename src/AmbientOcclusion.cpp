#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class AmbientOcclusion: public Integrator {
public:
    AmbientOcclusion(const PropertyList& props) {
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);
        const auto& frame = its.shFrame;
        auto sample =  sampler->next2D();
        Vector3f dir = frame.toWorld(Warp::squareToCosineHemisphere(sample));
        bool v = !scene->rayIntersect(Ray3f(its.p, dir));
        if (!v) return Color3f(0.f);

        return 1;
    }

    std::string toString() const {
        return "AmbientOcclusion[]";
    }
};

NORI_REGISTER_CLASS(AmbientOcclusion, "ao");

NORI_NAMESPACE_END
