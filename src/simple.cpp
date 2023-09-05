#include <nori/integrator.h>
#include <nori/scene.h>
#include <algorithm>

NORI_NAMESPACE_BEGIN

class SimpleIntegrator: public Integrator {
public:
    SimpleIntegrator(const PropertyList& props) {
       mPosition = props.getPoint("position");
       mEnergy = props.getColor("energy");
    }

    /// Compute the radiance value for a given ray. Just return green here
    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {

        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            return Color3f(0.0f);
        }
        
        const auto & frame = its.shFrame;
        auto x = its.p;
        auto dir = (mPosition-x).normalized();
        bool v = !scene->rayIntersect(Ray3f(x, dir));
        if (!v) return Color3f(0.f);
        float cosTheta = std::max(0.f, frame.cosTheta(frame.toLocal(dir)));
        float norm = (x - mPosition).norm();
        return cosTheta * INV_FOURPI * INV_PI / (norm * norm) * mEnergy;
    }

    /// Return a human-readable description for debugging purposes
    std::string toString() const {
        return tfm::format(
            "SimpleIntegrator[\n"
            "  Position = (%f, %f, %f) \n"
            "]",
            mPosition.x(), mPosition.y(), mPosition.z()
        );
    }
protected:
    Vector3f mPosition;
    Color3f mEnergy;

};

NORI_REGISTER_CLASS(SimpleIntegrator, "simple");
NORI_NAMESPACE_END