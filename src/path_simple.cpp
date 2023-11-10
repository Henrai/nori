#include <nori/integrator.h>
#include <nori/scene.h>
#include <algorithm>

#include "nori/bsdf.h"
#include "nori/warp.h"

NORI_NAMESPACE_BEGIN
class PathSimpleIntegrator : public Integrator {
public:
	PathSimpleIntegrator(const PropertyList& props) {}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
		return Li(scene, sampler, ray, 0, false);
	}
	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray, int depth, bool isdiffuse) const {
		constexpr float RR = 0.95f;
		Intersection its;
		if (!scene->rayIntersect(ray, its) || sampler->next1D() > RR) {
			return Color3f(0.f);
		}

		Color3f Le = 0.f;
		if (its.mesh->isEmitter() && ( depth == 0 || !isdiffuse)) {
			EmitterQueryRecord record = EmitterQueryRecord(ray.o, its.p, its.shFrame.n);
			Le = its.mesh->getEmitter()->eval(record);
		}

	
		float lightPdf;
		Emitter* light = scene->sampleLight(sampler->next1D(), lightPdf);


		EmitterQueryRecord eRec(its.p);
		Color3f L0 = light->sample(eRec, sampler);

		BSDFQueryRecord bRec = BSDFQueryRecord(its.toLocal(eRec.wi), its.toLocal(-ray.d), EMeasure::ESolidAngle);
		Color3f bsdf = its.mesh->getBSDF()->eval(bRec);
		float distance = (eRec.light - its.p).squaredNorm();

		float cosShading = fabs(Frame::cosTheta(its.shFrame.toLocal(eRec.wi)));
		float coslight = fabs(eRec.normal.dot(-eRec.wi));
		Color3f dir = 0.f;
		if (!scene->rayIntersect(eRec.shadowRay)) {
			dir = L0 * bsdf * cosShading * coslight / distance / lightPdf;
		}

		Color3f indir = 0.f;
	
		BSDFQueryRecord bsdfRec = BSDFQueryRecord(its.toLocal(-ray.d));
		Color3f c = its.mesh->getBSDF()->sample(bsdfRec, sampler->next2D());
		Ray3f bsdfRay = Ray3f(its.p, its.toWorld(bsdfRec.wo));
		
		indir = Li(scene, sampler, bsdfRay, depth + 1, its.mesh->getBSDF()->isDiffuse()) *c;
		
		return (Le + indir + dir)/RR;
	}

	std::string toString() const {
		return tfm::format("PathSimpleIntegrator");
	}
private:

};

NORI_REGISTER_CLASS(PathSimpleIntegrator, "path_simple");
NORI_NAMESPACE_END