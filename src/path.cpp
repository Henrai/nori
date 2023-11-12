#include <nori/integrator.h>
#include <nori/scene.h>
#include <algorithm>

#include "nori/bsdf.h"
#include "nori/warp.h"

NORI_NAMESPACE_BEGIN
class PathIntegrator : public Integrator {
public:
	PathIntegrator(const PropertyList& props) {}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
		constexpr float RR = 0.95f;
		Intersection its;
		if (!scene->rayIntersect(ray, its) || sampler->next1D() > RR) {
			return Color3f(0.f);
		}

		Color3f Le = 0.f;
		if (its.mesh->isEmitter()) {
			EmitterQueryRecord record = EmitterQueryRecord(ray.o, its.p, its.shFrame.n);
			Le = its.mesh->getEmitter()->eval(record);
		}

		Color3f dir = sampleLight(scene, sampler, ray, its);

		Color3f indir = sampleBsdf(scene, sampler, ray, its);

		return (Le + indir + dir) / RR;
	}



	std::string toString() const {
		return tfm::format("PathSimpleIntegrator");
	}
private:
	Color3f sampleLight(const Scene* scene, Sampler* sampler, const Ray3f& ray, const Intersection& its) const {
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

		float lightWeight = 0.f;
		float bsdfWeight = its.mesh->getBSDF()->pdf(bRec);
		if (coslight > Epsilon) {
		
			float tlightPdf = lightPdf * distance / coslight;
			lightWeight = 1.0f / (tlightPdf + bsdfWeight);
		}

		if (!scene->rayIntersect(eRec.shadowRay)) {
			dir = L0 * bsdf * cosShading * lightWeight;
		}

		return dir;
	}

	Color3f sampleBsdf(const Scene* scene, Sampler* sampler, const Ray3f& ray, const Intersection& its) const {
		Color3f indir = 0.f;
		BSDFQueryRecord bsdfRec = BSDFQueryRecord(its.toLocal(-ray.d));
		Color3f c = its.mesh->getBSDF()->sample(bsdfRec, sampler->next2D());
		Ray3f bsdfRay = Ray3f(its.p, its.toWorld(bsdfRec.wo));
		Intersection next_its;
		float lightWeight = 0.f;
		float bsdfWeight = its.mesh->getBSDF()->pdf(bsdfRec);
		if (scene->rayIntersect(bsdfRay, next_its) && next_its.mesh->isEmitter()) {
			float distance = (next_its.p - its.p).squaredNorm();
			float coslight =  fabs(next_its.shFrame.n.dot(ray.d));
			
			if (coslight > Epsilon) {
				lightWeight =  distance / coslight / next_its.mesh->getTotalArea();
				
			}
		}
		float weight = bsdfWeight + lightWeight > 0 ? bsdfWeight / (bsdfWeight + lightWeight) : bsdfWeight;
		indir = Li(scene, sampler, bsdfRay) * c * (its.mesh->getBSDF()->isDiffuse() ? weight : 1.f);
		return indir;
	}
};

NORI_REGISTER_CLASS(PathIntegrator, "path");
NORI_NAMESPACE_END