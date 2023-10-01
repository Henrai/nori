#include <nori/integrator.h>
#include <nori/scene.h>
#include <algorithm>

#include "nori/bsdf.h"

NORI_NAMESPACE_BEGIN
	class WhittedIntegrator : public Integrator {
public:
	WhittedIntegrator(const PropertyList& props){}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
		Intersection its;
		if(!scene->rayIntersect(ray, its)) {
			return Color3f(0.f);
		}

		Color3f Le =  0.f;
		if (its.mesh->isEmitter()) {
			EmitterQueryRecord record = EmitterQueryRecord(ray.o, its.p,  its.shFrame.n);
			 Le = its.mesh->getEmitter()->eval(record);
		}


		EmitterQueryRecord eRec(its.p);

		Emitter* light = scene->sampleLight(sampler->next1D(), eRec);

		

		Color3f Li = light->sample(eRec, sampler);
		if(scene->rayIntersect( eRec.shadowRay)) {
			return Le;
		}

		BSDFQueryRecord bRec = BSDFQueryRecord(its.toLocal(-ray.d), its.toLocal(eRec.wi), EMeasure::ESolidAngle);
		Color3f bsdf = its.mesh->getBSDF()->eval(bRec);
		float distance = (eRec.light - its.p).squaredNorm();

		float cosShading = fabs( Frame::cosTheta(its.shFrame.toLocal(eRec.wi)));
		float coslight = fabs(eRec.normal.dot(-eRec.wi));

		return Le + Li * bsdf * cosShading * coslight / distance / eRec.pdf;
		
	}

	std::string toString() const {
		return tfm::format("WhittedIntegrator");
	}
};

NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END