#include <nori/integrator.h>
#include <nori/scene.h>
#include <algorithm>

#include "nori/bsdf.h"

NORI_NAMESPACE_BEGIN
	class WhittedIntegrator : public Integrator {
public:
	WhittedIntegrator(const PropertyList& props){}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
		constexpr float RR = 0.95;
		Intersection its;
		if(!scene->rayIntersect(ray, its)) {
			return Color3f(0.f);
		}

		Color3f Le =  0.f;
		if (its.mesh->isEmitter()) {
			EmitterQueryRecord record = EmitterQueryRecord(ray.o, its.p,  its.shFrame.n);
			 Le = its.mesh->getEmitter()->eval(record);
		}

		if (its.mesh->getBSDF()->isDiffuse()) {

		float lightPdf;
		Emitter* light = scene->sampleLight(sampler->next1D(), lightPdf);

		
		EmitterQueryRecord eRec(its.p);
		Color3f L0 = light->sample(eRec, sampler);
		if(scene->rayIntersect( eRec.shadowRay)) {
			return Le;
		}

		
			BSDFQueryRecord bRec = BSDFQueryRecord(its.toLocal(eRec.wi), its.toLocal(-ray.d), EMeasure::ESolidAngle);
			Color3f bsdf = its.mesh->getBSDF()->eval(bRec);
			float distance = (eRec.light - its.p).squaredNorm();

			float cosShading = fabs(Frame::cosTheta(its.shFrame.toLocal(eRec.wi)));
			float coslight = fabs(eRec.normal.dot(-eRec.wi));

			return Le + L0 * bsdf * cosShading * coslight / distance / lightPdf;
		} else {
			BSDFQueryRecord bRec(its.toLocal(-ray.d));
			Color3f c = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
			if(sampler->next1D() < 0.95 && c.getLuminance() > 0.f) {
				return Li(scene, sampler, Ray3f(its.p,its.toWorld(bRec.wo))) / RR * c;
			} else {
				return 0.f;
			}
		}
	}

	std::string toString() const {
		return tfm::format("WhittedIntegrator");
	}

	
};

NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END