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
		constexpr float RR = 0.95f;
		Intersection its;
		if (!scene->rayIntersect(ray, its)) {
			return Color3f(0.f);
		}

		Color3f Le = 0.f;
		if (its.mesh->isEmitter()) {
			EmitterQueryRecord record = EmitterQueryRecord(ray.o, its.p, its.shFrame.n);
			Le = its.mesh->getEmitter()->eval(record);
		}

		if (its.mesh->getBSDF()->isDiffuse()) {

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
			Vector3f sample = Warp::squareToUniformHemisphere(sampler->next2D());
			Color3f indir = 0.f;
			Ray3f shadingRay = Ray3f(its.p, its.toWorld(sample));

			bool isIntersect = scene->rayIntersect(shadingRay);
			
			if (sampler->next1D() < RR && isIntersect) {
				Color3f li = Li(scene, sampler, shadingRay);
				BSDFQueryRecord indERec = BSDFQueryRecord(its.toLocal(ray.d), sample, EMeasure::ESolidAngle);
				indir = li * its.mesh->getBSDF()->eval(indERec) * std::max(0.f, Frame::cosTheta(sample)) / Warp::squareToUniformHemispherePdf(sample) / RR;
			}
			if (!dir.isValid()) {
				cout << "dir: " << bsdf.toString() << " " << cosShading << " " << coslight << " " << distance << " " << lightPdf << endl;
			}
			
			return Le + indir  + dir;
		}
		else {
			BSDFQueryRecord bRec(its.toLocal(-ray.d));
			Color3f c = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
			if (sampler->next1D() < RR && c.getLuminance() > 0.f) {
				return Li(scene, sampler, Ray3f(its.p, its.toWorld(bRec.wo))) / RR * c;
			}
			else {
				return 0.f;
			}
		}
	}

	std::string toString() const {
		return tfm::format("PathSimpleIntegrator");
	}
private:

};

NORI_REGISTER_CLASS(PathSimpleIntegrator, "path_simple");
NORI_NAMESPACE_END