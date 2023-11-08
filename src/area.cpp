#include<nori/emitter.h>

#include "nori/mesh.h"

NORI_NAMESPACE_BEGIN
	class Area : public Emitter {
public:
	Area(const PropertyList& propList) {
		radiance_ = propList.getColor("radiance", Color3f(0.f));
	}

	Color3f sample(EmitterQueryRecord& record, Sampler* sampler) override
	{
		mesh_->sampleTriangle( sampler, record);
		record.shadowRay = Ray3f(record.obj, record.wi, Epsilon, (record.light - record.obj).norm() - Epsilon);
		float p = pdf(record);
		if (p > 0.0f && !std::isnan(p) && !std::isinf(p)) {
			return eval(record) / pdf(record);
		}
		return 0.f;

	}

	Color3f eval(const EmitterQueryRecord& eRec) const override
	{
		return eRec.normal.dot(eRec.wi) < 0.f ? radiance_ : 0.f;
	}

	float pdf(const EmitterQueryRecord& eRec) const override
	{
		float cosTheta = eRec.normal.dot(-eRec.wi);
		if (cosTheta > 0.0f) return eRec.pdf;
		return 0.f;
	}

	

	std::string toString() const override {
		return tfm::format("AreaLight[\n "
            "  Radiance = (%f, %f, %f) \n"
			"]",
			radiance_.x(), radiance_.y(), radiance_.z()
		);

	}

	
private:
	Color3f radiance_;
};

NORI_REGISTER_CLASS(Area, "area")
NORI_NAMESPACE_END