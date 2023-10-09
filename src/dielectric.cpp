/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/bsdf.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

/// Ideal dielectric BSDF
class Dielectric : public BSDF {
public:
    Dielectric(const PropertyList &propList) {
        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);
    }

    Color3f eval(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return Color3f(0.0f);
    }

    float pdf(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return 0.0f;
    }

    static Vector3f refract(const Vector3f& wi,  Vector3f& n, float& eta) {
        float cosTheta_i = Frame::cosTheta(wi);
        if (cosTheta_i < 0.f) {
            eta =1.f / eta;
            n.z() = -1.0f;
            cosTheta_i = -cosTheta_i;
        }
        float sin2Theta_i = 1 - cosTheta_i * cosTheta_i;
        float sin2Theta_t = sin2Theta_i / (eta * eta);
        if (sin2Theta_t >  1.0f)
            return Vector3f(0.0f);
        float cosTheta_t = sqrt(1 - sin2Theta_t);
        return -wi / eta + (cosTheta_i / eta - cosTheta_t) * n;
    }

    Color3f sample(BSDFQueryRecord& bRec, const Point2f& sample) const {
        bRec.measure = EDiscrete;
        float cosTheta_i = Frame::cosTheta(bRec.wi);
        float kr = fresnel(cosTheta_i, m_extIOR, m_intIOR);
        if (sample.x() < kr) {
            bRec.wo = Vector3f(-bRec.wi.x(), -bRec.wi.y(), bRec.wi.z());
            bRec.eta = 1.f;
            return Color3f(1.0f);
        } else {
            Vector3f n = Vector3f(0.0f, 0.0f, 1.0f);
            bRec.eta = m_intIOR / m_extIOR;
            bRec.wo = refract(bRec.wi, n, bRec.eta);
            return Color3f(1.0f);
        }
    }

    std::string toString() const {
        return tfm::format(
            "Dielectric[\n"
            "  intIOR = %f,\n"
            "  extIOR = %f\n"
            "]",
            m_intIOR, m_extIOR);
    }
private:
    float m_intIOR, m_extIOR;
};

NORI_REGISTER_CLASS(Dielectric, "dielectric");
NORI_NAMESPACE_END
