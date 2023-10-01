/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#pragma once

#include <nori/object.h>

#include "mesh.h"

NORI_NAMESPACE_BEGIN

struct EmitterQueryRecord {
    /// Incident direction (in the local frame)

    Point3f obj;
    Point3f light;
    Vector3f normal;
    Vector3f wi;
    Ray3f shadowRay;
    float pdf;

    /// sampling on the light source
    EmitterQueryRecord(const Point3f& p) 
        : obj(p), pdf(1.0){}

    // shading on the light source
    EmitterQueryRecord(const Point3f& p, const Point3f& light, const Vector3f& normal)
        : obj(p), pdf(1.0), light(light), normal(normal)
    {
        wi = (light - p).normalized();
    }
};


	/**
 * \brief Superclass of all emitters
 */
class Emitter : public NoriObject {
public:

    virtual Color3f sample(EmitterQueryRecord& record, Sampler* sampler) = 0;

    virtual Color3f eval(const EmitterQueryRecord& bRec) const = 0;

    virtual float pdf(const EmitterQueryRecord& bRec) const = 0;

    /**
    * \brief Return the type of object (i.e. Mesh/Emitter/etc.)
    * provided by this instance
    * */
    EClassType getClassType() const override { return EEmitter; }

    void setParent(NoriObject* parent) override{
        mesh_ = dynamic_cast<Mesh*>(parent);
    }

	Mesh* getMesh() const {
        return mesh_;
    }

protected:
    Mesh* mesh_;
};

NORI_NAMESPACE_END
