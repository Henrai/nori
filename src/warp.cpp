/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}


float tent(float u) {
    if (u >= 0 && u < 0.5) {
        return sqrt(2 * u) - 1;
    }
    else if (u >= 0.5 && u < 1) {
        return 1 - sqrt(2- 2 * u);
    }
    return 0.f;
}

Point2f Warp::squareToTent(const Point2f &sample) {
    return Point2f(tent(sample.x()), tent(sample.y()));
}


float tentPdf(float u) {
    return u >= -1 && u <= 1 ? 1 - abs(u) : 0.f;
}

float Warp::squareToTentPdf(const Point2f &p) {
    return  tentPdf(p.x()) * tentPdf(p.y());
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    float r = sqrt(sample.x());
    float theta = 2 * M_PI * sample.y();
    return Point2f(r * sin(theta), r * cos(theta));
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    return p.squaredNorm() <= 1.f ? 1.f / M_PI : 0.f;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    float theta = acos(1 - 2 * sample.x());
    float phi = 2 * M_PI * sample.y();
    float sinTheta = sin(theta);
    float cosTheta = cos(theta);
    float sinPhi = sin(phi);
    float cosPhi = cos(phi);

    return Vector3f(sinTheta * cosPhi, sinTheta * sinPhi, cosTheta);
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    return INV_FOURPI;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    float theta = acos(1 - sample.x());
    float phi = 2 * M_PI * sample.y();
    float sintheta = sin(theta);
    float costheta = cos(theta);
    float sinphi = sin(phi);
    float cosphi = cos(phi);

    return Vector3f(sintheta * cosphi, sintheta * sinphi, costheta);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    return v.z() >= 0 ? INV_TWOPI : 0;
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    Point2f disk = squareToUniformDisk(sample);
    float x = disk.x();
    float y = disk.y();
    return Vector3f(x, y, sqrt(1 - x * x - y * y));
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    return v.z() >= 0.f ? v.z() * INV_PI : 0.f;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    float phi = M_PI * 2 * sample.x();
    float theta = atan(sqrt(-alpha * alpha * log(1 - sample.y())));
    float cosPhi = cos(phi);
    float sinPhi = sin(phi);
    float cosTheta = cos(theta);
    float sinTheta = sin(theta);
    float x = sinTheta * cosPhi;
    float y = sinTheta * sinPhi;
    float z = cosTheta;
    return Vector3f(x, y, z);
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    if (m.z() <= 0) {
        return 0;
    }
    float alpha2 = alpha * alpha;
    float cosTheta = m.z();
    float tanTheta2 = (m.x() * m.x() + m.y() * m.y()) / (cosTheta * cosTheta);
    float cosTheta3 = cosTheta * cosTheta * cosTheta;
    float azimuthal = INV_PI;
    float longitudinal = exp(-tanTheta2 / alpha2) / (alpha2 * cosTheta3);
    return azimuthal * longitudinal;
}


Vector3f Warp::squreToTriangle(const Point2f& sample) {
    float alpha = 1 - sqrt(1 - sample.x());
    float beta = sample.y() * sqrt(1 - sample.x());
    float gama = 1 - alpha - beta;

    return { alpha, beta, gama};
}

NORI_NAMESPACE_END
