#include "bsdf.h"

#include <iostream>
#include <algorithm>
#include <utility>

using std::min;
using std::max;
using std::swap;

namespace CGL {

void make_coord_space(Matrix3x3& o2w, const Vector3D& n) {

    Vector3D z = Vector3D(n.x, n.y, n.z);
    Vector3D h = z;
    if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z)) h.x = 1.0;
    else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z)) h.y = 1.0;
    else h.z = 1.0;

    z.normalize();
    Vector3D y = cross(h, z);
    y.normalize();
    Vector3D x = cross(z, y);
    x.normalize();

    o2w[0] = x;
    o2w[1] = y;
    o2w[2] = z;
}


Spectrum DiffuseBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  // Part 3, Task 1: 
  // This function takes in both wo and wi and returns the evaluation of
  // the BSDF for those two directions.
  return reflectance/PI;
}

Spectrum DiffuseBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // Part 3, Task 1: 
  // This function takes in only wo and provides pointers for wi and pdf,
  // which should be assigned by this function.
  // After sampling a value for wi, it returns the evaluation of the BSDF
  // at (wo, *wi).

  *pdf = 1.0 / PI;
  *wi  = sampler.get_sample(pdf);
  return reflectance/PI;
}

// Mirror BSDF //

Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

  // TODO: 3-2 Part 1 Task 2
  // Implement MirrorBSDF
  *pdf =  float(1.0);
  reflect(wo, wi);
  return reflectance / abs_cos_theta(*wi);

}


// Microfacet BSDF //

Spectrum MicrofacetBSDF::F(const Vector3D& wi) {
    // TODO: proj3-2, part 2
    // Compute Fresnel term for reflection on dielectric-conductor interface.
    // You will need both eta and etaK, both of which are Spectrum.
    double cos_theta_i = cos_theta(wi);
    Spectrum Rs = ((eta*eta + k*k) - 2*eta*cos_theta_i + cos_theta_i*cos_theta_i)/
                ((eta*eta + k*k) + 2*eta*cos_theta_i + cos_theta_i*cos_theta_i);

    Spectrum Rp = ((eta*eta + k*k)*cos_theta_i*cos_theta_i - 2*eta*cos_theta_i + 1)/
                ((eta*eta + k*k)*cos_theta_i*cos_theta_i + 2*eta*cos_theta_i + 1);
    return (Rs + Rp)/2.0;
}

double MicrofacetBSDF::G(const Vector3D& wo, const Vector3D& wi) {
    return 1.0 / (1.0 + Lambda(wi) + Lambda(wo));
}

double MicrofacetBSDF::D(const Vector3D& h) {
    // TODO: proj3-2, part 2
    // Compute Beckmann normal distribution function (NDF) here.
    // You will need the roughness alpha.
    double tan_h = sin_theta(h)/cos_theta(h);
    double alpha_sqr = pow(alpha, 2);
    return exp( (-1.0 * tan_h*tan_h)/alpha_sqr ) / (PI * alpha_sqr * pow(cos_theta(h), 4 ));
    
}

Spectrum MicrofacetBSDF::f(const Vector3D& wo, const Vector3D& wi) {
    // TODO: proj3-2, part 2
    // Implement microfacet model here.
    Vector3D n = Vector3D(0.0, 0.0, 1.0);
    if (wi.z <= 0.0 || wo.z <= 0.0) {
      return Spectrum();
    }
    Vector3D h = (wi + wo)/((wi + wo).norm());
    
    Spectrum s =  F(wi)*G(wo, wi)*D(h) / (4.0*dot(n,wo)*dot(n,wi));    
    return s;
    
}

Spectrum MicrofacetBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
    // TODO: proj3-2, part 2
    // *Importance* sample Beckmann normal distribution function (NDF) here.
    // Note: You should fill in the sampled direction *wi and the corresponding *pdf,
    //       and return the sampled BRDF value.

    double r1, r2;
    Vector2D tmp = sampler.get_sample();
    r1 = tmp.x; r2 = tmp.y;
    double theta_h = atan(sqrt(-1.0*alpha*alpha*log(1.0 - r1) ));

    
    double phi_h = 2*PI*r2;
    Vector3D h = Vector3D(cos(phi_h) * sin(theta_h),  sin(phi_h) * sin(theta_h), cos(theta_h));
    if (h.z <= 0 || dot(wo, h) <= 0) {
      *pdf = float(0);
      return Spectrum();
    }

    *wi = -wo + 2 * dot(wo, h) * h;
    if (wi->z <= 0 || dot(*wi, h) <= 0 ) {
      *pdf = float(0);
      return Spectrum();
    }

    double p_theta = ( ( 2*sin(theta_h) )/( alpha*alpha*pow(cos(theta_h), 3 ) ) )
      *exp(-1.0 * pow(sin(theta_h)/cos(theta_h), 2 )/(alpha*alpha));
    double p_phi = 1.0/(2.0*PI);
    double p_omega_h = p_theta*p_phi/sin(theta_h);

    *pdf = p_omega_h/(4.0*(dot(*wi, h)));

    
    if (*pdf < 0) {
      *pdf = float(0);
      return Spectrum();
    }

    // *pdf = 1.0/(2.0*PI);
    // *wi = cosineHemisphereSampler.get_sample(pdf);
    return MicrofacetBSDF::f(wo, *wi);
}


// Refraction BSDF //

Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

  
  return Spectrum();
}

// Glass BSDF //

Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

  // TODO: 3-2 Part 1 Task 4
  // Compute Fresnel coefficient and either reflect or refract based on it.
  
    if (refract(wo, wi, ior)) {
      double R_0 = pow(  ((1.0 - ior)/(1.0 + ior)) , 2);
      double R = R_0 + (1.0 - R_0)*( pow( 1.0 - abs_cos_theta(wo),5) );
      
      if (coin_flip(R)) {
        reflect(wo, wi);
        *pdf = R;
        return R * reflectance / abs_cos_theta(*wi);
      } else {
        *pdf = 1.0 - R;
        double eta;
        if (wo.z >= 0.0) {
          eta = 1.0/ior;
        } else {
          eta = ior;
        }
        return (1.0 - R) * transmittance / abs_cos_theta(*wi) / (eta*eta);
      }
  } else {
    reflect(wo, wi);
    *pdf =  float(1.0);
    return reflectance / abs_cos_theta(*wi);
  } 
}

void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {

  // TODO: 3-2 Part 1 Task 1
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
  *wi = Vector3D(-wo[0],-wo[1],wo[2]);

}

bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {

  // TODO: 3-2 Part 1 Task 3
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When wo.z is positive, then wo corresponds to a
  // ray entering the surface through vacuum.

  double eta;
  if (wo.z > 0.0) {
    eta = 1.0/ior;
    if ( (1.0 - eta*eta*(1.0 - wo.z*wo.z)) < 0.0 ) {
      return false;
    } else {
      *wi = Vector3D(-1.0*eta*wo.x, -1.0*eta*wo.y, -1.0*sqrt( (1.0 - eta*eta*(1.0 - wo.z*wo.z)) ));
      return true;
    }
  } else {
      eta = ior;
      if ((1.0 - eta*eta*(1.0 - wo.z*wo.z)) < 0.0) {
        return false;
      } else {
        *wi = Vector3D(-1.0*eta*wo.x, -1.0*eta*wo.y, sqrt( (1.0 - eta*eta*(1.0 - wo.z*wo.z) )));
        return true;
      }
  }
  return true;

}

// Emission BSDF //

Spectrum EmissionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum EmissionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *pdf = 1.0 / PI;
  *wi  = sampler.get_sample(pdf);
  return Spectrum();
}

} // namespace CGL
