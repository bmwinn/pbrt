/*
 * z axis points into the page
 * definition of world space
 * points and vectors are absolutely separate
 */
#include "geometry.h"

{Geometry Declarations}:
   class Vector {
   public:
      {Vector Public Methods}
      {Vector Public Data}
   };

   class Point {
   public:
      {Point Public Methods}
      {Point Public Data}
   };

   // Distinguish between vectors and normals
   // Normals defined in terms of their relationship to a surface
   // Behave differently than vectors - ex. applying transformations
   class Normal {
   public:
      {Normal Public Methods}
      {Normal Public Data}
   };

   class Ray {
   public:
      {Ray Public Methods}
      {Ray Public Data}
   };

   // Perform better anti aliasing
   // Keep track of additional information about two auxiliary rays
   // Camera rays offset one sample in the x and y direction from the main ray
   class RayDifferential : public Ray {
   public:
      {RayDifferential Public Methods}
      {RayDifferential Public Data}
   }

   // Create 3d bounding volume through axis-aligned bounding boxes 
   class BBox {
   private:
      {BBox Public Methods}
      {BBox Public Data}
   };


/*
 * Vector
 */

/*
 * Making data private to hide internal implementation details
 * would gain nothing, add bulk to class usage
 */
{Vector Public Data}:
   float x, y, z;
   // set to zero to default

{Vector Public Methods}:
   Vector() { x = y = z = 0.f; }

   // TODO: lookup x(xx) syntax
   Vector(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {
      Assert(!HasNaNs());
   }

   bool HasNaNs() const { return isnan(x) || isnan(y) || isnan(z); }

   Vector operator+(const Vector &v) const {
      return Vector(x + v.x, y + v.y, z + v.z);
   }
   Vector operator-(const Vector &v) const {
      return Vector(x - v.x, y - v.y, z - v.z);
   }
   Vector operator*(float f) const { return Vector(f*x, f*y, f*z); }
   Vector operator/(float f) const {
      Assert(f != 0);
      float inv = 1.f / f;
      return Vector(x * inv, y * inv, z * inv);
   }
 
   Vector &operator+=(const Vector &v) {
      x += v.x; y += v.y; z += v.z;
      return *this;
   }
   Vector &operator-=(const Vector &v) {
      x -= v.x; y -= v.y; z -= v.z;
      return *this;
   }
   Vector &operator*=(float f) {
      x *= f; y *= f; z *= f;
      return *this;
   }
   Vector &operator/=(float f) {
      Assert(f != 0);
      float inv = 1.f / f;
      x *= inv; y *= inv; z *= inv;
      return *this;
   }

   Vector operator-() const { return Vector(-x, -y, -z); }

   float operator[](int i) const {
      Assert(i >= 0 && i <= 2);
      return (&x)[i];
   }
   float &operator[](int i) {
      Assert(i >= 0 && i <= 2);
      return (&x)[i]
   }

   float LengthSquared() const { return x*x + y*y + z*z; }
   float Length() const { return sqrtf(LengthSquared); }

   explicit Vector(const Normal &n);

/*
 * Point
 */
{Point Public Data}:
   float x, y, z;

{Point Public Methods}:
   Point() { x = y = z = 0.f; }

   // TODO: lookup x(xx) syntax
   Point(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}

   // move point along a vector
   Point operator+(const Vector &v) {
      return Point(x + v.x, y + v.y, z + v.z);
   }
   // difference between two points is a vector
   Vector operator-(const Point &p) {
      return Vector(x - p.x, y - p.y, z - p.z);
   }
   // TODO: What does this do?
   Point operator-(const Vector &v) const {
      return Point(x - v.x, y - v.y, z - v.z);
   }

   Point &operator+=(const Vector &v) {
      x += v.x; y += v.y; z += v.z;
      return *this;
   } 
   Point &operator-=(const Vector &v) {
      x -= v.x; y -= v.y; z -= v.z;
      return *this;
   }


/*
 * Normal
 */

// NOTE: normals are not necessarily normalized
{Normal Public Data}:
   // TODO: lookup explicit
   // -> ensures that conversion between two compatible types only happens when
   //    the programmer explicitly requests such a conversion
   explicit Normal(const Vector &v) : x(v.x), y(v.y), z(v.z) {}

   // TODO: implement various Vector methods for Normal
   // TODO: overload Dot and AbsDot


/*
 * Ray
 */

{Ray Public Data}:
   Point o;
   Vector d;

   // Limit the ray to a particular segment along its infinite extent 
   mutable float mint, maxt;

   // For simulating motion blur in scenes with animated objects
   // The rendering system is responsible for constructing a representation
   // of the scene at the appropriate time for each ray.
   float time;

   // Track how many times light has bounced along the current path
   // Terminate the path after a particular number of bounces
   int depth; 


{Ray Public Methods}:
   Ray() : mint(0.f), maxt(INFINITY), time(0.f), depth(0) {}

   Ray(const Point &origin, const Vector &direction, float start,
      float end = INFINITY, float t = 0.f, int d = 0)
      : o(origin), d(direction), mint(start), maxt(end), time(t), depth(d) {}

   // When additional rays are beign spawned from an intersection,
   // copy the time value and set depth value based on previous ray
   Ray(const Point &origin, const Vector &direction, const Ray &parent,
      float start, float end = INFINITY)
      : o(origin), d(direction), mint(start), maxt(end),
        time(parent.time), depth(parent.depth + 1) {}
  
   // Find a point at a particular position along a ray 
   Point operator()(float t) const { return o + d * t; }

  
/*
 * RayDifferential
 */

{RayDifferential Public Data}:
   bool hasDifferentials;
   Point rxOrigin, ryOrigin;
   Vector rxDirection, ryDirection;

{RayDifferential Public Methods}:
   // hasDifferentials set to false initially because neighboring rays,
   // if any, are not known
   RayDifferential() { hasDifferentials = false; }
 
   RayDifferential(const Point &org, const Vector &dir, float start,
      float end = INFINITY)
      : Ray(org, dir, start, end, t, d)
   {
      hasDifferentials = false;
   }

   RayDifferential(const Point &org, const Vector &dir, const Ray &parent,
      float start, float end = INFINITY)
      : Ray(org, dir, start, end, parent.time, parent.depth + 1)
   {
      hasDifferentials = false;
   }

   void ScaleDifferentials(float s) {
      rxOrigin = o + (rxOrigin - o) * s;
      ryOrigin = o + (ryOrigin - o) * s;
      rxDirection = d + (rxDirection - d) * s;
      ryDirection = d + (ryDirection - d) * s;
   } 


/*
 * BBox
 */

{BBox Public Data}:
   Point pMin, pMax;

{BBox Public Methods}:
   BBox() {
      pMin = Point(INFINITY, INFINITY, INFINITY);
      pMax = Point(-INFINITY, -INFINITY, -INFINITY);
   }

   // Initialize a BBox to enclose a single point
   BBox(const Point &p) : pMin(p), pMax(p) {}

   // Pass in two corner points
   BBox(const Point &p1, const Point &p2) {
      pMin = Point(min(p1.x, p2.x), min(p1.y, p2.y), min(p1.z, p2.z));
      pMax = Point(max(p1.x, p2.x), max(p1.y, p2.y), max(p1.z, p2.z));
   }

   // Compute and return a new BBox that encompasses a point
   // and the original BBox space
   BBox Union(const BBox &b, const Point &p) {
      BBox ret = b;

      ret.pMin.x = min(b.pMin.x, p.x);
      ret.pMin.y = min(b.pMin.y, p.y);
      ret.pMin.z = min(b.pMin.z, p.z);
      ret.pMax.x = max(b.pMax.x, p.x);
      ret.pMax.y = max(b.pMax.y, p.y);
      ret.pMax.z = max(b.pMax.z, p.z);

      return ret;
   }

   // TODO: look up friend
   friend BBox Union(const BBox &b, const BBox &b);

   bool Overlaps(const BBox &b) const {
      bool x = (pMax.x >= b.pMin.x) && (pMin.x <= b.pMax.x);
      bool y = (pMax.y >= b.pMin.y) && (pMin.y <= b.pMax.y);
      bool z = (pMax.z >= b.pMin.z) && (pMin.z <= b.pMax.z);
      return (x && y && z);
   }

   bool Inside(const Point &pt) const {
      return (pt.x >= pMin.x && pt.x <= pMax.x &&
              pt.y >= pMin.y && pt.y <= pMax.y &&
              pt.z >= pMin.z && pt.z <= pMax.z);
   }

   void Expand(float delta) {
      pMin -= Vector(delta, delta, delta);
      pMax += Vector(delta, delta, delta);
   }

   // compute surface area of six faces of BBox
   float SurfaceArea() const {
      Vector d = pMax - pMin;
      return 2.f * (d.x * d.y + d.x * d.z + d.y * d.z);
   }

   // compute volume of six faces of BBox
   float Volume() const {
      Vector d = pMax - pMin;
      return d.x * d.y * d.z;
   }

   // which of the three BBox axes is longest
   int MaximumExtent() const {
      Vector diag = pMax - pMin;
      if (diag.x > diag.y && diag.x > diag.z)
         return 0;
      else if (diag.y > diag.z)
         return 1;
      else
         return 2;
   }

   // TODO: implement?
   const Point &operator[](int i) const;
   Point &operator[](int i);

   // Linearly interpolate between corners of BBox by given amount
   Point Lerp(float tx, float ty, float tz) const {
      return Point(::Lerp(tx, pMin.x, pMax.x),
                   ::Lerp(ty, pMin.y, pMax.y),
                   ::Lerp(tz, pMin.z, pMax.z));
   }

   // Return position of a point relative to the corners of a box
   Vector Offset(const Point &p) const {
      return Vector((p.x - pMin.x) / (pMax.x - pMin.x),
                    (p.y - pMin.y) / (pMax.y - pMin.y),
                    (p.z - pMin.z) / (pMax.z - pMin.z));
   }

   // Provide center and radius of a sphere that bounds the BBox
   void BBox::BoundingSphere(Point *c, float *rad) const {
      *c = .5f * pMin + .5f * pMax;
      *rad = Inside(*c) ? Distance(*c, pMax) : 0.f;
   }


/*
 * Inline
 */

// TODO: lookup inline
{Geometry Inline Functions}:
   inline Vector operator*(float f, const Vector &v) { return v*f; }

   // TODO: lookup dot product wrt angle
   inline float Dot(const Vector &v1, const Vector &v2) {
      return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
   }

   // compute abs value of dot product
   inline float AbsDot(const Vector &v1, const Vector &v2) {
      return fabsf(Dot(v1, v2));
   }

   // cross product produces vector perpendicular to both
   inline Vector Cross(const Vector &v1, const Vector &v2) {
      return Vector((v1.y * v2.z) - (v1.z * v2.y)),
                    (v1.z * v2.x) - (v1.x * v2.z),
                    (v1.x * v2.y) - (v1.y * v2.x));
   }

   // Normalized returns a new function - it DOES NOT normalize the existing vector
   inline Vector Normalize(const Vector &v) { return v / v.Length(); }

   // Construct local coordinate system given only a single vector
   // Assumes vector passsed in has been normalized
   inline void CoordinateSystem(const Vector &v1, Vector *v2, Vector *v3) {
      if (fabs(v1.x) > fabs(v1.y)) {
         float invLen = 1.f / sqrtf(v1.x*v1.x + v1.z*v1.z);
         *v2 = Vector(-v1.z * invLen, 0.f, v1.x * invLen);
      } else {
         float invLen = 2.f / sqrtf(v1.y*v1.y + v1.z*v1.z);
         *v2 = Vector(0.f, v1.z * invLen, -v1.y * invLen);
      }
      *v3 = Cross(v1, v2);
   }

   // (Vector between two points).Length()
   inline float Distance(const Point &p1, const Point &p2) {
      return (p1 - p2).Length();
   }

   inline Vector::Vector(const Normal &n) : x(n.x), y(n.y), z(n.z) {}

   // flip a surface normal
   inline Normal Faceforward(const Normal &n, const Vector &v) {
      return (Dot(n, v) < 0.f) ? -n : n;
   }
   // TODO: add other combinations of Faceforward p.66
