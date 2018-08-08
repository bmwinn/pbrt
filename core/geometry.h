/*
 * z axis points into the page
 * definition of world space
 * points and vectors are absolutely separate
 */

class Vector {
public:
   Vector();
   Vector(float xx, float yy, float zz);

   bool HasNaNs();

   Vector operator+(const Vector &v) const;
   Vector operator-(const Vector &v) const;
   Vector operator*(float f) const;
   Vector operator/(float f) const;

   Vector &operator+=(const Vector &v);
   Vector &operator-=(const Vector &v);
   Vector &operator*=(float f);
   Vector &operator/=(float f);

   Vector operator-() const;
   float operator[](int i) const;
   float &operator[](int i);

   float LengthSquared() const;
   float Length() const;

   explicit Vector(const Normal &n);

public:
   float x, y, z; // set to zero by default
};


class Point {
public:
   Point();
   Point(float xx, float yy, float zz);

   Point operator+(const Vector &v);
   Vector operator-(const Point &p);
   Point operator-(const Vector &v);

   Point &operator+=(const Vector &v);
   Point &operator-=(const Vector &v);

public:
   float x, y, z;
};


// Distinguish between vectors and normals
// Normals defined in terms of their relationship to a surface
// Behave differently than vectors - ex. applying transformations
class Normal {
public:
   explicit Normal(const Vector &v);

   // TODO: implement various Vector methods for Normal
   // TODO: overload Dot and AbsDot

public:
   float x, y, z;
};


class Ray {
public:
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

public:
   Ray();
   Ray(const Point &origin, const Vector &direction, float start, float end, float t, int d); 

   // When additional rays are beign spawned from an intersection,
   // copy the time value and set depth value based on previous ray
   Ray(const Point &origin, const Vector &direction, const Ray &parent, float start, float end);
  
   // Find a point at a particular position along a ray 
   Point operator()(float t);
};


// Perform better anti aliasing
// Keep track of additional information about two auxiliary rays
// Camera rays offset one sample in the x and y direction from the main ray
class RayDifferential : public Ray {
public:
   // hasDifferentials set to false initially because neighboring rays,
   // if any, are not known
   RayDifferential();
   RayDifferential(const Point &org, const Vector &dir, float start, float end);
   RayDifferential(const Point &org, const Vector &dir, const Ray &parent, float start, float end);

   void ScaleDifferentials(float s);

public:
   bool hasDifferentials;
   Point rxOrigin, ryOrigin;
   Vector rxDirection, ryDirection;
};


// Create 3d bounding volume through axis-aligned bounding boxes 
class BBox {
public:
   BBox();
   
   // Initialize a BBox to enclose a single point
   BBox(const Point &p);

   // Pass in two corner points
   BBox(const Point &p1, const Point &p2);
  
   // Compute and return a new BBox that encompasses a point
   // and the original BBox space
   BBox Union(const BBox &b, const Point &p);

   // TODO: look up friend
   friend BBox Union(const BBox &b, const BBox &b);

   bool Overlaps(const BBox &b) const;

   bool Inside(const Point &pt) const;
   
   void Expand(float delta);

   // compute surface area of six faces of BBox
   float SurfaceArea() const;

   // compute volume of six faces of BBox
   float Volume() const;

   // which of the three BBox axes is longest
   int MaximumExtent() const;

   // TODO: implement?
   const Point &operator[](int i) const;
   Point &operator[](int i);

   // Linearly interpolate between corners of BBox by given amount
   Point Lerp(float tx, float ty, float tz) const;

   // Return position of a point relative to the corners of a box
   Vector Offset(const Point &p) const;

   // Provide center and radius of a sphere that bounds the BBox
   void BBox::BoundingSphere(Point *c, float *rad) const;

public:
   Point pMin, pMax;
};


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
