///////////////////////////////////////////////////////////////////////
// A framework for a raytracer.
////////////////////////////////////////////////////////////////////////
#include <limits>
#include <Eigen_unsupported/Eigen/BVH>
#undef max
#undef min
const float INF = std::numeric_limits<float>::max();
typedef Eigen::AlignedBox<float, 3> Box3d; // The BV type provided by Eigen
class Shape;
class IntersectRecord;
class Ray;
class Material;
class Interval;
class Minimizer;
const float PI = 3.14159f;
const float EPSILON = 0.0001;
const Vector3f ZEROES = Vector3f(0, 0, 0);
const Vector3f ZAXIS = Vector3f(0, 0, 1);
const Vector3f YAXIS = Vector3f(0, 1, 0);
const Vector3f XAXIS = Vector3f(1, 0, 0);




class Material
{
public:
	Vector3f Kd, Ks;
	float alpha;
	unsigned int texid;

	virtual bool isLight() { return false; }

	Material() : Kd(Vector3f(1.0, 0.5, 0.0)), Ks(Vector3f(1, 1, 1)), alpha(1.0), texid(0) {}
	Material(const Vector3f d, const Vector3f s, const float a)
		: Kd(d), Ks(s), alpha(a), texid(0) {}
	Material(Material& o) { Kd = o.Kd;  Ks = o.Ks;  alpha = o.alpha;  texid = o.texid; }

	virtual Material& operator=(Material& other)
	{
		Kd = other.Kd;
		Ks = other.Ks;
		alpha = other.alpha;
		texid = other.texid;

		return *this;
	}


	void setTexture(const std::string path);
	//virtual void apply(const unsigned int program);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};



class Camera
{

public:
	// Camera/viewing parameters
	Vector3f ambient;
	Vector3f eye;      // Position of eye for viewing scene
	Quaternionf orient;   // Represents rotation of -Z to view direction
	float ry;
	float front, back;
	float spin, tilt;
	float cDist;              // Distance from eye to center of scene
							  //float lightSpin, lightTilt, lightDist;

	int width, height;
	void setScreen(const int _width, const int _height) { width = _width;  height = _height; }
	void setCamera(const Vector3f& _eye, const Quaternionf& _o, const float _ry)
	{
		eye = _eye; orient = _o; ry = _ry;
	}
	void setAmbient(const Vector3f& _a) { ambient = _a; }


	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

};




class Interval
{
public:


	Vector3f normal0, normal1;
	float t0, t1;


	Interval(float t00, float t11, Vector3f n0, Vector3f n1) : t0(t00), t1(t11), normal0(n0), normal1(n1) { if (t0 > t1) { float temp = t1; t1 = t0; t0 = temp; Vector3f temp1 = normal0; normal0 = normal1; normal1 = temp1; } }
	Interval(float x, float y) : t0(x), t1(y), normal0(0, 0, 0), normal1(normal0) { if (t0 > t1) { float temp = t1; t1 = t0; t0 = temp; Vector3f temp1 = normal0; normal0 = normal1; normal1 = temp1; } }
	Interval() : t0(1), t1(0), normal0(Vector3f(0,0,0)), normal1(Vector3f(0,0,0)) {}  //Default is empty interval
	Interval(float f) : t0(0), t1(INF), normal0(0,0,0), normal1(normal0) {}  //Add in a single float for infinite interval--- [0, INFINITY]
	bool isValidInterval() { return (t1 <= t0); }



	bool intersect(const Interval& other) {
		t0 = std::max(t0, other.t0); 
		t1 = std::min<float>(t1, other.t1); 
		if (t0 > t1) { return false; }
		else {
			if (t0 == other.t0) 
			{ normal0 = other.normal0; }  
			
			if (t1 == other.t1) 
			{ normal1 = other.normal1; return true; }
		}
	}
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};


class Ray
{
public:
	Ray(Vector3f q, Vector3f d) : startingPoint(q), direction(d.normalized()) {}

	Vector3f startingPoint;
	Vector3f direction;

	Vector3f pointAtDistance(float t) { return startingPoint + (direction*t); }

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	
};



class Shape
{


public:
	Shape() : mat(NULL) {}
	Shape(Material* m) : mat(m), center(0,0,0) {}
	Shape(Material* m , Vector3f v) : mat(m), center(v) {}
	virtual bool Intersect(Ray* r, IntersectRecord* i) = 0;
	virtual Box3d* bbox() const = 0;
	Material* mat;
	Vector3f center;
	//	virtual Shape& operator=(Shape& other) { *mat = *(other.mat); }
	virtual const Shape& operator=(Shape& other) { *mat = (*other.mat); center = other.center;  return *this; }
	//Make a new intersect record per-ray, so one for every pixel for proj. 1?
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;



};


class IntersectRecord
{
public:
	IntersectRecord() : normal(Vector3f(0,0,0)), intersectionPoint(normal), t(INF), intersectedShape(NULL), boundingBox(NULL) {}
	IntersectRecord(Vector3f n, Vector3f p, float t, Shape* s) : normal(n), intersectionPoint(p), t(t), intersectedShape(s) {}
	void Clear() { normal = Vector3f(0, 0, 0); intersectionPoint=normal; t = INF; }  //Get rid of the normal, intersection point, t, keep the shape & bounding box

	Vector3f normal, intersectionPoint;
	float t;
	Shape* intersectedShape;
	Box3d* boundingBox;
	//Some kind of bounding box too?
	//Note: removed the pointer dereferencing!
	const IntersectRecord& operator=(IntersectRecord& other) { normal = other.normal; t = other.t;
	
	
	if (intersectedShape != NULL && other.intersectedShape != NULL)
	{
		*intersectedShape = *(other.intersectedShape);
	}
	if (boundingBox != NULL && other.boundingBox != NULL)
	{
		*boundingBox = *(other.boundingBox);
	}
	return *this;}


	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};


class Slab
{
	//Infinite volume bounded by 2 parallel planes
	//Made up of 2 plane eqns, share normals
public:
	Slab() : d0(0), d1(0), normal(0,0,0) {}
	Slab(float a, float b, Vector3f c) : d0(a), d1(b), normal(c) {}

	const Slab& operator=(Slab& other)
	{
		d0 = other.d0;
		d1 = other.d1;
		normal = other.normal;
		return *this;
	}


	Interval Intersect(Ray* r)
	{
		if (normal.dot(r->direction) != 0)
		{
			return  Interval(-1 * (d0 + (normal.dot(r->startingPoint)) / normal.dot(r->direction)), -1 * (d1 + (normal.dot(r->startingPoint)) / normal.dot(r->direction)), -normal, normal);
			//Some combination of the normals with sign changes?
		}
		else
		{
		
			if ((normal.dot(r->startingPoint) + d0) * (normal.dot(r->startingPoint) + d1) < 0)
			{
				//Signs differ, ray is 100% between planes, return infinite interval
				return  Interval(1);
			}
			else
			{
				//Otherwise, no intersect, ray is 100% outside planes, return closed interval
				return  Interval();
			}
		}
	}
	float d0, d1;
	Vector3f normal;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};



class Sphere : public Shape
{public:
	Sphere(Vector3f c, float f) : centerPoint(c), radius(f), radiusSquared(f*f), Shape() {}
	Sphere(Vector3f c, float f, Material* m) : centerPoint(c), radius(f), radiusSquared(f*f), Shape(m, c) {}
	Box3d* bbox() const { return new Box3d(centerPoint - Vector3f(radius, radius, radius), centerPoint + Vector3f(radius, radius, radius)); }
	
	
	const Sphere& operator=(Sphere& other)
	{
		Shape::operator=(other);
		centerPoint = other.centerPoint;
		radius = other.radius;
		radiusSquared = other.radiusSquared;
		return *this;
	}



	Vector3f centerPoint;
	float radius, radiusSquared;

bool Intersect(Ray* r, IntersectRecord* i)
	{
		//On no intersect, return null
		float tPlus, tMinus, c, b, determinant;
		Vector3f qPrime = r->startingPoint - centerPoint;
		 b = qPrime.dot(r->direction);  //2 * qPrime dot direction = b 
		 c = qPrime.dot(qPrime) - radiusSquared; // qPrime dot qPrime - radius^2 = c
		 determinant = (b*b - 4 * c);
		 if (determinant<0)
		 {
			 i = NULL;
			 return false;
			 //NULL;
		 }

		 else
		 {
			 tMinus = (-b - sqrtf(determinant)) / 2;
			 tPlus = (-b + sqrtf(determinant)) / 2;
			 
			 if(tMinus <0 && tPlus <0)
			 {
				 i = NULL;
				 return false;
					 
			 }

			 else if (tMinus < 0)
			 {
				 //return intersect with tPlus
				 //normal is (point - center) normalized
				 i->normal = (r->pointAtDistance(tPlus) - centerPoint).normalized();
				 i->intersectedShape = this;
				 i->intersectionPoint = (r->pointAtDistance(tPlus));
				 i->t = tPlus;
				 i->boundingBox = this->bbox();
				 return true; //new IntersectRecord(((r->pointAtDistance(tPlus) - centerPoint).normalized()), r->pointAtDistance(tPlus), tPlus, this);
			 }

			 else
			 {
				 //return intersect with tMinus
				 i->normal = (r->pointAtDistance(tMinus) - centerPoint).normalized();
				 i->intersectedShape = this;
				 i->intersectionPoint = (r->pointAtDistance(tMinus));
				 i->t = tPlus;
				 i->boundingBox = this->bbox();
				 return true;
					 //new IntersectRecord(((r->pointAtDistance(tMinus) - centerPoint).normalized()), r->pointAtDistance(tMinus), tMinus, this);
			 }
		 }
	}
};

class Cylander : public Shape
{
public:
	Cylander(Vector3f b, Vector3f a, float r) : basePoint(b), axis(a), radius(r), Shape(), toZAxis(Quaternionf::FromTwoVectors(a,Vector3f::UnitZ())), endPlates(0, -1*(sqrtf(axis.dot(axis))), ZAXIS), radiusSquared(r*r) {}
	
	Cylander(Vector3f b, Vector3f a, float r, Material* m) : basePoint(b), axis(a), radius(r), Shape(m, basePoint + (0.5*axis)), toZAxis(Quaternionf::FromTwoVectors(a, Vector3f::UnitZ())), radiusSquared(r*r), endPlates(0,-1*(sqrtf(axis.dot(axis))),ZAXIS) {}

	const Cylander& operator=(Cylander& other) {
		Shape::operator=(other);
		basePoint = other.basePoint;
		axis = other.axis;
		endPlates = other.endPlates;
		toZAxis = other.toZAxis;
		radius = other.radius;
		radiusSquared = other.radiusSquared;
		return *this;
	}

	Vector3f basePoint, axis;
	Slab endPlates;
	Quaternionf toZAxis;
	float radius, radiusSquared;

	bool	 Intersect(Ray* r, IntersectRecord* i)
	{
		Vector3f transformedStartPoint = toZAxis._transformVector(r->startingPoint - basePoint);
		Vector3f transformedDirection = toZAxis._transformVector(r->direction);

		Interval test(1);
	bool endPlateIntersect =	test.intersect(endPlates.Intersect(r));
		


			float a, b, c, tPlus, tMinus, det, tMax, tMin;

			a = transformedDirection(0) *( transformedDirection(0)) + transformedDirection(1) *( transformedDirection(1));
			b = 2 * ((transformedDirection(0) * transformedStartPoint(0) + transformedDirection(1) * transformedStartPoint(1)));
			c = (transformedStartPoint(0)* transformedStartPoint(0)) + transformedStartPoint(1)* transformedStartPoint(1) - radiusSquared;

			det = (b*b) - (4 * a*c);

			if (det < 0)
			{
				//No intersect with cylander body
			//	i = NULL;
			//	return false;


				//Return endplate intersection if it exists
				if (endPlateIntersect)
				{
					i->boundingBox = this->bbox();
					i->intersectedShape = this;
					i->intersectionPoint = r->pointAtDistance(test.t0);
					i->t = test.t0;
					i->normal = test.normal0;
				}

				else
				{
					i = NULL;
						return false;
				}
			}

			else
			{
				tPlus = ((-1 * b) + sqrtf(det)) / (2 * a);
				tMinus = ((-1 * b) - sqrtf(det)) / (2 * a);

				if (test.intersect(Interval(tMinus, tPlus)))
				{
					if (test.t0 < EPSILON && test.t1 < EPSILON)
					{
						//Both less than 0, no intersect
						i = NULL;
						return false;
					}
					//How do you tell if it's intersecting the endplate vs. the cylander part?
					else if (tMinus < 0)
					{
						//If intersecting with endplate, NPrime = +/- ZAXIS depending on the endplate
						//Else, it's:  M = (transformedStartPoint) + t(transformedDirection),   NPrime = (Mx, My, 0)
						Vector3f m( transformedStartPoint(0) + tPlus*transformedDirection(0), transformedStartPoint(1) + tPlus*transformedDirection(1), 0);
						//m.z.setZero();// = 0;
						//m.
						i->normal = toZAxis.conjugate()._transformVector(m);
						i->intersectedShape = this;
						i->intersectionPoint = (r->pointAtDistance(tPlus));
						i->t = tPlus;
						i->boundingBox = this->bbox();
						return true; //new IntersectRecord(((r->pointAtDistance(tPlus) - centerPoint).normalized()), r->pointAtDistance(tPlus), tPlus, this);

					}

					else
					{

						Vector3f m( transformedStartPoint(0) + tMinus*transformedDirection(0), transformedStartPoint(1) + tMinus*transformedDirection(1), 0);
					//	m.z.setZero();// = 0;
						i->normal = toZAxis.conjugate()._transformVector(m);
						i->intersectedShape = this;
						i->intersectionPoint = (r->pointAtDistance(tMinus));
						i->t = tMinus;
						i->boundingBox = this->bbox();
					}


				
			
			}


		}

	}
	Box3d* bbox() const {
		Box3d* t = new Box3d(basePoint + Vector3f(radius, radius, radius), (basePoint + axis) + Vector3f(radius, radius, radius)); t->extend(basePoint - Vector3f(radius, radius, radius));  t->extend((basePoint + axis) - Vector3f(radius, radius, radius));	return t;
	}
};
/*
class Plane : public Shape
{

};
*/
class AABB : public Shape
{

	//Just 3 intervals?
	//Herron's ver: corner, xDiag, yDiag, zDiag


public:
	Vector3f corner, diag;
	Slab x, y, z;

	AABB(Vector3f c, Vector3f d) : Shape(), corner(c), diag(d), x(-1*c(0), (-1*c(0)) - (d(0)), XAXIS), y(-1*c(1), -1*c(1) - d(1), YAXIS), z(-1*c(2), -1*c(2) - d(2), ZAXIS) {}
	AABB(Vector3f c, Vector3f d, Material* m) : Shape(m, corner + (0.5*diag)), corner(c), diag(d), x(-1*c(0), -1*c(0) - d(0), XAXIS), y(-1*c(1), -1*c(1) - d(1), YAXIS), z(-1*c(2), -1*c(2) - d(2), ZAXIS) {}


	AABB& operator=(AABB& other)
	{
		Shape::operator=(other);
		corner = other.corner;
		diag = other.diag;
		x = other.x;
		y = other.y;
		z = other.z;
		return *this;
	}

	bool Intersect(Ray* r, IntersectRecord* i)
	{
		Interval test = x.Intersect(r);
		if (  test.isValidInterval() )
		{
			if (test.intersect((y.Intersect(r)))  && test.intersect((z.Intersect(r))))
			{
				i->t = test.t0;
				i->intersectedShape = this;
				i->intersectionPoint = r->pointAtDistance(test.t0);
				i->normal = test.normal0;
				i->boundingBox = this->bbox();


				return true;
			}

			else
			{
				//No intersect
				i = NULL;
				return false;
			}
		}
		else
		{
			//No intersection
			i = NULL;
			return false;
		}


	}

	Box3d* bbox() const { return new Box3d(corner, corner + diag); }

};

class Triangle : public Shape
{
public:
	Vector3f v0, v1, v2, e1, e2, n0, n1, n2;
	Triangle(Vector3f a, Vector3f b, Vector3f c) : v0(a), v1(b), v2(c), e1(v1 - v0), e2(v2 - v0), n0(Vector3f(0, 0, 0)), n1(n0), n2(n0), Shape() {}
	Triangle(Vector3f a, Vector3f b, Vector3f c, Material* m) : v0(a), v1(b), v2(c), e1(v1 - v0), e2(v2 - v0), Shape(m, Vector3f((a(0) + b(0) + c(0)) / 3, (a(1) + b(1) + c(1)) / 3, (a(2) + b(2) + c(2)) / 3)), n0(Vector3f(0, 0, 0)), n1(n0), n2(n0) {}
	Triangle(Vector3f a, Vector3f b, Vector3f c,  Vector3f x, Vector3f y, Vector3f z)  : v0(a), v1(b), v2(c), e1(v1-v0), e2(v2-v0), n0(x), n1(y), n2(z), Shape() {}
	Triangle(Vector3f a, Vector3f b, Vector3f c, Vector3f x, Vector3f y, Vector3f z, Material* m) : v0(a), v1(b), v2(c), e1(v1 - v0), e2(v2 - v0), Shape(m, Vector3f((a(0)+b(0)+c(0))/3, (a(1) + b(1) + c(1)) / 3,(a(2) + b(2) + c(2)) / 3)) , n0(x),n1(y),n2(z) {}
	

	Triangle& operator=(Triangle& other)
	{
		Shape::operator=(other);
		v0 = other.v0;
		v1 = other.v1;
		v2 = other.v2;
		e1 = other.e1;
		e2 = other.e2;
		n0 = other.n0;
		n1 = other.n1;
		n2 = other.n2;
		return *this;
	}
	
	bool Intersect(Ray* r, IntersectRecord* i)
	{
		Vector3f s, dE2, sE1;
		s = r->startingPoint - v0;
		float d, t, u, v;
		dE2 = r->direction.cross(e2);
		sE1 = s.cross(e1);
		d = dE2.dot(e1);
		if (d == 0)
		{
			//No intersect
			i = NULL;
			return false;
		}

		u = (dE2.dot(s)) / d;
		if (u < 0 || u>1)
		{
			//No intersect
			i = NULL;
			return false;
		}
		v = (sE1.dot(r->direction)) / d;
		if (v < 0 || v + u >1)
		{
			//No intersect
			i = NULL;
			return false;
			//return NULL;
		}

		t = (sE1.dot(e2)) / d;
		if (t < EPSILON)
		{
			//No intersect
			i = NULL;
			return false;
		}

		//If you have normals at each point, normal to return is:  (1-u-v)N0 + (u)N1 + (v)N2
		//Else, it's E2 cross E1
		
			if (n0 != ZEROES && n1 != ZEROES && n2 != ZEROES)
			{
				i->normal = (1 - u - v)*n0 + (u)*n1 + (v)*n2;
			}
			else
			{
				i->normal = e2.cross(e1);
			}
			i->boundingBox = this->bbox();
		i->t = t;
		i->intersectedShape = this;
		i->intersectionPoint = r->pointAtDistance(t);
		return true; //new IntersectRecord(e2.cross(e1), r->pointAtDistance(t), t, this);
	}



	Box3d* bbox() const { Box3d* t = new Box3d(v1, v2); t->extend(v0); return t; }

};

class Minimizer
{
public:
	typedef float Scalar;
	Ray ray;
	IntersectRecord* record;
	IntersectRecord* smallest;
	
	Box3d bbox( Shape* obj)
	{
		return *(obj->bbox());
	}


	Minimizer(const Ray& r) : ray(r), record(), smallest() {}

	float minimumOnObject(Shape* sh)
	{
		smallest->t = INF;
	 
		if (record != NULL &&  sh->Intersect(&ray, record) && record->t < smallest->t)
		{
			*smallest = *record;
		}
		//Keep track of nearest and intersect record???
		return smallest->t;
	}


	float minimumOnVolume(const Box3d& box)
	{
		Vector3f L = box.corner(Box3d::BottomLeftFloor);
		Vector3f R = box.corner(Box3d::TopRightCeil);
		//bottom left corner and top right corner?

		Vector3f diag = R - L;

		//Slab x, y, z;
		Slab x(-1*L(0), -1*L(0) - diag(0), XAXIS);
		Slab y(-1*L(1), -1*L(1) - diag(1), YAXIS);
		Slab z(-1*L(2), -1*L(2) - diag(2), ZAXIS);


		float xDot, yDot, zDot;
		xDot = ray.direction.dot(x.normal);
		yDot = ray.direction.dot(y.normal);
		zDot = ray.direction.dot(z.normal);

		if (xDot == 0 || yDot == 0 || zDot == 0) //Parallel to at least 1 plane
		{
			
			float x0, y0, z0, x1, y1, z1;
			x0 = ray.startingPoint.dot(x.normal) + x.d0;
			y0 = ray.startingPoint.dot(y.normal) + y.d0;
			z0 = ray.startingPoint.dot(z.normal) + z.d0;
			x1 = ray.startingPoint.dot(x.normal) + x.d1;
			y1 = ray.startingPoint.dot(y.normal) + y.d1;
			z1 = ray.startingPoint.dot(z.normal) + z.d1;

			if(x0*x1 <0 && y0*y1 <0 && z0*z1 <0)
			{
				return 0;
			}

			//Parallel to at least one, intersects with others, therefore no cube intersect
			else
			{
				float tX0, tY0, tZ0, tZ1, tX1, tY1, tOut0, tOut1;
				tX0 = std::min(-(x.d0 + x.normal.dot(ray.startingPoint)) / (x.normal.dot(ray.direction)), -(x.d1 + x.normal.dot(ray.startingPoint)) / (x.normal.dot(ray.direction)));
				tX1 = std::max(-(x.d0 + x.normal.dot(ray.startingPoint)) / (x.normal.dot(ray.direction)), -(x.d1 + x.normal.dot(ray.startingPoint)) / (x.normal.dot(ray.direction)));

				tY0 = std::min(-(y.d0 + y.normal.dot(ray.startingPoint)) / (y.normal.dot(ray.direction)), -(y.d1 + y.normal.dot(ray.startingPoint)) / (y.normal.dot(ray.direction)));
				tY1 = std::max(-(y.d0 + y.normal.dot(ray.startingPoint)) / (y.normal.dot(ray.direction)), -(y.d1 + y.normal.dot(ray.startingPoint)) / (y.normal.dot(ray.direction)));

				
				
				tZ0 = std::min(-(z.d0 + z.normal.dot(ray.startingPoint)) / (z.normal.dot(ray.direction)), -(z.d1 + z.normal.dot(ray.startingPoint)) / (z.normal.dot(ray.direction)));
				tZ1 = std::max(-(z.d0 + z.normal.dot(ray.startingPoint)) / (z.normal.dot(ray.direction)), -(z.d1 + z.normal.dot(ray.startingPoint)) / (z.normal.dot(ray.direction)));

				tOut0 = std::max(std::max(tX0, tY0),tZ0);
				tOut1 = std::min(std::min(tX1,tY1), tZ1);

				if(tOut0 > tOut1  || (tOut0 <EPSILON && tOut1 <EPSILON))
				{
					return INF;
				}

				else
				{
					if (tOut0 < EPSILON)
					{
						return tOut1;
					}

					else
					{
						return tOut0;
					}
				}



			}
		}


		/*
		Interval test = x.Intersect(&ray);
		if (test.isValidInterval())
		{
			if (test.intersect((y.Intersect(&ray))) && test.intersect((z.Intersect(&ray))))
			{


				return true;
			}

			else
			{
				//No intersect
				i = NULL;
				return false;
			}
		}
		else
		{
			//No intersection
			i = NULL;
			return false;
		}

		*/

	}

};




////////////////////////////////////////////////////////////////////////
// Material: encapsulates a BRDF and communication with a shader.
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
// Data structures for storing meshes -- mostly used for model files
// read in via ASSIMP.
//
// A MeshData holds two lists (stl::vector) one for vertices
// (VertexData: consisting of point, normal, texture, and tangent
// vectors), and one for triangles (TriData: consisting of three
// indices into the vertex array).
typedef Eigen::Matrix<unsigned int, 3, 1 > TriData;
    
class VertexData
{
 public:
    Vector3f pnt;
    Vector3f nrm;
    Vector2f tex;
    Vector3f tan;
    VertexData(const Vector3f& p, const Vector3f& n, const Vector2f& t, const Vector3f& a) 
        : pnt(p), nrm(n), tex(t), tan(a) 
    {}
};

struct MeshData
{
    std::vector<VertexData> vertices;
    std::vector<TriData> triangles;
    Material *mat;
};

////////////////////////////////////////////////////////////////////////
// Light: encapsulates a light and communiction with a shader.
////////////////////////////////////////////////////////////////////////
class Light: public Material
{
public:

    Light(const Vector3f e) : Material() { Kd = e; }
    virtual bool isLight() { return true; }

	Light& operator=(Light& other)
	{
		Material::operator=(other);
		Kd = other.Kd;
		return *this;
		
	}


    //virtual void apply(const unsigned int program);
};

////////////////////////////////////////////////////////////////////////////////
// Scene
class Realtime;

class Scene {
public:
    int width, height;
    Realtime* realtime;         // Remove this (realtime stuff)
    Material* currentMat;
	std::vector<Shape*> shapes;
	std::vector<Shape*> lights;
	Camera camera;
    Scene();
    void Finit();

    // The scene reader-parser will call the Command method with the
    // contents of each line in the scene file.
    void Command(const std::vector<std::string>& strings,
                 const std::vector<float>& f);

    // To read a model file into the scene via ASSIMP, call ReadAssimpFile.  
    void ReadAssimpFile(const std::string& path, const Matrix4f& M);

    // Once ReadAssimpFile parses the information from the model file,
    // it will call:
    void triangleMesh(MeshData* mesh);



    // The main program will call the TraceImage method to generate
    // and return the image.  This is the Ray Tracer!
    void TraceImage(Color* image, const int pass);
};
