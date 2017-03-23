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
const Vector3f ONES = Vector3f(1, 1, 1);
const Vector3f ZAXIS = Vector3f(0, 0, 1);
const Vector3f YAXIS = Vector3f(0, 1, 0);
const Vector3f XAXIS = Vector3f(1, 0, 0);
const float RUSSIAN_ROULETTE=0.8f;

enum class BRDFChoice
{
	NONE,
	DIFFUSE,
	REFLECTION,
	TRANSMISSION
};


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

	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
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


	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

};




class Interval
{
public:


	Vector3f normal0, normal1;
	float t0, t1;
	//bool isDefaultInterval;

	Interval(float t00, float t11, Vector3f n0, Vector3f n1) : t0(t00), t1(t11), normal0(n0), normal1(n1)
	{
		if (t0 > t1) 
		{
			float temp = t1;
			t1 = t0; 
			t0 = temp;
			Vector3f temp1 = normal0; 
			normal0 = normal1; 
			normal1 = temp1; }
	
	}
	Interval(float x, float y) : t0(std::max(0.f,x)), t1(y), normal0(0, 0, 0), normal1(normal0) { if (t0 > t1) { float temp = t1; t1 = t0; t0 = temp; Vector3f temp1 = normal0; normal0 = normal1; normal1 = temp1; } }
	Interval() : t0(1), t1(0), normal0(Vector3f(0,0,0)), normal1(Vector3f(0,0,0)) {}  //Default is empty interval
//	Interval(float f) : t0(0), t1(INF), normal0(0,0,0), normal1(normal0) {}  //Add in a single float for infinite interval--- [0, INFINITY]
	bool isValidInterval() { return (t0 <= t1); }
	void intersectWithInfiniteInterval()
	{
		t0 = std::max(t0, 0.f);
		t1 = std::min(t1, INF);
		//if (t0 > t1) { float temp = t1; t1 = t0; t0 = temp; Vector3f temp1 = normal0; normal0 = normal1; normal1 = temp1; };
	}


	bool intersect(const Interval& other) {
		t0 = std::max(0.f,std::max(t0, other.t0)); 
		t1 = std::min(t1, other.t1); 
		if (t0 > t1) { 
			
			return false;
		
		}
		

			
				if (t0 == other.t0)
					{
					normal0 = other.normal0;
					}

				if (t1 == other.t1)
					{
						normal1 = other.normal1;
					}
				
			return true;
		
		
	}
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};


class Ray
{
public:
	Ray(Vector3f q, Vector3f d) : startingPoint(q), direction(d.normalized()) {}

	Vector3f startingPoint;
	Vector3f direction;

	Vector3f pointAtDistance(float t) const { return startingPoint + (direction*t); }

	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	
};



class Shape
{


public:
	Shape() : mat(), center(0,0,0) {}
	Shape(Material* m) : mat(m), center(0,0,0), area(0) {}
	Shape(Material* m , Vector3f v, float f) : mat(m), center(v),area(f)  {}
	virtual bool Intersect(const Ray& r, IntersectRecord* i) = 0;
	virtual Box3d bbox() const = 0;
	Material* mat;
	Vector3f center;
	//	virtual Shape& operator=(Shape& other) { *mat = *(other.mat); }
	
	//virtual Shape& CopyCtor(Shape& other) {  return Shape(*this); }
	virtual Shape* Copy() = 0;
	virtual Shape* CopyPointer() = 0;
	virtual const Shape& operator=(Shape& other) { mat = (other.mat); center = other.center;  return *this; }
	virtual float calculateArea() = 0;
	float area;
	//Make a new intersect record per-ray, so one for every pixel for proj. 1?
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW;



};


class IntersectRecord
{
public:
	IntersectRecord() : normal(Vector3f(0, 0, 0)), intersectionPoint(normal), t(INF), intersectedShape(nullptr) {  }
	IntersectRecord(Vector3f n, Vector3f p, float t, Shape* s) : normal(n), intersectionPoint(p), t(t), intersectedShape(s) {}
	IntersectRecord(IntersectRecord& other) : normal(other.normal), intersectionPoint(other.intersectionPoint), t(other.t), intersectedShape(other.intersectedShape) {}




	void Clear() { normal = Vector3f(0, 0, 0); intersectionPoint=normal; t = INF; }  //Get rid of the normal, intersection point, t, keep the shape & bounding box


	Vector3f normal, intersectionPoint;
	float t;
	Shape* intersectedShape;
	//Box3d* boundingBox;

	//Some kind of bounding box too?
	//Note: removed the pointer dereferencing!
	const IntersectRecord& operator=(IntersectRecord& other) { normal = other.normal; t = other.t;
	intersectionPoint = other.intersectionPoint;
	
	if ( other.intersectedShape != NULL)
	{
		//if (intersectedShape == NULL)
	//	{
			intersectedShape = (other.intersectedShape->CopyPointer());
		//}
		//else
		//{
		//	*intersectedShape = *(other.intersectedShape);
		//}
	}

	else
	{
		std::cout << "Some intersect record has a NULL shape, something's wrong." << std::endl;
	}
	/*
	if (other.boundingBox != NULL)
	{
	//	if (boundingBox == NULL)
	//	{
			boundingBox = intersectedShape->bbox();
	//	}
	///	else {
	//		*boundingBox = *(other.boundingBox);
	//	}
		
	}
	*/
	return *this;}


	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};


class Slab
{
	//Infinite volume bounded by 2 parallel planes
	//Made up of 2 plane eqns, share normals
public:
	Slab() : d0(0), d1(0), normal(0,0,0) {}
	Slab(float a, float b, Vector3f c) : d0(a), d1(b), normal(c) {}
	Slab(Slab& other) : d0(other.d0), d1(other.d1), normal(other.normal) {}

	

	const Slab& operator=(Slab& other)
	{
		d0 = other.d0;
		d1 = other.d1;
		normal = other.normal;
		return *this;
	}

	

	Interval Intersect(const Ray& r) const
	{
		
		float nDir = normal.dot(r.direction);
		float nStart = normal.dot(r.startingPoint);
		if (nDir != 0)
		{
			
			return  Interval(-(d0 + nStart) / nDir, -(d1 + (nStart)) / nDir, -normal, normal);
			//Some combination of the normals with sign changes?
		}
		else
		{
		
			if ((nStart + d0) * (nStart + d1) < 0)
			{
				//Signs differ, ray is 100% between planes, return infinite interval
				return  Interval(0,INF);
			}
			else
			{
				//Otherwise, no intersect, ray is 100% outside planes, return closed interval
				return  Interval();
				//
			}
		}
	}


	float d0, d1;
	Vector3f normal;

	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};



class Sphere : public Shape
{public:
	Sphere(Vector3f c, float f) : centerPoint(c), radius(f), radiusSquared(f*f), Shape() { center = centerPoint; }
	Sphere(Vector3f c, float f, Material* m) : centerPoint(c), radius(f), radiusSquared(f*f), Shape(m, c, 4 * radiusSquared*PI) { center = centerPoint; }
	Box3d bbox() const { 
		Vector3f xR, yR, zR;
		xR = Vector3f(radius+1, 0, 0);
		yR = Vector3f(0, radius+1, 0);
		zR = Vector3f(0, 0, radius+1);
		Box3d b =  Box3d(centerPoint + xR, centerPoint - xR);
		b.extend(centerPoint + yR);
		b.extend(centerPoint - yR);
		b.extend(centerPoint + zR);
		b.extend(centerPoint - zR);
		//b->extend(center);
		return b;
	
	
	}
	float calculateArea() { return 4 * radiusSquared*PI; }
	Sphere(Sphere& other) : centerPoint(other.centerPoint), radius(other.radius), radiusSquared(radius*radius), Shape(other.mat, centerPoint, 4 * radiusSquared*PI) {}
	//float area() { return area; }
	Sphere* Copy() { return new Sphere(*this); }
	Sphere* CopyPointer() { return this; }
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



	

	IntersectRecord SampleSphere();  //(Vector3f center, float radius)
	void SampleSphere(IntersectRecord& i);
	



bool Intersect(const Ray& r, IntersectRecord* i)
	{
		//On no intersect, return null
		float tPlus, tMinus, a, c, b, determinant;
		Vector3f qPrime = r.startingPoint - centerPoint;
		//a = r.direction.dot(r.direction);
	
		b = 2* qPrime.dot(r.direction);  //2 * qPrime dot direction = b 
		 c = qPrime.dot(qPrime) - radiusSquared; // qPrime dot qPrime - radius^2 = c
		 determinant = (b*b) - (4 * c);
		 if (determinant<0)
		 {
			 i = NULL;
			 return false;
			 //NULL;
		 }

		 else
		 {
			 tMinus = (-b - sqrt(determinant)) / (2.f);
			 tPlus = (-b + sqrt(determinant)) / (2.f);
			 
			 if(tMinus <EPSILON && tPlus <EPSILON)
			 {
				 i = NULL;
				 return false;
					 
			 }

			 else if (tMinus < EPSILON)
			 {
				 //return intersect with tPlus
				 //normal is (point - center) normalized
				 i->normal = (r.pointAtDistance(tPlus) - centerPoint).normalized();
				 i->intersectedShape = this;
				 i->intersectionPoint = (r.pointAtDistance(tPlus));
				 i->t = tPlus;
			//	 i->boundingBox = this->bbox();
				 return true; //new IntersectRecord(((r.pointAtDistance(tPlus) - centerPoint).normalized()), r.pointAtDistance(tPlus), tPlus, this);
			 }

			 else
			 {
				 //return intersect with tMinus
				 i->normal = (r.pointAtDistance(tMinus) - centerPoint).normalized();
				 i->intersectedShape = this;
				 i->intersectionPoint = (r.pointAtDistance(tMinus));
				 i->t = tMinus;
				// i->boundingBox = this->bbox();
				 return true;
					 //new IntersectRecord(((r.pointAtDistance(tMinus) - centerPoint).normalized()), r.pointAtDistance(tMinus), tMinus, this);
			 }
		 }
	}
};

class Cylander : public Shape
{
public:
	Cylander(Vector3f b, Vector3f a, float r) : basePoint(b), axis(a), radius(r), Shape(), toZAxis(Quaternionf::FromTwoVectors(a,Vector3f::UnitZ())), endPlates(0, -(axis.norm()), ZAXIS), radiusSquared(r*r) {}
	
	Cylander(Vector3f b, Vector3f a, float r, Material* m) : basePoint(b), axis(a), radius(r), Shape(m, basePoint + (0.5*axis), 2*PI*radius*axis.norm() + 2*PI*radiusSquared), toZAxis(Quaternionf::FromTwoVectors(a, Vector3f::UnitZ())), radiusSquared(r*r), endPlates(0,-(axis.norm()),ZAXIS) {}

	Cylander(Cylander& other) : basePoint(other.basePoint), axis(other.axis), radius(other.radius), radiusSquared(other.radiusSquared), Shape(other.mat, basePoint+(0.5*axis), 2 * PI*radius*axis.norm() + 2 * PI*radiusSquared), toZAxis(other.toZAxis),  endPlates(other.endPlates) {}
	float calculateArea() {		return (2 * PI*radius*axis.norm() + 2 * PI*radiusSquared);	}
	Cylander* Copy() { return new Cylander(*this); }
	Cylander* CopyPointer() { return this; }
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

	bool	 Intersect(const Ray& r, IntersectRecord* i)
	{

		//Redo aaaall this bit
		//Need to figure out how to get the normal of the points where the ray intersects the infinite cylander bit


		Vector3f transformedStartPoint = toZAxis._transformVector(r.startingPoint - basePoint);
		Vector3f transformedDirection = toZAxis._transformVector(r.direction);
		Interval eP = endPlates.Intersect( Ray(transformedStartPoint, transformedDirection));
		//Interval test(1.f);
	//bool endPlateIntersect =	test.intersect(endPlates.Intersect(r));
	//Vector3f endPlateNormals[2] = { eP.normal0, eP.normal1 };
		//Interval endPlates(0, -(sqrtf(axis.dot(axis))), -ZAXIS, ZAXIS);
		//test.intersect(eP);
		eP.intersectWithInfiniteInterval();
			float a, b, c, tPlus, tMinus, det, tMax, tMin, tOut;

			a = (transformedDirection(0) * transformedDirection(0)) + (transformedDirection(1) * transformedDirection(1));
			b = 2 * ((transformedDirection(0) * transformedStartPoint(0)) + (transformedDirection(1) * transformedStartPoint(1)));
			c = (transformedStartPoint(0)* transformedStartPoint(0)) + (transformedStartPoint(1)* transformedStartPoint(1)) - radiusSquared;

			det = (b*b) - (4 * a*c);

			if (det < 0)
			{
				
				//No intersect with cylander body
				/*
				if (test.isValidInterval() && (test.t0 > EPSILON || test.t1 > EPSILON))
				{
					if (test.t0 < EPSILON)
					{
						i->t = test.t1;
						i->normal = toZAxis.conjugate()._transformVector(test.normal1);//toZAxis.conjugate()._transformVector(endPlateNormals[1].normalized());
						i->intersectedShape = this;
						i->intersectionPoint = (r.pointAtDistance(i->t));
						i->boundingBox = this->bbox();
						return true;
						
					}

					else
					{

						i->t = test.t0;
						i->normal = toZAxis.conjugate()._transformVector(test.normal0);//toZAxis.conjugate()._transformVector(endPlateNormals[0].normalized());
						i->intersectedShape = this;
						i->intersectionPoint = (r.pointAtDistance(i->t));
						i->boundingBox = this->bbox();
						return true;
					}
					
				}
				else {
				*/
				
				
					i = NULL;
					return false;


				//	}
			}

			else
			{
				tPlus = ((-b) + sqrt(det)) / (2 * a);
				tMinus = ((-b) - sqrt(det)) / (2 * a);
				//bool hasT = false;

				
					//if (test.t0 < EPSILON && test.t1 < EPSILON)
					if(eP.t0 <EPSILON && eP.t1 <EPSILON)
					{
						//Both less than 0, no intersect
						i = NULL;
						return false;
					}
					//How do you tell if it's intersecting the endplate vs. the cylander part?
					//else if (tMinus<EPSILON)
					else if (tMinus < EPSILON)
					{
						tOut = tPlus;
						//hasT = true;

						//cylander part normal - (x,y) of intersection, 0   (x,y,0)  ?
						

						//If intersecting with endplate, NPrime = +/- ZAXIS depending on the endplate
						//Else, it's:  M = (transformedStartPoint) + t(transformedDirection),   NPrime = (Mx, My, 0)
					
						
						//Vector3f m( transformedStartPoint(0) + tPlus*transformedDirection(0), transformedStartPoint(1) + tPlus*transformedDirection(1), 0);
						//m.z.setZero();// = 0;
						//m.

				
						/*
						i->normal = toZAxis.conjugate()._transformVector(m);
						i->intersectedShape = this;
						i->intersectionPoint = (r.pointAtDistance(tPlus));
						i->t = tPlus;
						i->boundingBox = this->bbox();
						return true; //new IntersectRecord(((r.pointAtDistance(tPlus) - centerPoint).normalized()), r.pointAtDistance(tPlus), tPlus, this);
						*/
					}

					else
					{
						tOut = tMinus;
						
							
							/*
							Vector3f m( transformedStartPoint(0) + tMinus*transformedDirection(0), transformedStartPoint(1) + tMinus*transformedDirection(1), 0);
					//	m.z.setZero();// = 0;
						i->normal = toZAxis.conjugate()._transformVector(m);
						i->intersectedShape = this;
						i->intersectionPoint = (r.pointAtDistance(tMinus));
						i->t = tMinus;
						i->boundingBox = this->bbox();
					*/
					
					}

					Vector3f cylanderNormalPlus(transformedStartPoint(0) + tPlus*transformedDirection(0), transformedStartPoint(1) + tPlus*transformedDirection(1), 0);
					Vector3f cylanderNormalMinus(transformedStartPoint(0) + tMinus*transformedDirection(0), transformedStartPoint(1) + tMinus*transformedDirection(1), 0);
					Interval cylander(tMinus, tPlus, cylanderNormalMinus, cylanderNormalPlus);
					cylander.intersect(eP);
					cylander.intersectWithInfiniteInterval();
					//cylander.intersect(endPlates);


					if (cylander.isValidInterval() && (cylander.t0 > EPSILON || cylander.t1 > EPSILON))
					{
						if (cylander.t0 < EPSILON)
						{
							i->t = cylander.t1;
							i->normal = (toZAxis.conjugate()._transformVector(cylander.normal1)).normalized();
							
						

						}

						else
						{
							i->t = cylander.t0;
							i->normal = (toZAxis.conjugate()._transformVector(cylander.normal0)).normalized();
						}

						i->intersectionPoint = (r.pointAtDistance(i->t));
						i->intersectedShape = this;
					//	i->boundingBox = this->bbox();
						return true;
					}

					else
					{
						i = NULL;
						return false;
					}
			


		}

	}
	Box3d bbox() const {
		Vector3f xR, yR, zR, aB;
		xR = Vector3f(radius, 0, 0);
		yR = Vector3f(0,radius,0);
		zR = Vector3f(0,0,radius);
		aB = axis + basePoint;
		Box3d t =  Box3d(basePoint + xR, aB +xR);
		t.extend(basePoint - xR); 
		t.extend(aB - xR);
		t.extend(basePoint + yR);
		t.extend(basePoint - yR);
		t.extend(basePoint + zR);
		t.extend(basePoint - zR);
		t.extend(aB + yR);
		t.extend(aB - yR);
		t.extend(aB + zR);
		t.extend(aB - zR);
		//t->extend(basePoint);
		//t->extend(aB);
		
		
		
		return t;
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

	AABB(Vector3f c, Vector3f d) : Shape(), corner(c), diag(d), x(-c(0), (-c(0)) - (d(0)), XAXIS), y(-c(1), -c(1) - d(1), YAXIS), z(-c(2), -c(2) - d(2), ZAXIS) {}
	AABB(Vector3f c, Vector3f d, Material* m) : Shape(m, corner + (0.5*diag), 0), corner(c), diag(d), x(-c(0), -c(0) - d(0), XAXIS), y(-c(1), -c(1) - d(1), YAXIS), z(-c(2), -c(2) - d(2), ZAXIS) { Vector3f farCorner = c + d;   this->area = 2 * (fabs(farCorner(0) - corner(0)) * fabs(farCorner(1) - corner(1)) + fabs(farCorner(0) - corner(0)) * fabs(farCorner(2) - corner(2)) + fabs(farCorner(1) - corner(1)) * fabs(farCorner(2) - corner(2))); }
	AABB(AABB& other) : corner(other.corner), diag(other.diag), Shape(other.mat, corner + (0.5*diag), other.area),  x(-corner(0), -corner(0) - diag(0), XAXIS), y(-corner(1), -corner(1) - diag(1), YAXIS), z(-corner(2), -corner(2) - diag(2), ZAXIS) {}

	float calculateArea() {
		Vector3f farCorner = corner + diag;  
		return 2 * (fabs(farCorner(0) - corner(0)) * fabs(farCorner(1) - corner(1)) + fabs(farCorner(0) - corner(0)) * fabs(farCorner(2) - corner(2)) + fabs(farCorner(1) - corner(1)) * fabs(farCorner(2) - corner(2)));
	}


	AABB* Copy() { return new AABB(*this); }
	AABB* CopyPointer() { return this; }
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

	bool Intersect(const Ray& r, IntersectRecord* i)
	{
		
		//Interval initial(1.f);
		//initial.intersect(test[0]);
		//std::cout << "x(t0, t1):  (" << test[0].t0 << ", " << test[0].t1 << ").  " << std::endl;
		//std::cout << "y(t0, t1):  (" << test[1].t0 << ", " << test[1].t1 << ").  " << std::endl;
		//std::cout << "z(t0, t1):  (" << test[2].t0 << ", " << test[2].t1 << ").  " << std::endl;

		Interval test[3] = { x.Intersect(r), y.Intersect(r), z.Intersect(r) };
		
		


			if (test[0].intersect(test[1])  && test[0].intersect(test[2]) && test[0].isValidInterval() && (test[0].t0 > EPSILON || test[0].t1 > EPSILON))
			{
				
					if(test[0].t0>EPSILON)
					{
						i->t = test[0].t0;
						i->normal = test[0].normal0;
					}
					else
					{
						i->t = test[0].t1;
						i->normal = test[0].normal1;
					}
					//i->t = test[0].t0;
					i->intersectedShape = this;
					i->intersectionPoint = r.pointAtDistance(i->t);
					//i->normal = test[0].normal0;
				//	i->boundingBox = this->bbox();


					return true;

				



			}

			else
			{
				//No intersect
				i = NULL;
				return false;
			}
	

	}

	Box3d bbox() const { return  Box3d(corner, corner + diag); }

};

class Triangle : public Shape
{
public:
	Vector3f v0, v1, v2, e1, e2, n0, n1, n2;
	Triangle(Vector3f a, Vector3f b, Vector3f c) : v0(a), v1(b), v2(c), e1(v1 - v0), e2(v2 - v0), n0(Vector3f(0, 0, 0)), n1(n0), n2(n0), Shape() {}
	Triangle(Vector3f a, Vector3f b, Vector3f c, Material* m) : v0(a), v1(b), v2(c), e1(v1 - v0), e2(v2 - v0), Shape(m, Vector3f((a(0) + b(0) + c(0)) / 3, (a(1) + b(1) + c(1)) / 3, (a(2) + b(2) + c(2)) / 3), 0.5f * sqrtf(e1.squaredNorm() * e2.squaredNorm() - e1.dot(e2))), n0(Vector3f(0, 0, 0)), n1(n0), n2(n0) {}
	Triangle(Vector3f a, Vector3f b, Vector3f c,  Vector3f x, Vector3f y, Vector3f z)  : v0(a), v1(b), v2(c), e1(v1-v0), e2(v2-v0), n0(x), n1(y), n2(z), Shape() {}
	Triangle(Vector3f a, Vector3f b, Vector3f c, Vector3f x, Vector3f y, Vector3f z, Material* m) : v0(a), v1(b), v2(c), e1(v1 - v0), e2(v2 - v0), Shape(m, Vector3f((a(0)+b(0)+c(0))/3, (a(1) + b(1) + c(1)) / 3,(a(2) + b(2) + c(2)) / 3), 0.5f * sqrtf( e1.squaredNorm() * e2.squaredNorm() - e1.dot(e2))) , n0(x),n1(y),n2(z) {}
	Triangle(Triangle& other) : v0(other.v0), v1(other.v1), v2(other.v2), e1(other.e1), e2(other.e2), Shape(other.mat, other.center,other.area) , n0(other.n0), n1(other.n1), n2(other.n2) {}
	float calculateArea() {return 0.5f * sqrtf(e1.squaredNorm() * e2.squaredNorm() - e1.dot(e2)); }
	Triangle* Copy() { return new Triangle(*this); }
	Triangle* CopyPointer() { return this; }
	
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
	
	bool Intersect(const Ray& r, IntersectRecord* i)
	{
		Vector3f s, dE2, sE1;
		s = r.startingPoint - v0;
		float d, t, u, v;
		dE2 = r.direction.cross(e2);
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
		v = (r.direction.dot(sE1)) / d;
		if (v < 0 || v + u >1)
		{
			//No intersect
			i = NULL;
			return false;
			//return NULL;
		}

		t = (e2.dot(sE1)) / d;
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
				i->normal = ((1 - u - v)*n0 + (u)*n1 + (v)*n2).normalized();
			}
			else
			{
				i->normal = (e2.cross(e1)).normalized();
			}
		//	i->boundingBox = this->bbox();
		i->t = t;
		i->intersectedShape = this;
		i->intersectionPoint = r.pointAtDistance(t);
		return true; //new IntersectRecord(e2.cross(e1), r.pointAtDistance(t), t, this);
	}



	Box3d bbox() const { //Box3d t =  Box3d(v0+ONES*EPSILON, v1+EPSILON*ONES); 
	//t.extend(v2+ONES*EPSILON); 
		Box3d t = Box3d(v0, v1);
		t.extend(v2);
		Vector3f diag = (t.min() - t.max())*5;// / 2;   //.normalized();
	//	diag *= EPSILON*700;
	t.extend(t.min() + diag);
	t.extend(t.max() - diag);


	return t; }

};

class Minimizer
{
public:





	void CheckIntersectRecordRay(IntersectRecord* i, Ray* r)
	{
		bool ex = false;
		bool shapeExists = false;
		bool intersectionRecordExists = true;
		bool rayExists = true;
		Sphere* s = NULL;
		if (r == NULL)
		{
			std::cout << "Ray does not exist, exiting." << std::endl;
			ex = true;
			rayExists = false;
		}

		if (i == NULL)
		{
			std::cout << "IntersectRecord does not exist, exiting." << std::endl;
			intersectionRecordExists = false;
			ex = true;
		}


		//Assume sphere only for now
		//Check if point is on object
		if (intersectionRecordExists)
		{
			if (i->intersectedShape == NULL)
			{
				std::cout << "Intersected shape does not exist, skipping past." << std::endl;
			}


			else
			{
				s = static_cast<Sphere*>(i->intersectedShape);
				shapeExists = true;
			}
		}




		if (shapeExists)
		{
			if (fabs(s->radiusSquared - (s->centerPoint - i->intersectionPoint).squaredNorm()) > EPSILON)
			{
				std::cout << "Intersection point is not (within reasonable precision of 0.0001) on sphere, exiting." << std::endl;
				ex = true;
			}
		}

		if (ex)
		{
			exit(-111);
		}


	}










	typedef float Scalar;
	Ray ray;
	IntersectRecord smallest, record;
	
	Box3d bbox( Shape* obj)
	{
		//Box3d testBox = obj->bbox();
		//testBox.extend(Vector3f(10,10,10));
		//testBox.extend(Vector3f(-10, -10, -10));
		//return testBox;
		return (obj->bbox());
	}


	Minimizer(const Ray& r) : ray(r), smallest(), record() { smallest.t = 99999; record.t = 99999; }

	float minimumOnObject(Shape* sh)
	{
		//IntersectRecord record;
		if ( sh->Intersect(ray, &record))
		{
			if (record.t < smallest.t)
			{
				smallest.t  = record.t;
				smallest.intersectionPoint = record.intersectionPoint;
				smallest.normal = record.normal;
				smallest.intersectedShape = record.intersectedShape;
				
				if (smallest.intersectionPoint == ZEROES)
				{
					int br = 0;
					br++;
				}
				//smallest.intersectionPoint = ray.pointAtDistance(smallest.t);
			}
			//delete record.intersectedShape;
			//return record.t;
			return smallest.t;
		}
		//Keep track of nearest and intersect record???
		else
		{
			return INF;
		}

	//	return minimumOnVolume(*sh->bbox());

	}


	float minimumOnVolume(const Box3d& box)
	{
		//Vector3f L = box.corner(Box3d::BottomRightFloor);
		//Vector3f R = box.corner(Box3d::TopLeftCeil);
		//bottom left corner and top right corner?


		Vector3f L = box.min();
		Vector3f R = box.max();
		//Vector3f diag = R - L;
		/*


		AABB testBox(L, diag);
		IntersectRecord testRecord;
		testRecord.t = INF;
		testBox.Intersect(&ray, &testRecord);
		if (testRecord.intersectedShape != NULL) //If it intersects something and exists
		{
		

			if (testRecord.t == 0)
			{
				int breakHere = 1;
				breakHere++;
			}

			return testRecord.t;

		}
		else  //Doesn't intersect anything
		{
			return INF;
		}
		*/
		//Slab x, y, z;
		Slab x(-L(0), -R(0), XAXIS);
		Slab y(-L(1), -R(1), YAXIS);
		Slab z(-L(2), -R(2), ZAXIS);
		

		//Interval test[3] = { x.Intersect(&ray), y.Intersect(&ray), z.Intersect(&ray) };
		
		Interval testX( x.Intersect(ray));
		Interval testY( y.Intersect(ray));
		Interval testZ( z.Intersect(ray));

		//Ray starts inside box?
	//	testX.intersect(testY);
	//	testX.intersect(testZ);
		//float tOut0, tOut1;
		
		//tOut0 = std::max(testZ.t0, std::max(testY.t0,std::max(0.f, testX.t0)));
		//tOut1 = std::min(testZ.t1,std::min(testX.t1,testY.t1));


/*
		AABB testBox(L, R - L);


		Interval test[3] = { testBox.x.Intersect(&ray), testBox.y.Intersect(&ray), testBox.z.Intersect(&ray) };



		//test[0].intersectWithInfiniteInterval();
		testX.intersect(testY);
		testX.intersect(testZ);
		test[0].intersect(test[1]);
		test[0].intersect(test[2]);
		
		//if (testX.intersect(testY) && testX.intersect(testZ) && test[0].intersect(test[1]) && test[0].intersect(test[2]))
		{
			//if (test[0].t0 != tOut0 || test[0].t1 != tOut1)
			if(fabs(test[0].t0 - testX.t0) > EPSILON || fabs(test[0].t1 - testX.t1) > EPSILON)
			{
				int george = 1;
				george++;

				exit(-1);
			}

			
		}


		*/
		/*
		
		else
		{
			int bob = 1;
			bob++;
			exit(-2);
		}

		*/


		if (testX.intersect(testY) && testX.intersect(testZ) && testX.isValidInterval() )
		{
			return testX.t0;
		}

		return INF;
			
		

		/*
		if (test[0].t0 == test[1].t0 && test[1].t0 == test[2].t0 && test[0].t1 == test[1].t1 && test[1].t1 == test[2].t1 && test[0].t0 == 1.f && test[0].t1 == 1.f)
		{
			return 0;
		}
	
		else
		{
			//test[0].intersectWithInfiniteInterval();
			if (test[0].intersect(test[1]) && test[0].intersect(test[2]) && test[0].isValidInterval() && (test[0].t0 > 0 || test[0].t1 > 0))
			{

				if (test[0].t0>0)
				{
					//i->t = test[0].t0;
					return test[0].t0;
				}
				else
				{
					//i->t = test[0].t1;
					return test[0].t1;
				}
				//i->t = test[0].t0;
				//i->intersectedShape = this;
				//i->intersectionPoint = r.pointAtDistance(test[0].t0);
				//i->normal = test[0].normal0;
				//	i->boundingBox = this->bbox();


			//	return true;





			}

			else
			{
				//No intersect
			//	i = NULL;
			//	return false;
				return INF;
			}
		}
		*/
		/*
		float rayDotXNorm, rayDotYNorm, rayDotZNorm;
		rayDotXNorm = ray.direction.dot(x.normal);
		rayDotYNorm = ray.direction.dot(y.normal);
		rayDotZNorm = ray.direction.dot(z.normal);


		if (rayDotXNorm != 0 &&  rayDotYNorm != 0 && rayDotZNorm != 0) 
		{
			
		

	
		
				float tX0, tY0, tZ0, tZ1, tX1, tY1, tOut0, tOut1;
				tX0 = std::min(-(x.d0 + x.normal.dot(ray.startingPoint)) / (x.normal.dot(ray.direction)), -(x.d1 + x.normal.dot(ray.startingPoint)) / (x.normal.dot(ray.direction)));
				tX1 = std::max(-(x.d0 + x.normal.dot(ray.startingPoint)) / (x.normal.dot(ray.direction)), -(x.d1 + x.normal.dot(ray.startingPoint)) / (x.normal.dot(ray.direction)));

				tY0 = std::min(-(y.d0 + y.normal.dot(ray.startingPoint)) / (y.normal.dot(ray.direction)), -(y.d1 + y.normal.dot(ray.startingPoint)) / (y.normal.dot(ray.direction)));
				tY1 = std::max(-(y.d0 + y.normal.dot(ray.startingPoint)) / (y.normal.dot(ray.direction)), -(y.d1 + y.normal.dot(ray.startingPoint)) / (y.normal.dot(ray.direction)));

				
				
				tZ0 = std::min(-(z.d0 + z.normal.dot(ray.startingPoint)) / (z.normal.dot(ray.direction)), -(z.d1 + z.normal.dot(ray.startingPoint)) / (z.normal.dot(ray.direction)));
				tZ1 = std::max(-(z.d0 + z.normal.dot(ray.startingPoint)) / (z.normal.dot(ray.direction)), -(z.d1 + z.normal.dot(ray.startingPoint)) / (z.normal.dot(ray.direction)));

				tOut0 = std::max(0.f,(std::max(std::max(tX0, tY0),tZ0)));
				tOut1 = std::min(INF,(std::min(std::min(tX1,tY1), tZ1)));

				if(tOut0 > tOut1  || (tOut0 <0 && tOut1 <0))
				{
					return INF;
				}

				else
				{
					if (tOut0 < 0)
					{
						return tOut1;
					}

					else
					{
						return tOut0;
					}
				}



			}
		


		else
		{
			float x0, y0, z0, x1, y1, z1;
			x0 = ray.startingPoint.dot(x.normal) + x.d0;
			y0 = ray.startingPoint.dot(y.normal) + y.d0;
			z0 = ray.startingPoint.dot(z.normal) + z.d0;
			x1 = ray.startingPoint.dot(x.normal) + x.d1;
			y1 = ray.startingPoint.dot(y.normal) + y.d1;
			z1 = ray.startingPoint.dot(z.normal) + z.d1;

			if (x0*x1 <0 && y0*y1 <0 && z0*z1 <0)
			{
				return 0;
			}

			else {
					return INF;
				}
			}
			*/
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

	Vector3f Radiance(Vector3f point) { return Kd; }  //Return the radiance of the light


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
	std::vector<MeshData*> meshes;
	
	Camera camera;
    Scene();
    void Finit();
	Vector3f TracePath(Ray& r, KdBVH<float, 3, Shape*>& Tree);
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
