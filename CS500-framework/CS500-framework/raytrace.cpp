//////////////////////////////////////////////////////////////////////
// Provides the framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include <vector>

#ifdef _WIN32
    // Includes for Windows
    #include <windows.h>
    #include <cstdlib>
    #include <limits>
    #include <crtdbg.h>
#else
    // Includes for Linux
#endif
#include <ctime>
#include "geom.h"
#include "raytrace.h"
#include "realtime.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"




	// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::mt19937_64 RNGen;
std::uniform_real_distribution<> myrandom(0.0, 1.0);
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].

//Material m;








Scene::Scene() : shapes(), lights()
{ 
    realtime = new Realtime(); 
}

void Scene::Finit()
{
}

void Scene::triangleMesh(MeshData* mesh) 
{ 
    realtime->triangleMesh(mesh); 
	meshes.push_back(mesh);
}

Quaternionf Orientation(int i, 
                        const std::vector<std::string>& strings,
                        const std::vector<float>& f)
{
    Quaternionf q(1,0,0,0); // Unit quaternion
    while (i<strings.size()) {
        std::string c = strings[i++];
        if (c == "x")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitX());
        else if (c == "y")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitY());
        else if (c == "z")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitZ());
        else if (c == "q")  {
            q *= Quaternionf(f[i+0], f[i+1], f[i+2], f[i+3]);
            i+=4; }
        else if (c == "a")  {
            q *= angleAxis(f[i+0]*Radians, Vector3f(f[i+1], f[i+2], f[i+3]).normalized());
            i+=4; } }


    return q;
}

////////////////////////////////////////////////////////////////////////
// Material: encapsulates surface properties
void Material::setTexture(const std::string path)
{
    int width, height, n;
    stbi_set_flip_vertically_on_load(true);
    unsigned char* image = stbi_load(path.c_str(), &width, &height, &n, 0);

    // Realtime code below:  This sends the texture in *image to the graphics card.
    // The raytracer will not use this code (nor any features of OpenGL nor the graphics card).
    glGenTextures(1, &texid);
    glBindTexture(GL_TEXTURE_2D, texid);
    glTexImage2D(GL_TEXTURE_2D, 0, n, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 100);
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, (int)GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, (int)GL_LINEAR_MIPMAP_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    stbi_image_free(image);
}

void Scene::Command(const std::vector<std::string>& strings,
                    const std::vector<float>& f)
{
	bool newMat = false;
    if (strings.size() == 0) return;
    std::string c = strings[0];
	if (c[0] != '#')
	{
		if (c == "screen") {
			// syntax: screen width height
			realtime->setScreen(int(f[1]), int(f[2]));
			width = int(f[1]);
			height = int(f[2]);

			camera.setScreen(int(f[1]), int(f[2]));
		}
		else if (c == "camera") {
			// syntax: camera x y z   ry   <orientation spec>
			// Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry
			realtime->setCamera(Vector3f(f[1], f[2], f[3]), Orientation(5, strings, f), f[4]);
			camera.setCamera(Vector3f(f[1], f[2], f[3]), Orientation(5, strings, f), f[4]);
			//newMat = true;
		}

		else if (c == "ambient") {
			// syntax: ambient r g b
			// Sets the ambient color.  Note: This parameter is temporary.
			// It will be ignored once your raytracer becomes capable of
			// accurately *calculating* the true ambient light.
			realtime->setAmbient(Vector3f(f[1], f[2], f[3]));
			camera.setAmbient(Vector3f(f[1], f[2], f[3]));

		}

		else if (c == "brdf") {
			// syntax: brdf  r g b   r g b  alpha
			// later:  brdf  r g b   r g b  alpha  r g b ior
			// First rgb is Diffuse reflection, second is specular reflection.
			// third is beer's law transmission followed by index of refraction.
			// Creates a Material instance to be picked up by successive shapes
			if (f.size() < 11)
			{
				currentMat = new Material(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7]);
			}
			else
			{
				currentMat = new Material(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], Vector3f(f[8], f[9], f[10]), f[11]);
			}
			newMat = true;
		}

		else if (c == "light") {
			// syntax: light  r g b   
			// The rgb is the emission of the light
			// Creates a Material instance to be picked up by successive shapes
			currentMat = new Light(Vector3f(f[1], f[2], f[3]));
			//newMat = true;
		}

		else if (c == "sphere") {
			// syntax: sphere x y z   r
			// Creates a Shape instance for a sphere defined by a center and radius
			realtime->sphere(Vector3f(f[1], f[2], f[3]), f[4], currentMat);
			Sphere* s = new Sphere(Vector3f(f[1], f[2], f[3]), f[4], currentMat);
			shapes.push_back(s);
			if (currentMat->isLight())
			{
				lights.push_back(s);
			}


		}

		else if (c == "box") {
			// syntax: box bx by bz   dx dy dz
			// Creates a Shape instance for a box defined by a corner point and diagonal vector
			realtime->box(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), currentMat);
			AABB* b = new AABB(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), currentMat);
			shapes.push_back(b);
			if (currentMat->isLight())
			{
				lights.push_back(b);
			}
		}

		else if (c == "cylinder") {
			// syntax: cylinder bx by bz   ax ay az  r
			// Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
			realtime->cylinder(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], currentMat);
			Cylander* c = new Cylander(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], currentMat);
			shapes.push_back(c);
			if (currentMat->isLight())
			{
				lights.push_back(c);
			}

		}

		else if (c == "mesh") {
			// syntax: mesh   filename   tx ty tz   s   <orientation>
			// Creates many Shape instances (one per triangle) by reading
			// model(s) from filename. All triangles are rotated by a
			// quaternion (qw qx qy qz), uniformly scaled by s, and
			// translated by (tx ty tz) .
			Matrix4f modelTr = translate(Vector3f(f[2], f[3], f[4]))
				*scale(Vector3f(f[5], f[5], f[5]))
				*toMat4(Orientation(6, strings, f));
			ReadAssimpFile(strings[1], modelTr);
		}

		else if (c == "intersect")
		{
			//Pop off the previous two shapes, intersect them, push the resulting object back onto the list of shapes

			Shape* s(shapes.back()->Copy());
			shapes.pop_back();
			Shape* s1(shapes.back()->Copy());
			shapes.pop_back();

			

			shapes.push_back(new Intersect(s->Copy(), s1->Copy()));
			//shapes.push_back(new Intersect(s1, s));
		}

		else if (c == "union")
		{
			//Pop off the previous two shapes, intersect them, push the resulting object back onto the list of shapes

			Shape* s(shapes.back()->Copy());
			shapes.pop_back();
			Shape* s1(shapes.back()->Copy());
				shapes.pop_back();

	

			shapes.push_back(new Union(s1->Copy(), s->Copy()));
			//shapes.push_back(new Union(s1, s));
		}

		else if (c == "difference")
		{
			//Pop off the previous two shapes, intersect them, push the resulting object back onto the list of shapes

			Shape* s(shapes.back()->Copy());
			shapes.pop_back();
			Shape* s1(shapes.back()->Copy());
			shapes.pop_back();

			shapes.push_back(new Difference(s1->Copy(), s->Copy()));
			//shapes.push_back(new Difference(s1, s));
		}


		else {
			fprintf(stderr, "\n*********************************************\n");
			fprintf(stderr, "* Unknown command: %s\n", c.c_str());
			fprintf(stderr, "*********************************************\n\n");
		}
	}
}

Box3d bounding_box(Shape* s)
{
	return (s->bbox());
}


// D, G (G1), AND F FUNCTIONS
float D(const Vector3f& m, const Vector3f& norm, float alpha)
{
	float mN = m.dot(norm);
	if(mN<=0)
	{
		return 0;
	}

	else
	{
		return (powf(mN, alpha)) * ((alpha + 2.f) / (2.f * PI));
	}

}

Vector3f F(float dot, const Vector3f& Ks)
{
	float a = 1.f-abs(dot);
	return Ks + ((ONES - Ks)*(a*a*a*a*a));
}

float G1(const Vector3f& v,const Vector3f& m,const Vector3f& norm, float alpha) 
{
	float vM = v.dot(m);
	float vN = v.dot(norm);
	if(vM/vN <=0)
	{
		return 0;
	}
	else
	{
		if(vN >1)
		{
			return 1.f;
		}
		else
		{
			float tan = sqrtf(1.f - (vN*vN)) / vN;
			if (tan == 0)
			{
				return 1.f;
			}

			else {

				float a = (sqrtf((alpha / 2.f) + 1.f)) / abs(tan);

				if (a < 1.6)
				{
					return ((3.535f*a) + (2.181f*a*a)) / (1.f + (2.276*a) + (2.577*a*a));
				}
				else
				{
					return 1.f;
				}
			}
		}
	}
}

float G(const Vector3f& wI, const Vector3f& wO, const Vector3f& m, const Vector3f& norm, float alpha)
{
	return G1(wI, m,norm, alpha) * G1(wO, m,norm, alpha);
}




//PROJECT 3
BRDFChoice Chooser(float randNum, Shape* s)
{


	float sum1;// , sum2;
	sum1 = s->mat->Pd + s->mat->Pr;
	//sum2 = sum1 + s->mat->Pt;

	if (randNum < s->mat->Pd)
	{
		return BRDFChoice::DIFFUSE;
	}
	else if(randNum < (sum1))
	{
		return BRDFChoice::REFLECTION;
	}

	else
	{
	 return BRDFChoice::TRANSMISSION;
	}


}

//PROJECT 2 STUFF GOES HERE
//
IntersectRecord Sphere::SampleSphere()//(Vector3f center, float radius)
{
	float rand[2] = { myrandom(RNGen), myrandom(RNGen) };
	float z, r, a;
	z = 2 * (rand[0]) - 1;
	r = sqrtf(1 - powf(z, 2));
	a = 2 * PI * rand[1];
	Vector3f norm(r*cosf(a), r*sinf(a), z);
	return IntersectRecord(norm, norm*radius + center, 0, this);
}

void Sphere::SampleSphere(IntersectRecord& i)
{
	float rand[2] = { myrandom(RNGen), myrandom(RNGen) };
	float z, r, a;
	z = 2 * (rand[0]) - 1;
	r = sqrtf(1 - powf(z, 2));
	a = 2 * PI * rand[1];
	Vector3f norm(r*cosf(a), r*sinf(a), z);
	//return IntersectRecord(norm, norm*radius + center, 0, this);
	i.normal = norm;
	i.intersectionPoint = norm*radius + center;
	i.intersectedShape = this;
	
}

IntersectRecord SampleLight(Scene& s)
{
	Sphere* a = static_cast<Sphere*>(s.lights[0]);
	return a->SampleSphere();
}
void SampleLight(Scene& s, IntersectRecord& r)
{
	Sphere* a = static_cast<Sphere*>(s.lights[0]);
	return a->SampleSphere(r);
}
Vector3f SampleLobe(const Vector3f& norm, float cosineOfAngleBetweenReturnAndNormal, float phi)
{
float s = sqrtf(1 - powf(cosineOfAngleBetweenReturnAndNormal, 2));
Vector3f K(s*cosf(phi), s*sinf(phi),cosineOfAngleBetweenReturnAndNormal);
Quaternionf q = Quaternionf::FromTwoVectors(Vector3f::UnitZ(), norm);
return q._transformVector(K);
}

float GeometryFactor(IntersectRecord& a, IntersectRecord& b)
{
Vector3f d = a.intersectionPoint - b.intersectionPoint;
return fabs(((a.normal.dot(d))*(b.normal.dot(d))) / powf(d.dot(d), 2));
}
//For proj 3
float PDFBRDF(BRDFChoice choice, const Vector3f& wO, const Vector3f& norm, const Vector3f& wI, float alpha,float indexOfRefraction, float indexOfRefractionRatio, float wON)  //Project 4
{
	if (choice == BRDFChoice::DIFFUSE)
	{
		return abs(wI.dot(norm)) / PI;
	}

	else if(choice==BRDFChoice::REFLECTION)
	{
		Vector3f m = (wO + wI).normalized();
		return (D(m, norm, alpha) * abs(m.dot(norm))  / (4 * abs(wI.dot(m))));

	}

	else
	{  Vector3f m;
	float n0, n1;
	n0 = n1 = 1.f;
		if(wON >=0)
	{ n0 = indexOfRefraction;
		m = -(wI*indexOfRefraction + wO).normalized();
	}
	else
	{ n1=indexOfRefraction;
		m = -(wI + wO*indexOfRefraction).normalized();
	}
	
				float wOM = wO.dot(m);
			float r = 1 - ((indexOfRefractionRatio*indexOfRefractionRatio)*(1-(wOM*wOM)));
	if(r<0)  //Total internal reflection
	{
	//	return (D(m, norm, alpha) * abs(m.dot(norm)) / (4 * abs(wI.dot(m))));
	 return PDFBRDF(BRDFChoice::REFLECTION, wO, norm, wI, alpha, indexOfRefraction, indexOfRefractionRatio, wON);
	 //Might need to double check this
	}
	else
	{ float wIM = (wI.dot(m));
	float t = n0*wIM + n1*wOM;
	  return D(m,norm,alpha) * abs(m.dot(norm)) * ( ( (n0*n0) *abs(wIM))/(t*t));
	}
	}
}

float PDFBRDF(Vector3f& norm, Vector3f& wI)
{

return fabs(wI.dot(norm))/PI;
//return PDFBRDF(ZEROES, norm, ZEROES);
}

Vector3f EvalBRDF(IntersectRecord& P)
{
if(P.intersectedShape == NULL || P.intersectedShape->mat == NULL)
{
std::cout << "Shape / Material pointer in EvalBRDF is NULL, exiting." << std::endl;
exit(-123);
}

else if(P.intersectedShape->mat->isLight())
{
std::cout<<"Material pointer in EvalBRDF points to a LIGHT, exiting." << std::endl;
exit(-456);
}

else
{
return P.intersectedShape->mat->Kd / PI;
}

}


Vector3f EvalBRDF(BRDFChoice choice, const Vector3f& wO, const Vector3f& norm, const Vector3f& wI, const Vector3f& Kd, float alpha, const Vector3f& Ks, const Vector3f& Kt, float indexOfRefraction, float indexOfRefractionRatio, float wON, float tVal)  //Project 4
{
	if (choice == BRDFChoice::DIFFUSE)
	{
		return Kd / PI;
	}

	else if(choice == BRDFChoice::REFLECTION)
	{
		Vector3f m = (wO + wI).normalized();

//		Vector3f f = F(wI.dot(m), Ks);
//		float d = D(m, norm, alpha);
//		float g1 = G1(wI, m, norm, alpha);
//		float g2 = G1(wO, m, norm, alpha);


		return (D(m, norm, alpha) * G(wI, wO, m, norm, alpha) * F(wI.dot(m), Ks)) / (4.f * abs(wI.dot(norm)) * abs(wO.dot(norm)));
	}

	else
	{
		//Beers law values:  f(t) = e ^ (t ln Kt) 
		Vector3f beersLawVals(exp(tVal*logf(Kt(0))), exp(tVal*logf(Kt(1))), exp(tVal*logf(Kt(2))));
		Vector3f m;
		float n0, n1;
		n0 = n1 = 1.f;
		if (wON >= 0)
		{
			n0 = indexOfRefraction;
			m = -(wI*indexOfRefraction + wO).normalized();
		}
		else
		{
			n1 = indexOfRefraction;
			m = -(wI + wO*indexOfRefraction).normalized();
		}

		float wOM = wO.dot(m);
		float wIM = wI.dot(m);
		float r = 1 - powf(indexOfRefractionRatio, 2)*(1 - powf(wOM, 2));
		if (r < 0)  //Total internal reflection
		{
			return beersLawVals.cwiseProduct((D(m, norm, alpha) * G(wI, wO, m, norm, alpha) * F(wI.dot(m), Ks)) / (4.f * abs(wI.dot(norm)) * abs(wO.dot(norm))));

			//return EvalBRDF(BRDFChoice::REFLECTION, wO, norm, wI, Kd, alpha, Ks, Kt, indexOfRefraction, indexOfRefractionRatio, wON, tVal);  //CHECK THIS--may not be correct due to incorrect 'm' vector
		}
		else
		{
			float t = n0*wIM + n1*wOM;
			return   beersLawVals.cwiseProduct(((D(m, norm, alpha) * G(wI, wO, m, norm, alpha) * (ONES - F(wIM, Ks))) / (abs(wON) * abs(wI.dot(norm)))) * ((abs(wIM) * abs(wOM) * (n0*n0)) / (t*t)));
		}
	}
}

Vector3f SampleBRDF(const Vector3f& norm)
{
	return SampleLobe(norm, sqrtf(myrandom(RNGen)), 2 * PI * myrandom(RNGen));
}

Vector3f SampleBRDF(BRDFChoice choice, const Vector3f& wO, const Vector3f& norm, float alpha,float indexOfRefractionRatio, float wON)  //Project 4
{
	if (choice == BRDFChoice::DIFFUSE)
	{
		return SampleBRDF(norm);
	}
	else if(choice==BRDFChoice::REFLECTION)
	{
		Vector3f m = SampleLobe(norm, powf(myrandom(RNGen), (1.f / (1 + alpha))), 2.f*PI*myrandom(RNGen));
		return (2.f*(wO.dot(m))*m) - wO;
	}
	else
	{	Vector3f m = SampleLobe(norm, powf(myrandom(RNGen), (1.f / (1 + alpha))), 2.f*PI*myrandom(RNGen));
			float wOM = wO.dot(m);
			float r = 1 - ((indexOfRefractionRatio*indexOfRefractionRatio)*(1-(wOM*wOM)));
			if(r<0)  //Total internal reflection
			 { 
				//return 2.f*wOM*m - wO;
				return SampleBRDF(BRDFChoice::REFLECTION, wO, norm, alpha, indexOfRefractionRatio, wON);
			}
		
			else
			{  
	
				if (wON >= 0)
				{
				return (indexOfRefractionRatio *wOM - sqrtf(r))*m - (indexOfRefractionRatio *wO);
				}

				else
				{
				return (indexOfRefractionRatio *wOM + sqrtf(r))*m - (indexOfRefractionRatio *wO);
				}
	
			}
	}
}




float PDFLight(IntersectRecord& r ) //, Scene& s)
{
//If this doesn't work, just use the explicit area formula for the light
//return 1/(s.shapes.size() * r.intersectedShape->area);
	float a = r.intersectedShape->area;
	if (a == 0)
	{
		a = r.intersectedShape->calculateArea();
		r.intersectedShape->area = a;
	}
	return 1/a;
}






bool isZeroes(Vector3f& v)
{
	return ((v(1) > -EPSILON && v(1) <= EPSILON) && (v(0) > -EPSILON && v(0) <= EPSILON) && (v(2) > -EPSILON && v(2) <= EPSILON));
}

bool isfabsoluteZero(Vector3f& v)
{
	return (v(0) == v(1) && v(1) == v(2) && v(2) == 0);
}



Vector3f Scene::TracePath(Ray& ray, KdBVH<float, 3, Shape*>& Tree)
{
	//int count = 0;

	//Intersect ray w/ scene
	//IntersectRecord rec;


	/*
	IntersectRecord smallest =  IntersectRecord();
	IntersectRecord temp =  IntersectRecord();
	smallest.t = INF;
	for (int i = 0; i < shapes.size(); i++)
	{

	if (shapes[i]->Intersect(&r, &temp) && temp.t < smallest.t) //(temp.t - smallest.t) < EPSILON )
	{
	smallest = temp;
	}
	}
	*/
	



	Minimizer m(ray);
	float minDist = BVMinimize(Tree, m);
	if (m.smallest.intersectedShape == NULL)
	{
		return ZEROES;
	}

	else
	{
		IntersectRecord P = m.smallest;
		if (m.smallest.intersectedShape->mat->isLight())
		{
			return static_cast<Light*>(m.smallest.intersectedShape->mat)->Radiance(P.intersectionPoint);
		}


		else
		{
			Vector3f outColor( ZEROES);
			Vector3f weights (ONES);
			Vector3f w0 = -ray.direction;
			while (myrandom(RNGen) < RUSSIAN_ROULETTE)
			{
				//count++;
				//Explicit light connection
				//Generate lightray from P -> carefully chosen rand. pt. on a light
				//Use floor(lights.size() * myrandom(RNGen)    to randomly choose a light from the vector of lights
				//IntersectRecord L = SampleLight(*this); // PART OF SECTION 2
				IntersectRecord L;
				SampleLight(*this, L);
				float wON = w0.dot(P.normal);
				float ratio = P.intersectedShape->mat->indexOfRefraction;
				if (wON > 0 && ratio > 0)
				{
					ratio = 1.f / ratio;
				}


				float pExplicit = PDFLight(L) / GeometryFactor(P, L);



				Vector3f wIToExplicitLight(L.intersectionPoint - P.intersectionPoint);
				//Ray I(P, wIToExplicitLight);
				//Ray 
				//Minimizer explicitMinimizer


				//Actually this might all do the SECTION 2 stuff already
				//IntersectRecord randomLight = static_cast<Sphere*>(lights[0])->SampleSphere();
			
				Ray explicitLightRay(P.intersectionPoint, wIToExplicitLight);
				Minimizer explicitLightRayMinimizer(explicitLightRay);
				float minExplicitLightRayDistance = BVMinimize(Tree, explicitLightRayMinimizer);
				if (pExplicit > 0 && explicitLightRayMinimizer.smallest.intersectedShape != NULL && (fabs(explicitLightRayMinimizer.smallest.intersectionPoint(0) - L.intersectionPoint(0)) < EPSILON) && (fabs(explicitLightRayMinimizer.smallest.intersectionPoint(1) - L.intersectionPoint(1)) < EPSILON) && (fabs(explicitLightRayMinimizer.smallest.intersectionPoint(2) - L.intersectionPoint(2)) < EPSILON))
				{

					//PROJECT 3
					if (pExplicit < EPSILON)
					{
						break;
					}

					//Vector3f w02(-wIToExplicitLight.normalized());
					//BRDFChoice c = Chooser(myrandom(RNGen), P.intersectedShape);
					//Calculate extra MIS weights here
					//outColor += this light's contribution

					//PROJECT 4
					//float wON = wO.dot(P.normal);
					//float r = P.intersectedShape->mat->indexOfRefraction;
					//if(wON >0) 
					//{r = 1.f/r;}

					//PART OF SECTION 2
			

					
					Vector3f f(fabs(P.normal.dot(wIToExplicitLight)) * EvalBRDF(BRDFChoice::DIFFUSE, w0, P.normal, wIToExplicitLight, P.intersectedShape->mat->Kd, P.intersectedShape->mat->alpha, P.intersectedShape->mat->Ks,P.intersectedShape->mat->Kt, P.intersectedShape->mat->indexOfRefraction, ratio, wON, P.t)); // Project 2 version EvalBRDF(P));
					f += fabs(P.normal.dot(wIToExplicitLight)) * EvalBRDF(BRDFChoice::REFLECTION, w0, P.normal, wIToExplicitLight, P.intersectedShape->mat->Kd, P.intersectedShape->mat->alpha, P.intersectedShape->mat->Ks, P.intersectedShape->mat->Kt, P.intersectedShape->mat->indexOfRefraction, ratio, wON, P.t);
					f += fabs(P.normal.dot(wIToExplicitLight)) * EvalBRDF(BRDFChoice::TRANSMISSION, w0, P.normal, wIToExplicitLight, P.intersectedShape->mat->Kd, P.intersectedShape->mat->alpha, P.intersectedShape->mat->Ks,P.intersectedShape->mat->Kt, P.intersectedShape->mat->indexOfRefraction, ratio, wON, P.t);
					
					
					//SECTION 3:  add in MIS factor
					float q = RUSSIAN_ROULETTE * PDFBRDF(P.normal, wIToExplicitLight);  //Removed for project 3
					//float MIS = (pExplicit*pExplicit) / (pExplicit*pExplicit + q*q);   Removed for project 3
					//float MIS = 1.f;  //Project 3


					//project 4
					float qD, qR, qT, MIS;
					qD = PDFBRDF(BRDFChoice::DIFFUSE, w0, P.normal, wIToExplicitLight,P.intersectedShape->mat->alpha, P.intersectedShape->mat->indexOfRefraction,ratio,wON);
					qR = PDFBRDF(BRDFChoice::REFLECTION, w0, P.normal, wIToExplicitLight, P.intersectedShape->mat->alpha, P.intersectedShape->mat->indexOfRefraction, ratio, wON);
					qT = PDFBRDF(BRDFChoice::TRANSMISSION, w0, P.normal, wIToExplicitLight, P.intersectedShape->mat->alpha, P.intersectedShape->mat->indexOfRefraction, ratio, wON);
					MIS = (pExplicit*pExplicit) / (q*q + qD*qD + qR*qR+qT*qT);
					//if ((f / pExplicit).isZero())
					//{
					//	std::cout << "f / pExplicit is 0, something's wrong prolly." << std::endl;
					//}

					//else if (weights.isZero())
					//{
					//	std::cout << "weights almost 0, something might be wrong." << std::endl;

					//
					//}

					//else if ((static_cast<Light*>(L.intersectedShape->mat)->Radiance(L.intersectionPoint)).isZero())
					//{
					//	std::cout << "Radiance at intersection point is 0, this one might be ok." << std::endl;
					//}

					

					outColor += weights.cwiseProduct((f / pExplicit).cwiseProduct(static_cast<Light*>(L.intersectedShape->mat)->Radiance(L.intersectionPoint)));//*MIS;
				
				}
				
				
				/*
				else if (explicitLightRayMinimizer.smallest.intersectedShape == NULL)
				{
					int sss = 1;
					sss++;
					break;

				}
				*/


				BRDFChoice c = Chooser(myrandom(RNGen), P.intersectedShape);

				//Extend path
				//Generate new ray in some carefully chosen random direction from P
				//Vector3f wI ( SampleBRDF(P.normal));  // PART OF SECTION 1
				Vector3f wI(SampleBRDF(c, w0, P.normal, P.intersectedShape->mat->alpha,ratio, wON));  //Project 3 - reflections
				Ray newRay(P.intersectionPoint, wI);  // PART OF SECTION 1
				Minimizer newRayMinimizer(newRay);
				float minNewRayDistance = BVMinimize(Tree, newRayMinimizer);
				IntersectRecord Q = newRayMinimizer.smallest;
				if (Q.intersectedShape == NULL) //If the ray hits something, it probably exists
				{
					break;
					

					//Figure out where this chunk goes exactly?
					//PART OF SECTION ONE
					
					/*
					Vector3f f = fabs(P.normal.dot(wI)) * EvalBRDF(P.normal);
					float p1 = RUSSIAN_ROULETTE * PDFBRDF(P.normal, wI);
					if(p<EPSILON)
					{
						//Avoid div. by 0 or almost 0
						break;
						
					}
					else
					{
						weights *= (f / p1);
					}
					
					
					*/



				}
				else
				{
					//Calculate MIS weights here
					//outColor += this light's contribution

					//PART OF SECTION ONE

//					Vector3f eBRDF(EvalBRDF(c, w0, P.normal, wI, P.intersectedShape->mat->Kd, P.intersectedShape->mat->alpha, P.intersectedShape->mat->Ks));

					Vector3f f ( fabs(P.normal.dot(wI)) *EvalBRDF(c, w0, P.normal, wI, P.intersectedShape->mat->Kd, P.intersectedShape->mat->alpha, P.intersectedShape->mat->Ks, P.intersectedShape->mat->Kt, P.intersectedShape->mat->indexOfRefraction, ratio,wON, P.t)); // Project 2 version EvalBRDF(P));
					float p = RUSSIAN_ROULETTE * PDFBRDF(c, w0, P.normal, wI, P.intersectedShape->mat->alpha,P.intersectedShape->mat->indexOfRefraction,ratio,wON);//PDFBRDF(P.normal, wI);
					if (p<EPSILON)
					{
						//Avoid div. by 0 or almost 0
						break;

					}
					else
					{
						weights = weights.cwiseProduct((f / p));



						float q = PDFLight(Q) / GeometryFactor(P, Q);
						


						//float MIS = (p*p) / (p*p + q*q);
						//MIS = 1.f;   //Project 3
									 //project 4
									 //float qD, qR, qT;
						float qD, qR, qT, MIS;
						qD = PDFBRDF(BRDFChoice::DIFFUSE, w0, P.normal, wIToExplicitLight, P.intersectedShape->mat->alpha, P.intersectedShape->mat->indexOfRefraction, ratio, wON);
						qR = PDFBRDF(BRDFChoice::REFLECTION, w0, P.normal, wIToExplicitLight, P.intersectedShape->mat->alpha, P.intersectedShape->mat->indexOfRefraction, ratio, wON);
						qT = PDFBRDF(BRDFChoice::TRANSMISSION, w0, P.normal, wIToExplicitLight, P.intersectedShape->mat->alpha, P.intersectedShape->mat->indexOfRefraction, ratio, wON);

									 MIS = (p*p) / (q*q + qD*qD + qR*qR+qT*qT);
						if (Q.intersectedShape->mat->isLight())
						{



							//if ((f / p).isZero())
							//{
							//	std::cout << "f / p is 0, something's wrong prolly." << std::endl;
							//}

							//else if (weights.isZero())
							//{
							//	std::cout << "weights is 0, something is VERY wrong." << std::endl;
							//}

							//else if ((static_cast<Light*>(Q.intersectedShape->mat)->Radiance(Q.intersectionPoint)).isZero())
							//{
							//	std::cout << "Radiance at intersection point is 0, this one might be ok." << std::endl;
							//}





							outColor += weights.cwiseProduct(static_cast<Light*>(Q.intersectedShape->mat)->Radiance(Q.intersectionPoint));//*MIS;
							break;
						}
					}
				}

				P = Q;
				w0 = -wI;
			}
		
			//if (isfabsoluteZero(outColor))
			//{
			//	std::cout << "Why is the output Zeroes ajls;afjas" << std::endl;
			//}
			return outColor;
		}
	}


}


void Scene::TraceImage(Color* image, const int pass)
{
   // realtime->run();                          // Remove this (realtime stuff)

	/*

	for (int i = 0; i < meshes.size(); ++i)
	{
		for (int j = 0; j < meshes[i]->triangles.size(); ++j)
		{
			shapes.push_back(new Triangle(
				meshes[i]->vertices[meshes[i]->triangles[j](0)].pnt,
				meshes[i]->vertices[meshes[i]->triangles[j](1)].pnt,
				meshes[i]->vertices[meshes[i]->triangles[j](2)].pnt,
				meshes[i]->vertices[meshes[i]->triangles[j](0)].nrm,
				meshes[i]->vertices[meshes[i]->triangles[j](1)].nrm,
				meshes[i]->vertices[meshes[i]->triangles[j](2)].nrm,
				meshes[i]->mat)
			);
		}
	}
	*/
	//MeshData* temp;
	for (int i = 0; i < meshes.size(); ++i)
	{
		//temp = meshes[i];
		for (int j = 0; j < meshes[i]->triangles.size(); ++j)
		{
			
			shapes.push_back(new Triangle(
				meshes[i]->vertices[meshes[i]->triangles[j][0]].pnt,
				meshes[i]->vertices[meshes[i]->triangles[j][1]].pnt,
				meshes[i]->vertices[meshes[i]->triangles[j][2]].pnt,
				meshes[i]->vertices[meshes[i]->triangles[j][0]].nrm,
				meshes[i]->vertices[meshes[i]->triangles[j][1]].nrm,
				meshes[i]->vertices[meshes[i]->triangles[j][2]].nrm,
				meshes[i]->mat)
			);
		}
	}


	KdBVH<float, 3, Shape*> Tree(shapes.begin(), shapes.end());

std::cout << "Number of shapes:  " << shapes.size() << std::endl;





//Project 2+ forever loop
std::string timingFilename("Timing info P5 - ");
timingFilename+=std::to_string(pass);
timingFilename += " Passes.txt";
std::ofstream fileOut(timingFilename, std::fstream::out | std::fstream::trunc);


for (int passes = 0; passes < pass; ++passes)

{
	std::cout << "Loop " << passes + 1 << " started." << std::endl;
	fileOut << "Loop " << passes + 1 << " started." << std::endl;
	std::chrono::time_point<std::chrono::system_clock> start, end;
	start = std::chrono::system_clock::now();
#pragma omp parallel for schedule(dynamic,1)
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{

			float dx, dy, minDist;
			float rx = (camera.ry * camera.width) / (camera.height);
			
			
			Vector3f bigX, bigY, bigZ;
			bigX = rx * camera.orient._transformVector(Vector3f::UnitX());
			bigY = camera.ry * camera.orient._transformVector(Vector3f::UnitY());
			bigZ = -1 * camera.orient._transformVector(Vector3f::UnitZ());





			int xCopy = x;
			int yCopy = y;
			
			
			//Depth-of-field stuff
			Vector3f P1, E1;
			float rxD, ryD, r1, theta;

			
			P1 = camera.eye + dx *camera.inFocus* bigX + dy*camera.inFocus * bigY + camera.inFocus*bigZ;

			r1 = camera.diskWidth *sqrtf( myrandom(RNGen));
			theta = 2.f*M_PI*myrandom(RNGen);
			rxD = r1*cosf(theta);
			ryD = r1*sinf(theta);
			E1 = camera.eye + rxD*bigX + rxD*bigY;
			

			
			
			
			Color color(0, 0, 0);
			//dx = (2 * ((xCopy + 0.5) / width)) - 1;
			//dy = (2 * ((yCopy + 0.5) / height)) - 1;

			//Use these for later
			dx = (2 * ((xCopy + myrandom(RNGen)) / width) - 1);
			dy = (2 * ((yCopy + myrandom(RNGen)) / height) - 1);



			Ray r(camera.eye, ((dx * bigX) + (dy*bigY) + bigZ));
			
		
		//Depth of field ray
		//	Ray r(E1, P1-E1);
			/*
			
			
			
			
			*/


			
			color = TracePath(r, Tree);
			if (color(0) != NAN && color(0) != INF &&color(1) != NAN && color(1) != INF &&color(2) != NAN && color(2) != INF && color(0) > 0 && color(1) > 0 &&color(2) > 0 )
			{
				image[yCopy*width + xCopy] += color;
			}

			if(x==width/2 && y==height/2)
			{
				std::cout << "Halfway there!" << std::endl;
			}

			



			//COMMENTING OUT PROJECT 1 STUFF STARTING HERE

//
///*			
//			IntersectRecord smallest =  IntersectRecord(); 
//			IntersectRecord temp =  IntersectRecord();
//			smallest.t = INF;
//			
//*/
//
//			//bool intersectionFound = false;
//			/*
//			if (x == 0 && y == 0)
//			{
//				int bob = 2;
//				bob++;
//
//				std::cout << "Ray direction at (0,0):  (" << r->direction(0) << ", "<< r->direction(1) << ", " << r->direction(2) << ") ." << std::endl;
//				std::cout << "Ray origin at (0,0):  (" << r->startingPoint(0) << ", " << r->startingPoint(1) << ", " << r->startingPoint(2) << ") ." << std::endl;
//
//
//
//
//			}
//
//			if (x == 399 && y == 299)
//			{
//				std::cout << "Ray direction at (399,299):  (" << r->direction(0) << ", " << r->direction(1) << ", " << r->direction(2) << ") ." << std::endl;
//				std::cout << "Ray origin at (399,299):  (" << r->startingPoint(0) << ", " << r->startingPoint(1) << ", " << r->startingPoint(2) << ") ." << std::endl;
//
//			}
//			
//
//			
//
//			if (x == 150 && y == 200)
//			{
//				int bob = 0;
//				bob++;
//			}
//			*/
//			/*
//			for (int i = 0; i < shapes.size(); i++)
//			{
//				
//				if (shapes[i]->Intersect(&r, &temp) && temp.t < smallest.t) //(temp.t - smallest.t) < EPSILON )
//				{
//					smallest = temp;
//				}
//			}
//			
//			if (smallest.intersectedShape != NULL)
//			{
//				bool containsPoint = smallest.intersectedShape->bbox().contains(smallest.intersectionPoint);
//				bool onSphere = static_cast<Sphere*>(smallest.intersectedShape)->radiusSquared < (smallest.intersectionPoint - static_cast<Sphere*>(smallest.intersectedShape)->center).squaredNorm();
//				if (!(containsPoint) || !(onSphere))
//				{
//					int a = 1;
//					a++;
//				}
//			}
//
//			*/
//			/*
//			if (y == 0 && x == 1)
//			{
//				int bob = 0;
//				bob++;
//
//				std::cout << "Intersection values at (0, 0):  " << std::endl;
//				std::cout << "Intersected object exists:  ";
//				if (smallest.intersectedShape != NULL)
//				{
//					std::cout << "YES" << std::endl;
//					std::cout << "Object's t value:  " << smallest.t << std::endl;
//					std::cout << "Point of intersection:  (" << smallest.intersectionPoint(0) << ", " << smallest.intersectionPoint(1) << ", " << smallest.intersectionPoint(2) << ").  " << std::endl;
//					std::cout << "Normal:  (" << smallest.normal(0) << ", " << smallest.normal(1) << ", " << smallest.normal(2) << ").  " << std::endl;
//					
//				}
//
//				else
//				{
//					std::cout << "NO";
//				}
//
//				std::cout << std::endl;
//			}
//
//			*/
//		
//			
//
//			
//				Minimizer m = Minimizer(r);
//				 minDist = BVMinimize(Tree, m);
//				 if (  m.smallest.intersectedShape != NULL)
//				 {
//			
//					 color = m.smallest.intersectedShape->mat->Kd;
//					// color = Vector3f(fabs(m.smallest.normal(0)), fabs(m.smallest.normal(1)), fabs(m.smallest.normal(2)));
//					// color = (((m->smallest.normal).dot((lights[0]->center - m->smallest.intersectionPoint))) * m->smallest.intersectedShape->mat->Kd) / PI;
//					// color = m->smallest.normal;
//				}
//
//				 /*
//				 if (m.smallest.intersectedShape != NULL)
//				 {
//					 bool containsPoint = m.smallest.intersectedShape->bbox().contains(m.smallest.intersectionPoint);
//					 Box3d box = m.smallest.intersectedShape->bbox();
//					 bool onSphere = static_cast<Sphere*>(m.smallest.intersectedShape)->radiusSquared < (m.smallest.intersectionPoint - static_cast<Sphere*>(m.smallest.intersectedShape)->center).squaredNorm();
//					 
//					 if (!(containsPoint) || !(onSphere))
//					 {
//
//
//						 std::cout << "Intersection Point:  (" << m.smallest.intersectionPoint(0) << ", " << m.smallest.intersectionPoint(1) << ", " << m.smallest.intersectionPoint(2) << ")." << std::endl;
//						 std::cout << "VS." << std::endl;
//						 std::cout << "Bounding box Max, Min points:  Max - (" << box.max()(0) << ", " << box.max()(1) << ", " << box.max()(2) << "),    Min - (" << box.min()(0) << ", " << box.min()(1) << ", " << box.min()(2) << ")." << std::endl;
//
//						 int a = 1;
//						 a++;
//					 }
//				 }
//				 */
//				//Cast ray here?
//				//Write color to image
//			//color = Vector3f(fabs(smallest.normal(0)), fabs(smallest.normal(1)), fabs(smallest.normal(2)));
//			//	color = Vector3f(1.0, 0, 0);
//			//image[y*width + x] = color;
///*
//			if (smallest.t == INF)
//			{
//				color = Vector3f(0, 1, 0);
//			}
//			else
//			{
//				color = ((smallest.t - 5) / 4) * Vector3f(1, 1, 1);
//			}
//
//			image[y*width + x] = color;
//			image[y*width + x] = Vector3f(x / 400.0, y / 300.0, 0);
//			image[y*width + x] = Vector3f(-1*r->direction(2), -1*r->direction(2), -1*r->direction(2));
//
//			
//			//color = ((smallest.t - 5) / 4) * Vector3f(1, 1, 1);
//
//			if (intersectionFound)
//			{
//				color = Vector3f(1, 0, 0);
//			}
//			else
//			{
//				color = Vector3f(0, 1, 0);
//			}
//		
//
//			
//			
//			
//			if (smallest.t == INF)
//			{
//				color = Vector3f(0, 0, 0);
//			}
//			else
//			{
//				color = ((smallest.t - 5) / 4) * Vector3f(1, 1, 1);
//			}
//			
//			
//			
//			if (smallest.intersectedShape != NULL)
//			{
//
//				color = smallest.intersectedShape->mat->Kd;
//			}
//			else
//			{
//				color = Vector3f(0, 0, 0);
//			}
//			*/
//			//color = Vector3f(fabs(smallest.normal(0)), fabs(smallest.normal(1)), fabs(smallest.normal(2)));
//				image[yCopy*width + xCopy] = color;
//
//
//

/*
Minimizer m(r);
minDist = BVMinimize(Tree, m);
if (m.smallest.intersectedShape != NULL)
{
	image[yCopy*width + xCopy] = m.smallest.intersectedShape->mat->Kd;
}
*/

				//COMMENTING OUT PROJECT 1 STUFF ENDING HERE
		}



	}
	std::cout << "Loop " << passes + 1 << " ended." << std::endl;
	fileOut << "Loop " << passes+1 << " ended." << std::endl;
	end = std::chrono::system_clock::now();

	std::chrono::duration<double> elapsed_seconds = end - start;
	std::time_t end_time = std::chrono::system_clock::to_time_t(end);

	fileOut << "finished computation at " << std::ctime(&end_time)
		<< "elapsed time: " << elapsed_seconds.count() << "s\n\n";
}

	



/*
#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
    for (int y=0;  y<height;  y++) {

        fprintf(stderr, "Rendering %4d\r", y);
        for (int x=0;  x<width;  x++) {
            Color color;
            if ((x-width/2)*(x-width/2)+(y-height/2)*(y-height/2) < 100*100)
                color = Color(myrandom(RNGen), myrandom(RNGen), myrandom(RNGen));
            else if (fabs(x-width/2)<4 || fabs(y-height/2)<4)
                color = Color(0.0, 0.0, 0.0);
            else 
                color = Color(1.0, 1.0, 1.0);
            image[y*width + x] = color;
        }
    }
    fprintf(stderr, "\n");
	*/


}
