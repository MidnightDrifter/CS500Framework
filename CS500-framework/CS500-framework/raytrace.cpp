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

#include "geom.h"
#include "raytrace.h"
#include "realtime.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"


//Material m;

// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::mt19937_64 RNGen;
std::uniform_real_distribution<> myrandom(0.0, 1.0);
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].

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
			currentMat = new Material(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7]);
			//newMat = true;
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


		else {
			fprintf(stderr, "\n*********************************************\n");
			fprintf(stderr, "* Unknown command: %s\n", c.c_str());
			fprintf(stderr, "*********************************************\n\n");
		}
	}
}

Box3d bounding_box(Shape* s)
{
	return *(s->bbox());
}


void Scene::TraceImage(Color* image, const int pass)
{
   // realtime->run();                          // Remove this (realtime stuff)


	KdBVH<float, 3, Shape*> Tree(shapes.begin(), shapes.end());

	
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
			Color color;
			dx = (2 * ((xCopy + 0.5) / width)) - 1;
			dy = (2 * ((yCopy + 0.5) / height)) - 1;

			//Use these for later
			// dx = 2 * ((x + myrandom(RNGen)) / (width - 1));
			// dy = 2 * ((y + myrandom(RNGen)) / (height - 1));



			Ray* r = new Ray(camera.eye, ((dx * bigX) + (dy*bigY) + bigZ));
			IntersectRecord smallest =  IntersectRecord(); 
			IntersectRecord temp =  IntersectRecord();
			smallest.t = INF;
			//bool intersectionFound = false;
			/*
			if (x == 0 && y == 0)
			{
				int bob = 2;
				bob++;

				std::cout << "Ray direction at (0,0):  (" << r->direction(0) << ", "<< r->direction(1) << ", " << r->direction(2) << ") ." << std::endl;
				std::cout << "Ray origin at (0,0):  (" << r->startingPoint(0) << ", " << r->startingPoint(1) << ", " << r->startingPoint(2) << ") ." << std::endl;




			}

			if (x == 399 && y == 299)
			{
				std::cout << "Ray direction at (399,299):  (" << r->direction(0) << ", " << r->direction(1) << ", " << r->direction(2) << ") ." << std::endl;
				std::cout << "Ray origin at (399,299):  (" << r->startingPoint(0) << ", " << r->startingPoint(1) << ", " << r->startingPoint(2) << ") ." << std::endl;

			}
			

			*/

			if (x == 150 && y == 200)
			{
				int bob = 0;
				bob++;
			}

			for (int i = 0; i < shapes.size(); i++)
			{
				//if (shapes[i]->Intersect(r, temp))
				//{
					//intersectionFound = true;
				//}
				
				if (shapes[i]->Intersect(r, &temp) && temp.t < smallest.t) //(temp.t - smallest.t) < EPSILON )
				{
					smallest = temp;
				}
			}
			
			/*

			if (y == 0 && x == 1)
			{
				int bob = 0;
				bob++;

				std::cout << "Intersection values at (0, 0):  " << std::endl;
				std::cout << "Intersected object exists:  ";
				if (smallest.intersectedShape != NULL)
				{
					std::cout << "YES" << std::endl;
					std::cout << "Object's t value:  " << smallest.t << std::endl;
					std::cout << "Point of intersection:  (" << smallest.intersectionPoint(0) << ", " << smallest.intersectionPoint(1) << ", " << smallest.intersectionPoint(2) << ").  " << std::endl;
					std::cout << "Normal:  (" << smallest.normal(0) << ", " << smallest.normal(1) << ", " << smallest.normal(2) << ").  " << std::endl;
					
				}

				else
				{
					std::cout << "NO";
				}

				std::cout << std::endl;
			}

			*/
			/*
				Minimizer* m = new Minimizer(*r);
				 minDist = BVMinimize(Tree, *m);
				 if (NULL != m && NULL != m->smallest && m->smallest->intersectedShape != NULL)
				 {
					 color = Vector3f(abs(m->smallest->normal(0)), abs(m->smallest->normal(1)), abs(m->smallest->normal(2)));
					// color = (((m->smallest.normal).dot((lights[0]->center - m->smallest.intersectionPoint))) * m->smallest.intersectedShape->mat->Kd) / PI;
					// color = m->smallest.normal;
				}
				 */
				//Cast ray here?
				//Write color to image
			//color = Vector3f(abs(smallest.normal(0)), abs(smallest.normal(1)), abs(smallest.normal(2)));
			//	color = Vector3f(1.0, 0, 0);
			//image[y*width + x] = color;
/*
			if (smallest.t == INF)
			{
				color = Vector3f(0, 1, 0);
			}
			else
			{
				color = ((smallest.t - 5) / 4) * Vector3f(1, 1, 1);
			}

			image[y*width + x] = color;
			image[y*width + x] = Vector3f(x / 400.0, y / 300.0, 0);
			image[y*width + x] = Vector3f(-1*r->direction(2), -1*r->direction(2), -1*r->direction(2));

			
			//color = ((smallest.t - 5) / 4) * Vector3f(1, 1, 1);

			if (intersectionFound)
			{
				color = Vector3f(1, 0, 0);
			}
			else
			{
				color = Vector3f(0, 1, 0);
			}
		

			
			
			
			if (smallest.t == INF)
			{
				color = Vector3f(0, 0, 0);
			}
			else
			{
				color = ((smallest.t - 5) / 4) * Vector3f(1, 1, 1);
			}
			*/
			
			//color = Vector3f(abs(smallest.normal(0)), abs(smallest.normal(1)), abs(smallest.normal(2)));
			if (smallest.intersectedShape != NULL)
			{

				color = smallest.intersectedShape->mat->Kd;
			}
			else
			{
				color = Vector3f(0, 0, 0);
			}
				image[yCopy*width + xCopy] = color;
		}



	}






	

/*
#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
    for (int y=0;  y<height;  y++) {

        fprintf(stderr, "Rendering %4d\r", y);
        for (int x=0;  x<width;  x++) {
            Color color;
            if ((x-width/2)*(x-width/2)+(y-height/2)*(y-height/2) < 100*100)
                color = Color(myrandom(RNGen), myrandom(RNGen), myrandom(RNGen));
            else if (abs(x-width/2)<4 || abs(y-height/2)<4)
                color = Color(0.0, 0.0, 0.0);
            else 
                color = Color(1.0, 1.0, 1.0);
            image[y*width + x] = color;
        }
    }
    fprintf(stderr, "\n");
	*/
}
