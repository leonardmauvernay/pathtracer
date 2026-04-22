#define _CRT_SECURE_NO_WARNINGS 1
#include <cmath>
#include <random>
#include <vector>
#include <map>
#include <string>
#include <fstream>

 
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#ifndef M_PI
#define M_PI 3.14159265358979323856
#endif

static std::default_random_engine engine[32];
static std::uniform_real_distribution<double> uniform(0, 1);

double sqr(double x) { return x * x; };

class Vector {
public:
  explicit Vector(double x = 0, double y = 0, double z = 0) {
    data[0] = x;
    data[1] = y;
    data[2] = z;
  }
  double norm2() const {
    return data[0] * data[0] + data[1] * data[1] + data[2] * data[2];
  }
  double norm() const { return sqrt(norm2()); }
  void normalize() {
    double n = norm();
    data[0] /= n;
    data[1] /= n;
    data[2] /= n;
  }
  double operator[](int i) const { return data[i]; };
  double &operator[](int i) { return data[i]; };
  double data[3];
};

Vector operator+(const Vector &a, const Vector &b) {
  return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}
Vector operator-(const Vector &a, const Vector &b) {
  return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
Vector operator*(const double a, const Vector &b) {
  return Vector(a * b[0], a * b[1], a * b[2]);
}
Vector operator*(const Vector &a, const double b) {
  return Vector(a[0] * b, a[1] * b, a[2] * b);
}
Vector operator/(const Vector &a, const double b) {
  return Vector(a[0] / b, a[1] / b, a[2] / b);
}
double dot(const Vector &a, const Vector &b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vector cross(const Vector &a, const Vector &b) {
  return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]);
}

class Ray {
public:
  Ray(const Vector &origin, const Vector &unit_direction)
      : O(origin), u(unit_direction) {};
  Vector O, u;
};

class Object {
public:
  Object(const Vector &albedo, bool mirror = false, bool transparent = false)
      : albedo(albedo), mirror(mirror), transparent(transparent) {};

  virtual bool intersect(const Ray &ray, Vector &P, double &t,
                         Vector &N) const = 0;

  Vector albedo;
  bool mirror, transparent;
};

class Sphere : public Object {
public:
  Sphere(const Vector &center, double radius, const Vector &albedo,
         bool mirror = false, bool transparent = false)
      : ::Object(albedo, mirror, transparent), C(center), R(radius) {};

  // returns true iif there is an intersection between the ray and the sphere
  // if there is an intersection, also computes the point of intersection P,
  // t>=0 the distance between the ray origin and P (i.e., the parameter along
  // the ray) and the unit normal N
  bool intersect(const Ray &ray, Vector &P, double &t, Vector &N) const {
    // TODO (lab 1) : compute the intersection (just true/false at the begining
    // of lab 1, then P, t and N as well)
    Vector o_c = ray.O - C;
    double dot_u_oc = dot(ray.u, o_c);
    double norm2_oc = o_c.norm2();
    double delta = dot_u_oc * dot_u_oc - (norm2_oc - R * R);

    if (delta < 0) {
      return false;
    }

    for (int i = -1; i < 2;
         i += 2) { // Louis gave me the idea of the loop for cleaner code
      t = -dot_u_oc + i * sqrt(delta);
      if (t >= 0) {
        P = ray.O + t * ray.u;
        N = P - C;
        N.normalize();
        return true;
      }
    }
    return false;
  }

  double R;
  Vector C;
};

// Class only used in labs 3 and 4 
class TriangleIndices {
public:
	TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1) {
		vtx[0] = vtxi; vtx[1] = vtxj; vtx[2] = vtxk;
		uv[0] = uvi; uv[1] = uvj; uv[2] = uvk;
		n[0] = ni; n[1] = nj; n[2] = nk;
		this->group = group;
	};
	int vtx[3]; // indices within the vertex coordinates array
	int uv[3];  // indices within the uv coordinates array
	int n[3];   // indices within the normals array
	int group;  // face group
};

// Class only used in labs 3 and 4 
class TriangleMesh : public Object {
public:
	TriangleMesh(const Vector& albedo, bool mirror = false, bool transparent = false) : ::Object(albedo, mirror, transparent) {};

	// first scale and then translate the current object
	void scale_translate(double s, const Vector& t) {
		for (int i = 0; i < vertices.size(); i++) {
			vertices[i] = vertices[i] * s + t;
		}
	}

	// read an .obj file
	void readOBJ(const char* obj) {
		std::ifstream f(obj);
		if (!f) return;

		std::map<std::string, int> mtls;
		int curGroup = -1, maxGroup = -1;

		// OBJ indices are 1-based and can be negative (relative), this normalizes them
		auto resolveIdx = [](int i, int size) {
			return i < 0 ? size + i : i - 1;
		};

		auto setFaceVerts = [&](TriangleIndices& t, int i0, int i1, int i2) {
			t.vtx[0] = resolveIdx(i0, vertices.size());
			t.vtx[1] = resolveIdx(i1, vertices.size());
			t.vtx[2] = resolveIdx(i2, vertices.size());
		};
		auto setFaceUVs = [&](TriangleIndices& t, int j0, int j1, int j2) {
			t.uv[0] = resolveIdx(j0, uvs.size());
			t.uv[1] = resolveIdx(j1, uvs.size());
			t.uv[2] = resolveIdx(j2, uvs.size());
		};
		auto setFaceNormals = [&](TriangleIndices& t, int k0, int k1, int k2) {
			t.n[0] = resolveIdx(k0, normals.size());
			t.n[1] = resolveIdx(k1, normals.size());
			t.n[2] = resolveIdx(k2, normals.size());
		};

		std::string line;
		while (std::getline(f, line)) {
			// Trim trailing whitespace
			line.erase(line.find_last_not_of(" \r\t\n") + 1);
			if (line.empty()) continue;

			const char* s = line.c_str();

			if (line.rfind("usemtl ", 0) == 0) {
				std::string matname = line.substr(7);
				auto result = mtls.emplace(matname, maxGroup + 1);
				if (result.second) {
					curGroup = ++maxGroup;
				} else {
					curGroup = result.first->second;
				}
			} else if (line.rfind("vn ", 0) == 0) {
				Vector v;
				sscanf(s, "vn %lf %lf %lf", &v[0], &v[1], &v[2]);
				normals.push_back(v);
			} else if (line.rfind("vt ", 0) == 0) {
				Vector v;
				sscanf(s, "vt %lf %lf", &v[0], &v[1]);
				uvs.push_back(v);
			} else if (line.rfind("v ", 0) == 0) {
				Vector pos, col;
				if (sscanf(s, "v %lf %lf %lf %lf %lf %lf", &pos[0], &pos[1], &pos[2], &col[0], &col[1], &col[2]) == 6) {
					for (int i = 0; i < 3; i++) col[i] = std::min(1.0, std::max(0.0, col[i]));
					vertexcolors.push_back(col);
				} else {
					sscanf(s, "v %lf %lf %lf", &pos[0], &pos[1], &pos[2]);
				}
				vertices.push_back(pos);
			}
			else if (line[0] == 'f') {
				int i[4], j[4], k[4], offset, nn;
				const char* cur = s + 1;
				TriangleIndices t;
				t.group = curGroup;

				// Try each face format: v/vt/vn, v/vt, v//vn, v
				if ((nn = sscanf(cur, "%d/%d/%d %d/%d/%d %d/%d/%d%n", &i[0], &j[0], &k[0], &i[1], &j[1], &k[1], &i[2], &j[2], &k[2], &offset)) == 9) {
					setFaceVerts(t, i[0], i[1], i[2]); 
					setFaceUVs(t, j[0], j[1], j[2]); 
					setFaceNormals(t, k[0], k[1], k[2]);
				} else if ((nn = sscanf(cur, "%d/%d %d/%d %d/%d%n", &i[0], &j[0], &i[1], &j[1], &i[2], &j[2], &offset)) == 6) {
					setFaceVerts(t, i[0], i[1], i[2]); 
					setFaceUVs(t, j[0], j[1], j[2]);
				} else if ((nn = sscanf(cur, "%d//%d %d//%d %d//%d%n", &i[0], &k[0], &i[1], &k[1], &i[2], &k[2], &offset)) == 6) {
					setFaceVerts(t, i[0], i[1], i[2]); 
					setFaceNormals(t, k[0], k[1], k[2]);
				} else if ((nn = sscanf(cur, "%d %d %d%n", &i[0], &i[1], &i[2], &offset)) == 3) {
					setFaceVerts(t, i[0], i[1], i[2]);
				}
				else continue;

				indices.push_back(t);
				cur += offset;

				// Fan triangulation for polygon faces (4+ vertices)
				while (*cur && *cur != '\n') {
					TriangleIndices t2;
					t2.group = curGroup;
					if ((nn = sscanf(cur, " %d/%d/%d%n", &i[3], &j[3], &k[3], &offset)) == 3) {
						setFaceVerts(t2, i[0], i[2], i[3]); 
						setFaceUVs(t2, j[0], j[2], j[3]); 
						setFaceNormals(t2, k[0], k[2], k[3]);
					} else if ((nn = sscanf(cur, " %d/%d%n", &i[3], &j[3], &offset)) == 2) {
						setFaceVerts(t2, i[0], i[2], i[3]); 
						setFaceUVs(t2, j[0], j[2], j[3]);
					} else if ((nn = sscanf(cur, " %d//%d%n", &i[3], &k[3], &offset)) == 2) {
						setFaceVerts(t2, i[0], i[2], i[3]); 
						setFaceNormals(t2, k[0], k[2], k[3]);
					} else if ((nn = sscanf(cur, " %d%n", &i[3], &offset)) == 1) {
						setFaceVerts(t2, i[0], i[2], i[3]);
					} else { 
						cur++; 
						continue; 
					}

					indices.push_back(t2);
					cur += offset;
					i[2] = i[3]; j[2] = j[3]; k[2] = k[3];
				}
			}
		}
	}
	

	// TODO ray-mesh intersection (labs 3 and 4)
	bool intersect(const Ray& ray, Vector& P, double& t, Vector& N) const {
    // lab 3 : once done, speed it up by first checking against the mesh bounding box
    Vector b_min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()); 
    Vector b_max(std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), std::numeric_limits<double>::min()); 
    for (int i = 1; i < vertices.size(); i++) {
      if (vertices[i][0] < b_min[0]) b_min[0] = vertices[i][0];
      if (vertices[i][1] < b_min[1]) b_min[1] = vertices[i][1];
      if (vertices[i][2] < b_min[2]) b_min[2] = vertices[i][2];

      if (vertices[i][0] > b_max[0]) b_max[0] = vertices[i][0];
      if (vertices[i][1] > b_max[1]) b_max[1] = vertices[i][1];
      if (vertices[i][2] > b_max[2]) b_max[2] = vertices[i][2];
    }

    Vector t_min;
    Vector t_max;
    for (int i=0; i < 3; i++){
      t_min[i] = (b_min[i] - ray.O[i]) / ray.u[i];
      t_max[i] = (b_max[i] - ray.O[i]) / ray.u[i];
      if (t_min[i] > t_max[i]) {
        std::swap(t_min[i], t_max[i]);
      }
    }

    double bt_min = std::max(t_min[0], std::max(t_min[1], t_min[2]));
    double bt_max = std::min(t_max[0], std::min(t_max[1], t_max[2]));

    if (bt_max < bt_min){
      return false;
    } 
		
		// lab 3 : for each triangle, compute the ray-triangle intersection with Moller-Trumbore algorithm
    double min_t = std::numeric_limits<double>::max();
    bool b = false;
    for (int i=0; i < indices.size(); i++){
      Vector A = vertices[indices[i].vtx[0]];
      Vector B = vertices[indices[i].vtx[1]];
      Vector C = vertices[indices[i].vtx[2]];

      Vector e1 = B - A;
      Vector e2 = C - A;
      Vector N_temp = cross(e1, e2);
      Vector v = cross(A - ray.O, ray.u);
      double d = dot(ray.u, N);

      double beta = dot(e2, v) / d;
      double gamma = -dot(e1, v) / d;
      double alpha = 1. - beta - gamma;
      double t_temp = dot(A - ray.O, N_temp) / d;

      if (alpha >= 0 && beta >= 0 && gamma >= 0 && t_temp >= 0 && t_temp < min_t) {
				min_t = t_temp;
				t = t_temp;
				P = ray.O + t * ray.u;
        N = N_temp;
        N.normalize();
				b = true;
			}
		}
		// lab 4 : recursively apply the bounding-box test from a BVH datastructure
		return b;
	}


	std::vector<TriangleIndices> indices;
	std::vector<Vector> vertices;
	std::vector<Vector> normals;
	std::vector<Vector> uvs;
	std::vector<Vector> vertexcolors;
};

class Scene {
public:
  Scene() {};
  void addObject(const Object *obj) { objects.push_back(obj); }

  // returns true iif there is an intersection between the ray and any object in
  // the scene
  // if there is an intersection, also computes the point of the *nearest*
  // intersection P, t>=0 the distance between the ray origin and P (i.e., the
  // parameter along the ray) and the unit normal N. Also returns the index of
  // the object within the std::vector objects in object_id
  bool intersect(const Ray &ray, Vector &P, double &t, Vector &N,
                 int &object_id) const {

    // TODO (lab 1): iterate through the objects and check the intersections
    // with all of them, and keep the closest intersection, i.e., the one if
    // smallest positive value of t
    double min_t = INT_MAX;
    bool b = false;

    for (int i = 0; i < objects.size(); i++) {
      Vector P_t, N_t;
      double t_t;
      if (objects[i]->intersect(ray, P_t, t_t, N_t)) {
        if (t_t < min_t) {
          P = P_t;
          N = N_t;
          min_t = t_t;
          t = t_t;
          object_id = i;
          b = true;
        }
      }
    }
    return b;
  }

  // return the radiance (color) along ray
  Vector getColor(const Ray &ray, int recursion_depth) {

    if (recursion_depth >= max_light_bounce)
      return Vector(0, 0, 0);

    // TODO (lab 1) : if intersect with ray, use the returned information to
    // compute the color ; otherwise black in lab 1, the color only includes
    // direct lighting with shadows

    Vector P, N;
    double t;
    int object_id;
    if (intersect(ray, P, t, N, object_id)) {

      if (objects[object_id]->mirror) {
        Vector i = ray.u;
        Vector v = i - 2 * dot(i, N) * N;
        Ray reflection(P, v);
        return getColor(reflection, recursion_depth + 1);
        // return getColor in the reflected direction, with recursion_depth+1
        // (recursively)
      } // else

      if (objects[object_id]->transparent) { // optional

        // return getColor in the refraction direction, with recursion_depth+1
        // (recursively)
      } // else

      // test if there is a shadow by sending a new ray
      // if there is no shadow, compute the formula with dot products etc.
      // TODO (lab 2) : add indirect lighting component with a recursive call
      Vector v = light_position - P;
      double a = light_intensity / (4 * M_PI * (v).norm2());
      Vector m = objects[object_id]->albedo / M_PI;
      double sa = std::max(0., dot(N, v / v.norm()));

      double epsilon = 1e-5; // advised by Xianjin GONG
      Vector P2 = P + epsilon * N;
      Vector l_dir = light_position - P2;
      double l_dist2 = l_dir.norm2();
      l_dir.normalize();
      Vector shadow_P, shadow_N;
      double shadow_t;
      int shadow_obj_id;
      Ray shadow_ray(P2, l_dir);

      bool s =
          intersect(shadow_ray, shadow_P, shadow_t, shadow_N, shadow_obj_id);
      Vector v2 = shadow_P - P2;

      Vector dir;
      if (s && v2.norm2() <= l_dist2) {
        dir = Vector(0, 0, 0);
      } else {
        dir = a * m * sa;
      }

      // indirect light part
      double r1 = uniform(engine[0]);
      double r2 = uniform(engine[0]);
      double x = cos(2 * M_PI * r1) * sqrt(1 - r2);
      double y = sin(2 * M_PI * r1) * sqrt(1 - r2);
      double z = sqrt(r2);

      double N_x = N[0];
      double N_y = N[1];
      double N_z = N[2];

      Vector T1;
      if (std::abs(N_x)<= std::abs(N_y) && std::abs(N_x) <= std::abs(N_z)) {
        T1 = Vector(0, -N_z, N_y);
      } else if (std::abs(N_y)<= std::abs(N_x) && std::abs(N_y) <= std::abs(N_z)) {
        T1 = Vector(-N_z, 0, N_x);
      } else {
        T1 = Vector(-N_y, N_x, 0);
      }
      T1.normalize();

      Vector T2 = cross(N, T1);
      Vector w = x * T1 + y * T2 + z * N; // random direction sampling

      Ray indir_ray(P2, w);
      Vector radiance = getColor(indir_ray, recursion_depth + 1);
      Vector indir(objects[object_id]->albedo[0] * radiance[0], objects[object_id]->albedo[1] * radiance[1], objects[object_id]->albedo[2] * radiance[2]);

      return dir + indir;
    }

    return Vector(0, 0, 0);
  }

  std::vector<const Object *> objects;

  Vector camera_center, light_position;
  double fov, gamma, light_intensity;
  int max_light_bounce;
};

int main() {
  int W = 512;
  int H = 512;
  int N = 30;

  for (int i = 0; i < 32; i++) {
    engine[i].seed(i);
  }

  Sphere center_sphere(Vector(0, 0, 0), 10., Vector(0.8, 0.8, 0.8));
  Sphere wall_left(Vector(-1000, 0, 0), 940, Vector(0.5, 0.8, 0.1));
  Sphere wall_right(Vector(1000, 0, 0), 940, Vector(0.9, 0.2, 0.3));
  Sphere wall_front(Vector(0, 0, -1000), 940, Vector(0.1, 0.6, 0.7));
  Sphere wall_behind(Vector(0, 0, 1000), 940, Vector(0.8, 0.2, 0.9));
  Sphere ceiling(Vector(0, 1000, 0), 940, Vector(0.3, 0.5, 0.3));
  Sphere floor(Vector(0, -1000, 0), 990, Vector(0.6, 0.5, 0.7));

  TriangleMesh cat(Vector(0.8, 0.8, 0.8));
  cat.readOBJ("./cat.obj");
  cat.scale_translate(0.6, Vector(0,-10,0));

  Scene scene;
  scene.camera_center = Vector(0, 0, 55);
  scene.light_position = Vector(-10, 20, 40);
  scene.light_intensity = 3E7;
  scene.fov = 60 * M_PI / 180.;
  scene.gamma = 2.2; // TODO (lab 1) : play with gamma ; typically, gamma = 2.2
  scene.max_light_bounce = 5;

  
  // scene.addObject(&center_sphere);

  scene.addObject(&wall_left);
  scene.addObject(&wall_right);
  scene.addObject(&wall_front);
  scene.addObject(&wall_behind);
  scene.addObject(&ceiling);
  scene.addObject(&floor); 
  
  
  scene.addObject(&cat);

  std::vector<unsigned char> image(W * H * 3, 0);

#pragma omp parallel for schedule(dynamic, 1)
  for (int i = 0; i < H; i++) {
    for (int j = 0; j < W; j++) {
      // TODO (lab 1) : correct ray_direction so that it goes through each pixel
      // (j, i)
      // TODO (lab 2) : add Monte Carlo / averaging of random ray contributions
      // here
      // TODO (lab 2) : add antialiasing by altering the ray_direction here
      // TODO (lab 2) : add depth of field effect by altering the ray origin
      // (and direction) here
      Vector sum_color(0,0,0);
      for (int k = 0; k < N; k++) {
        double r1 = uniform(engine[0]);
        double r2 = uniform(engine[0]);
        double sigma = 0.5;

        Vector ray_direction(j - W / 2 + 0.5 + sigma * sqrt(-2 * log(r1)) * cos(2 * M_PI * r2), H / 2 - i - 0.5 + sigma * sqrt(-2 * log(r1)) * sin(2 * M_PI * r2),
                             -W / (2 * tan(scene.fov / 2)));
        ray_direction.normalize();

        Ray ray(scene.camera_center, ray_direction);
        sum_color = sum_color + scene.getColor(ray, 0);
      }

      Vector color = sum_color / N;

      image[(i * W + j) * 3 + 0] = std::min(
          255.,
          std::max(0., 255. * std::pow(color[0] / 255., 1. / scene.gamma)));
      image[(i * W + j) * 3 + 1] = std::min(
          255.,
          std::max(0., 255. * std::pow(color[1] / 255., 1. / scene.gamma)));
      image[(i * W + j) * 3 + 2] = std::min(
          255.,
          std::max(0., 255. * std::pow(color[2] / 255., 1. / scene.gamma)));
    }
  }
  stbi_write_png("image.png", W, H, 3, &image[0], 0);

  return 0;
}