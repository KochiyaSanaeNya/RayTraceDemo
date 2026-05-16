#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <fstream>
#include <algorithm>
#include <memory> 

const double INF = std::numeric_limits<double>::infinity();

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(double d) const { return Vec3(x * d, y * d, z * d); }
    Vec3 operator*(const Vec3& v) const { return Vec3(x * v.x, y * v.y, z * v.z); }
    Vec3 normalize() const {
        double mg = std::sqrt(x*x + y*y + z*z);
        return Vec3(x/mg, y/mg, z/mg);
    }
};

inline double dot(const Vec3& a, const Vec3& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return Vec3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

struct Ray {
    Vec3 origin, direction;
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d.normalize()) {}
};

struct Light {
    Vec3 position;  
    Vec3 intensity; 
};

struct Material {
    Vec3 color;        
    double diffuse;    
    double specular;   
    double reflection; 
    double shininess;  
};

struct Intersection {
    bool hit;          
    double t;          
    Vec3 hitPoint;     
    Vec3 normal;       
    Material material; 
    Intersection() : hit(false), t(INF) {}
};

class Shape {
public:
    Material material;
    Shape(const Material& m) : material(m) {}
    virtual ~Shape() = default; 
    virtual Intersection intersect(const Ray& ray) const = 0;
};

class Sphere : public Shape {
    Vec3 center;
    double radius;
public:
    Sphere(const Vec3& c, double r, const Material& m) : Shape(m), center(c), radius(r) {}
    Intersection intersect(const Ray& ray) const override {
        Intersection ix;
        Vec3 oc = ray.origin - center;
        double b = 2.0 * dot(oc, ray.direction);
        double c = dot(oc, oc) - radius*radius;
        double discriminant = b*b - 4*c;
        if (discriminant > 0) {
            double t1 = (-b - std::sqrt(discriminant)) / 2.0;
            double t2 = (-b + std::sqrt(discriminant)) / 2.0;
            double t = (t1 > 1e-4) ? t1 : ((t2 > 1e-4) ? t2 : -1);
            if (t > 1e-4) {
                ix.hit = true;
                ix.t = t;
                ix.hitPoint = ray.origin + ray.direction * t;
                ix.normal = (ix.hitPoint - center).normalize();
                ix.material = material;
            }
        }
        return ix;
    }
};

class Triangle : public Shape {
    Vec3 v0, v1, v2;
public:
    Triangle(const Vec3& v0, const Vec3& v1, const Vec3& v2, const Material& m)
        : Shape(m), v0(v0), v1(v1), v2(v2) {}
    Intersection intersect(const Ray& ray) const override {
        Intersection ix;
        Vec3 edge1 = v1 - v0;
        Vec3 edge2 = v2 - v0;
        Vec3 h = cross(ray.direction, edge2);
        double a = dot(edge1, h);
        if (a > -1e-5 && a < 1e-5) return ix;
        double f = 1.0 / a;
        Vec3 s = ray.origin - v0;
        double u = f * dot(s, h);
        if (u < 0.0 || u > 1.0) return ix;
        Vec3 q = cross(s, edge1);
        double v = f * dot(ray.direction, q);
        if (v < 0.0 || u + v > 1.0) return ix;
        double t = f * dot(edge2, q);
        if (t > 1e-4) {
            ix.hit = true;
            ix.t = t;
            ix.hitPoint = ray.origin + ray.direction * t;
            ix.normal = cross(edge1, edge2).normalize();
            ix.material = material;
        }
        return ix;
    }
};

class Scene {
public:
    std::vector<std::unique_ptr<Shape>> shapes;
    std::vector<Light> lights;

    Intersection intersect(const Ray& ray) const {
        Intersection closest;
        for (const auto& s : shapes) {
            Intersection ix = s->intersect(ray);
            if (ix.hit && ix.t < closest.t) closest = ix;
        }
        return closest;
    }
};

Vec3 reflect(const Vec3& I, const Vec3& N) {
    return I - N * 2.0 * dot(I, N);
}

Vec3 castRay(const Ray& ray, const Scene& scene, int depth = 0) {
    if (depth > 3) return Vec3(0, 0, 0);

    Intersection ix = scene.intersect(ray);
    if (!ix.hit) return Vec3(0.1, 0.1, 0.15); 

    Vec3 ambientLight(0.25, 0.25, 0.25);
    Vec3 finalColor = ix.material.color * ambientLight;

    for (const auto& light : scene.lights) {
        Vec3 lightDir = (light.position - ix.hitPoint).normalize();

        Ray shadowRay(ix.hitPoint + ix.normal * 1e-4, lightDir);
        Intersection shadowIx = scene.intersect(shadowRay);

        double lightDist = std::sqrt(dot(light.position - ix.hitPoint, light.position - ix.hitPoint));

        bool inShadow = shadowIx.hit && shadowIx.t < lightDist;

        if (!inShadow) {
            double diff = std::max(0.0, dot(ix.normal, lightDir));
            Vec3 diffuse = light.intensity * ix.material.color * ix.material.diffuse * diff;

            Vec3 viewDir = (ray.origin - ix.hitPoint).normalize();
            Vec3 reflectDir = reflect(Vec3(0,0,0) - lightDir, ix.normal);
            double spec = std::pow(std::max(0.0, dot(viewDir, reflectDir)), ix.material.shininess);
            Vec3 specular = light.intensity * ix.material.specular * spec;

            finalColor = finalColor + diffuse + specular;
        }
    }

    if (ix.material.reflection > 0) {
        Vec3 reflectDir = reflect(ray.direction, ix.normal);
        Ray reflectRay(ix.hitPoint + ix.normal * 1e-4, reflectDir);
        Vec3 reflectColor = castRay(reflectRay, scene, depth + 1);
        finalColor = finalColor + reflectColor * ix.material.reflection;
    }

    return finalColor;
}

class Camera {
public:
    Vec3 origin;
    Vec3 forward, right, up;
    double fov;

    Camera(Vec3 lookfrom, Vec3 lookat, Vec3 vup, double f) : origin(lookfrom), fov(f) {
        forward = (lookat - lookfrom).normalize();

        if (std::abs(forward.y) > 0.999) {
            Vec3 tempUp = Vec3(0, 0, -1);
            right = cross(forward, tempUp).normalize();
        } else {
            right = cross(forward, vup).normalize();
        }

        up = cross(right, forward).normalize();
    }

    Ray getRay(double px, double py) const {
        Vec3 dir = (right * px + up * py + forward).normalize();
        return Ray(origin, dir);
    }
};

int main() {
    int width = 1920, height = 1080;
    Scene scene;

    Material redMat = {Vec3(1, 0.2, 0.2), 0.7, 0.3, 0.4, 30};
    Material greenMat = {Vec3(0.2, 1, 0.2), 0.7, 0.3, 0.2, 10};
    Material blueMat = {Vec3(0.2, 0.2, 1), 0.7, 0.3, 0.1, 20};
    Material floorMat = {Vec3(0.3, 0.3, 0.3), 0.8, 0, 0, 0};
    Material mirrorMat = {Vec3(1, 1, 1), 0.0, 0.0, 1.0, 100};
    Material wallMat = {Vec3(0.6, 0.6, 0.6), 0.8, 0.2, 0.2, 50};
    scene.shapes.push_back(std::make_unique<Sphere>(Vec3(2.5, 2, -3), 0.8, redMat));
    scene.shapes.push_back(std::make_unique<Sphere>(Vec3(0, 2, -3), 0.9, blueMat));
    scene.shapes.push_back(std::make_unique<Sphere>(Vec3(-2.5, 2, -3), 0.8, greenMat));
    
    scene.shapes.push_back(std::make_unique<Triangle>(Vec3(-10, -1, -10), Vec3(-10, -1, 1), Vec3(10, -1, 1), floorMat));
    scene.shapes.push_back(std::make_unique<Triangle>(Vec3(-10, -1, -10), Vec3(10, -1, 1), Vec3(10, -1, -10), floorMat));

    scene.shapes.push_back(std::make_unique<Triangle>(Vec3(-10, -1, -10), Vec3(10, -1, -10), Vec3(-10, 10, -10), mirrorMat));
    scene.shapes.push_back(std::make_unique<Triangle>(Vec3(10, 10, -10), Vec3(-10, 10, -10), Vec3(10, -1, -10), mirrorMat));

    scene.shapes.push_back(std::make_unique<Triangle>(Vec3(-10, -1, 1), Vec3(-10, -1, -10), Vec3(-10, 10, 1), wallMat));
    scene.shapes.push_back(std::make_unique<Triangle>(Vec3(-10, 10, -10), Vec3(-10, 10, 1), Vec3(-10, -1, -10), wallMat));

    scene.shapes.push_back(std::make_unique<Triangle>(Vec3(10, -1, -10), Vec3(10, -1, 1), Vec3(10, 10, -10), wallMat));
    scene.shapes.push_back(std::make_unique<Triangle>(Vec3(10, 10, 1), Vec3(10, 10, -10), Vec3(10, -1, 1), wallMat));
    
    scene.shapes.push_back(std::make_unique<Triangle>(Vec3(-10, 10, -10), Vec3(10, 10, -10), Vec3(-10, 10, -2), wallMat));
    scene.shapes.push_back(std::make_unique<Triangle>(Vec3(10, 10, -2), Vec3(-10, 10, -2), Vec3(10, 10, -10), wallMat));
    
    scene.lights.push_back({Vec3(-5, 5, 2), Vec3(0.5, 0.5, 0.5)});
    scene.lights.push_back({Vec3(5, 5, 2), Vec3(0.4, 0.4, 0.4)});

    double fov = 3.14159 / 1.6 ;

    Vec3 lookFrom(0, 5, -3);      
    Vec3 lookAt(0, 2, -5);       
    Vec3 vUp(0, 1, 0);           
    Camera camera(lookFrom, lookAt, vUp,fov);

    std::vector<Vec3> image(width * height);

    int samplesPerPixel = 4; 

#pragma omp parallel for schedule(dynamic)
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            Vec3 pixelColor(0, 0, 0);

            for (int sy = 0; sy < 2; ++sy) {
                for (int sx = 0; sx < 2; ++sx) {
                    double offsetX = (sx + 0.5) / 2.0;
                    double offsetY = (sy + 0.5) / 2.0;

                    double aspectRatio = (double)width / height;
                    double scale = std::tan(fov / 2.0);

                    double px = (2.0 * ((x + offsetX) / width) - 1.0) * scale * aspectRatio;
                    double py = (1.0 - 2.0 * ((y + offsetY) / height)) * scale;

                    Ray ray = camera.getRay(px, py);
                    pixelColor = pixelColor + castRay(ray, scene);
                }
            }
            image[y * width + x] = pixelColor * (1.0 / samplesPerPixel);
        }
    }

    std::ofstream ofs("output.ppm");
    ofs << "P3\n" << width << " " << height << "\n255\n";
    for (int i = 0; i < width * height; ++i) {
        double r_linear = std::min(1.0, std::max(0.0, image[i].x));
        double g_linear = std::min(1.0, std::max(0.0, image[i].y));
        double b_linear = std::min(1.0, std::max(0.0, image[i].z));

        int r = int(std::pow(r_linear, 1.0 / 2.2) * 255.0);
        int g = int(std::pow(g_linear, 1.0 / 2.2) * 255.0);
        int b = int(std::pow(b_linear, 1.0 / 2.2) * 255.0);

        ofs << r << " " << g << " " << b << "\n";
    }
    ofs.close();

    std::cout << "Render successful. Output saved to 'output.ppm'" << std::endl;
    return 0;
}