//#include <iostream>
//#include <vector>
//#include <cmath>
//#include <random>
//#include <algorithm>
//#include <fstream>
//#include <chrono>
//#include <string>
//#include "adaptive_sampler.h"
//
//#define M_PI 3.14159265358979323846
//
//// Vector3 class for 3D math
//struct Vec3 {
//    double x, y, z;
//
//    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
//
//    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
//    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
//    Vec3 operator*(double t) const { return Vec3(x * t, y * t, z * t); }
//    Vec3 operator/(double t) const { return Vec3(x / t, y / t, z / t); }
//
//    double dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
//    Vec3 cross(const Vec3& v) const {
//        return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
//    }
//
//    double length() const { return std::sqrt(x * x + y * y + z * z); }
//    Vec3 normalized() const { double l = length(); return Vec3(x / l, y / l, z / l); }
//
//    // Component-wise multiplication
//    Vec3 mult(const Vec3& v) const { return Vec3(x * v.x, y * v.y, z * v.z); }
//};
//
//// Ray structure
//struct Ray {
//    Vec3 origin, direction;
//    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d.normalized()) {}
//};
//
//// Material structure
//struct Material {
//    Vec3 color;
//    Vec3 emission;
//    double reflectivity;
//
//    Material(Vec3 c = Vec3(0.8, 0.8, 0.8), Vec3 e = Vec3(0, 0, 0), double r = 0.0)
//        : color(c), emission(e), reflectivity(r) {
//    }
//};
//
//// Sphere primitive
//struct Sphere {
//    Vec3 center;
//    double radius;
//    Material material;
//
//    Sphere(Vec3 c, double r, Material m) : center(c), radius(r), material(m) {}
//
//    bool intersect(const Ray& ray, double& t) const {
//        Vec3 oc = ray.origin - center;
//        double a = ray.direction.dot(ray.direction);
//        double b = 2.0 * oc.dot(ray.direction);
//        double c = oc.dot(oc) - radius * radius;
//        double discriminant = b * b - 4 * a * c;
//
//        if (discriminant < 0) return false;
//
//        double t1 = (-b - std::sqrt(discriminant)) / (2.0 * a);
//        double t2 = (-b + std::sqrt(discriminant)) / (2.0 * a);
//
//        t = (t1 > 0.001) ? t1 : t2;
//        return t > 0.001;
//    }
//
//    Vec3 getNormal(const Vec3& point) const {
//        return (point - center).normalized();
//    }
//};
//
//// Scene containing all geometry
//struct Scene {
//    std::vector<Sphere> spheres;
//
//    void createCornellBox() {
//        // Cornell Box dimensions: 1 unit cube centered at origin
//        double size = 1.0;
//
//        // Floor (white)
//        spheres.push_back(Sphere(Vec3(0, -1000.5, 0), 1000,
//            Material(Vec3(0.8, 0.8, 0.8), Vec3(0, 0, 0), 0.0)));
//
//        // Ceiling (white)
//        spheres.push_back(Sphere(Vec3(0, 1000.5, 0), 1000,
//            Material(Vec3(0.8, 0.8, 0.8), Vec3(0, 0, 0), 0.0)));
//
//        // Back wall (white)
//        spheres.push_back(Sphere(Vec3(0, 0, -1001), 1000,
//            Material(Vec3(0.8, 0.8, 0.8), Vec3(0, 0, 0), 0.0)));
//
//        // Left wall (red)
//        spheres.push_back(Sphere(Vec3(-1000.5, 0, 0), 1000,
//            Material(Vec3(0.8, 0.1, 0.1), Vec3(0, 0, 0), 0.0)));
//
//        // Right wall (green)
//        spheres.push_back(Sphere(Vec3(1000.5, 0, 0), 1000,
//            Material(Vec3(0.1, 0.8, 0.1), Vec3(0, 0, 0), 0.0)));
//
//        // Light (area light at top)
//        spheres.push_back(Sphere(Vec3(0, 0.45, 0), 0.15,
//            Material(Vec3(1, 1, 1), Vec3(12, 12, 12), 0.0)));
//
//        // Left sphere (diffuse)
//        spheres.push_back(Sphere(Vec3(-0.25, -0.25, -0.3), 0.25,
//            Material(Vec3(0.9, 0.9, 0.2), Vec3(0, 0, 0), 0.0)));
//
//        // Right sphere (more reflective)
//        spheres.push_back(Sphere(Vec3(0.25, -0.25, 0.1), 0.25,
//            Material(Vec3(0.2, 0.3, 0.9), Vec3(0, 0, 0), 0.3)));
//    }
//
//    bool intersect(const Ray& ray, double& t, int& hitIndex) const {
//        bool hit = false;
//        t = 1e20;
//        hitIndex = -1;
//
//        for (size_t i = 0; i < spheres.size(); i++) {
//            double tTemp;
//            if (spheres[i].intersect(ray, tTemp) && tTemp < t) {
//                t = tTemp;
//                hitIndex = i;
//                hit = true;
//            }
//        }
//
//        return hit;
//    }
//};
//
//// Camera structure
//struct Camera {
//    Vec3 position;
//    Vec3 lookAt;
//    Vec3 up;
//    double fov;
//    int width, height;
//
//    Camera(Vec3 pos, Vec3 at, Vec3 u, double f, int w, int h)
//        : position(pos), lookAt(at), up(u), fov(f), width(w), height(h) {
//    }
//
//    Ray getRay(int x, int y, std::mt19937& rng) const {
//        std::uniform_real_distribution<double> dist(0.0, 1.0);
//
//        // Add jittering for antialiasing
//        double u = (x + dist(rng)) / width;
//        double v = (y + dist(rng)) / height;
//
//        // Convert to NDC [-1, 1]
//        double ndcX = 2.0 * u - 1.0;
//        double ndcY = 1.0 - 2.0 * v;
//
//        // Calculate aspect ratio
//        double aspectRatio = (double)width / height;
//        double tanHalfFov = std::tan(fov * 0.5 * M_PI / 180.0);
//
//        // Calculate camera basis vectors
//        Vec3 forward = (lookAt - position).normalized();
//        Vec3 right = forward.cross(up).normalized();
//        Vec3 newUp = right.cross(forward).normalized();
//
//        // Calculate ray direction
//        Vec3 horizontal = right * (ndcX * aspectRatio * tanHalfFov);
//        Vec3 vertical = newUp * (ndcY * tanHalfFov);
//        Vec3 direction = (forward + horizontal + vertical).normalized();
//
//        return Ray(position, direction);
//    }
//};
//
//// Path tracer with adaptive sampling
//class PathTracer {
//private:
//    Scene scene;
//    Camera camera;
//    int maxBounces;
//
//    // Random sampling helper
//    Vec3 randomInHemisphere(const Vec3& normal, std::mt19937& rng) {
//        std::uniform_real_distribution<double> dist(0.0, 1.0);
//
//        double r1 = dist(rng);
//        double r2 = dist(rng);
//
//        double phi = 2.0 * M_PI * r1;
//        double cosTheta = std::sqrt(1.0 - r2);
//        double sinTheta = std::sqrt(r2);
//
//        Vec3 direction(std::cos(phi) * sinTheta, cosTheta, std::sin(phi) * sinTheta);
//
//        // Transform to world space
//        Vec3 w = normal;
//        Vec3 u = ((std::abs(w.x) > 0.1 ? Vec3(0, 1, 0) : Vec3(1, 0, 0)).cross(w)).normalized();
//        Vec3 v = w.cross(u);
//
//        return (u * direction.x + w * direction.y + v * direction.z).normalized();
//    }
//
//public:
//    PathTracer(const Scene& s, const Camera& c, int bounces = 5)
//        : scene(s), camera(c), maxBounces(bounces) {
//    }
//
//    // Trace single ray and return radiance (Equation from I.1.3)
//    Vec3 trace(const Ray& ray, int depth, std::mt19937& rng) {
//        if (depth >= maxBounces) return Vec3(0, 0, 0);
//
//        double t;
//        int hitIndex;
//
//        if (!scene.intersect(ray, t, hitIndex)) {
//            // Sky color
//            return Vec3(0.5, 0.7, 1.0) * 0.3;
//        }
//
//        const Sphere& sphere = scene.spheres[hitIndex];
//        Vec3 hitPoint = ray.origin + ray.direction * t;
//        Vec3 normal = sphere.getNormal(hitPoint);
//
//        // Add emission (lights)
//        Vec3 color = sphere.material.emission;
//
//        // Russian roulette for path termination
//        if (depth > 3) {
//            double continueProbability = 0.9;
//            std::uniform_real_distribution<double> dist(0.0, 1.0);
//            if (dist(rng) > continueProbability) return color;
//            color = color / continueProbability;
//        }
//
//        // Sample random direction in hemisphere
//        Vec3 newDirection = randomInHemisphere(normal, rng);
//        Ray newRay(hitPoint, newDirection);
//
//        // Recursive ray tracing (Monte Carlo integration)
//        Vec3 incoming = trace(newRay, depth + 1, rng);
//
//        // BRDF (Lambertian for now)
//        double cosTheta = std::max(0.0, normal.dot(newDirection));
//        color = color + sphere.material.color.mult(incoming) * cosTheta * 2.0;
//
//        return color;
//    }
//
//    // Render single sample for pixel (x, y) - Returns Y_i(x) from equation I.1.3
//    Vec3 renderSample(int x, int y, std::mt19937& rng) {
//        Ray ray = camera.getRay(x, y, rng);
//        return trace(ray, 0, rng);
//    }
//};
//
//// Image buffer
//struct Image {
//    int width, height;
//    std::vector<Vec3> pixels;
//
//    Image(int w, int h) : width(w), height(h), pixels(w* h, Vec3(0, 0, 0)) {}
//
//    void setPixel(int x, int y, const Vec3& color) {
//        if (x >= 0 && x < width && y >= 0 && y < height) {
//            pixels[y * width + x] = color;
//        }
//    }
//
//    Vec3 getPixel(int x, int y) const {
//        if (x >= 0 && x < width && y >= 0 && y < height) {
//            return pixels[y * width + x];
//        }
//        return Vec3(0, 0, 0);
//    }
//
//    // Calculate luminance L(x) for adaptive sampling
//    double getLuminance(int x, int y) const {
//        Vec3 color = getPixel(x, y);
//        return 0.2126 * color.x + 0.7152 * color.y + 0.0722 * color.z;
//    }
//
//    void savePPM(const std::string& filename) const {
//        std::ofstream file(filename);
//        file << "P3\n" << width << " " << height << "\n255\n";
//
//        for (int y = 0; y < height; y++) {
//            for (int x = 0; x < width; x++) {
//                Vec3 color = getPixel(x, y);
//
//                // Gamma correction
//                color.x = std::pow(std::clamp(color.x, 0.0, 1.0), 1.0 / 2.2);
//                color.y = std::pow(std::clamp(color.y, 0.0, 1.0), 1.0 / 2.2);
//                color.z = std::pow(std::clamp(color.z, 0.0, 1.0), 1.0 / 2.2);
//
//                int r = static_cast<int>(255.99 * color.x);
//                int g = static_cast<int>(255.99 * color.y);
//                int b = static_cast<int>(255.99 * color.z);
//
//                file << r << " " << g << " " << b << " ";
//            }
//            file << "\n";
//        }
//
//        file.close();
//        std::cout << "Saved image: " << filename << std::endl;
//    }
//};
//
////int main() {
////    // Setup scene
////    Scene scene;
////    scene.createCornellBox();
////
////    // Setup camera
////    int width = 512, height = 512;
////    Camera camera(Vec3(0, 0, 2.5), Vec3(0, 0, 0), Vec3(0, 1, 0), 40.0, width, height);
////
////    // Create path tracer
////    PathTracer tracer(scene, camera, 5);
////
////    // Setup rendering
////    int samplesPerPixel = 64;  // n(x) from equation I.1.3
////    Image image(width, height);
////
////    std::cout << "Rendering Cornell Box (" << width << "x" << height
////        << ") with " << samplesPerPixel << " samples per pixel..." << std::endl;
////
////    auto startTime = std::chrono::high_resolution_clock::now();
////
////    // Random number generator with seed
////    unsigned int seed = 42;
////    std::mt19937 rng(seed);
////
////    // Standard path tracing: uniform sampling
////    // Implements equation: I_hat(x) = (1/n(x)) * sum(Y_i(x))
////    for (int y = 0; y < height; y++) {
////        for (int x = 0; x < width; x++) {
////            Vec3 color(0, 0, 0);
////
////            // Monte Carlo integration with n(x) samples
////            for (int s = 0; s < samplesPerPixel; s++) {
////                color = color + tracer.renderSample(x, y, rng);
////            }
////
////            // Average: I_hat(x) = (1/n(x)) * sum
////            color = color / samplesPerPixel;
////            image.setPixel(x, y, color);
////        }
////
////        // Progress indicator
////        if ((y + 1) % 50 == 0) {
////            std::cout << "Progress: " << (y + 1) << "/" << height << " rows" << std::endl;
////        }
////    }
////
////    auto endTime = std::chrono::high_resolution_clock::now();
////    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
////
////    std::cout << "Rendering completed in " << duration.count() / 1000.0 << " seconds" << std::endl;
////
////    // Save image
////    image.savePPM("cornell_box_baseline.ppm");
////
////    std::cout << "\nNext steps:" << std::endl;
////    std::cout << "1. This is your BASELINE path tracer (uniform sampling)" << std::endl;
////    std::cout << "2. Next, implement adaptive sampling based on variance V(x)" << std::endl;
////    std::cout << "3. Add temporal reuse for dynamic scenes" << std::endl;
////    std::cout << "4. Implement SSIM for quality metrics" << std::endl;
////
////    return 0;
////}