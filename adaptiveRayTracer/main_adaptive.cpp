
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <numeric>
#include "adaptive_sampler.h"

#define M_PI 3.14159265358979323846


struct Ray {
    Vec3 origin, direction;
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d.normalized()) {}
};

struct Material {
    Vec3 color;
    Vec3 emission;
    double reflectivity;

    Material(Vec3 c = Vec3(0.8, 0.8, 0.8), Vec3 e = Vec3(0, 0, 0), double r = 0.0)
        : color(c), emission(e), reflectivity(r) {
    }
};

struct Sphere {
    Vec3 center;
    double radius;
    Material material;

    Sphere(Vec3 c, double r, Material m) : center(c), radius(r), material(m) {}

    bool intersect(const Ray& ray, double& t) const {
        Vec3 oc = ray.origin - center;
        double a = ray.direction.dot(ray.direction);
        double b = 2.0 * oc.dot(ray.direction);
        double c = oc.dot(oc) - radius * radius;
        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0) return false;

        double t1 = (-b - std::sqrt(discriminant)) / (2.0 * a);
        double t2 = (-b + std::sqrt(discriminant)) / (2.0 * a);

        t = (t1 > 0.001) ? t1 : t2;
        return t > 0.001;
    }

    Vec3 getNormal(const Vec3& point) const {
        return (point - center).normalized();
    }
};

struct Scene {
    std::vector<Sphere> spheres;

    void createCornellBox() {
        double size = 1.0;

        spheres.push_back(Sphere(Vec3(0, -1000.5, 0), 1000,
            Material(Vec3(0.8, 0.8, 0.8), Vec3(0, 0, 0), 0.0)));

        spheres.push_back(Sphere(Vec3(0, 1000.5, 0), 1000,
            Material(Vec3(0.8, 0.8, 0.8), Vec3(0, 0, 0), 0.0)));

        spheres.push_back(Sphere(Vec3(0, 0, -1001), 1000,
            Material(Vec3(0.8, 0.8, 0.8), Vec3(0, 0, 0), 0.0)));

        spheres.push_back(Sphere(Vec3(-1000.5, 0, 0), 1000,
            Material(Vec3(0.8, 0.1, 0.1), Vec3(0, 0, 0), 0.0)));

        spheres.push_back(Sphere(Vec3(1000.5, 0, 0), 1000,
            Material(Vec3(0.1, 0.8, 0.1), Vec3(0, 0, 0), 0.0)));

        spheres.push_back(Sphere(Vec3(0, 0.45, 0), 0.15,
            Material(Vec3(1, 1, 1), Vec3(12, 12, 12), 0.0)));

        spheres.push_back(Sphere(Vec3(-0.25, -0.25, -0.3), 0.25,
            Material(Vec3(0.9, 0.9, 0.2), Vec3(0, 0, 0), 0.0)));

        spheres.push_back(Sphere(Vec3(0.25, -0.25, 0.1), 0.25,
            Material(Vec3(0.2, 0.3, 0.9), Vec3(0, 0, 0), 0.3)));
    }

    bool intersect(const Ray& ray, double& t, int& hitIndex) const {
        bool hit = false;
        t = 1e20;
        hitIndex = -1;

        for (size_t i = 0; i < spheres.size(); i++) {
            double tTemp;
            if (spheres[i].intersect(ray, tTemp) && tTemp < t) {
                t = tTemp;
                hitIndex = i;
                hit = true;
            }
        }

        return hit;
    }
};

struct Camera {
    Vec3 position;
    Vec3 lookAt;
    Vec3 up;
    double fov;
    int width, height;

    Camera(Vec3 pos, Vec3 at, Vec3 u, double f, int w, int h)
        : position(pos), lookAt(at), up(u), fov(f), width(w), height(h) {
    }

    Ray getRay(int x, int y, std::mt19937& rng) const {
        std::uniform_real_distribution<double> dist(0.0, 1.0);

        double u = (x + dist(rng)) / width;
        double v = (y + dist(rng)) / height;

        double ndcX = 2.0 * u - 1.0;
        double ndcY = 1.0 - 2.0 * v;

        double aspectRatio = (double)width / height;
        double tanHalfFov = std::tan(fov * 0.5 * M_PI / 180.0);

        Vec3 forward = (lookAt - position).normalized();
        Vec3 right = forward.cross(up).normalized();
        Vec3 newUp = right.cross(forward).normalized();

        Vec3 horizontal = right * (ndcX * aspectRatio * tanHalfFov);
        Vec3 vertical = newUp * (ndcY * tanHalfFov);
        Vec3 direction = (forward + horizontal + vertical).normalized();

        return Ray(position, direction);
    }
};

class PathTracer {
private:
    Scene scene;
    Camera camera;
    int maxBounces;

    Vec3 randomInHemisphere(const Vec3& normal, std::mt19937& rng) {
        std::uniform_real_distribution<double> dist(0.0, 1.0);

        double r1 = dist(rng);
        double r2 = dist(rng);

        double phi = 2.0 * M_PI * r1;
        double cosTheta = std::sqrt(1.0 - r2);
        double sinTheta = std::sqrt(r2);

        Vec3 direction(std::cos(phi) * sinTheta, cosTheta, std::sin(phi) * sinTheta);

        Vec3 w = normal;
        Vec3 u = ((std::abs(w.x) > 0.1 ? Vec3(0, 1, 0) : Vec3(1, 0, 0)).cross(w)).normalized();
        Vec3 v = w.cross(u);

        return (u * direction.x + w * direction.y + v * direction.z).normalized();
    }

public:
    PathTracer(const Scene& s, const Camera& c, int bounces = 5)
        : scene(s), camera(c), maxBounces(bounces) {
    }

    Vec3 trace(const Ray& ray, int depth, std::mt19937& rng) {
        if (depth >= maxBounces) return Vec3(0, 0, 0);

        double t;
        int hitIndex;

        if (!scene.intersect(ray, t, hitIndex)) {
            return Vec3(0.5, 0.7, 1.0) * 0.3;
        }

        const Sphere& sphere = scene.spheres[hitIndex];
        Vec3 hitPoint = ray.origin + ray.direction * t;
        Vec3 normal = sphere.getNormal(hitPoint);

        Vec3 color = sphere.material.emission;

        if (depth > 3) {
            double continueProbability = 0.9;
            std::uniform_real_distribution<double> dist(0.0, 1.0);
            if (dist(rng) > continueProbability) return color;
            color = color / continueProbability;
        }

        Vec3 newDirection = randomInHemisphere(normal, rng);
        Ray newRay(hitPoint, newDirection);

        Vec3 incoming = trace(newRay, depth + 1, rng);

        double cosTheta = std::max(0.0, normal.dot(newDirection));
        color = color + sphere.material.color.mult(incoming) * cosTheta * 2.0;

        return color;
    }

    Vec3 renderSample(int x, int y, std::mt19937& rng) {
        Ray ray = camera.getRay(x, y, rng);
        return trace(ray, 0, rng);
    }
};


void renderAdaptive(PathTracer& tracer, Image& image, AdaptiveSampler& sampler,
    int totalBudget, unsigned int seed) {

    std::mt19937 rng(seed);
    int width = image.width;
    int height = image.height;

    std::cout << "Phase 1: Initial sampling for variance estimation..." << std::endl;

    std::vector<std::vector<Vec3>> initialSamples(width * height);
    int initialSamplesPerPixel = 8;  

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            initialSamples[idx].resize(initialSamplesPerPixel);

            Vec3 color(0, 0, 0);
            for (int s = 0; s < initialSamplesPerPixel; s++) {
                Vec3 sample = tracer.renderSample(x, y, rng);
                initialSamples[idx][s] = sample;
                color = color + sample;
            }

            color = color / initialSamplesPerPixel;
            image.setPixel(x, y, color);

            sampler.computeVariance(initialSamples[idx], x, y);
        }
    }

    std::cout << "Phase 2: Computing edge strength..." << std::endl;
    sampler.computeEdgeStrength(image);

    std::vector<Vec3> motionVectors(width * height, Vec3(0, 0, 0));  // Static scene
    sampler.computeMotion(motionVectors);

    sampler.applyTemporalReuse();

    // θ(x) = s_min + α_V*sqrt(V(x)) + α_D*D(x) + α_M*M(x)
    std::cout << "Phase 3: Computing adaptive heuristic..." << std::endl;
    sampler.computeHeuristic();

    // n(x) = N * θ(x) / Σθ(y)
    int remainingBudget = totalBudget - (width * height * initialSamplesPerPixel);
    sampler.allocateSamples(remainingBudget);

    std::cout << "Phase 4: Adaptive sampling..." << std::endl;
    std::cout << "Sample allocation - Min: " << sampler.getMinSamples()
        << ", Max: " << sampler.getMaxSamples()
        << ", Avg: " << sampler.getAverageSamples() << std::endl;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            int additionalSamples = sampler.getSampleCount(x, y);

            Vec3 color = image.getPixel(x, y) * initialSamplesPerPixel;

            for (int s = 0; s < additionalSamples; s++) {
                color = color + tracer.renderSample(x, y, rng);
            }

            int totalSamples = initialSamplesPerPixel + additionalSamples;
            color = color / totalSamples;
            image.setPixel(x, y, color);
        }

        if ((y + 1) % 50 == 0) {
            std::cout << "Progress: " << (y + 1) << "/" << height << " rows" << std::endl;
        }
    }

    sampler.updatePreviousFrame(image);
}

double computeSSIM(const Image& img1, const Image& img2) {
    if (img1.width != img2.width || img1.height != img2.height) {
        return 0.0;
    }

    double C1 = 0.01 * 0.01;
    double C2 = 0.03 * 0.03;

    double mean1 = 0.0, mean2 = 0.0;
    int n = img1.width * img1.height;

    for (int i = 0; i < n; i++) {
        mean1 += img1.getLuminance(i % img1.width, i / img1.width);
        mean2 += img2.getLuminance(i % img2.width, i / img2.width);
    }
    mean1 /= n;
    mean2 /= n;

    double var1 = 0.0, var2 = 0.0, covar = 0.0;
    for (int i = 0; i < n; i++) {
        double lum1 = img1.getLuminance(i % img1.width, i / img1.width);
        double lum2 = img2.getLuminance(i % img2.width, i / img2.width);

        var1 += (lum1 - mean1) * (lum1 - mean1);
        var2 += (lum2 - mean2) * (lum2 - mean2);
        covar += (lum1 - mean1) * (lum2 - mean2);
    }
    var1 /= (n - 1);
    var2 /= (n - 1);
    covar /= (n - 1);

    double numerator = (2 * mean1 * mean2 + C1) * (2 * covar + C2);
    double denominator = (mean1 * mean1 + mean2 * mean2 + C1) * (var1 + var2 + C2);

    return numerator / denominator;
}

double perceptualDistance(const Image& rendered, const Image& reference) {
    return 1.0 - computeSSIM(rendered, reference);
}

int main() {
    int width = 512, height = 512;
    int totalBudget = width * height * 256;  

    unsigned int seeds[20] = { 30, 35, 40, 42, 43, 50, 55, 60, 70, 79, 81, 85, 87, 93, 111, 135, 141, 145, 147, 211 };
    for (int i = 0; i < 20; i++) {
        unsigned int seed = seeds[i];

        std::cout << "\n\nSEED: " << seed << std::endl;

        Scene scene;
        scene.createCornellBox();
        Camera camera(Vec3(0, 0, 2.5), Vec3(0, 0, 0), Vec3(0, 1, 0), 40.0, width, height);
        PathTracer tracer(scene, camera, 5);

        std::cout << "Resolution: " << width << "x" << height << std::endl;
        std::cout << "Total sample budget N: " << totalBudget << std::endl;
        std::cout << "Average samples per pixel: " << (totalBudget / (width * height)) << std::endl;

        std::cout << "\nStandard Path Tracing" << std::endl;

        Image baselineImage(width, height);
        auto baselineStart = std::chrono::high_resolution_clock::now();

        std::mt19937 rngBaseline(seed);
        int uniformSPP = totalBudget / (width * height);

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                Vec3 color(0, 0, 0);
                for (int s = 0; s < uniformSPP; s++) {
                    color = color + tracer.renderSample(x, y, rngBaseline);
                }
                color = color / uniformSPP;
                baselineImage.setPixel(x, y, color);
            }

            if ((y + 1) % 50 == 0) {
                std::cout << "Progress: " << (y + 1) << "/" << height << " rows" << std::endl;
            }
        }

        auto baselineEnd = std::chrono::high_resolution_clock::now();
        double baselineTime = std::chrono::duration<double>(baselineEnd - baselineStart).count();

        std::cout << "Baseline rendering time T_baseline: " << baselineTime << " seconds" << std::endl;
        baselineImage.savePPM("baseline_uniform.ppm");

        std::cout << "\nAdaptive Sampling Algorithm" << std::endl;

        Image adaptiveImage(width, height);
        AdaptiveSampler sampler(width, height,
            0.2,
            0.0,
            0.0,
            8,
            1e-6,
            5.0
        );

        auto adaptiveStart = std::chrono::high_resolution_clock::now();
        renderAdaptive(tracer, adaptiveImage, sampler, totalBudget, seed);
        auto adaptiveEnd = std::chrono::high_resolution_clock::now();

        double adaptiveTime = std::chrono::duration<double>(adaptiveEnd - adaptiveStart).count();

        std::cout << "Adaptive rendering time T_adaptive: " << adaptiveTime << " seconds" << std::endl;
        adaptiveImage.savePPM("adaptive_variance.ppm");

        std::cout << "\nEvaluation Metrics" << std::endl;

        Image& referenceImage = baselineImage;

        double ssimBaseline = computeSSIM(baselineImage, referenceImage);
        double ssimAdaptive = computeSSIM(adaptiveImage, referenceImage);

        double distBaseline = perceptualDistance(baselineImage, referenceImage);
        double distAdaptive = perceptualDistance(adaptiveImage, referenceImage);

        std::cout << "\nQuality (SSIM):" << std::endl;
        std::cout << "  Baseline: " << ssimBaseline << std::endl;
        std::cout << "  Adaptive: " << ssimAdaptive << std::endl;

        std::cout << "\nPerceptual Distance D(I_hat, I_ref):" << std::endl;
        std::cout << "  Baseline: " << distBaseline << std::endl;
        std::cout << "  Adaptive: " << distAdaptive << std::endl;

        double speedup = baselineTime / adaptiveTime;
        std::cout << "\nSpeedup(q) = T_baseline / T_adaptive: " << speedup << "x" << std::endl;

        std::cout << "\nTime-to-Quality:" << std::endl;
        std::cout << "  Baseline: " << baselineTime << " s" << std::endl;
        std::cout << "  Adaptive: " << adaptiveTime << " s" << std::endl;

        std::cout << "\nSample Allocation Statistics:" << std::endl;
        std::cout << "  Min samples/pixel: " << sampler.getMinSamples() << std::endl;
        std::cout << "  Max samples/pixel: " << sampler.getMaxSamples() << std::endl;
        std::cout << "  Avg samples/pixel: " << sampler.getAverageSamples() << std::endl;
        std::cout << "  Ratio (max/min): " << (sampler.getMaxSamples() / sampler.getMinSamples()) << std::endl;
    }

    return 0;
}