// Complete adaptive ray tracer implementing all equations from your document
// Save this as a SEPARATE file to compare with baseline

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


// Ray structure
struct Ray {
    Vec3 origin, direction;
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d.normalized()) {}
};

// Material structure
struct Material {
    Vec3 color;
    Vec3 emission;
    double reflectivity;

    Material(Vec3 c = Vec3(0.8, 0.8, 0.8), Vec3 e = Vec3(0, 0, 0), double r = 0.0)
        : color(c), emission(e), reflectivity(r) {
    }
};

// Sphere primitive
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

// Scene containing all geometry
struct Scene {
    std::vector<Sphere> spheres;

    void createCornellBox() {
        // Cornell Box dimensions: 1 unit cube centered at origin
        double size = 1.0;

        // Floor (white)
        spheres.push_back(Sphere(Vec3(0, -1000.5, 0), 1000,
            Material(Vec3(0.8, 0.8, 0.8), Vec3(0, 0, 0), 0.0)));

        // Ceiling (white)
        spheres.push_back(Sphere(Vec3(0, 1000.5, 0), 1000,
            Material(Vec3(0.8, 0.8, 0.8), Vec3(0, 0, 0), 0.0)));

        // Back wall (white)
        spheres.push_back(Sphere(Vec3(0, 0, -1001), 1000,
            Material(Vec3(0.8, 0.8, 0.8), Vec3(0, 0, 0), 0.0)));

        // Left wall (red)
        spheres.push_back(Sphere(Vec3(-1000.5, 0, 0), 1000,
            Material(Vec3(0.8, 0.1, 0.1), Vec3(0, 0, 0), 0.0)));

        // Right wall (green)
        spheres.push_back(Sphere(Vec3(1000.5, 0, 0), 1000,
            Material(Vec3(0.1, 0.8, 0.1), Vec3(0, 0, 0), 0.0)));

        // Light (area light at top)
        spheres.push_back(Sphere(Vec3(0, 0.45, 0), 0.15,
            Material(Vec3(1, 1, 1), Vec3(12, 12, 12), 0.0)));

        // Left sphere (diffuse)
        spheres.push_back(Sphere(Vec3(-0.25, -0.25, -0.3), 0.25,
            Material(Vec3(0.9, 0.9, 0.2), Vec3(0, 0, 0), 0.0)));

        // Right sphere (more reflective)
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

// Camera structure
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

        // Add jittering for antialiasing
        double u = (x + dist(rng)) / width;
        double v = (y + dist(rng)) / height;

        // Convert to NDC [-1, 1]
        double ndcX = 2.0 * u - 1.0;
        double ndcY = 1.0 - 2.0 * v;

        // Calculate aspect ratio
        double aspectRatio = (double)width / height;
        double tanHalfFov = std::tan(fov * 0.5 * M_PI / 180.0);

        // Calculate camera basis vectors
        Vec3 forward = (lookAt - position).normalized();
        Vec3 right = forward.cross(up).normalized();
        Vec3 newUp = right.cross(forward).normalized();

        // Calculate ray direction
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

    // Random sampling helper
    Vec3 randomInHemisphere(const Vec3& normal, std::mt19937& rng) {
        std::uniform_real_distribution<double> dist(0.0, 1.0);

        double r1 = dist(rng);
        double r2 = dist(rng);

        double phi = 2.0 * M_PI * r1;
        double cosTheta = std::sqrt(1.0 - r2);
        double sinTheta = std::sqrt(r2);

        Vec3 direction(std::cos(phi) * sinTheta, cosTheta, std::sin(phi) * sinTheta);

        // Transform to world space
        Vec3 w = normal;
        Vec3 u = ((std::abs(w.x) > 0.1 ? Vec3(0, 1, 0) : Vec3(1, 0, 0)).cross(w)).normalized();
        Vec3 v = w.cross(u);

        return (u * direction.x + w * direction.y + v * direction.z).normalized();
    }

public:
    PathTracer(const Scene& s, const Camera& c, int bounces = 5)
        : scene(s), camera(c), maxBounces(bounces) {
    }

    // Trace single ray and return radiance (Equation from I.1.3)
    Vec3 trace(const Ray& ray, int depth, std::mt19937& rng) {
        if (depth >= maxBounces) return Vec3(0, 0, 0);

        double t;
        int hitIndex;

        if (!scene.intersect(ray, t, hitIndex)) {
            // Sky color
            return Vec3(0.5, 0.7, 1.0) * 0.3;
        }

        const Sphere& sphere = scene.spheres[hitIndex];
        Vec3 hitPoint = ray.origin + ray.direction * t;
        Vec3 normal = sphere.getNormal(hitPoint);

        // Add emission (lights)
        Vec3 color = sphere.material.emission;

        // Russian roulette for path termination
        if (depth > 3) {
            double continueProbability = 0.9;
            std::uniform_real_distribution<double> dist(0.0, 1.0);
            if (dist(rng) > continueProbability) return color;
            color = color / continueProbability;
        }

        // Sample random direction in hemisphere
        Vec3 newDirection = randomInHemisphere(normal, rng);
        Ray newRay(hitPoint, newDirection);

        // Recursive ray tracing (Monte Carlo integration)
        Vec3 incoming = trace(newRay, depth + 1, rng);

        // BRDF (Lambertian for now)
        double cosTheta = std::max(0.0, normal.dot(newDirection));
        color = color + sphere.material.color.mult(incoming) * cosTheta * 2.0;

        return color;
    }

    // Render single sample for pixel (x, y) - Returns Y_i(x) from equation I.1.3
    Vec3 renderSample(int x, int y, std::mt19937& rng) {
        Ray ray = camera.getRay(x, y, rng);
        return trace(ray, 0, rng);
    }
};

// Image buffer


// Include all previous structures (Vec3, Ray, Material, Sphere, Scene, Camera, PathTracer, Image)
// [Copy the Vec3, Ray, Material, Sphere, Scene, Camera, and Image structures from main.cpp here]
// [Copy the PathTracer class from main.cpp here]

// For brevity, I'll show the adaptive rendering part
// You'll need to combine this with the full code from main.cpp

// Adaptive rendering function
void renderAdaptive(PathTracer& tracer, Image& image, AdaptiveSampler& sampler,
    int totalBudget, unsigned int seed) {

    std::mt19937 rng(seed);
    int width = image.width;
    int height = image.height;

    // Step 1: Initial low-resolution sampling pass (to estimate variance)
    std::cout << "Phase 1: Initial sampling for variance estimation..." << std::endl;

    std::vector<std::vector<Vec3>> initialSamples(width * height);
    int initialSamplesPerPixel = 8;  // Low sample count for initial variance estimate

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

            // Compute variance V(x) from samples
            sampler.computeVariance(initialSamples[idx], x, y);
        }
    }

    // Step 2: Compute edge strength D(x)
    std::cout << "Phase 2: Computing edge strength..." << std::endl;
    sampler.computeEdgeStrength(image);

    // Step 3: Compute motion M(x) (for static scenes, this is zero)
    std::vector<Vec3> motionVectors(width * height, Vec3(0, 0, 0));  // Static scene
    sampler.computeMotion(motionVectors);

    // Step 4: Apply temporal reuse (if previous frame exists)
    sampler.applyTemporalReuse();

    // Step 5: Compute heuristic θ(x) = s_min + α_V*sqrt(V(x)) + α_D*D(x) + α_M*M(x)
    std::cout << "Phase 3: Computing adaptive heuristic..." << std::endl;
    sampler.computeHeuristic();

    // Step 6: Allocate samples: n(x) = N * θ(x) / Σθ(y)
    int remainingBudget = totalBudget - (width * height * initialSamplesPerPixel);
    sampler.allocateSamples(remainingBudget);

    std::cout << "Phase 4: Adaptive sampling..." << std::endl;
    std::cout << "Sample allocation - Min: " << sampler.getMinSamples()
        << ", Max: " << sampler.getMaxSamples()
        << ", Avg: " << sampler.getAverageSamples() << std::endl;

    // Step 7: Render with adaptive sample allocation
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            int additionalSamples = sampler.getSampleCount(x, y);

            // Get existing color from initial pass
            Vec3 color = image.getPixel(x, y) * initialSamplesPerPixel;

            // Add adaptive samples
            for (int s = 0; s < additionalSamples; s++) {
                color = color + tracer.renderSample(x, y, rng);
            }

            // Average over total samples: I_hat(x) = (1/n(x)) * Σ Y_i(x)
            int totalSamples = initialSamplesPerPixel + additionalSamples;
            color = color / totalSamples;
            image.setPixel(x, y, color);
        }

        if ((y + 1) % 50 == 0) {
            std::cout << "Progress: " << (y + 1) << "/" << height << " rows" << std::endl;
        }
    }

    // Step 8: Update previous frame for temporal reuse
    sampler.updatePreviousFrame(image);
}

// SSIM calculation (Equation I.1.6 - for quality metrics)
double computeSSIM(const Image& img1, const Image& img2) {
    // Simplified SSIM implementation
    // Full implementation would use sliding windows

    if (img1.width != img2.width || img1.height != img2.height) {
        return 0.0;
    }

    double C1 = 0.01 * 0.01;
    double C2 = 0.03 * 0.03;

    // Compute means
    double mean1 = 0.0, mean2 = 0.0;
    int n = img1.width * img1.height;

    for (int i = 0; i < n; i++) {
        mean1 += img1.getLuminance(i % img1.width, i / img1.width);
        mean2 += img2.getLuminance(i % img2.width, i / img2.width);
    }
    mean1 /= n;
    mean2 /= n;

    // Compute variances and covariance
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

    // SSIM formula
    double numerator = (2 * mean1 * mean2 + C1) * (2 * covar + C2);
    double denominator = (mean1 * mean1 + mean2 * mean2 + C1) * (var1 + var2 + C2);

    return numerator / denominator;
}

// Perceptual distance D(I_hat, I_ref) = 1 - SSIM
double perceptualDistance(const Image& rendered, const Image& reference) {
    return 1.0 - computeSSIM(rendered, reference);
}

// Main comparison experiment
int main() {
    // Configuration
    int width = 512, height = 512;
    int totalBudget = width * height * 1024;  // N = total sample budget
    unsigned int seed = 42;

    // Setup scene and camera
    Scene scene;
    scene.createCornellBox();
    Camera camera(Vec3(0, 0, 2.5), Vec3(0, 0, 0), Vec3(0, 1, 0), 40.0, width, height);
    PathTracer tracer(scene, camera, 5);

    std::cout << "=== ADAPTIVE RAY TRACING EXPERIMENT ===" << std::endl;
    std::cout << "Resolution: " << width << "x" << height << std::endl;
    std::cout << "Total sample budget N: " << totalBudget << std::endl;
    std::cout << "Average samples per pixel: " << (totalBudget / (width * height)) << std::endl;

    // === BASELINE: Standard Path Tracing ===
    std::cout << "\n--- BASELINE: Standard Path Tracing ---" << std::endl;

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

    // === ADAPTIVE: Adaptive Sampling Algorithm ===
    std::cout << "\n--- ADAPTIVE: Adaptive Sampling Algorithm ---" << std::endl;

    Image adaptiveImage(width, height);
    AdaptiveSampler sampler(width, height,
        //1.0,   // α_V (variance weight)
        //0.5,   // α_D (edge weight)
        //0.3,   // α_M (motion weight)
        //4,     // s_min (minimum samples)
        //1e-6,  // ε (epsilon)
        //5.0    // β (temporal decay)
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

    // === EVALUATION METRICS (Section I.1.6) ===
    std::cout << "\n=== EVALUATION METRICS ===" << std::endl;

    // Use baseline as reference (or load high-quality ground truth)
    Image& referenceImage = baselineImage;

    // Compute quality metrics
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

    // Speedup calculation (Equation I.1.6)
    double speedup = baselineTime / adaptiveTime;
    std::cout << "\nSpeedup(q) = T_baseline / T_adaptive: " << speedup << "x" << std::endl;

    // Time-to-quality metric
    std::cout << "\nTime-to-Quality:" << std::endl;
    std::cout << "  Baseline: " << baselineTime << " s" << std::endl;
    std::cout << "  Adaptive: " << adaptiveTime << " s" << std::endl;

    // Sample efficiency
    std::cout << "\nSample Allocation Statistics:" << std::endl;
    std::cout << "  Min samples/pixel: " << sampler.getMinSamples() << std::endl;
    std::cout << "  Max samples/pixel: " << sampler.getMaxSamples() << std::endl;
    std::cout << "  Avg samples/pixel: " << sampler.getAverageSamples() << std::endl;
    std::cout << "  Ratio (max/min): " << (sampler.getMaxSamples() / sampler.getMinSamples()) << std::endl;

    std::cout << "\n=== EXPERIMENT COMPLETE ===" << std::endl;
    std::cout << "\nOutput files:" << std::endl;
    std::cout << "  - baseline_uniform.ppm (uniform sampling)" << std::endl;
    std::cout << "  - adaptive_variance.ppm (adaptive sampling)" << std::endl;

    std::cout << "\nNext steps:" << std::endl;
    std::cout << "1. Run this 20-30 times with different seeds (Section I.3.2)" << std::endl;
    std::cout << "2. Perform paired t-test on results (Section I.1.7)" << std::endl;
    std::cout << "3. Run ablation study (Section I.1.8)" << std::endl;
    std::cout << "4. Test on other scenes (Sponza, Bistro, etc.)" << std::endl;

    return 0;
}