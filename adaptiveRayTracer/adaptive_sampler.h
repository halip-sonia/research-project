#ifndef ADAPTIVE_SAMPLER_H
#define ADAPTIVE_SAMPLER_H

#include <vector>
#include <cmath>
#include <algorithm>
#include "image.h"

// Implements the adaptive sampling algorithm from Section I.1.4
// Equation: θ(x) = s_min + α_V * sqrt(V(x)) + α_D * D(x) + α_M * M(x)


class AdaptiveSampler {
private:
    int width, height;
    double epsilon;  // Minimum variance threshold

    // Heuristic weights (α_V, α_D, α_M from equation I.1.4)
    double alphaV;  // Weight for variance term
    double alphaD;  // Weight for edge term
    double alphaM;  // Weight for motion term

    int sMin;  // Minimum samples per pixel

    // Per-pixel statistics
    std::vector<double> variance;        // V(x) - temporal variance estimate
    std::vector<double> edgeStrength;    // D(x) - edge strength
    std::vector<double> motion;          // M(x) - motion magnitude
    std::vector<double> theta;           // θ(x) - heuristic score
    std::vector<int> sampleCount;        // n(x) - allocated samples per pixel

    // Previous frame data for temporal reuse (Section I.1.5)
    std::vector<double> prevLuminance;   // L(x)^(t-1)
    std::vector<double> prevVariance;    // V_{t-1}(x)

    double beta;  // Decay parameter for temporal reuse: ρ(x) = exp(-β*M(x))

public:
    AdaptiveSampler(int w, int h,
        double alphaV = 1.0, double alphaD = 0.5, double alphaM = 0.3,
        int sMin = 4, double eps = 1e-6, double beta = 5.0)
        : width(w), height(h), alphaV(alphaV), alphaD(alphaD), alphaM(alphaM),
        sMin(sMin), epsilon(eps), beta(beta) {

        int totalPixels = width * height;
        variance.resize(totalPixels, 0.0);
        edgeStrength.resize(totalPixels, 0.0);
        motion.resize(totalPixels, 0.0);
        theta.resize(totalPixels, 0.0);
        sampleCount.resize(totalPixels, sMin);
        prevLuminance.resize(totalPixels, 0.0);
        prevVariance.resize(totalPixels, 0.0);
    }

    // Calculate variance V(x) from samples (Equation I.1.4)
    void computeVariance(const std::vector<Vec3>& samples, int x, int y) {
        int idx = y * width + x;
        int n = samples.size();

        if (n < 2) {
            variance[idx] = epsilon;
            return;
        }

        // Calculate mean luminance
        double meanLum = 0.0;
        for (const auto& sample : samples) {
            double lum = 0.2126 * sample.x + 0.7152 * sample.y + 0.0722 * sample.z;
            meanLum += lum;
        }
        meanLum /= n;

        // Calculate variance: Var[L(x)]
        double var = 0.0;
        for (const auto& sample : samples) {
            double lum = 0.2126 * sample.x + 0.7152 * sample.y + 0.0722 * sample.z;
            double diff = lum - meanLum;
            var += diff * diff;
        }
        var /= (n - 1);

        // V(x) = max(Var[L(x)], ε) - Equation I.1.4
        variance[idx] = std::max(var, epsilon);
    }

    // Calculate edge strength D(x) (Equation I.1.4: |L(x)^(t) - L(x)^(t-1)|)
    void computeEdgeStrength(const Image& currentFrame) {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;
                double currentLum = getLuminance(currentFrame, x, y);

               /* double gradX = 0.0, gradY = 0.0;

                if (x > 0 && x < width - 1) {
                    double lumLeft = getLuminance(currentFrame, x - 1, y);
                    double lumRight = getLuminance(currentFrame, x + 1, y);
                    gradX = std::abs(lumRight - lumLeft);
                }

                if (y > 0 && y < height - 1) {
                    double lumUp = getLuminance(currentFrame, x, y - 1);
                    double lumDown = getLuminance(currentFrame, x, y + 1);
                    gradY = std::abs(lumDown - lumUp);
                }*/

                // D(x) = |L(x)^(t) - L(x)^(t-1)|
                edgeStrength[idx] = std::abs(currentLum - prevLuminance[idx]);
                //edgeStrength[idx] = std::sqrt(gradX * gradX + gradY * gradY);

            }
        }
    }

    // Calculate motion magnitude M(x) (Equation I.1.4: ||m(x)||)
    // For static scenes, M(x) = 0. For dynamic, computed from motion vectors
    void computeMotion(const std::vector<Vec3>& motionVectors) {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;

                if (idx < motionVectors.size()) {
                    // M(x) = ||m(x)||
                    Vec3 m = motionVectors[idx];
                    motion[idx] = std::sqrt(m.x * m.x + m.y * m.y + m.z * m.z);
                }
                else {
                    motion[idx] = 0.0;  // Static scene
                }
            }
        }
    }

    // Compute heuristic θ(x) for all pixels (Equation I.1.4)
    // θ(x) = s_min + α_V * sqrt(V(x)) + α_D * D(x) + α_M * M(x)
    void computeHeuristic() {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;

                // Full heuristic equation from I.1.4
                theta[idx] = sMin
                    + alphaV * std::sqrt(variance[idx])
                    + alphaD * edgeStrength[idx]
                    + alphaM * motion[idx];
            }
        }
    }

    // Allocate samples according to heuristic (Equation I.1.4)
    // n(x) = N * θ(x) / Σ_y∈Ω θ(y)
    void allocateSamples(int totalSampleBudget) {
        // Compute sum of all heuristic scores: Σ_y∈Ω θ(y)
        double thetaSum = 0.0;
        for (double t : theta) {
            thetaSum += t;
        }

        if (thetaSum < epsilon) {
            // Fallback to uniform sampling
            int uniformSamples = totalSampleBudget / (width * height);
            std::fill(sampleCount.begin(), sampleCount.end(), uniformSamples);
            return;
        }

        // Allocate samples: n(x) = N * θ(x) / Σθ(y)
        int maxSamplesPerPixel = 128;
        int allocatedTotal = 0;
        for (int idx = 0; idx < width * height; idx++) {
            sampleCount[idx] = static_cast<int>(
                (totalSampleBudget * theta[idx]) / thetaSum
                );
            sampleCount[idx] = std::max(sampleCount[idx], sMin); 
            sampleCount[idx] = std::min(sampleCount[idx], maxSamplesPerPixel);
            // Enforce minimum
            allocatedTotal += sampleCount[idx];
        }

        // Distribute remaining samples if budget not fully used
        int remaining = totalSampleBudget - allocatedTotal;
        if (remaining > 0) {
            for (int i = 0; i < remaining && i < width * height; i++) {
                sampleCount[i]++;
            }
        }
    }

    // Temporal reuse: update variance estimate (Equation I.1.5)
    // V_t(x) = ρ(x) * V_{t-1}(x) + (1 - ρ(x)) * V_new(x)
    // where ρ(x) = exp(-β * M(x))
    void applyTemporalReuse() {
        for (int idx = 0; idx < width * height; idx++) {
            // Similarity measure: ρ(x) = exp(-β * M(x))
            double rho = std::exp(-beta * motion[idx]);

            // If M(x) ≈ 0, then ρ(x) ≈ 1 (pixel is stable, reuse history)
            // V_t(x) = ρ(x) * V_{t-1}(x) + (1 - ρ(x)) * V_new(x)
            double newVariance = rho * prevVariance[idx] + (1.0 - rho) * variance[idx];

            prevVariance[idx] = newVariance;
            variance[idx] = newVariance;
        }
    }

    // Update previous frame data for next iteration
    void updatePreviousFrame(const Image& currentFrame) {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;
                prevLuminance[idx] = getLuminance(currentFrame, x, y);
            }
        }
    }

    // Get sample count for pixel
    int getSampleCount(int x, int y) const {
        if (x < 0 || x >= width || y < 0 || y >= height) return sMin;
        return sampleCount[y * width + x];
    }

    // Get statistics for analysis
    double getAverageSamples() const {
        double sum = 0.0;
        for (int s : sampleCount) sum += s;
        return sum / (width * height);
    }

    double getMaxSamples() const {
        return *std::max_element(sampleCount.begin(), sampleCount.end());
    }

    double getMinSamples() const {
        return *std::min_element(sampleCount.begin(), sampleCount.end());
    }

    // Ablation study configurations (Section I.1.8)
    void setAblationMode(int mode) {
        switch (mode) {
        case 0:  // Full heuristic
            alphaV = 1.0; alphaD = 0.5; alphaM = 0.3;
            break;
        case 1:  // Omit variance term
            alphaV = 0.0; alphaD = 0.5; alphaM = 0.3;
            break;
        case 2:  // Omit edge term
            alphaV = 1.0; alphaD = 0.0; alphaM = 0.3;
            break;
        case 3:  // Omit motion term
            alphaV = 1.0; alphaD = 0.5; alphaM = 0.0;
            break;
        }
    }

private:
    // Helper: get luminance from image
    double getLuminance(const Image& img, int x, int y) const;
};

// Implementation moved here to avoid circular dependency
inline double AdaptiveSampler::getLuminance(const Image& img, int x, int y) const {
    if (x < 0 || x >= width || y < 0 || y >= height) return 0.0;
    Vec3 color = img.getPixel(x, y);
    return 0.2126 * color.x + 0.7152 * color.y + 0.0722 * color.z;
}

#endif // ADAPTIVE_SAMPLER_H