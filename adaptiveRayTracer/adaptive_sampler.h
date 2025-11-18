#ifndef ADAPTIVE_SAMPLER_H
#define ADAPTIVE_SAMPLER_H

#include <vector>
#include <cmath>
#include <algorithm>
#include "image.h"

// θ(x) = s_min + α_V * sqrt(V(x)) + α_D * D(x) + α_M * M(x)


class AdaptiveSampler {
private:
    int width, height;
    double epsilon; 

    double alphaV;  
    double alphaD; 
    double alphaM; 

    int sMin;

    std::vector<double> variance;        
    std::vector<double> edgeStrength;   
    std::vector<double> motion;         
    std::vector<double> theta;           
    std::vector<int> sampleCount;        

    std::vector<double> prevLuminance;  
    std::vector<double> prevVariance;   

    double beta;  // for ρ(x) = exp(-β*M(x))

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

    void computeVariance(const std::vector<Vec3>& samples, int x, int y) {
        int idx = y * width + x;
        int n = samples.size();

        if (n < 2) {
            variance[idx] = epsilon;
            return;
        }

        double meanLum = 0.0;
        for (const auto& sample : samples) {
            double lum = 0.2126 * sample.x + 0.7152 * sample.y + 0.0722 * sample.z;
            meanLum += lum;
        }
        meanLum /= n;

        double var = 0.0;
        for (const auto& sample : samples) {
            double lum = 0.2126 * sample.x + 0.7152 * sample.y + 0.0722 * sample.z;
            double diff = lum - meanLum;
            var += diff * diff;
        }
        var /= (n - 1);

        variance[idx] = std::max(var, epsilon);
    }

    // |L(x)^(t) - L(x)^(t-1)|
    void computeEdgeStrength(const Image& currentFrame) {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;
                double currentLum = getLuminance(currentFrame, x, y);

                edgeStrength[idx] = std::abs(currentLum - prevLuminance[idx]);
            }
        }
    }


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
                    motion[idx] = 0.0;  
                }
            }
        }
    }

    void computeHeuristic() {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;

                theta[idx] = sMin
                    + alphaV * std::sqrt(variance[idx])
                    + alphaD * edgeStrength[idx]
                    + alphaM * motion[idx];
            }
        }
    }

    // n(x) = N * θ(x) / Σ_y∈Ω θ(y)
    void allocateSamples(int totalSampleBudget) {
        double thetaSum = 0.0;
        for (double t : theta) {
            thetaSum += t;
        }

        if (thetaSum < epsilon) {
            int uniformSamples = totalSampleBudget / (width * height);
            std::fill(sampleCount.begin(), sampleCount.end(), uniformSamples);
            return;
        }

        int maxSamplesPerPixel = 128;
        int allocatedTotal = 0;
        for (int idx = 0; idx < width * height; idx++) {
            sampleCount[idx] = static_cast<int>(
                (totalSampleBudget * theta[idx]) / thetaSum
                );
            sampleCount[idx] = std::max(sampleCount[idx], sMin); 
            sampleCount[idx] = std::min(sampleCount[idx], maxSamplesPerPixel);
            allocatedTotal += sampleCount[idx];
        }

        int remaining = totalSampleBudget - allocatedTotal;
        if (remaining > 0) {
            for (int i = 0; i < remaining && i < width * height; i++) {
                sampleCount[i]++;
            }
        }
    }

    // V_t(x) = ρ(x) * V_{t-1}(x) + (1 - ρ(x)) * V_new(x)
    void applyTemporalReuse() {
        for (int idx = 0; idx < width * height; idx++) {
            double rho = std::exp(-beta * motion[idx]);

            double newVariance = rho * prevVariance[idx] + (1.0 - rho) * variance[idx];

            prevVariance[idx] = newVariance;
            variance[idx] = newVariance;
        }
    }

    void updatePreviousFrame(const Image& currentFrame) {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;
                prevLuminance[idx] = getLuminance(currentFrame, x, y);
            }
        }
    }

    int getSampleCount(int x, int y) const {
        if (x < 0 || x >= width || y < 0 || y >= height) return sMin;
        return sampleCount[y * width + x];
    }

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

    // ablation study configurations
    void setAblationMode(int mode) {
        switch (mode) {
        case 0:  
            alphaV = 1.0; alphaD = 0.5; alphaM = 0.3;
            break;
        case 1:  
            alphaV = 0.0; alphaD = 0.5; alphaM = 0.3;
            break;
        case 2:  
            alphaV = 1.0; alphaD = 0.0; alphaM = 0.3;
            break;
        case 3: 
            alphaV = 1.0; alphaD = 0.5; alphaM = 0.0;
            break;
        }
    }

private:
    double getLuminance(const Image& img, int x, int y) const;
};

inline double AdaptiveSampler::getLuminance(const Image& img, int x, int y) const {
    if (x < 0 || x >= width || y < 0 || y >= height) return 0.0;
    Vec3 color = img.getPixel(x, y);
    return 0.2126 * color.x + 0.7152 * color.y + 0.0722 * color.z;
}

#endif 