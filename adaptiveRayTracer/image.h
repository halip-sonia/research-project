#include <vector>
#include "vec3.h"

struct Image {
    int width, height;
    std::vector<Vec3> pixels;

    Image(int w, int h) : width(w), height(h), pixels(w* h, Vec3(0, 0, 0)) {}

    void setPixel(int x, int y, const Vec3& color) {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            pixels[y * width + x] = color;
        }
    }

    Vec3 getPixel(int x, int y) const {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            return pixels[y * width + x];
        }
        return Vec3(0, 0, 0);
    }

    double getLuminance(int x, int y) const {
        Vec3 color = getPixel(x, y);
        return 0.2126 * color.x + 0.7152 * color.y + 0.0722 * color.z;
    }

    void savePPM(const std::string& filename) const {
        std::ofstream file(filename);
        file << "P3\n" << width << " " << height << "\n255\n";

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                Vec3 color = getPixel(x, y);

                color.x = std::pow(std::clamp(color.x, 0.0, 1.0), 1.0 / 2.2);
                color.y = std::pow(std::clamp(color.y, 0.0, 1.0), 1.0 / 2.2);
                color.z = std::pow(std::clamp(color.z, 0.0, 1.0), 1.0 / 2.2);

                int r = static_cast<int>(255.99 * color.x);
                int g = static_cast<int>(255.99 * color.y);
                int b = static_cast<int>(255.99 * color.z);

                file << r << " " << g << " " << b << " ";
            }
            file << "\n";
        }

        file.close();
        std::cout << "Saved image: " << filename << std::endl;
    }
};