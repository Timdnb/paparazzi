#include <iostream>
#include <vector>
#include <cmath>

std::vector<std::vector<int>> project_points(std::vector<std::vector<float>> projection_matrix, std::vector<std::vector<float>> point_3D) {
    std::vector<std::vector<int>> uvs;

    for (const auto& row : point_3D) {
        std::vector<int> uv(2, 0);

        // Calculate the projected point
        std::vector<float> projected_points_homogeneous(3, 0.0f);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                projected_points_homogeneous[i] += projection_matrix[i][j] * row[j];
            }
        }

        // Normalize the homogeneous coordinates
        float w = projected_points_homogeneous[2]; // Third entry
        for (int i = 0; i < 2; ++i) {
            projected_points_homogeneous[i] /= w;
        }

        // Round the coordinates to integers
        
        for (int i = 0; i < 2; ++i) {
            uv[i] = static_cast<int>(std::round(projected_points_homogeneous[i]));
        }


        uvs.push_back(uv);
    }

    return uvs;
}

int main() {
    // Example usage
    std::vector<std::vector<float>> projection_matrix = {{300.0f, 0.0f, 120.0f, 0.0f},
                                                         {0.0f, 300.0f, 260.0f, 0.0f},
                                                         {0.0f, 0.0f, 1.0f, 0.0f}};
    //here corners are given as in drone frame
    //std::vector<std::vector<float>> point_3D = {{4.0f, 4.0f, 0.0f, 1.0f},
    //                                                  {-4.0f, -4.0f, 0.0f, 1.0f},
    //                                                  {4.0f, -4.0f, 0.0f, 1.0f}};
    // here corners are given as in camera frame
    std::vector<std::vector<float>> point_3D = {{0.0f, 4.0f, 4.0f, 1.0f},
                                                      {0.0f, -4.0f, -4.0f, 1.0f},
                                                      {0.0f, 4.0f, -4.0f, 1.0f}};

    std::vector<std::vector<int>> uvs = project_points(projection_matrix, point_3D);

    // Output the results
    for (const auto& uv : uvs) {
        std::cout << "(" << uv[0] << ", " << uv[1] << ")" << std::endl;
    }

    return 0;
}

