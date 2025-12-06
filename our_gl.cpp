#include <cmath>
#include <limits>
#include <cstdlib>
#include "our_gl.h"

Matrix ModelView;
Matrix Viewport;
Matrix Projection;

IShader::~IShader() {}

void viewport(int x, int y, int w, int h) {
    Viewport = Matrix::identity();
    Viewport[0][3] = x + w / 2.f;
    Viewport[1][3] = y + h / 2.f;
    Viewport[2][3] = depth / 2.f;
    Viewport[0][0] = w / 2.f;
    Viewport[1][1] = h / 2.f;
    Viewport[2][2] = depth / 2.f;
}

void projection(float coeff) {
    Projection = Matrix::identity();
    Projection[3][2] = coeff;
}

void lookat(Vec3f eye, Vec3f center, Vec3f up) {
    Vec3f z = (eye - center).normalize();
    Vec3f x = cross(up, z).normalize();
    Vec3f y = cross(z, x).normalize();
    ModelView = Matrix::identity();
    for (int i = 0; i < 3; i++) {
        ModelView[0][i] = x[i];
        ModelView[1][i] = y[i];
        ModelView[2][i] = z[i];
        ModelView[i][3] = -center[i];
    }
}

Vec3f barycentric(Vec2f A, Vec2f B, Vec2f C, Vec2f P) {
    Vec3f s[2];
    for (int i = 2; i--; ) {
        s[i][0] = C[i] - A[i];
        s[i][1] = B[i] - A[i];
        s[i][2] = A[i] - P[i];
    }
    Vec3f u = cross(s[0], s[1]);
    if (std::abs(u[2]) > 1e-2)
        return Vec3f(1.f - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z);
    return Vec3f(-1, 1, 1);
}

void triangle(Vec4f* pts, IShader& shader, TGAImage& image, float* zbuffer) {
    Vec2f bboxmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            bboxmin[j] = std::min(bboxmin[j], pts[i][j] / pts[i][3]);
            bboxmax[j] = std::max(bboxmax[j], pts[i][j] / pts[i][3]);
        }
    }

    // Ограничиваем bounding box размерами изображения
    bboxmin.x = std::max(0.f, std::min((float)image.get_width() - 1, bboxmin.x));
    bboxmin.y = std::max(0.f, std::min((float)image.get_height() - 1, bboxmin.y));
    bboxmax.x = std::max(0.f, std::min((float)image.get_width() - 1, bboxmax.x));
    bboxmax.y = std::max(0.f, std::min((float)image.get_height() - 1, bboxmax.y));

    Vec2i P;
    TGAColor color;

    for (P.x = (int)bboxmin.x; P.x <= (int)bboxmax.x; P.x++) {
        for (P.y = (int)bboxmin.y; P.y <= (int)bboxmax.y; P.y++) {
            Vec3f c = barycentric(proj<2>(pts[0] / pts[0][3]),
                proj<2>(pts[1] / pts[1][3]),
                proj<2>(pts[2] / pts[2][3]),
                proj<2>(P));

            if (c.x < 0 || c.y < 0 || c.z < 0) continue;

            float z = pts[0][2] * c.x + pts[1][2] * c.y + pts[2][2] * c.z;
            float w = pts[0][3] * c.x + pts[1][3] * c.y + pts[2][3] * c.z;
            int frag_depth = z / w;

            bool discard = shader.fragment(c, color);
            if (!discard) {
                // Проверяем прозрачность
                if (color.bytespp == 4 && color[3] == 255) {
                    // Непрозрачный объект - обычный z-буфер
                    if (zbuffer[P.x + P.y * image.get_width()] > frag_depth) continue;
                    zbuffer[P.x + P.y * image.get_width()] = frag_depth;
                    image.set(P.x, P.y, color);
                }
                else {
                    // Полупрозрачный объект - только если ближе
                    // (но не обновляем z-буфер для прозрачных объектов)
                    if (zbuffer[P.x + P.y * image.get_width()] > frag_depth) continue;
                    image.set_with_blend(P.x, P.y, color);
                }
            }
        }
    }
}