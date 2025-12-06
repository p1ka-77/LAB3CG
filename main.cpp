#include <vector>
#include <limits>
#include <iostream>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include "our_gl.h"

Model* model = NULL;
float* shadowbuffer = NULL;

const int width = 800;
const int height = 800;

Vec3f light_dir(1, 2, 1);      // Свет сверху-сбоку
Vec3f eye(2, 1.5, 4);         // Камера снаружи куба
Vec3f center(0, 0, 0);        // Смотрим в центр куба
Vec3f up(0, 1, 0);

// ШЕЙДЕР ДЛЯ ПОЛУПРОЗРАЧНОГО КУБА
struct TransparentCubeShader : public IShader {
    mat<3, 3, float> varying_tri;
    float cube_size;
    TGAColor cube_color;
    float transparency;

    TransparentCubeShader(float size, TGAColor color, float alpha)
        : varying_tri(), cube_size(size), cube_color(color), transparency(alpha) {
    }

    virtual Vec4f vertex(int iface, int nthvert) {
        // Вершины куба
        Vec3f cube_verts[8] = {
            {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
            {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}
        };

        // Грани куба (12 треугольников)
        int faces[12][3] = {
            {0,1,2}, {0,2,3}, // задняя
            {4,5,6}, {4,6,7}, // передняя
            {0,3,7}, {0,7,4}, // левая
            {1,2,6}, {1,6,5}, // правая
            {0,1,5}, {0,5,4}, // нижняя
            {3,2,6}, {3,6,7}  // верхняя
        };

        int idx = faces[iface][nthvert];
        Vec3f v = cube_verts[idx] * cube_size;

        Vec4f gl_Vertex = Viewport * Projection * ModelView * embed<4>(v);
        varying_tri.set_col(nthvert, proj<3>(gl_Vertex / gl_Vertex[3]));
        return gl_Vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor& color) {
        // Простое освещение для куба
        Vec3f light = light_dir.normalize();
        Vec3f normal(0, 0, 1);  // Упрощенная нормаль

        float intensity = std::max(0.1f, normal * light);

        // Синий цвет с прозрачностью
        color = cube_color;
        for (int i = 0; i < 3; i++) {
            color[i] = std::min<float>(cube_color[i] * intensity, 255);
        }
        color[3] = (unsigned char)(transparency * 255);  // Альфа-канал

        return false;
    }
};

// ШЕЙДЕР ДЛЯ МОДЕЛИ (голова)
struct ModelShader : public IShader {
    mat<4, 4, float> uniform_M;
    mat<4, 4, float> uniform_MIT;
    mat<4, 4, float> uniform_Mshadow;
    mat<2, 3, float> varying_uv;
    mat<3, 3, float> varying_tri;

    ModelShader(Matrix M, Matrix MIT, Matrix MS)
        : uniform_M(M), uniform_MIT(MIT), uniform_Mshadow(MS),
        varying_uv(), varying_tri() {
    }

    virtual Vec4f vertex(int iface, int nthvert) {
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));
        Vec4f gl_Vertex = Viewport * Projection * ModelView *
            embed<4>(model->vert(iface, nthvert));
        varying_tri.set_col(nthvert, proj<3>(gl_Vertex / gl_Vertex[3]));
        return gl_Vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor& color) {
        // Упрощенный шейдер без теней, чтобы избежать артефактов
        Vec2f uv = varying_uv * bar;

        // Нормаль из модели
        Vec3f n = model->normal(uv).normalize();

        // Освещение
        Vec3f l = light_dir.normalize();
        float diff = std::max(0.0f, n * l);

        // Спекуляр
        Vec3f r = (n * (n * l * 2.0f) - l).normalize();
        float spec = pow(std::max(r.z, 0.0f), model->specular(uv));

        // Цвет из текстуры
        TGAColor c = model->diffuse(uv);

        // Итоговый цвет
        for (int i = 0; i < 3; i++) {
            color[i] = std::min<float>(c[i] * (0.3f + 0.7f * diff) + 100 * spec, 255);
        }
        color[3] = 255;  // Непрозрачная модель

        return false;
    }
};

struct DepthShader : public IShader {
    mat<3, 3, float> varying_tri;

    DepthShader() : varying_tri() {}

    virtual Vec4f vertex(int iface, int nthvert) {
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert));
        gl_Vertex = Viewport * Projection * ModelView * gl_Vertex;
        varying_tri.set_col(nthvert, proj<3>(gl_Vertex / gl_Vertex[3]));
        return gl_Vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor& color) {
        Vec3f p = varying_tri * bar;
        color = TGAColor(255, 255, 255) * (p.z / depth);
        return false;
    }
};

// ФУНКЦИЯ ДЛЯ РЕНДЕРИНГА КУБА
void renderTransparentCube(TGAImage& image, float* zbuffer, float size,
    TGAColor color, float transparency) {
    TransparentCubeShader cube_shader(size, color, transparency);

    // Рендерим все 12 треугольников куба
    for (int i = 0; i < 12; i++) {
        Vec4f screen_coords[3];
        for (int j = 0; j < 3; j++) {
            screen_coords[j] = cube_shader.vertex(i, j);
        }
        triangle(screen_coords, cube_shader, image, zbuffer);
    }
}

// ФУНКЦИЯ ДЛЯ РЕНДЕРИНГА КАРКАСА КУБА (опционально)
void renderCubeWireframe(TGAImage& image, float size, TGAColor color) {
    // Вершины куба
    Vec3f cube_verts[8] = {
        {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
        {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}
    };

    // Ребра куба
    int edges[12][2] = {
        {0,1}, {1,2}, {2,3}, {3,0},  // задняя грань
        {4,5}, {5,6}, {6,7}, {7,4},  // передняя грань  
        {0,4}, {1,5}, {2,6}, {3,7}   // боковые ребра
    };

    // Проецируем вершины в 2D
    Vec2i proj_verts[8];
    for (int i = 0; i < 8; i++) {
        Vec3f v = cube_verts[i] * size;
        Vec4f p = Viewport * Projection * ModelView * embed<4>(v);
        p = p / p[3];
        proj_verts[i] = Vec2i(p[0], p[1]);
    }

    // Функция для рисования линии (упрощенная)
    auto draw_line = [&](int x0, int y0, int x1, int y1) {
        bool steep = false;
        if (std::abs(x0 - x1) < std::abs(y0 - y1)) {
            std::swap(x0, y0);
            std::swap(x1, y1);
            steep = true;
        }
        if (x0 > x1) {
            std::swap(x0, x1);
            std::swap(y0, y1);
        }

        int dx = x1 - x0;
        int dy = y1 - y0;
        int derror2 = std::abs(dy) * 2;
        int error2 = 0;
        int y = y0;

        for (int x = x0; x <= x1; x++) {
            if (steep) {
                if (y >= 0 && y < width && x >= 0 && x < height)
                    image.set(y, x, color);
            }
            else {
                if (x >= 0 && x < width && y >= 0 && y < height)
                    image.set(x, y, color);
            }
            error2 += derror2;
            if (error2 > dx) {
                y += (y1 > y0 ? 1 : -1);
                error2 -= dx * 2;
            }
        }
        };

    // Рисуем все ребра
    for (int i = 0; i < 12; i++) {
        Vec2i p1 = proj_verts[edges[i][0]];
        Vec2i p2 = proj_verts[edges[i][1]];
        draw_line(p1.x, p1.y, p2.x, p2.y);
    }
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " obj/model.obj" << std::endl;
        return 1;
    }

    // Загружаем модель
    model = new Model(argv[1]);
    light_dir.normalize();

    // Создаем буферы
    float* zbuffer = new float[width * height];
    for (int i = 0; i < width * height; i++) {
        zbuffer[i] = -std::numeric_limits<float>::max();
    }

    // Создаем изображение
    TGAImage frame(width, height, TGAImage::RGBA);

    // Заливаем фон (светло-серый)
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            frame.set(x, y, TGAColor(240, 240, 240, 255));
        }
    }

    std::cout << "Начинаем рендеринг сцены..." << std::endl;
    std::cout << "Модель: " << argv[1] << std::endl;
    std::cout << "Вершин: " << model->nverts() << std::endl;
    std::cout << "Полигонов: " << model->nfaces() << std::endl;

    // 1. Настраиваем камеру и проекцию
    lookat(eye, center, up);
    viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
    projection(-1.0f / (eye - center).norm());

    std::cout << "Камера: eye(" << eye.x << ", " << eye.y << ", " << eye.z << ")" << std::endl;
    std::cout << "Центр: (" << center.x << ", " << center.y << ", " << center.z << ")" << std::endl;

    // 2. ПЕРВЫМ рендерим модель (африканскую голову)
    std::cout << "Рендерим модель..." << std::endl;
    ModelShader model_shader(ModelView, (Projection * ModelView).invert_transpose(),
        Matrix::identity());

    Vec4f screen_coords[3];
    int triangles_rendered = 0;

    for (int i = 0; i < model->nfaces(); i++) {
        for (int j = 0; j < 3; j++) {
            screen_coords[j] = model_shader.vertex(i, j);
        }
        triangle(screen_coords, model_shader, frame, zbuffer);
        triangles_rendered++;

        // Выводим прогресс каждые 500 треугольников
        if (triangles_rendered % 500 == 0) {
            std::cout << "  Отрендерено треугольников: " << triangles_rendered
                << " / " << model->nfaces() << std::endl;
        }
    }

    std::cout << "Модель отрендерена. Треугольников: " << triangles_rendered << std::endl;

    // 3. ВТОРЫМ рендерим полупрозрачный куб ВОКРУГ модели
    std::cout << "Рендерим полупрозрачный куб..." << std::endl;

    // Кубик больше модели (голова должна помещаться внутри)
    float cube_size = 1.2f;

    // Полупрозрачный голубой куб (как стекло аквариума)
    TGAColor cube_color(100, 150, 255, 100);  // Голубой, 40% прозрачность

    renderTransparentCube(frame, zbuffer, cube_size, cube_color, 0.4f);

    // 4. Опционально: добавляем каркас куба для лучшей видимости граней
    std::cout << "Добавляем каркас куба..." << std::endl;
    renderCubeWireframe(frame, cube_size, TGAColor(0, 0, 200, 255));  // Темно-синий каркас

    // Сохраняем результат
    frame.flip_vertically();
    frame.write_tga_file("aquarium.tga");

    std::cout << "\nРендеринг завершен!" << std::endl;
    std::cout << "Файл сохранен: aquarium.tga" << std::endl;
    std::cout << "Размер: " << width << "x" << height << std::endl;
    std::cout << "\nОписание сцены:" << std::endl;
    std::cout << "- Африканская голова внутри полупрозрачного куба" << std::endl;
    std::cout << "- Кубик как аквариум: голубые прозрачные грани + синий каркас" << std::endl;
    std::cout << "- Камера снаружи: смотрит на сцену под углом" << std::endl;

    delete model;
    delete[] zbuffer;

    return 0;
}