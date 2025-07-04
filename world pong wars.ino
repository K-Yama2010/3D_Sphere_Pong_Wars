#include <Arduino.h>
#include <M5Unified.h>
#include <vector>
#include <algorithm>
#include <array>

// =================================================================
// 設定 (Configuration)
// =================================================================

// ボールのペア数を設定します (1～　数十にすると固まりはしませんがfps下がります)
const int NUM_AGENT_PAIRS = 3;

// 反射角度のランダム性の強さ。0にすると完全な反射になる。
// 少し値を加えることで、ボールの動きが単調になるのを防ぐ。
const float REFLECTION_RANDOMNESS = 0.15f;


const int GRID_SIZE = 8;
const float SPHERE_RADIUS = 100.0f; // 球の半径
const float BALL_SPEED = 6.0f;
const float BALL_SIZE = 6.0f; // ボールポリゴンの大きさ

// スプライト（描画領域）のサイズを定義
const int CANVAS_WIDTH = 200;
const int CANVAS_HEIGHT = 200;

// チームID定義
const int TEAM_GREEN = 0; // 緑陣地
const int TEAM_TRANSPARENT = 1; // 無色透明陣地

// 色定義
const uint16_t COLOR_GREEN = TFT_GREEN;
const uint16_t COLOR_DARK_GREEN = M5.Lcd.color565(0, 100, 0); // 暗い緑
const uint16_t COLOR_BLACK = TFT_BLACK; // 背景色

// カメラ設定
const float CAMERA_Z = SPHERE_RADIUS * 3.0f;
const float FOV = 200.0f;

// =================================================================
// 3D演算用の構造体とヘルパー関数
// =================================================================
struct Vec3 {
    float x, y, z;
    Vec3(float x=0, float y=0, float z=0) : x(x), y(y), z(z) {}
    float magnitude() const { return sqrt(x*x + y*y + z*z); }
    void normalize() { float mag = magnitude(); if (mag > 0) { x /= mag; y /= mag; z /= mag; } }
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
};
struct Vec2 { float x, y; };

struct Quad {
    int vertex_indices[4];
    int team_id;
    Vec3 center;
};

struct Agent {
    int id;
    Vec3 pos;
    Vec3 vel;
    int current_quad_idx;
};

float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 cross(const Vec3& a, const Vec3& b) { return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }

// 2Dの点が三角形の内部にあるかチェックするヘルパー関数
float sign(const Vec2& p1, const Vec2& p2, const Vec2& p3) {
    return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}
bool is_point_in_triangle(const Vec2& pt, const Vec2& v1, const Vec2& v2, const Vec2& v3) {
    float d1 = sign(pt, v1, v2);
    float d2 = sign(pt, v2, v3);
    float d3 = sign(pt, v3, v1);
    bool has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    bool has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
    return !(has_neg && has_pos);
}

// =================================================================
// グローバル変数
// =================================================================
M5Canvas canvas(&M5.Display);
std::vector<Vec3> vertices;
std::vector<Quad> quads;
std::vector<Agent> agents;
float rotationY = 0.0f;

// =================================================================
// クアッドスフィア生成
// =================================================================
void createQuadSphere() {
    vertices.clear(); quads.clear();
    Vec3 face_normals[] = { Vec3(1,0,0), Vec3(-1,0,0), Vec3(0,1,0), Vec3(0,-1,0), Vec3(0,0,1), Vec3(0,0,-1) };
    Vec3 face_tangents[] = { Vec3(0,1,0), Vec3(0,1,0), Vec3(1,0,0), Vec3(1,0,0), Vec3(1,0,0), Vec3(1,0,0) };
    int vertex_offset = 0;

    for (int f = 0; f < 6; ++f) {
        Vec3 normal = face_normals[f];
        Vec3 tangent = face_tangents[f];
        Vec3 bitangent = cross(normal, tangent);
        for (int j = 0; j <= GRID_SIZE; ++j) {
            for (int i = 0; i <= GRID_SIZE; ++i) {
                float u = (float)i / GRID_SIZE * 2.0f - 1.0f;
                float v = (float)j / GRID_SIZE * 2.0f - 1.0f;
                Vec3 point = normal + tangent * u + bitangent * v;
                point.normalize();
                vertices.push_back(point * SPHERE_RADIUS);
            }
        }
        for (int j = 0; j < GRID_SIZE; ++j) {
            for (int i = 0; i < GRID_SIZE; ++i) {
                Quad q;
                q.vertex_indices[0] = vertex_offset + j * (GRID_SIZE + 1) + i;
                q.vertex_indices[1] = vertex_offset + j * (GRID_SIZE + 1) + (i + 1);
                q.vertex_indices[2] = vertex_offset + (j + 1) * (GRID_SIZE + 1) + (i + 1);
                q.vertex_indices[3] = vertex_offset + (j + 1) * (GRID_SIZE + 1) + i;
                q.team_id = (f < 3) ? TEAM_GREEN : TEAM_TRANSPARENT;
                q.center = (vertices[q.vertex_indices[0]] + vertices[q.vertex_indices[1]] + vertices[q.vertex_indices[2]] + vertices[q.vertex_indices[3]]) * 0.25f;
                quads.push_back(q);
            }
        }
        vertex_offset += (GRID_SIZE + 1) * (GRID_SIZE + 1);
    }
}

int findClosestQuadIdx(const Vec3& pos) {
    float min_dist_sq = 1e9;
    int closest_idx = -1;
    for (size_t i = 0; i < quads.size(); ++i) {
        Vec3 diff = pos - quads[i].center;
        float dist_sq = dot(diff, diff);
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            closest_idx = i;
        }
    }
    return closest_idx;
}

// =================================================================
// セットアップ
// =================================================================
void setup(void) {
    auto cfg = M5.config();
    M5.begin(cfg);
    
    canvas.createSprite(CANVAS_WIDTH, CANVAS_HEIGHT);
    M5.Display.setBrightness(128);

    // 乱数の初期化
    randomSeed(esp_random());

    createQuadSphere();

    agents.resize(NUM_AGENT_PAIRS * 2);

    for (int i = 0; i < NUM_AGENT_PAIRS; ++i) {
        float angle_offset = (float)i / NUM_AGENT_PAIRS * TWO_PI;
        float y_offset = sin((float)i * PI / 2.0f) * 0.5f;

        int agent_idx1 = i * 2;
        agents[agent_idx1].id = TEAM_TRANSPARENT;
        agents[agent_idx1].pos = Vec3(-SPHERE_RADIUS * cos(angle_offset), 0, -SPHERE_RADIUS * sin(angle_offset));
        agents[agent_idx1].vel = Vec3(sin(angle_offset) * 0.5f, BALL_SPEED, -cos(angle_offset) * 0.5f + y_offset);

        int agent_idx2 = i * 2 + 1;
        agents[agent_idx2].id = TEAM_GREEN;
        agents[agent_idx2].pos = Vec3(SPHERE_RADIUS * cos(angle_offset), 0, SPHERE_RADIUS * sin(angle_offset));
        agents[agent_idx2].vel = Vec3(-sin(angle_offset) * 0.5f, -BALL_SPEED, cos(angle_offset) * 0.5f - y_offset);
    }
    
    for(auto& agent : agents) {
        agent.pos.normalize(); agent.pos = agent.pos * SPHERE_RADIUS;
        agent.current_quad_idx = findClosestQuadIdx(agent.pos);
    }
}

// =================================================================
// メインループ (Loop)
// =================================================================
void loop(void) {
    M5.update();
    canvas.fillSprite(TFT_BLACK);
    rotationY += 0.015f;
    if (rotationY > TWO_PI) rotationY -= TWO_PI;

    // 1. エージェントのロジック更新
    for (auto& agent : agents) {
        Vec3 next_pos = agent.pos + agent.vel;
        next_pos.normalize(); next_pos = next_pos * SPHERE_RADIUS;
        Vec3 new_vel = next_pos - agent.pos;
        new_vel.normalize(); agent.vel = new_vel * BALL_SPEED;
        int next_quad_idx = findClosestQuadIdx(next_pos);
        if (next_quad_idx != -1 && agent.current_quad_idx != -1) {
            if (quads[next_quad_idx].team_id != agent.id) {
                quads[next_quad_idx].team_id = agent.id;
                Vec3 current_quad_center = quads[agent.current_quad_idx].center;
                Vec3 next_quad_center = quads[next_quad_idx].center;
                Vec3 wall_normal = (next_quad_center - current_quad_center);
                wall_normal.normalize();

                // 反射ベクトルにランダムな要素を加える
                if (REFLECTION_RANDOMNESS > 0) {
                    float r_x = ((float)random(-100, 101) / 100.0f) * REFLECTION_RANDOMNESS;
                    float r_y = ((float)random(-100, 101) / 100.0f) * REFLECTION_RANDOMNESS;
                    float r_z = ((float)random(-100, 101) / 100.0f) * REFLECTION_RANDOMNESS;
                    wall_normal = wall_normal + Vec3(r_x, r_y, r_z);
                    wall_normal.normalize(); // 再度正規化
                }
                agent.vel = agent.vel - wall_normal * (2 * dot(agent.vel, wall_normal));       
            }
        }
        agent.pos = agent.pos + agent.vel;
        agent.pos.normalize(); agent.pos = agent.pos * SPHERE_RADIUS;
        agent.current_quad_idx = findClosestQuadIdx(agent.pos);
    }

    // 2. 3D->2D投影の準備
    std::vector<Vec2> projected_vertices(vertices.size());
    std::vector<Vec3> rotated_vertices(vertices.size());
    
    for (size_t i = 0; i < vertices.size(); ++i) {
        const auto& v = vertices[i];
        Vec3& rotated_v = rotated_vertices[i];
        rotated_v.x = v.x * cos(rotationY) - v.z * sin(rotationY);
        rotated_v.y = v.y;
        rotated_v.z = v.x * sin(rotationY) + v.z * cos(rotationY);
        rotated_v.z = rotated_v.z + CAMERA_Z; 
        if (rotated_v.z <= 0) {
            projected_vertices[i].x = -9999;
            projected_vertices[i].y = -9999;
            continue;
        }
        float scale = FOV / rotated_v.z;
        projected_vertices[i].x = rotated_v.x * scale + canvas.width() / 2;
        projected_vertices[i].y = rotated_v.y * scale + canvas.height() / 2;
    }

    // 3. 描画処理
    std::vector<std::pair<float, int>> z_sorted_quads;
    for(size_t i = 0; i < quads.size(); ++i) {
        Vec3 center_rotated;
        center_rotated.x = quads[i].center.x * cos(rotationY) - quads[i].center.z * sin(rotationY);
        center_rotated.y = quads[i].center.y;
        center_rotated.z = quads[i].center.x * sin(rotationY) + quads[i].center.z * cos(rotationY);
        z_sorted_quads.push_back({center_rotated.z, i});
    }
    std::sort(z_sorted_quads.begin(), z_sorted_quads.end(), std::greater<std::pair<float, int>>());

    std::vector<std::array<Vec2, 4>> visible_front_green_quads_p;

    for (const auto& p : z_sorted_quads) {
        int i = p.second;
        if (quads[i].team_id == TEAM_TRANSPARENT) {
            continue;
        }
        Vec3 r_v[4]; 
        bool quad_visible = true;
        for(int j=0; j<4; ++j) {
            r_v[j] = rotated_vertices[quads[i].vertex_indices[j]];
            if (projected_vertices[quads[i].vertex_indices[j]].x == -9999) {
                quad_visible = false;
                break;
            }
        }
        if (!quad_visible) continue;
        Vec3 edge1 = r_v[1] - r_v[0];
        Vec3 edge2 = r_v[2] - r_v[0];
        Vec3 normal = cross(edge1, edge2);
        Vec3 view_vector = r_v[0] * -1.0f;
        bool is_backface = dot(normal, view_vector) < 0;
        uint16_t quad_color = is_backface ? COLOR_DARK_GREEN : COLOR_GREEN;
        Vec2 proj_p[4];
        for(int j=0; j<4; ++j) {
            proj_p[j] = projected_vertices[quads[i].vertex_indices[j]];
        }
        if (!is_backface) {
            visible_front_green_quads_p.push_back({proj_p[0], proj_p[1], proj_p[2], proj_p[3]});
        }
        canvas.fillTriangle(proj_p[0].x, proj_p[0].y, proj_p[1].x, proj_p[1].y, proj_p[2].x, proj_p[2].y, quad_color);
        canvas.fillTriangle(proj_p[0].x, proj_p[0].y, proj_p[2].x, proj_p[2].y, proj_p[3].x, proj_p[3].y, quad_color);
    }

    // 4. ボールの描画
    for (const auto& agent : agents) {
        Vec3 ball_pos_rotated;
        ball_pos_rotated.x = agent.pos.x * cos(rotationY) - agent.pos.z * sin(rotationY);
        ball_pos_rotated.y = agent.pos.y;
        float ball_z_rotated_local = agent.pos.x * sin(rotationY) + agent.pos.z * cos(rotationY);
        ball_pos_rotated.z = ball_z_rotated_local + CAMERA_Z;

        if (ball_pos_rotated.z <= 0) continue;

        bool is_ball_on_back = ball_z_rotated_local > 0;

        bool is_occluded = false;
        if (is_ball_on_back) {
            float center_scale = FOV / ball_pos_rotated.z;
            Vec2 ball_p_center = {
                ball_pos_rotated.x * center_scale + canvas.width() / 2,
                ball_pos_rotated.y * center_scale + canvas.height() / 2
            };
            for (const auto& quad_p : visible_front_green_quads_p) {
                if (is_point_in_triangle(ball_p_center, quad_p[0], quad_p[1], quad_p[2]) ||
                    is_point_in_triangle(ball_p_center, quad_p[0], quad_p[2], quad_p[3])) {
                    is_occluded = true;
                    break;
                }
            }
        }

        if (is_ball_on_back && is_occluded) {
            continue; 
        }

        uint16_t ball_color;
        if (agent.id == TEAM_GREEN) {
            Vec3 antipodal_pos = agent.pos * -1.0f;
            int behind_quad_idx = findClosestQuadIdx(antipodal_pos);
            if (behind_quad_idx != -1 && quads[behind_quad_idx].team_id == TEAM_GREEN) {
                ball_color = COLOR_DARK_GREEN;
            } else {
                ball_color = COLOR_BLACK;
            }
        } else {
            ball_color = is_ball_on_back ? COLOR_DARK_GREEN : COLOR_GREEN;
        }

        float scale = FOV / ball_pos_rotated.z;
        float current_ball_size = BALL_SIZE * scale;
        Vec3 normal = agent.pos; normal.normalize();
        Vec3 world_up = (abs(normal.y) > 0.9) ? Vec3(1,0,0) : Vec3(0,1,0);
        Vec3 tangent = cross(world_up, normal); tangent.normalize();
        Vec3 bitangent = cross(normal, tangent); bitangent.normalize();
        Vec3 ball_v[4];
        ball_v[0] = agent.pos - tangent * current_ball_size + bitangent * current_ball_size;
        ball_v[1] = agent.pos + tangent * current_ball_size + bitangent * current_ball_size;
        ball_v[2] = agent.pos + tangent * current_ball_size - bitangent * current_ball_size;
        ball_v[3] = agent.pos - tangent * current_ball_size - bitangent * current_ball_size;
        Vec2 ball_p[4]; Vec3 ball_r_v[4];
        bool ball_visible = true;
        for(int j=0; j<4; ++j) {
            ball_v[j].normalize(); ball_v[j] = ball_v[j] * SPHERE_RADIUS;
            ball_r_v[j].x = ball_v[j].x * cos(rotationY) - ball_v[j].z * sin(rotationY);
            ball_r_v[j].y = ball_v[j].y;
            ball_r_v[j].z = ball_v[j].x * sin(rotationY) + ball_v[j].z * cos(rotationY);
            ball_r_v[j].z += CAMERA_Z;
            if (ball_r_v[j].z <= 0) { ball_visible = false; break; }
            float vertex_scale = FOV / ball_r_v[j].z;
            ball_p[j].x = ball_r_v[j].x * vertex_scale + canvas.width() / 2;
            ball_p[j].y = ball_r_v[j].y * vertex_scale + canvas.height() / 2;
        }
        if (ball_visible) {
            canvas.fillTriangle(ball_p[0].x, ball_p[0].y, ball_p[1].x, ball_p[1].y, ball_p[2].x, ball_p[2].y, ball_color);
            canvas.fillTriangle(ball_p[0].x, ball_p[0].y, ball_p[2].x, ball_p[2].y, ball_p[3].x, ball_p[3].y, ball_color);
        }
    }
    
    canvas.pushSprite((M5.Display.width() - CANVAS_WIDTH) / 2, (M5.Display.height() - CANVAS_HEIGHT) / 2);
}

#if defined(ESP_PLATFORM) && !defined(ARDUINO)
extern "C" {
int app_main(int, char **) {
    setup();
    for(;;) { loop(); }
    return 0;
}
}
#endif