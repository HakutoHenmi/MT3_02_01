/*****************************************************************
 *  MT3 02-01  球と球の衝突判定サンプル（改訂版）
 *  - ImGui で 2 球の中心 / 半径を操作
 *  - 当たり判定で色変更
 *  - カメラ : WSAD / ↑↓ で移動、右ドラッグで視点回転
 *****************************************************************/

#define _USE_MATH_DEFINES
#include <Novice.h>
#include <algorithm>
#include <cmath>
#include <imgui.h>

constexpr char kWindowTitle[] = "LE2B_20_ヘンミ_ハクト";
constexpr float kPI = 3.14159265358979323846f; // float 版 π

//================================================================
// 基本データ構造
//================================================================
struct Vector3 {
  float x, y, z;
}; // 3 次元ベクトル
struct Matrix4x4 {
  float m[4][4];
}; // 4×4 行列
struct Sphere {
  Vector3 center;
  float radius;
};

//----------------------------------------------------------------
// 便利なベクトル演算
//----------------------------------------------------------------
inline Vector3 Add(const Vector3 &a, const Vector3 &b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
inline Vector3 Subtract(const Vector3 &a, const Vector3 &b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline Vector3 Scale(const Vector3 &v, float s) {
  return {v.x * s, v.y * s, v.z * s};
}
inline float Dot(const Vector3 &a, const Vector3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline float LengthSq(const Vector3 &v) { return Dot(v, v); }
inline float Length(const Vector3 &v) { return std::sqrt(LengthSq(v)); }

//----------------------------------------------------------------
// 球と球の衝突判定
//----------------------------------------------------------------
inline bool IsCollision(const Sphere &s1, const Sphere &s2) {
  float rSum = s1.radius + s2.radius;
  return LengthSq(Subtract(s1.center, s2.center)) <= rSum * rSum;
}

//----------------------------------------------------------------
// 行列ユーティリティ
//----------------------------------------------------------------
Matrix4x4 MakeIdentity() {
  Matrix4x4 r{};
  for (int i = 0; i < 4; i++)
    r.m[i][i] = 1.0f;
  return r;
}
Matrix4x4 MakeTranslate(const Vector3 &t) {
  Matrix4x4 r = MakeIdentity();
  r.m[3][0] = t.x;
  r.m[3][1] = t.y;
  r.m[3][2] = t.z;
  return r;
}
Matrix4x4 MakeRotateX(float a) {
  Matrix4x4 r = MakeIdentity();
  r.m[1][1] = cosf(a);
  r.m[1][2] = sinf(a);
  r.m[2][1] = -sinf(a);
  r.m[2][2] = cosf(a);
  return r;
}
Matrix4x4 MakeRotateY(float a) {
  Matrix4x4 r = MakeIdentity();
  r.m[0][0] = cosf(a);
  r.m[0][2] = -sinf(a);
  r.m[2][0] = sinf(a);
  r.m[2][2] = cosf(a);
  return r;
}
Matrix4x4 Mul(const Matrix4x4 &A, const Matrix4x4 &B) {
  Matrix4x4 r{};
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      for (int k = 0; k < 4; k++)
        r.m[i][j] += A.m[i][k] * B.m[k][j];
  return r;
}
Vector3 Transform(const Vector3 &v, const Matrix4x4 &m) {
  Vector3 r;
  float w;
  r.x = v.x * m.m[0][0] + v.y * m.m[1][0] + v.z * m.m[2][0] + m.m[3][0];
  r.y = v.x * m.m[0][1] + v.y * m.m[1][1] + v.z * m.m[2][1] + m.m[3][1];
  r.z = v.x * m.m[0][2] + v.y * m.m[1][2] + v.z * m.m[2][2] + m.m[3][2];
  w = v.x * m.m[0][3] + v.y * m.m[1][3] + v.z * m.m[2][3] + m.m[3][3];
  r.x /= w;
  r.y /= w;
  r.z /= w;
  return r;
}
Matrix4x4 MakePerspectiveFov(float fovY, float aspect, float n, float f) {
  Matrix4x4 r{};
  float s = 1.0f / tanf(fovY * 0.5f);
  r.m[0][0] = s / aspect;
  r.m[1][1] = s;
  r.m[2][2] = f / (n - f);
  r.m[2][3] = -1.0f;
  r.m[3][2] = (n * f) / (n - f);
  return r;
}
Matrix4x4 MakeViewport(float l, float t, float w, float h, float minD,
                       float maxD) {
  Matrix4x4 r{};
  r.m[0][0] = w * 0.5f;
  r.m[1][1] = h * 0.5f;
  r.m[2][2] = maxD - minD;
  r.m[3][0] = l + w * 0.5f;
  r.m[3][1] = t + h * 0.5f;
  r.m[3][2] = minD;
  r.m[3][3] = 1.0f;
  return r;
}

//----------------------------------------------------------------
// 補助描画（グリッド／ワイヤ球）
//----------------------------------------------------------------
void DrawGrid(const Matrix4x4 &vp, const Matrix4x4 &vm) {
  constexpr float half = 4.0f;
  constexpr int div = 20;
  const float step = (half * 2.0f) / div;
  for (int i = 0; i <= div; i++) {
    float o = -half + i * step;
    Vector3 a{o, 0, -half}, b{o, 0, half};
    a = Transform(Transform(a, vp), vm);
    b = Transform(Transform(b, vp), vm);
    Novice::DrawLine((int)a.x, (int)a.y, (int)b.x, (int)b.y, 0x444444FF);

    a = {-half, 0, o};
    b = {half, 0, o};
    a = Transform(Transform(a, vp), vm);
    b = Transform(Transform(b, vp), vm);
    Novice::DrawLine((int)a.x, (int)a.y, (int)b.x, (int)b.y, 0x444444FF);
  }
}
void DrawSphereWire(const Sphere &sp, const Matrix4x4 &vp, const Matrix4x4 &vm,
                    uint32_t color) {
  constexpr int latDiv = 12, lonDiv = 24;
  for (int lat = 0; lat <= latDiv; ++lat) {
    float latA = (-0.5f + static_cast<float>(lat) / latDiv) * kPI;
    for (int lon = 0; lon < lonDiv; ++lon) {
      float lonA = 2.0f * kPI * static_cast<float>(lon) / lonDiv;
      float lonB = 2.0f * kPI * static_cast<float>(lon + 1) / lonDiv;

      Vector3 p1{sp.radius * cosf(latA) * cosf(lonA) + sp.center.x,
                 sp.radius * sinf(latA) + sp.center.y,
                 sp.radius * cosf(latA) * sinf(lonA) + sp.center.z};
      Vector3 p2{sp.radius * cosf(latA) * cosf(lonB) + sp.center.x,
                 sp.radius * sinf(latA) + sp.center.y,
                 sp.radius * cosf(latA) * sinf(lonB) + sp.center.z};

      p1 = Transform(Transform(p1, vp), vm);
      p2 = Transform(Transform(p2, vp), vm);
      Novice::DrawLine((int)p1.x, (int)p1.y, (int)p2.x, (int)p2.y, color);
    }
  }
}

//------------------------------------------------------------------
// エントリポイント
//------------------------------------------------------------------
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance,
                   LPSTR lpCmdLine, int nCmdShow) {

  (void)hInstance;
  (void)hPrevInstance;
  (void)lpCmdLine;
  (void)nCmdShow;

  Novice::Initialize(kWindowTitle, 1280, 720);
  char keys[256]{}, preKeys[256]{};

  //============= 初期パラメータ =============
  Vector3 camPos{0.0f, 2.0f, -8.0f}; // カメラ位置
  Vector3 camRot{0.0f, 0.0f, 0.0f};  // カメラ回転 (pitch, yaw)

  Sphere sphere1{{-1.5f, 0.5f, 0.0f}, 1.0f};
  Sphere sphere2{{1.0f, 0.5f, 0.5f}, 1.2f};

  // マウス位置保持（右ドラッグ用）
  int prevMouseX = 0, prevMouseY = 0;

  //------------- メインループ -------------
  while (Novice::ProcessMessage() == 0) {
    Novice::BeginFrame();
    memcpy(preKeys, keys, 256);
    Novice::GetHitKeyStateAll(keys);

    //--------------------------------------------------
    // カメラ移動 : WSAD / ↑↓
    //--------------------------------------------------
    constexpr float moveSpd = 0.05f, rotSpd = 0.005f;
    if (keys[DIK_W])
      camPos.z += moveSpd;
    if (keys[DIK_S])
      camPos.z -= moveSpd;
    if (keys[DIK_A])
      camPos.x -= moveSpd;
    if (keys[DIK_D])
      camPos.x += moveSpd;
    if (keys[DIK_UP])
      camPos.y += moveSpd;
    if (keys[DIK_DOWN])
      camPos.y -= moveSpd;

    //--------------------------------------------------
    // マウス右ドラッグで視点回転
    //--------------------------------------------------
    int mouseX, mouseY;
    Novice::GetMousePosition(&mouseX, &mouseY);
    if (Novice::IsPressMouse(1)) { // 右ボタン押下中
      int dx = mouseX - prevMouseX;
      int dy = mouseY - prevMouseY;
      camRot.y += dx * rotSpd;
      camRot.x += dy * rotSpd;
      camRot.x = std::clamp(camRot.x, -kPI * 0.49f,
                            kPI * 0.49f); // 視線が裏返らないよう制限
    }
    prevMouseX = mouseX;
    prevMouseY = mouseY;

    //--------------------------------------------------
    // ImGui UI
    //--------------------------------------------------
    ImGui::Begin("Control");
    ImGui::Text("Camera");
    ImGui::DragFloat3("Pos", &camPos.x, 0.01f);
    ImGui::DragFloat3("Rot", &camRot.x, 0.01f);
    ImGui::Separator();
    ImGui::Text("Sphere 1");
    ImGui::DragFloat3("S1 Ctr", &sphere1.center.x, 0.01f);
    ImGui::DragFloat("S1 Rad", &sphere1.radius, 0.01f, 0.01f);
    ImGui::Text("Sphere 2");
    ImGui::DragFloat3("S2 Ctr", &sphere2.center.x, 0.01f);
    ImGui::DragFloat("S2 Rad", &sphere2.radius, 0.01f, 0.01f);
    bool hit = IsCollision(sphere1, sphere2);
    ImGui::Separator();
    ImGui::Text("Collision : %s", hit ? "YES" : "NO");
    ImGui::End();

    //--------------------------------------------------
    // 行列計算
    //--------------------------------------------------
    Matrix4x4 view = Mul(MakeRotateY(-camRot.y),
                         Mul(MakeRotateX(-camRot.x),
                             MakeTranslate({-camPos.x, -camPos.y, -camPos.z})));

    Matrix4x4 proj = MakePerspectiveFov(0.45f, 1280.0f / 720.0f, 0.1f, 100.0f);
    Matrix4x4 vp = Mul(view, proj);
    Matrix4x4 vpm = MakeViewport(0, 0, 1280, 720, 0.0f, 1.0f);

    //--------------------------------------------------
    // 描画
    //--------------------------------------------------
    DrawGrid(vp, vpm);

    uint32_t col1 = hit ? 0xFF4444FF : 0x4444FFFF; // 衝突時：赤
    uint32_t col2 = hit ? 0xFF4444FF : 0xFFFFFFFF;

    DrawSphereWire(sphere1, vp, vpm, col1);
    DrawSphereWire(sphere2, vp, vpm, col2);

    Novice::EndFrame();
    if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0)
      break;
  }

  Novice::Finalize();
  return 0;
}
