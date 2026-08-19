// urdf_fk_single.cpp
//
// 一个文件完成：
//   1. 读取 URDF；
//   2. 解析 link、joint、parent、child、origin、axis；
//   3. 输入一组关节值；
//   4. 计算 base_link 到目标末端 link 的正运动学；
//   5. 输出 4x4 矩阵和自定义的 XYZWPR。
//
// 只使用 C++14 标准库，不依赖 Qt、OpenCASCADE、OSG、Eigen、KDL 等库。
//
// 编译示例（MSVC）：
//   cl /EHsc /utf-8 urdf_fk_single.cpp
//
// 编译示例（GCC/MinGW）：
//   g++ -std=c++14 -O2 urdf_fk_single.cpp -o urdf_fk_single.exe

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

constexpr double kPi = 3.1415926535897932384626433832795;

double degToRad(double degree) {
    return degree * kPi / 180.0;
}

double radToDeg(double radian) {
    return radian * 180.0 / kPi;
}

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

Vec3 operator*(const Vec3& v, double scale) {
    return {v.x * scale, v.y * scale, v.z * scale};
}

double length(const Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vec3 normalized(const Vec3& v) {
    const double len = length(v);
    if (len < 1e-15) {
        throw std::runtime_error("关节 axis 长度为 0，无法构造旋转轴");
    }
    return {v.x / len, v.y / len, v.z / len};
}

struct Mat4 {
    double m[4][4]{};

    static Mat4 identity() {
        Mat4 result;
        for (int i = 0; i < 4; ++i) {
            result.m[i][i] = 1.0;
        }
        return result;
    }
};

// 本文件采用列向量约定：p_parent = T_parent_child * p_child。
// 因而 A * B 表示先执行 B，再执行 A。
Mat4 operator*(const Mat4& a, const Mat4& b) {
    Mat4 result;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            for (int k = 0; k < 4; ++k) {
                result.m[row][col] += a.m[row][k] * b.m[k][col];
            }
        }
    }
    return result;
}

Mat4 makeTranslation(const Vec3& xyz) {
    Mat4 result = Mat4::identity();
    result.m[0][3] = xyz.x;
    result.m[1][3] = xyz.y;
    result.m[2][3] = xyz.z;
    return result;
}

Mat4 makeRotationX(double angle) {
    Mat4 result = Mat4::identity();
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    result.m[1][1] = c;
    result.m[1][2] = -s;
    result.m[2][1] = s;
    result.m[2][2] = c;
    return result;
}

Mat4 makeRotationY(double angle) {
    Mat4 result = Mat4::identity();
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    result.m[0][0] = c;
    result.m[0][2] = s;
    result.m[2][0] = -s;
    result.m[2][2] = c;
    return result;
}

Mat4 makeRotationZ(double angle) {
    Mat4 result = Mat4::identity();
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    result.m[0][0] = c;
    result.m[0][1] = -s;
    result.m[1][0] = s;
    result.m[1][1] = c;
    return result;
}

// URDF 的 rpy 是固定轴 Roll(X)、Pitch(Y)、Yaw(Z)。
// 在列向量约定下：R = Rz(yaw) * Ry(pitch) * Rx(roll)。
Mat4 makeTransformFromXyzRpy(const Vec3& xyz, const Vec3& rpy) {
    return makeTranslation(xyz)
        * makeRotationZ(rpy.z)
        * makeRotationY(rpy.y)
        * makeRotationX(rpy.x);
}

// Rodrigues 公式：绕任意单位轴旋转。
// axis 属于 joint 坐标系；URDF 中 revolute/continuous 的 q 使用弧度。
Mat4 makeAxisRotation(const Vec3& rawAxis, double angle) {
    const Vec3 axis = normalized(rawAxis);
    const double x = axis.x;
    const double y = axis.y;
    const double z = axis.z;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double v = 1.0 - c;

    Mat4 result = Mat4::identity();
    result.m[0][0] = x * x * v + c;
    result.m[0][1] = x * y * v - z * s;
    result.m[0][2] = x * z * v + y * s;
    result.m[1][0] = y * x * v + z * s;
    result.m[1][1] = y * y * v + c;
    result.m[1][2] = y * z * v - x * s;
    result.m[2][0] = z * x * v - y * s;
    result.m[2][1] = z * y * v + x * s;
    result.m[2][2] = z * z * v + c;
    return result;
}

struct Joint {
    std::string name;
    std::string type;
    std::string parentLink;
    std::string childLink;
    Vec3 originXyz{0.0, 0.0, 0.0};  // URDF 标准单位：米
    Vec3 originRpy{0.0, 0.0, 0.0};  // URDF 标准单位：弧度
    Vec3 axis{1.0, 0.0, 0.0};       // URDF 未填写 axis 时默认 X 轴
};

struct RobotModel {
    std::unordered_set<std::string> links;
    std::vector<Joint> joints;
};

std::string readTextFile(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("无法打开 URDF 文件: " + path);
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

std::string removeXmlComments(std::string xml) {
    static const std::regex commentPattern(
        R"(<!--[\s\S]*?-->)",
        std::regex_constants::icase
    );
    return std::regex_replace(xml, commentPattern, "");
}

std::string getAttribute(
    const std::string& startTagAttributes,
    const std::string& attributeName,
    const std::string& defaultValue = "")
{
    // 同时支持 name="value" 和 name='value'。
    const std::regex pattern(
        "\\b" + attributeName + R"(\s*=\s*["']([^"']*)["'])",
        std::regex_constants::icase
    );
    std::smatch match;
    return std::regex_search(startTagAttributes, match, pattern)
        ? match[1].str()
        : defaultValue;
}

std::string findTagAttributes(const std::string& body, const std::string& tagName) {
    const std::regex pattern(
        "<\\s*" + tagName + R"(\b([^>]*)/?>)",
        std::regex_constants::icase
    );
    std::smatch match;
    return std::regex_search(body, match, pattern) ? match[1].str() : "";
}

Vec3 parseVec3(const std::string& text, const Vec3& defaultValue) {
    if (text.empty()) {
        return defaultValue;
    }

    std::istringstream stream(text);
    Vec3 result;
    if (!(stream >> result.x >> result.y >> result.z)) {
        throw std::runtime_error("无法解析三维向量: " + text);
    }
    return result;
}

RobotModel parseUrdf(const std::string& urdfPath) {
    const std::string xml = removeXmlComments(readTextFile(urdfPath));
    RobotModel model;

    // 读取 link 名称。这里只需要拓扑，不需要 visual/mesh。
    const std::regex linkPattern(
        R"(<\s*link\b([^>]*)>)",
        std::regex_constants::icase
    );
    for (std::sregex_iterator it(xml.begin(), xml.end(), linkPattern), end; it != end; ++it) {
        const std::string name = getAttribute((*it)[1].str(), "name");
        if (!name.empty()) {
            model.links.insert(name);
        }
    }

    // 读取完整 joint 块。标准 URDF 的 joint 是成对标签，不是自闭合标签。
    const std::regex jointPattern(
        R"(<\s*joint\b([^>]*)>([\s\S]*?)<\s*/\s*joint\s*>)",
        std::regex_constants::icase
    );

    for (std::sregex_iterator it(xml.begin(), xml.end(), jointPattern), end; it != end; ++it) {
        Joint joint;
        const std::string jointAttributes = (*it)[1].str();
        const std::string jointBody = (*it)[2].str();

        joint.name = getAttribute(jointAttributes, "name");
        joint.type = getAttribute(jointAttributes, "type", "fixed");

        const std::string parentTag = findTagAttributes(jointBody, "parent");
        const std::string childTag = findTagAttributes(jointBody, "child");
        const std::string originTag = findTagAttributes(jointBody, "origin");
        const std::string axisTag = findTagAttributes(jointBody, "axis");

        joint.parentLink = getAttribute(parentTag, "link");
        joint.childLink = getAttribute(childTag, "link");
        joint.originXyz = parseVec3(getAttribute(originTag, "xyz"), {0.0, 0.0, 0.0});
        joint.originRpy = parseVec3(getAttribute(originTag, "rpy"), {0.0, 0.0, 0.0});
        joint.axis = parseVec3(getAttribute(axisTag, "xyz"), {1.0, 0.0, 0.0});

        if (joint.name.empty() || joint.parentLink.empty() || joint.childLink.empty()) {
            throw std::runtime_error("URDF 中存在缺少 name/parent/child 的 joint");
        }

        model.links.insert(joint.parentLink);
        model.links.insert(joint.childLink);
        model.joints.push_back(std::move(joint));
    }

    if (model.joints.empty()) {
        throw std::runtime_error("URDF 中没有解析到 joint");
    }
    return model;
}

std::string findRootLink(const RobotModel& model) {
    std::unordered_set<std::string> childLinks;
    for (const Joint& joint : model.joints) {
        childLinks.insert(joint.childLink);
    }

    std::vector<std::string> roots;
    for (const std::string& link : model.links) {
        if (childLinks.count(link) == 0) {
            roots.push_back(link);
        }
    }

    if (roots.size() != 1) {
        std::ostringstream message;
        message << "URDF 应当只有一个根 link，当前找到 " << roots.size() << " 个";
        throw std::runtime_error(message.str());
    }
    return roots.front();
}

std::vector<const Joint*> buildChain(
    const RobotModel& model,
    const std::string& rootLink,
    const std::string& targetLink)
{
    std::unordered_map<std::string, const Joint*> childToJoint;
    for (const Joint& joint : model.joints) {
        if (!childToJoint.emplace(joint.childLink, &joint).second) {
            throw std::runtime_error("一个 child link 被多个 joint 指向: " + joint.childLink);
        }
    }

    std::vector<const Joint*> reversedChain;
    std::unordered_set<std::string> visited;
    std::string current = targetLink;

    while (current != rootLink) {
        if (!visited.insert(current).second) {
            throw std::runtime_error("URDF 运动链中检测到环: " + current);
        }

        const auto it = childToJoint.find(current);
        if (it == childToJoint.end()) {
            throw std::runtime_error(
                "无法从目标 link 回溯到根 link，断开位置: " + current
            );
        }

        reversedChain.push_back(it->second);
        current = it->second->parentLink;
    }

    std::reverse(reversedChain.begin(), reversedChain.end());
    return reversedChain;
}

// targetLink 留空时，选择从根节点出发、joint 数最多的叶子节点。
// 如果机器人有多个工具分支，建议在 main() 里明确填写目标 link。
std::string chooseDeepestLeaf(const RobotModel& model, const std::string& rootLink) {
    std::unordered_set<std::string> parentLinks;
    for (const Joint& joint : model.joints) {
        parentLinks.insert(joint.parentLink);
    }

    std::string bestLeaf;
    std::size_t bestDepth = 0;
    for (const std::string& link : model.links) {
        if (parentLinks.count(link) != 0) {
            continue;
        }

        try {
            const std::size_t depth = buildChain(model, rootLink, link).size();
            if (bestLeaf.empty() || depth > bestDepth) {
                bestLeaf = link;
                bestDepth = depth;
            }
        }
        catch (const std::exception&) {
            // 跳过不属于该根节点的异常分支。
        }
    }

    if (bestLeaf.empty()) {
        throw std::runtime_error("没有找到可达的叶子 link");
    }
    return bestLeaf;
}

bool isMovingJoint(const Joint& joint) {
    return joint.type == "revolute"
        || joint.type == "continuous"
        || joint.type == "prismatic";
}

// 计算 T_root_target。
// movingJointValues 的顺序就是运动链上非 fixed joint 的顺序：
//   revolute / continuous：输入单位为度；
//   prismatic：输入单位为米。
Mat4 forwardKinematics(
    const std::vector<const Joint*>& chain,
    const std::vector<double>& movingJointValues)
{
    std::size_t requiredValueCount = 0;
    for (const Joint* joint : chain) {
        if (isMovingJoint(*joint)) {
            ++requiredValueCount;
        }
    }

    if (movingJointValues.size() != requiredValueCount) {
        std::ostringstream message;
        message << "关节值数量错误：需要 " << requiredValueCount
                << " 个，实际输入 " << movingJointValues.size() << " 个";
        throw std::runtime_error(message.str());
    }

    Mat4 transform = Mat4::identity();
    std::size_t valueIndex = 0;

    for (const Joint* joint : chain) {
        // URDF 定义：T_parent_child(q) = T_origin * T_motion(q)。
        transform = transform * makeTransformFromXyzRpy(joint->originXyz, joint->originRpy);

        if (joint->type == "revolute" || joint->type == "continuous") {
            const double angleRad = degToRad(movingJointValues[valueIndex++]);
            transform = transform * makeAxisRotation(joint->axis, angleRad);
        }
        else if (joint->type == "prismatic") {
            const double distanceMetre = movingJointValues[valueIndex++];
            transform = transform * makeTranslation(normalized(joint->axis) * distanceMetre);
        }
        else if (joint->type == "fixed") {
            // fixed joint 只有 origin，没有可变运动量。
        }
        else {
            throw std::runtime_error(
                "本示例不支持 joint 类型 " + joint->type + "，joint=" + joint->name
            );
        }
    }

    return transform;
}

// 为了让四个品牌共用同一种输出类型，这里把姿态分量统一放在
// orientation1～orientation4 中；实际名称由对应的 Name 字段给出：
//   FANUC:   W, P, R
//   KUKA:    A, B, C
//   YASKAWA: Rx, Ry, Rz
//   ABB:     q1, q2, q3, q4（RAPID orient 四元数）
struct BrandPose {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double orientation1 = 0.0;
    double orientation2 = 0.0;
    double orientation3 = 0.0;
    double orientation4 = 0.0;
    int orientationCount = 3;
    std::string brand;
    std::string orientation1Name;
    std::string orientation2Name;
    std::string orientation3Name;
    std::string orientation4Name;
    std::string orientationUnit;
    std::string convention;
};

// 矩阵转姿态时采用哪一个机器人品牌的控制器原生约定。
// 只影响最终 4x4 矩阵 -> WPR/ABC/RxRyRz/ABB四元数的转换和显示，
// 不影响前面的 URDF 正运动学矩阵计算。
enum class PoseBrand {
    FANUC,
    KUKA,
    YASKAWA,
    ABB
};

struct EulerXYZ {
    double x = 0.0; // 绕 X 轴的角，弧度
    double y = 0.0; // 绕 Y 轴的角，弧度
    double z = 0.0; // 绕 Z 轴的角，弧度
};

BrandPose makeBrandPosePosition(
    const Mat4& transform,
    double positionScale,
    const std::string& brand,
    const std::string& angle1Name,
    const std::string& angle2Name,
    const std::string& angle3Name,
    const std::string& convention)
{
    BrandPose pose;
    pose.x = transform.m[0][3] * positionScale;
    pose.y = transform.m[1][3] * positionScale;
    pose.z = transform.m[2][3] * positionScale;
    pose.brand = brand;
    pose.orientation1Name = angle1Name;
    pose.orientation2Name = angle2Name;
    pose.orientation3Name = angle3Name;
    pose.orientationUnit = "deg";
    pose.convention = convention;
    return pose;
}

// ------------------------------------------------------------
// 分解形式 1：Rmatrix = Rz(z) * Ry(y) * Rx(x)
// ------------------------------------------------------------
// 在本文件采用的列向量约定下，它可以从两个角度描述：
//   1. 外旋 XYZ：依次绕固定 X、固定 Y、固定 Z 旋转；
//   2. 内旋 ZYX：依次绕运动后的 Z、Y、X 轴旋转。
// 二者生成相同的旋转矩阵，但品牌角度的名称和排列顺序不同。
EulerXYZ decomposeRzRyRx(const Mat4& transform) {
    EulerXYZ result;

    // y = asin(-R20)。clamp 用于消除浮点误差造成的越界。
    // C++14 兼容写法，作用等同 C++17 的 std::clamp。
    const double sinY = std::max(-1.0, std::min(1.0, -transform.m[2][0]));
    result.y = std::asin(sinY);
    const double cosY = std::cos(result.y);

    if (std::abs(cosY) > 1e-10) {
        result.x = std::atan2(transform.m[2][1], transform.m[2][2]);
        result.z = std::atan2(transform.m[1][0], transform.m[0][0]);
    }
    else {
        // 万向锁：Y 接近 +/-90 度时 X、Z 不再唯一。
        // 本实现固定 Z=0，把耦合角放入 X，保证结果确定且可重建矩阵。
        result.x = std::atan2(-transform.m[1][2], transform.m[1][1]);
        result.z = 0.0;
    }
    return result;
}

// ============================================================
// 1. FANUC：外旋 XYZ，输出 X/Y/Z/W/P/R
// ============================================================
// 与当前工程中 FANUC 使用的 gp_Extrinsic_XYZ 映射一致。
// 定义：
//   W：绕固定 X 轴
//   P：绕固定 Y 轴
//   R：绕固定 Z 轴
// 依次做外旋 X(W)、Y(P)、Z(R)，列向量矩阵为：
//   Rmatrix = Rz(R) * Ry(P) * Rx(W)
BrandPose matrixToFanucPose(const Mat4& transform, double positionScale = 1000.0) {
    const EulerXYZ euler = decomposeRzRyRx(transform);
    BrandPose pose = makeBrandPosePosition(
        transform, positionScale,
        "FANUC", "W", "P", "R",
        "外旋 XYZ；Rmatrix = Rz(R) * Ry(P) * Rx(W)"
    );
    pose.orientation1 = radToDeg(euler.x); // W
    pose.orientation2 = radToDeg(euler.y); // P
    pose.orientation3 = radToDeg(euler.z); // R
    return pose;
}

// ============================================================
// 2. KUKA：内旋 ZYX，输出 X/Y/Z/A/B/C
// ============================================================
// 与当前工程中 KUKA 使用的 gp_Intrinsic_ZYX 映射一致。
// 定义：
//   A：绕运动坐标系 Z 轴
//   B：绕运动坐标系 Y 轴
//   C：绕运动坐标系 X 轴
// 内旋 Z(A)、Y(B)、X(C)，列向量矩阵为：
//   Rmatrix = Rz(A) * Ry(B) * Rx(C)
// 注意：矩阵形式和 FANUC 上面的公式相同，但角度名称顺序是 A/B/C=Z/Y/X。
BrandPose matrixToKukaPose(const Mat4& transform, double positionScale = 1000.0) {
    const EulerXYZ euler = decomposeRzRyRx(transform);
    BrandPose pose = makeBrandPosePosition(
        transform, positionScale,
        "KUKA", "A", "B", "C",
        "内旋 ZYX；Rmatrix = Rz(A) * Ry(B) * Rx(C)"
    );
    pose.orientation1 = radToDeg(euler.z); // A，Z
    pose.orientation2 = radToDeg(euler.y); // B，Y
    pose.orientation3 = radToDeg(euler.x); // C，X
    return pose;
}

// ============================================================
// 3. YASKAWA：官方 Rz -> Ry -> Rx 姿态顺序，输出 X/Y/Z/Rx/Ry/Rz
// ============================================================
// YASKAWA YRC 官方手册说明：指定坐标系依次旋转 Rz、Ry、Rx，
// 得到控制点坐标系姿态。列向量矩阵写成：
//   Rmatrix = Rz(Rz) * Ry(Ry) * Rx(Rx)
//
// 注意：这与原工程给 YASKAWA 设置的 gp_Intrinsic_XYZ 不同；
// 这里按品牌官方控制器定义实现。
BrandPose matrixToYaskawaPose(const Mat4& transform, double positionScale = 1000.0) {
    const EulerXYZ euler = decomposeRzRyRx(transform);
    BrandPose pose = makeBrandPosePosition(
        transform, positionScale,
        "YASKAWA", "Rx", "Ry", "Rz",
        "官方顺序 Rz -> Ry -> Rx；Rmatrix = Rz(Rz) * Ry(Ry) * Rx(Rx)"
    );
    pose.orientation1 = radToDeg(euler.x); // Rx
    pose.orientation2 = radToDeg(euler.y); // Ry
    pose.orientation3 = radToDeg(euler.z); // Rz
    return pose;
}

// ============================================================
// 4. ABB：RAPID 原生 orient 四元数，输出 X/Y/Z/q1/q2/q3/q4
// ============================================================
// ABB RAPID 的 orient 类型是 [q1,q2,q3,q4]：
//   q1：四元数标量项 w；q2/q3/q4：向量项 x/y/z。
// 单位为无量纲，且 q1^2+q2^2+q3^2+q4^2=1。
// q 和 -q 表示同一姿态；为了让输出唯一，本实现固定 q1 >= 0。
BrandPose matrixToAbbPose(const Mat4& transform, double positionScale = 1000.0) {
    BrandPose pose = makeBrandPosePosition(
        transform, positionScale,
        "ABB RAPID", "q1", "q2", "q3",
        "RAPID orient 四元数 [q1,q2,q3,q4] = [w,x,y,z]"
    );
    pose.orientationCount = 4;
    pose.orientation4Name = "q4";
    pose.orientationUnit = "无量纲";

    const double r00 = transform.m[0][0];
    const double r01 = transform.m[0][1];
    const double r02 = transform.m[0][2];
    const double r10 = transform.m[1][0];
    const double r11 = transform.m[1][1];
    const double r12 = transform.m[1][2];
    const double r20 = transform.m[2][0];
    const double r21 = transform.m[2][1];
    const double r22 = transform.m[2][2];

    double qw = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    const double trace = r00 + r11 + r22;

    // 按最大对角分支计算，避免接近 180 度时除以很小的数。
    if (trace > 0.0) {
        const double s = 2.0 * std::sqrt(std::max(0.0, trace + 1.0));
        qw = 0.25 * s;
        qx = (r21 - r12) / s;
        qy = (r02 - r20) / s;
        qz = (r10 - r01) / s;
    }
    else if (r00 > r11 && r00 > r22) {
        const double s = 2.0 * std::sqrt(std::max(0.0, 1.0 + r00 - r11 - r22));
        qw = (r21 - r12) / s;
        qx = 0.25 * s;
        qy = (r01 + r10) / s;
        qz = (r02 + r20) / s;
    }
    else if (r11 > r22) {
        const double s = 2.0 * std::sqrt(std::max(0.0, 1.0 + r11 - r00 - r22));
        qw = (r02 - r20) / s;
        qx = (r01 + r10) / s;
        qy = 0.25 * s;
        qz = (r12 + r21) / s;
    }
    else {
        const double s = 2.0 * std::sqrt(std::max(0.0, 1.0 + r22 - r00 - r11));
        qw = (r10 - r01) / s;
        qx = (r02 + r20) / s;
        qy = (r12 + r21) / s;
        qz = 0.25 * s;
    }

    const double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm < 1e-15) {
        throw std::runtime_error("旋转矩阵无法转换为有效的 ABB 四元数");
    }
    qw /= norm;
    qx /= norm;
    qy /= norm;
    qz /= norm;

    if (qw < 0.0) {
        qw = -qw;
        qx = -qx;
        qy = -qy;
        qz = -qz;
    }

    pose.orientation1 = qw; // q1，标量 w
    pose.orientation2 = qx; // q2，X 向量项
    pose.orientation3 = qy; // q3，Y 向量项
    pose.orientation4 = qz; // q4，Z 向量项
    return pose;
}

// 统一入口：main() 只需要传入 poseBrand，不必在文件后面寻找并修改转换函数。
BrandPose matrixToBrandPose(
    const Mat4& transform,
    PoseBrand poseBrand,
    double positionScale = 1000.0)
{
    switch (poseBrand) {
    case PoseBrand::FANUC:
        return matrixToFanucPose(transform, positionScale);
    case PoseBrand::KUKA:
        return matrixToKukaPose(transform, positionScale);
    case PoseBrand::YASKAWA:
        return matrixToYaskawaPose(transform, positionScale);
    case PoseBrand::ABB:
        return matrixToAbbPose(transform, positionScale);
    }

    throw std::runtime_error("未知的姿态输出品牌");
}

void printMatrix(const Mat4& transform) {
    std::cout << "T_root_target =\n";
    std::cout << std::fixed << std::setprecision(9);
    for (int row = 0; row < 4; ++row) {
        std::cout << "  [ ";
        for (int col = 0; col < 4; ++col) {
            std::cout << std::setw(14) << transform.m[row][col];
            if (col != 3) {
                std::cout << ", ";
            }
        }
        std::cout << " ]\n";
    }
}

void printChainAndInputs(
    const std::vector<const Joint*>& chain,
    const std::vector<double>& values)
{
    std::cout << "运动链：\n";
    std::size_t valueIndex = 0;
    for (const Joint* joint : chain) {
        std::cout << "  " << joint->parentLink << " --[" << joint->name
                  << ", " << joint->type << "]--> " << joint->childLink;

        if (joint->type == "revolute" || joint->type == "continuous") {
            std::cout << "    q=" << values.at(valueIndex++) << " deg";
        }
        else if (joint->type == "prismatic") {
            std::cout << "    q=" << values.at(valueIndex++) << " m";
        }
        std::cout << '\n';
    }
}

} // namespace

int main() {
    try {
        // ====================== 只需要修改这里 ======================

        // 1. URDF 文件路径。
        const std::string urdfPath =
            R"(E:\lixiang\PH\robotLibrary\FANUC\R-2000iB210F\urdf\R-2000iB210F.urdf)";

        // 2. 要输出姿态的目标 link。
        //    当前只计算 base_link -> Link_6，不再继续乘 Link_6 -> tool0 的固定变换。
        //    名称必须与 URDF 中 <link name="..."> 完全一致（包括大小写）。
        //    留空 "" 才会自动选择层级最深的叶子节点。
        const std::string targetLink = "Link_6";

        // 3. 按 base -> target 运动链顺序输入所有“可运动关节”的值。
        //    revolute / continuous 的单位是度；prismatic 的单位是米。
        //    典型六轴机器人就是 J1、J2、J3、J4、J5、J6。
        const std::vector<double> jointValues = {
            0.0,    // J1，deg
            -30.0,  // J2，deg
            45.0,   // J3，deg
            0.0,    // J4，deg
            30.0,   // J5，deg
            0.0     // J6，deg
        };

        // 4. “URDF 零姿态”在机器人界面上显示的关节角。
        //
        //    这不是 Home 点，也不是 URDF <origin rpy>：
        //      - URDF <origin>       定义相邻关节坐标系之间的固定安装变换；
        //      - 本数组             定义 URDF q=0 时，控制器/界面显示的轴角；
        //      - Home               只是用户保存的一组回零/待机姿态。
        //
        //    本程序采用：
        //      q_URDF = axisSign * (q_界面输入 - q_URDF零姿态时的界面显示值)
        //
        //    四行只启用一行。下面数值中，KUKA 和 YASKAWA 是当前工程
        //    RobotLoadCommand::execute() 写入界面的初始显示值；FANUC/ABB/UR
        //    在当前工程中没有额外写入，所以均为 0。
        //
        //    注意：真实控制器的校准零位可能因具体型号、机械校准数据而不同。
        //    如果要严格匹配官方离线软件/示教器，应把该型号在官方软件中
        //    “模型处于 URDF q=0 姿态时显示的六轴角”填到这里。
        const std::vector<double> displayedJointsAtUrdfZero = { 0, 0, 0, 0, 0, 0 };       // FANUC / ABB / UR（当前工程）
        // const std::vector<double> displayedJointsAtUrdfZero = { 0, -90, 90, 0, 0, 0 };  // KUKA（当前工程）
        // const std::vector<double> displayedJointsAtUrdfZero = { 0, 90, -90, 0, 0, 0 };  // YASKAWA（当前工程）

        // 各轴正方向是否与 URDF <axis xyz="..."> 定义一致。
        // 一致填 1，方向相反填 -1。当前工程直接使用 URDF 轴方向，所以全为 1。
        const std::vector<double> jointAxisSigns = { 1, 1, 1, 1, 1, 1 };

        // 5. 是否复现当前工程中 FANUC 专用的 J2 -> J3 联动。
        //
        //    true：匹配你当前工程 Robot::updateJointValue() 的显示结果。
        //          工程在更新 J2 时还会把同一个增量施加到 J3 及其后续连杆，
        //          因此用于矩阵累乘的有效第三轴角度为 J3_effective = J3 + J2。
        //
        //    false：严格按照 URDF 标准，把 J1～J6 当作相互独立的关节角。
        //
        // 当前默认 true，所以本文件输出会与你截图中的工程结果一致。
        const bool matchCurrentProjectFanucJ2J3Linkage = true;

        // 6. 选择“末端矩阵 -> 品牌控制器姿态”的转换方式。
        //    四行只能启用一行：启用的行前面不要加 //，其余三行保持注释。
        //
        //    FANUC：   外旋 XYZ，输出 X/Y/Z/W/P/R
        //    KUKA：    内旋 ZYX，输出 X/Y/Z/A/B/C
        //    YASKAWA： 官方 Rz -> Ry -> Rx，输出 X/Y/Z/Rx/Ry/Rz
        //    ABB：     RAPID 原生四元数，输出 X/Y/Z/q1/q2/q3/q4
        const PoseBrand poseBrand = PoseBrand::FANUC;
        // const PoseBrand poseBrand = PoseBrand::KUKA;
        // const PoseBrand poseBrand = PoseBrand::YASKAWA;
        // const PoseBrand poseBrand = PoseBrand::ABB;

        // ==================== 以下通常无需修改 ====================

        const RobotModel model = parseUrdf(urdfPath);
        const std::string rootLink = findRootLink(model);
        const std::string resolvedTarget = targetLink.empty()
            ? chooseDeepestLeaf(model, rootLink)
            : targetLink;

        if (model.links.count(resolvedTarget) == 0) {
            throw std::runtime_error("URDF 中不存在目标 link: " + resolvedTarget);
        }

        const std::vector<const Joint*> chain =
            buildChain(model, rootLink, resolvedTarget);

        std::cout << "URDF:  " << urdfPath << '\n';
        std::cout << "Root:  " << rootLink << '\n';
        std::cout << "Target:" << resolvedTarget << "\n\n";

        printChainAndInputs(chain, jointValues);

        // 先把界面/控制器显示角换算成 URDF 关节变量，不修改 jointValues 原始输入。
        if (displayedJointsAtUrdfZero.size() != jointValues.size() ||
            jointAxisSigns.size() != jointValues.size()) {
            throw std::runtime_error("零位数组、轴方向数组必须与输入关节数量一致");
        }

        std::vector<double> fkJointValues = jointValues;
        for (std::size_t i = 0; i < fkJointValues.size(); ++i) {
            fkJointValues[i] = jointAxisSigns[i] *
                (jointValues[i] - displayedJointsAtUrdfZero[i]);
        }

        std::cout << "\n界面显示角 -> URDF 关节变量：\n";
        for (std::size_t i = 0; i < fkJointValues.size(); ++i) {
            std::cout << "  J" << (i + 1)
                      << ": input=" << jointValues[i]
                      << ", zeroDisplay=" << displayedJointsAtUrdfZero[i]
                      << ", sign=" << jointAxisSigns[i]
                      << ", q_URDF=" << fkJointValues[i] << '\n';
        }

        // 需要匹配当前工程的 FANUC 显示联动时，再调整参与累乘的副本。
        if (matchCurrentProjectFanucJ2J3Linkage) {
            if (fkJointValues.size() < 3) {
                throw std::runtime_error("J2 -> J3 联动至少需要三个可运动关节");
            }
            fkJointValues[2] = fkJointValues[2] + fkJointValues[1];
            std::cout << "\n当前工程 FANUC J2->J3 联动：已启用\n"
                      << "  联动前 q3_URDF       = "
                      << jointAxisSigns[2] * (jointValues[2] - displayedJointsAtUrdfZero[2]) << " deg\n"
                      << "  有效 q3_URDF=q3+q2   = " << fkJointValues[2] << " deg\n";
        }
        else {
            std::cout << "\n当前工程 FANUC J2->J3 联动：未启用（严格 URDF）\n";
        }

        const Mat4 endTransform = forwardKinematics(chain, fkJointValues);
        std::cout << '\n';
        printMatrix(endTransform);

        // 根据上面配置的 poseBrand 转换末端矩阵。
        // positionScale=1000：URDF 的米转换为毫米。
        // FANUC/KUKA/YASKAWA 姿态角输出度；ABB 四元数无量纲。
        const BrandPose pose = matrixToBrandPose(endTransform, poseBrand, 1000.0);

        std::cout << "\n品牌姿态输出：" << pose.brand << '\n';
        std::cout << "旋转约定：" << pose.convention << '\n';
        std::cout << "单位：XYZ=mm，姿态=" << pose.orientationUnit << '\n';
        std::cout << std::fixed << std::setprecision(6)
                  << "  X  = " << pose.x << '\n'
                  << "  Y  = " << pose.y << '\n'
                  << "  Z  = " << pose.z << '\n'
                  << "  " << pose.orientation1Name << " = " << pose.orientation1 << '\n'
                  << "  " << pose.orientation2Name << " = " << pose.orientation2 << '\n'
                  << "  " << pose.orientation3Name << " = " << pose.orientation3 << '\n';
        if (pose.orientationCount == 4) {
            std::cout << "  " << pose.orientation4Name
                      << " = " << pose.orientation4 << '\n';
        }

        return 0;
    }
    catch (const std::exception& error) {
        std::cerr << "错误: " << error.what() << '\n';
        return 1;
    }
}
