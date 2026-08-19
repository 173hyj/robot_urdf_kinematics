# GitHub 上传步骤

## 1. 提交本地仓库

```bash
git status
git add .
git commit -m "Prepare URDF kinematics toolkit and MATLAB visualizer"
```

构建目录、编译产物和 MATLAB 导出的 PDF 已由 `.gitignore` 排除。

## 2. 创建空仓库并添加远程地址

在 GitHub 新建空仓库，例如 `robot_urdf_kinematics`，然后执行：

```bash
git remote add origin https://github.com/<用户名>/robot_urdf_kinematics.git
```

也可以使用 SSH：

```bash
git remote add origin git@github.com:<用户名>/robot_urdf_kinematics.git
```

## 3. 推送

```bash
git branch -M main
git push -u origin main
```

首次使用 SSH 时，请在 GitHub 的 **Settings → SSH and GPG keys** 添加你自己的公钥。不要把私钥、访问令牌或账号密码写入仓库。
