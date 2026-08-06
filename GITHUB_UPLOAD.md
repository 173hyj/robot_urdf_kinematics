# GitHub Upload Setup

## SSH Key

Open GitHub:

```text
Settings -> SSH and GPG keys -> New SSH key
```

Use this title:

```text
ubuntu22-to-gl-be6500-20260712
```

Use this key. Copy it as one single line:

```text
ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIAOk7d+jJ58On8DDvcsYP19Wr4NBLN1aVPZKAU+MvqI6 ubuntu22-to-gl-be6500-20260712
```

The expected key fingerprint is:

```text
SHA256:kqXkFP7uQA9zM62cZJ8pyLQAJau4NH7Wxzx98sqTKFc
```

## Repository Remote

After creating an empty GitHub repository, copy its SSH URL. It should look like this:

```text
git@github.com:YOUR_USERNAME/robot_urdf_kinematics.git
```

Send that SSH URL back to me and I can run:

```bash
git remote add origin git@github.com:YOUR_USERNAME/robot_urdf_kinematics.git
git branch -M main
git push -u origin main
```
