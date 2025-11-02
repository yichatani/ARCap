# 🥽 Ubuntu 下连接与安装 Meta Quest 3 教程

**适用于：** Ubuntu 20.04+ / 22.04 / 24.04  
**目标：** 在 Ubuntu 上完成 Meta Quest 3 的开发者连接、调试与安装 APK（如 arcap_release.apk）

---

## 🧭 目录

- [前提条件](#前提条件)
- [安装 ADB 工具](#安装-adb-工具)
- [开启开发者模式](#开启开发者模式)
- [连接 Quest 3](#连接-quest-3)
- [修复权限问题](#修复权限问题)
- [授权 USB 调试](#授权-usb-调试)
- [安装 APK 应用](#安装-apk-应用)
- [查看日志与卸载](#查看日志与卸载)
- [可选工具](#可选工具)
- [常见问题](#常见问题)
- [总结](#总结)

---

## 🧰 前提条件

你需要准备：

- 一台运行 Ubuntu 的电脑
- 一台 Meta Quest 3
- 一根 **支持数据传输** 的 USB-C 线
- 手机上安装并登录 **Meta Horizon App**

---

## ⚙️ 安装 ADB 工具

在终端执行以下命令安装 Android 调试工具：

```bash
sudo apt update
sudo apt install android-tools-adb android-tools-fastboot -y
```

验证安装是否成功：

```bash
adb version
```

输出示例：

```
Android Debug Bridge version 1.0.41
```

---

## 🧩 开启开发者模式

这一步需要在手机上完成：

1. 打开 **Meta Horizon App**
2. 登录与你头显绑定的账号
3. 点击右上角 **☰ → Devices**
4. 选择你的 **Quest 3**
5. 点击 **Developer Mode**
6. 打开开关 ✅
7. **重启头显**

---

## 🔌 连接 Quest 3

使用 USB-C 线连接头显与 Ubuntu 电脑，然后执行：

```bash
adb devices
```

第一次运行时，头显会弹出授权提示：

```
Allow USB debugging?
```

- ✅ 勾选 **"Always allow from this computer"**
- 然后点击 **"OK"**

再次运行：

```bash
adb devices
```

成功连接后，输出示例：

```
List of devices attached
1WMHH1234567	device
```

✅ 表示连接成功！

---

## 🧑‍🔧 修复权限问题

如果出现以下错误：

```
no permissions (user ani is not in the plugdev group)
```

需要执行以下修复步骤：

### 1️⃣ 将用户加入 plugdev 组

```bash
sudo usermod -aG plugdev $USER
sudo systemctl restart udev
```

**注意：** 需要登出或重启电脑以使更改生效。

### 2️⃣ 添加 udev 规则

创建或编辑 udev 规则文件：

```bash
sudo nano /etc/udev/rules.d/51-android.rules
```

粘贴以下内容：

```
SUBSYSTEM=="usb", ATTR{idVendor}=="2833", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="18d1", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="2c3f", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="1d6b", MODE="0666", GROUP="plugdev"
```

保存后执行：

```bash
sudo chmod a+r /etc/udev/rules.d/51-android.rules
sudo udevadm control --reload-rules
sudo service udev restart
```

最后，**重新插拔 Quest 3**。

---

## 🔐 授权 USB 调试

如果执行 `adb devices` 后显示：

```
List of devices attached
XXXXXXX	unauthorized
```

说明需要在头显中确认授权：

1. 保持 USB 连接
2. 戴上头显
3. 在弹出的提示窗口中：
   - ✅ 勾选 **"Always allow from this computer"**
   - 点击 **"OK"**
4. 再次运行 `adb devices`

输出应为：

```
XXXXX	device
```

---

## 📦 安装 APK 应用

假设 `arcap_release.apk` 位于当前目录，执行：

```bash
adb install arcap_release.apk
```

### 覆盖安装旧版本

如果需要覆盖已安装的旧版本：

```bash
adb install -r -d arcap_release.apk
```

成功后输出：

```
Success
```

### 备用方案

如果安装过程卡在 `Performing Streamed Install`，尝试：

```bash
adb push arcap_release.apk /sdcard/Download/
adb shell pm install -r /sdcard/Download/arcap_release.apk
```

<!-- ---

## 🧠 查看日志与卸载

### 查看实时日志

```bash
adb logcat
```

### 仅查看特定包名的日志

```bash
adb logcat | grep arcap
```

### 卸载应用

```bash
adb uninstall com.example.arcap
```

**注意：** 包名可通过以下命令查询：

```bash
adb shell pm list packages | grep arcap
```

---

## 🖥️ 可选工具

### 屏幕镜像与调试

安装 **scrcpy** 工具，可以在 Ubuntu 上实时查看 Quest 屏幕：

```bash
sudo apt install scrcpy
```

启动实时画面镜像：

```bash
scrcpy
```

可以直接在 Ubuntu 上看到 Quest 屏幕并进行交互调试。

---

## ❓ 常见问题

| 问题 | 解决方法 |
|------|---------|
| `no permissions` | 添加 plugdev 组 + udev 规则 |
| `unauthorized` | 在头显中授权 USB 调试 |
| `Performing Streamed Install` 卡住 | 使用 `adb push` + `pm install` |
| `INSTALL_FAILED_VERSION_DOWNGRADE` | 使用 `-d` 参数允许降级安装 |
| 无法识别设备 | 检查数据线是否为「支持数据传输」线 |

---

## ✅ 总结

### Windows MQDH vs Ubuntu 对照表

| 操作 | Windows MQDH | Ubuntu 替代方式 |
|------|--------------|----------------|
| 查看设备 | MQDH Device Manager | `adb devices` |
| 安装 APK | MQDH → Install APK | `adb install arcap_release.apk` |
| 查看日志 | MQDH Console | `adb logcat` |
| 屏幕镜像 | MQDH Preview | `scrcpy` |
| 启用开发者模式 | Meta Horizon App | 同步操作（与系统无关） |

---

## 💡 自动化脚本

如果你经常调试，可以把这些命令写入脚本：

创建 `install_arcap.sh` 文件：

```bash
#!/bin/bash
adb devices
adb install -r -d arcap_release.apk
adb logcat | grep arcap
```

添加执行权限：

```bash
chmod +x install_arcap.sh
```

以后执行一行命令即可自动安装：

```bash
./install_arcap.sh
```

---

**教程完成！** 🎉 现在你可以在 Ubuntu 上顺畅地开发和调试 Meta Quest 3 应用了。 -->