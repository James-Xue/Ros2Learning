# C++ VS Code Linux 练习区（独立小工程）

这个文件夹是一个**与 ROS2 工作区解耦**的纯 C++ 小工程，用来练习在 Linux 下使用 VS Code 进行：
- CMake 配置/编译（Debug/Release）
- gdb 断点调试（F5）
- 基本 C++ 语法与常见坑（RAII、容器、线程等）

建议做法：在 VS Code 里 **File → Open Folder…** 打开本文件夹 `cpp_vscode_lab/`（而不是打开仓库根目录），这样这里的 `.vscode/` 配置不会和仓库根目录的 ROS2 `.vscode/` 互相影响。

## 0. 依赖准备

Ubuntu/Debian：

```bash
sudo apt update
sudo apt install -y build-essential cmake gdb
```

VS Code 推荐扩展：
- `ms-vscode.cpptools`（C/C++）
- `ms-vscode.cmake-tools`（CMake Tools，可选但推荐）

## 1. 目录结构

- `CMakeLists.txt`：构建配置
- `src/`：多个 demo 源文件
- `.vscode/`：本文件夹专用 VS Code 构建/调试配置
- `build/`：构建输出（已在本文件夹 `.gitignore` 忽略）

## 2. 在 VS Code 里一键编译

打开 `cpp_vscode_lab/` 后：

- `Ctrl+Shift+B`：执行 **CMake Build**（会先自动 Configure）
- 或者命令面板运行 Task：`Tasks: Run Task` → `CMake Build`

产物在：
- `build/bin/01_hello`
- `build/bin/02_args`
- ...

## 3. 在 VS Code 里一键调试（F5）

1) 在任意 demo 里打断点
2) 按 `F5`
3) 选择一个调试配置（比如 `Debug: 03_vector_sort`）

这些调试配置都带 `preLaunchTask: CMake Build`，所以你不用手动编译。

## 4. 命令行方式（不依赖 VS Code）

```bash
cd cpp_vscode_lab
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
./build/bin/01_hello
```

## 5. Demo 列表（建议练习顺序）

- `01_hello.cpp`：最小可调试程序（断点、变量查看）
- `02_args.cpp`：命令行参数与基础 I/O
- `03_vector_sort.cpp`：`std::vector`、排序、lambda、常见边界
- `04_class_raii.cpp`：RAII、构造/析构、资源管理、异常安全
- `05_threads.cpp`：`std::thread`、互斥锁、数据竞争示例

## 6. 常见问题（速查）

- **F5 启动但断点灰色**：
  - 确认是 `Debug` 构建（本工程默认 Debug）
  - 确认启动的是 `build/bin/...` 里的程序（不是旧路径）

- **找不到 gdb**：
  - `sudo apt install gdb`

- **想新增 demo**：
  - 在 `src/` 新增 `06_xxx.cpp`
  - 在 `CMakeLists.txt` 里加一行 `add_demo(06_xxx src/06_xxx.cpp)`
  - 复制一份 `launch.json` 里的配置改 program 名字即可
