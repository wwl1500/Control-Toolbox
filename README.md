# Control Toolbox 安装与使用


## 目录

1. [系统要求](#系统要求)
2. [依赖安装](#依赖安装)
3. [Control Toolbox 编译安装](#control-toolbox-编译安装)
4. [运行论文代码示例](#运行论文代码示例)
---

## 系统要求

- **操作系统**: Ubuntu 24.04
---

## 依赖安装

### 步骤 1: 更新系统包管理器

```bash
sudo apt-get update
sudo apt-get upgrade -y
```

### 步骤 2: 安装基础依赖

```bash
# 安装编译工具链
sudo apt-get install -y build-essential cmake git

# 安装线性代数库
sudo apt-get install -y liblapack-dev libeigen3-dev

# 安装 Boost 库
sudo apt-get install -y libboost-all-dev

# 安装 OpenMP (并行计算支持)
sudo apt-get install -y libomp-dev

# 安装 Clang (可选，用于代码生成)
sudo apt-get install -y clang

# 安装 Python 3 及相关包 (用于绘图功能)
sudo apt-get install -y python3 python3-dev python3-numpy python3-matplotlib

# 安装 IPOPT (优化求解器，可选但推荐)
sudo apt-get install -y coinor-libipopt-dev
```

### 步骤 3: 安装 Kindr (运动学库)

```bash
# 创建工作目录
mkdir -p ~/workspace
cd ~/workspace

# 克隆 Kindr 仓库
git clone https://github.com/ethz-asl/kindr.git
cd kindr

# 编译安装 Kindr
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
cd ../..
```

### 步骤 4: 安装 CppAD 和 CppADCodeGen (自动微分库)

```bash
# 安装 CppAD
cd ~/workspace
wget https://github.com/coin-or/CppAD/archive/20240000.0.tar.gz
tar -xzf 20240000.0.tar.gz
cd CppAD-20240000.0
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
cd ../..

# 安装 CppADCodeGen
git clone https://github.com/joaoleal/CppADCodeGen.git
cd CppADCodeGen
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
cd ../..
```

### 步骤 5: 安装 Google Test (用于单元测试，可选)

```bash
sudo apt-get install -y libgtest-dev
cd /usr/src/gtest
sudo cmake -DBUILD_SHARED_LIBS=ON .
sudo make -j$(nproc)
sudo cp libgtest*.so /usr/lib
```

---

## Control Toolbox 编译安装

### 步骤 1: 克隆仓库

```bash
cd ~/workspace
git clone https://github.com/ethz-adrl/control-toolbox.git
cd control-toolbox
```

### 步骤 2: 安装依赖脚本 (可选，如果上述依赖已手动安装可跳过)

```bash
cd ct
chmod +x install_deps.sh
# 注意：此脚本会安装大部分依赖，但可能不包含所有最新版本
# 建议手动安装以确保版本兼容性
```

### 步骤 3: 编译安装 ct_core 模块

```bash
cd ~/workspace/control-toolbox/ct_core
mkdir -p build && cd build

# 配置构建
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=$HOME/.local

# 编译
make -j$(nproc)

# 安装到本地目录
make install
cd ../..
```

### 步骤 4: 编译安装 ct_rbd 模块

```bash
cd ~/workspace/control-toolbox/ct_rbd
mkdir -p build && cd build

# 配置构建 (指定 ct_core 位置)
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=$HOME/.local \
    -Dct_core_DIR=$HOME/.local/share/ct_core/cmake

# 编译
make -j$(nproc)

# 安装
make install
cd ../..
```

### 步骤 5: 编译安装 ct_models 模块 (包含论文代码示例)

```bash
cd ~/workspace/control-toolbox/ct_models
mkdir -p build && cd build

# 配置构建 (启用示例编译)
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=$HOME/.local \
    -Dct_rbd_DIR=$HOME/.local/share/ct_rbd/cmake \
    -DBUILD_EXAMPLES=ON

# 编译
make -j$(nproc)

# 安装
make install
cd ../..
```

### 步骤 6: 设置环境变量 (可选，但推荐)

将以下内容添加到 `~/.bashrc` 或 `~/.zshrc`:

```bash
# Control Toolbox 环境变量
export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig:$PKG_CONFIG_PATH
export CMAKE_PREFIX_PATH=$HOME/.local:$CMAKE_PREFIX_PATH
```

然后重新加载 shell 配置:

```bash
source ~/.bashrc  # 或 source ~/.zshrc
```

---

## 运行论文代码示例

### 步骤 1: 定位示例程序

论文代码示例位于 `ct_models/examples/UnderwaterDoubleLinkExample.cpp`，编译后的可执行文件位于:

```bash
~/workspace/control-toolbox/ct_models/build/examples/ex_underwater_double_link
```

### 步骤 2: 运行示例

```bash
# 方法 1: 直接运行
~/workspace/control-toolbox/ct_models/build/examples/ex_underwater_double_link

# 方法 2: 进入构建目录运行
cd ~/workspace/control-toolbox/ct_models/build/examples
./ex_underwater_double_link
```

### 步骤 3: 查看输出

程序会输出仿真 5 秒后的最终状态向量:

```
Final state after 5 seconds:
  [θ₁]  [θ₂]  [p_x]  [p_y]  [θ̇₁]  [θ̇₂]  [v_x]  [v_y]
```

其中:
- `θ₁, θ₂`: 两个连杆的角度 (弧度)
- `p_x, p_y`: 机器人质心位置 (米)
- `θ̇₁, θ̇₂`: 连杆角速度 (弧度/秒)
- `v_x, v_y`: 质心速度 (米/秒)

### 步骤 4: 修改参数和初始条件

编辑源代码文件以修改参数:

```bash
nano ~/workspace/control-toolbox/ct_models/examples/UnderwaterDoubleLinkExample.cpp
```

主要可修改的参数包括:
- **物理参数**: 质量 `m_`, 长度 `l_`, 阻尼系数 `c_t_`, `c_n_` 等
- **水动力参数**: `mu_n_`, `lambda1_`, `lambda2_`, `lambda3_`
- **环境流速**: `v_c_` (在构造函数中设置)
- **初始条件**: 在 `main()` 函数中修改 `state` 向量
- **仿真时间**: 修改 `tf` 变量

修改后重新编译:

```bash
cd ~/workspace/control-toolbox/ct_models/build
make -j$(nproc)
```

---

## 验证安装

运行以下命令验证安装是否成功:

```bash
# 检查可执行文件是否存在
ls -lh ~/workspace/control-toolbox/ct_models/build/examples/ex_underwater_double_link

# 检查库文件是否安装
ls -lh $HOME/.local/lib/libct_*.so

# 运行示例程序
~/workspace/control-toolbox/ct_models/build/examples/ex_underwater_double_link
```

如果程序能正常运行并输出状态向量，说明安装成功

---

## 快速参考

### 常用命令总结

```bash
# 重新编译单个模块
cd ~/workspace/control-toolbox/ct_models/build
make -j$(nproc)

# 清理构建文件
cd ~/workspace/control-toolbox/ct_models/build
make clean

# 完全重新构建
rm -rf ~/workspace/control-toolbox/*/build
# 然后重新执行编译步骤

# 查看编译选项
cd ~/workspace/control-toolbox/ct_models/build
ccmake ..  # 交互式配置
```

### 代码结构

```
control-toolbox/
├── ct/              # 构建脚本和配置
├── ct_core/         # 核心模块
├── ct_rbd/          # 刚体动力学模块
├── ct_optcon/       # 最优控制模块
└── ct_models/       # 机器人模型和示例
    └── examples/
        └── UnderwaterDoubleLinkExample.cpp  # 论文代码示例
```

---

## 参考资料

- **官方文档**: https://ethz-adrl.github.io/ct/ct_doc/doc/html/index.html
- **源码库**: https://github.com/ethz-adrl/control-toolbox

---

