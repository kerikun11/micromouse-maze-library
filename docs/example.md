## コンピュータでの使用例

コマンドラインでの使用例を示す。

--------------------------------------------------------------------------------

### Docker による環境構築

```sh
# build dev env
./docker/build.sh
# run dev env
./docker/run.sh
# configure
rm -rf build
./docker/run.sh cmake ..
# build
./docker/run.sh make
```

--------------------------------------------------------------------------------

### 動作確認済みの環境

- Linux
  - Ubuntu 20.04
  - Manjaro Linux 20.0.3
- Windows
  - [MSYS2 MinGW 64 bit](https://www.msys2.org/)

--------------------------------------------------------------------------------

### 必要なパッケージ

- 必須
  - git
  - gcc, g++
  - make
  - cmake
- オプション
  - 可視化のために必要
    - python3
    - python3-matplotlib
  - Pythonモジュール化のために必要
    - pybind11
  - リファレンスの自動生成のために必要
    - doxygen
    - graphviz
  - ユニットテストのために必要
    - gtest
  - カバレッジテストのために必要
    - lcov
  - Qt GUI アプリのために必要
    - qt5

--------------------------------------------------------------------------------

### インストールコマンドの例

```sh
# Ubuntu 20.04
apt install git make cmake gcc g++ \
    python3-matplotlib \
    python3-dev \
    python3-distutils \
    python3-pybind11 \
    doxygen graphviz \
    libgtest-dev lcov \
    qt5-default
# Arch Linux
yay -S --needed git make cmake gcc \
    python-matplotlib \
    pybind11 \
    doxygen graphviz \
    gtest lcov \
    qt5-base
# MSYS2 MinGW 64bit
pacman -S --needed git make \
    $MINGW_PACKAGE_PREFIX-cmake \
    $MINGW_PACKAGE_PREFIX-toolchain \
    $MINGW_PACKAGE_PREFIX-python-matplotlib \
    $MINGW_PACKAGE_PREFIX-pybind11 \
    $MINGW_PACKAGE_PREFIX-doxygen \
    $MINGW_PACKAGE_PREFIX-graphviz \
    $MINGW_PACKAGE_PREFIX-gtest \
    $MINGW_PACKAGE_PREFIX-lcov \
    $MINGW_PACKAGE_PREFIX-qt5
```

--------------------------------------------------------------------------------

### リポジトリの取得

このリポジトリは [CMake](https://cmake.org/) プロジェクトになっている。

はじめに以下のコマンドで初期化する。

```sh
## 迷路データ (サブモジュール) を含めて clone
git clone --recursive https://github.com/kerikun11/micromouse-maze-library.git
## 移動
cd micromouse-maze-library
## 作業ディレクトリを作成
mkdir build
cd build
## 初期化 (Makefile の生成); MSYS2 の場合 -G"MSYS Makefiles" オプションを付加
cmake .. ${MSYSTEM:+-G"MSYS Makefiles"}
## ビルド
make
```

以降、コマンド `make` は上記 `build` ディレクトリで実行すること。

--------------------------------------------------------------------------------

### 探索走行

サンプルコード [examples/search/main.cpp](/examples/search/main.cpp) を実行するコマンドの例

```sh
## 実行 (examples/search/main.cpp を実行)
make search
## コマンドラインにアニメーションが流れる
```

--------------------------------------------------------------------------------

### リファレンスの生成

コード中のコメントは [Doxygen](http://www.doxygen.jp/) に準拠しているので、API リファレンスを自動生成することができる。

```sh
# ドキュメントの自動生成
make docs
# ブラウザで開く (open コマンドは環境依存)
open docs/html/index.html
```

上記コマンドにより `build/docs/html/index.html` にリファレンスが生成される。

--------------------------------------------------------------------------------

### ユニットテスト

[GoogleTest](https://github.com/google/googletest) によるユニットテストと [LCOV](https://github.com/linux-test-project/lcov) によるカバレッジテストを実行する

```sh
# カバレッジ結果の初期化
make lcov_init
# テストを実行
make test_run
# カバレッジ結果の収集
make lcov
# ブラウザでカバレッジ結果をみる (open コマンドは環境依存)
open test/html/index.html
```

上記コマンドにより `build/test/html/index.html` にカバレッジ結果が生成される。
