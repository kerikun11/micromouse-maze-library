## コンピュータでの使用例

--------------------------------------------------------------------------------

### 動作確認済みの環境

- Windows 10 (MSYS2 MinGW)
- Ubuntu 20.04
- Arch Linux (Manjaro Linux 20)

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
    lcov \
    qt5-default
# Arch Linux
yay -S --needed git make cmake gcc \
    python-matplotlib \
    pybind11 \
    doxygen graphviz \
    lcov \
    qt5-base
# MSYS2 MinGW 64bit
pacman -S --needed git make \
    mingw-w64-x86_64-cmake \
    mingw-w64-x86_64-toolchain \
    mingw-w64-x86_64-python-matplotlib \
    mingw-w64-x86_64-pybind11 \
    mingw-w64-x86_64-doxygen \
    mingw-w64-x86_64-graphviz \
    mingw-w64-x86_64-lcov \
    mingw-w64-x86_64-qt5
```

--------------------------------------------------------------------------------

### リポジトリの取得

```sh
## 迷路データ (サブモジュール) を含めて複製
git clone --recursive https://github.com/kerikun11/micromouse-maze-library.git
## 移動
cd micromouse-maze-library
## 作業ディレクトリを作成
mkdir build
cd build
## 初期化 (Makefile の生成)
cmake .. # MSYS2 の場合 -G "MSYS Makefiles" オプションを付加
## ビルド
make
```

--------------------------------------------------------------------------------

### 探索走行

サンプルコード [/examples/search/main.cpp](/examples/search/main.cpp) を実行するコマンドの例

```sh
# CMake により初期化された作業ディレクトリへ移動する
cd build
## 実行 (examples/search/main.cpp を実行)
make search
## コマンドラインにアニメーションが流れる
```

--------------------------------------------------------------------------------

## リファレンスの生成

コード中のコメントは [Doxygen](http://www.doxygen.jp/) に準拠しているので，API リファレンスを自動生成することができる．

```sh
# CMake により初期化された作業ディレクトリへ移動する
cd build
# ドキュメントの自動生成
make docs
# ブラウザで開く
open docs/html/index.html
```

上記コマンドにより `/build/docs/html/index.html` にリファレンスが生成される．

--------------------------------------------------------------------------------

## ユニットテスト

[GoogleTest](https://github.com/google/googletest) によるユニットテストと [LCOV](https://github.com/linux-test-project/lcov) によるカバレッジテストを実行する

```sh
# CMake により初期化された作業ディレクトリへ移動する
cd build
# テストを実行
make test_run
# カバレッジ結果の収集
make lcov
# ブラウザで開く
open test/html/index.html
# カバレッジのクリア
make lcov_clean
```

上記コマンドにより `/build/test/html/index.html` にカバレッジ結果が生成される．
