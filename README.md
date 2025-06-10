[![Build & Test](https://github.com/YukiShirai/Noir/actions/workflows/ci.yml/badge.svg)](https://github.com/YukiShirai/Noir/actions)
[![codecov](https://codecov.io/gh/YukiShirai/Noir/branch/main/graph/badge.svg)](https://codecov.io/gh/YukiShirai/Noir)


# Noir
Implementation in C++ for Numerical Optimization in Robotics. This repo is still under development.


<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may get started working on this repository.

### Prerequisites
* gcc-11 or clang-14
* Python 3.10, 3.11
* pip3
* Ubuntu >= 22.04
* Eigen3.4


### Installation


1. Create virtualenvironment. It can be either using conda or venv.
2. Run the following command in the root directory of this project:

```
python3 -m pip install .
```
3. Then, download eigen and build Noir using cmake:
```
chmod +x setup/install_eigen3.4.sh
sudo ./setup/install_eigen3.4.sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```


<!-- Pre-commit -->
## Pre-commit
We use [pre-commit](https://pre-commit.com/) to run simple checks such as auto-formattting and linting. Follow the instructions in the hyperlink to setup pre-commit  on laptop.

```
pre-commit install
```
In this way, pre-commit is called everytime you do git commit.

## Code Style
We follow [Google's code stype](https://google.github.io/styleguide/pyguide.html).


## Code Pull Request Review
We do PR with ideally at most 500 lines. Please feel free to request PR and I am happy to review it.
