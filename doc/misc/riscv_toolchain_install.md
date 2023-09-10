# RISC-V Tool Chain Installation

## Introduction

**GNU MCU Eclipse RISC-V Embedded GCC** is used to compile the C code into RISC-V ISA using newlib as the c standard library.

- Reference website for: <https://gnu-mcu-eclipse.github.io/blog/2019/05/21/riscv-none-gcc-v8-2-0-2-2-20190521-released>
- Reference website for installation: <https://xpack.github.io/riscv-none-embed-gcc/install/>

## Prerequisites

1. node.js and npm (required by xpm)

Installation guide can be found at <https://nodejs.org/en/download/package-manager/>

For Arch Linux:

```shell
sudo pacman -S nodejs npm
```

For Debian Linux:

```shell
sudo apt-get install npm
```

2. xpm

Installation guide can be found at <https://xpack.github.io/xpm/install/>

```shell
sudo npm install --global xpm@latest --no-audit
```

## Installation

Installation guide can be found at <https://xpack.github.io/riscv-none-embed-gcc/install/>

**xpm** is required for the installation. See **Prerequisites** to install **xpm** if it is not installed

```shell
xpm install --global @xpack-dev-tools/riscv-none-embed-gcc@latest --verbose
```

The tool chain is installed in the following path: `~/.local/xPacks/@xpack-dev-tools/riscv-none-embed-gcc/10.2.0-1.2.1/.content/`. You might have a different version or path `10.2.0-1.2.1` depending on your installation.

Add the path to the $PATH variable by adding the following into your `.bashrc` file

`export PATH=$PATH:~/.local/xPacks/@xpack-dev-tools/riscv-none-embed-gcc/10.2.0-1.2.1/.content/bin/`