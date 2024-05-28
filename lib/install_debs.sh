#!/bin/bash
# Check if script is being run as root
if [ "$EUID" -ne 0 ]; then
    echo -e "\e[1;31m[ERROR]:\e[0m Please run as root"
    exit 1
fi
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
echo -e "\e[1;34m[INFO]:\e[0m Running: apt update"
apt update
echo -e "\e[1;34m[INFO]:\e[0m Running: apt install ${script_dir}/*.deb -y"
if ! apt install "${script_dir}"/*.deb -y; then
    echo -e "\e[1;31m[ERROR]:\e[0m Error installing debs, please check package.xml to see if all dependencies are satisfied"
    exit 1
fi
