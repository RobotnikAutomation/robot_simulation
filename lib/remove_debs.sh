#!/bin/bash
# Check if script is being run as root
if [ "$EUID" -ne 0 ]; then
    echo -e "\e[1;31m[ERROR]:\e[0m Please run as root"
    exit 1
fi
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# List of packages of debs to remove
packages_to_remove=""
files=("${script_dir}"/*.deb)
for file in "${files[@]}"; do
    packages_to_remove="${packages_to_remove} $(dpkg-deb -f "${file}" Package)"
done
echo -e "\e[1;34m[INFO]:\e[0m Running: apt remove ${packages_to_remove} -y"
if ! apt remove ${packages_to_remove} -y; then
    echo -e "\e[1;31m[ERROR]:\e[0m Error removing debs"
    exit 1
fi
