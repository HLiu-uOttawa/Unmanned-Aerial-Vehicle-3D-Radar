#!/usr/bin/env bash
# ============================================
# ROS 2 Humble automatic installer for WSL2
# Compatible with Ubuntu 22.04 (Jammy)
# ============================================

set -e

echo "---------------------------------------------"
echo ">>> Checking Ubuntu version..."
echo "---------------------------------------------"

UBUNTU_VERSION=$(lsb_release -rs)
if [[ "$UBUNTU_VERSION" != "22.04" ]]; then
    echo "❌ Error: This script only supports Ubuntu 22.04 (Jammy)."
    echo "You are running: Ubuntu $UBUNTU_VERSION"
    echo "Please use an Ubuntu 22.04 environment (e.g., WSL2 Ubuntu 22.04)."
    exit 1
fi

echo "✅ Ubuntu 22.04 detected. Proceeding with installation..."
echo

# --------------------------------------------
# Step 1. Set Locale
# --------------------------------------------
echo ">>> Setting locale..."
sudo apt update -y
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale | grep LANG
echo "Locale configured to en_US.UTF-8"
echo

# --------------------------------------------
# Step 2. Add ROS 2 sources
# --------------------------------------------
echo ">>> Adding ROS 2 apt sources..."
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo apt update -y
sudo apt install -y curl gnupg lsb-release

ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
echo "ROS 2 apt source added successfully."
echo

# --------------------------------------------
# Step 3. Install ROS 2 packages
# --------------------------------------------
echo ">>> Installing ROS 2 Humble Desktop..."
sudo apt update -y
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop

echo ">>> Installing ROS development tools..."
sudo apt install -y ros-dev-tools

echo ">>> Sourcing ROS 2 environment..."
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
echo "ROS 2 environment added to ~/.bashrc"
echo

# --------------------------------------------
# Step 4. Verify installation
# --------------------------------------------
echo "---------------------------------------------"
echo "✅ Installation complete!"
echo "ROS 2 installation path: /opt/ros/humble"
echo "ROS 2 distribution: $ROS_DISTRO"
echo "---------------------------------------------"
echo

# --------------------------------------------
# Step 5. Post-install: setup SSH key and clone repo
# --------------------------------------------
echo ">>> Setting up SSH keys and cloning repository..."

# Switch to normal user if running as root
if [ "$EUID" -eq 0 ]; then
    echo "Switching to user account..."
    su - user <<'EOSU'
        set -e
        echo "Creating ~/.ssh folder..."
        mkdir -p ~/.ssh

        echo "Copying SSH keys from Windows path..."
        sudo cp /mnt/c/Users/Brian/.ssh/id_rsa ~/.ssh/
        sudo cp /mnt/c/Users/Brian/.ssh/id_rsa.pub ~/.ssh/

        echo "Fixing SSH key permissions..."
        chmod 700 ~/.ssh
        chmod 600 ~/.ssh/id_rsa
        chmod 644 ~/.ssh/id_rsa.pub

        echo "Creating workspace and cloning Git repository..."
        mkdir -p ~/ws_ros2
        cd ~/ws_ros2
        git clone git@github.com:HLiu-uOttawa/Unmanned-Aerial-Vehicle-3D-Radar.git

        echo "Entering repository folder..."
        cd ~/ws_ros2/Unmanned-Aerial-Vehicle-3D-Radar
        echo "Repository cloned successfully."
EOSU
else
    echo "Creating ~/.ssh folder..."
    mkdir -p ~/.ssh

    echo "Copying SSH keys from Windows path..."
    sudo cp /mnt/c/Users/Brian/.ssh/id_rsa ~/.ssh/
    sudo cp /mnt/c/Users/Brian/.ssh/id_rsa.pub ~/.ssh/

    echo "Fixing SSH key permissions..."
    chmod 700 ~/.ssh
    chmod 600 ~/.ssh/id_rsa
    chmod 644 ~/.ssh/id_rsa.pub

    echo "Creating workspace and cloning Git repository..."
    mkdir -p ~/ws_ros2
    cd ~/ws_ros2
    git clone git@github.com:HLiu-uOttawa/Unmanned-Aerial-Vehicle-3D-Radar.git

    echo "Entering repository folder..."
    cd ~/ws_ros2/Unmanned-Aerial-Vehicle-3D-Radar
    echo "Repository cloned successfully."
fi

echo
echo "---------------------------------------------"
echo "🎯 ROS 2 Humble + GitHub repo setup completed!"
echo "You can now work inside:"
echo "~/ws_ros2/Unmanned-Aerial-Vehicle-3D-Radar"
echo "---------------------------------------------"
