#!/bin/bash
set -euo pipefail

if [ ! -d "teleimager" ] || [ ! -f "requirements.txt" ]; then
    echo "Error: Please run this script from the root of the unitree_sim_isaaclab project!"
    exit 1
fi

UNITREE_DIR="$(pwd)"
DEFAULT_ENV_PATH="$UNITREE_DIR/.venv"
DEFAULT_CYCLONEDDS_DIR="/data/jun7.shi/code/poc/unitree/thirdparty/cyclonedds"
DEFAULT_UNITREE_SDK2_PYTHON_DIR="/data/jun7.shi/code/poc/unitree/Robot_SDK/unitree_sdk2_python"
PYTHON_VER="3.11"
TORCH_INDEX_URL="https://download.pytorch.org/whl/cu128"
NVIDIA_INDEX_URL="https://pypi.nvidia.com"
UV_CACHE_DIR="${UV_CACHE_DIR:-/data/jun7.shi/cache/uv}"
CC_LIBSTDCPP_DIR="/data/jun7.shi/tools/miniforge3/envs/cc-libstdcpp/lib"

confirm_clone() {
    local label="$1"
    local repo_url="$2"
    local repo_ref="$3"
    local dest_dir="$4"

    echo "$label not found at: $dest_dir"
    echo "The script can clone:"
    echo "  repo: $repo_url"
    if [ -n "$repo_ref" ]; then
        echo "  ref: $repo_ref"
    fi
    echo "  dest: $dest_dir"

    if [ ! -t 0 ]; then
        echo "Error: $label is missing at $dest_dir and cloning requires interactive confirmation."
        exit 1
    fi

    if ! read -r -p "Continue? [y/N] " response < /dev/tty; then
        echo "Cancelled."
        exit 1
    fi

    case "$response" in
        y|Y|yes|YES)
            mkdir -p "$(dirname "$dest_dir")"
            return 0
            ;;
        *)
            echo "Cancelled."
            exit 1
            ;;
    esac
}

ensure_cyclonedds_dir() {
    if [ -d "$CYCLONEDDS_DIR" ]; then
        return 0
    fi

    if [ -e "$CYCLONEDDS_DIR" ]; then
        echo "Error: CYCLONEDDS_DIR exists but is not a directory: $CYCLONEDDS_DIR"
        exit 1
    fi

    confirm_clone "CycloneDDS" "https://github.com/eclipse-cyclonedds/cyclonedds" "releases/0.10.x" "$CYCLONEDDS_DIR"
    git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x "$CYCLONEDDS_DIR"
}

ensure_unitree_sdk2_python_dir() {
    if [ -d "$UNITREE_SDK2_PYTHON_DIR" ]; then
        return 0
    fi

    if [ -e "$UNITREE_SDK2_PYTHON_DIR" ]; then
        echo "Error: UNITREE_SDK2_PYTHON_DIR exists but is not a directory: $UNITREE_SDK2_PYTHON_DIR"
        exit 1
    fi

    confirm_clone "unitree_sdk2_python" "https://github.com/unitreerobotics/unitree_sdk2_python" "" "$UNITREE_SDK2_PYTHON_DIR"
    git clone https://github.com/unitreerobotics/unitree_sdk2_python "$UNITREE_SDK2_PYTHON_DIR"
}

case "$#" in
    0)
        ENV_PATH="$DEFAULT_ENV_PATH"
        CYCLONEDDS_DIR="${CYCLONEDDS_DIR:-$DEFAULT_CYCLONEDDS_DIR}"
        UNITREE_SDK2_PYTHON_DIR="${UNITREE_SDK2_PYTHON_DIR:-$DEFAULT_UNITREE_SDK2_PYTHON_DIR}"
        ;;
    2)
        ENV_PATH="$DEFAULT_ENV_PATH"
        CYCLONEDDS_DIR="$1"
        UNITREE_SDK2_PYTHON_DIR="$2"
        ;;
    3)
        ENV_PATH="$1"
        CYCLONEDDS_DIR="$2"
        UNITREE_SDK2_PYTHON_DIR="$3"
        ;;
    *)
        echo "Usage: bash auto_uv_setup_env.sh [ENV_PATH] <CYCLONEDDS_DIR> <UNITREE_SDK2_PYTHON_DIR>"
        echo "Example: bash auto_uv_setup_env.sh $UNITREE_DIR/.venv /data/jun7.shi/code/poc/unitree/thirdparty/cyclonedds /data/jun7.shi/code/poc/unitree/Robot_SDK/unitree_sdk2_python"
        echo "You can also omit ENV_PATH and use the default: $DEFAULT_ENV_PATH"
        echo "Or run with no positional arguments to use default dependency locations:"
        echo "  CYCLONEDDS_DIR=$DEFAULT_CYCLONEDDS_DIR"
        echo "  UNITREE_SDK2_PYTHON_DIR=$DEFAULT_UNITREE_SDK2_PYTHON_DIR"
        exit 1
        ;;
esac

if ! command -v git >/dev/null 2>&1; then
    echo "Error: git is not installed or not in PATH."
    exit 1
fi

ensure_cyclonedds_dir
ensure_unitree_sdk2_python_dir

CYCLONEDDS_DIR="$(realpath "$CYCLONEDDS_DIR")"
UNITREE_SDK2_PYTHON_DIR="$(realpath "$UNITREE_SDK2_PYTHON_DIR")"
CYCLONEDDS_INSTALL_DIR="${CYCLONEDDS_DIR}/install"

if [ ! -f "$CYCLONEDDS_DIR/CMakeLists.txt" ]; then
    echo "Error: Invalid CYCLONEDDS_DIR: $CYCLONEDDS_DIR"
    exit 1
fi

if [ ! -f "$UNITREE_SDK2_PYTHON_DIR/setup.py" ] && [ ! -f "$UNITREE_SDK2_PYTHON_DIR/pyproject.toml" ]; then
    echo "Error: Invalid UNITREE_SDK2_PYTHON_DIR: $UNITREE_SDK2_PYTHON_DIR"
    exit 1
fi

if ! command -v uv >/dev/null 2>&1; then
    echo "Error: uv is not installed or not in PATH."
    exit 1
fi

if ! command -v cmake >/dev/null 2>&1; then
    echo "Error: cmake is not installed or not in PATH."
    exit 1
fi

if ! command -v openssl >/dev/null 2>&1; then
    echo "Error: openssl is not installed or not in PATH."
    exit 1
fi

if [ ! -d "$CC_LIBSTDCPP_DIR" ]; then
    echo "Error: cc-libstdcpp runtime not found at $CC_LIBSTDCPP_DIR"
    exit 1
fi

mkdir -p "$UV_CACHE_DIR"
mkdir -p "$(dirname "$ENV_PATH")"

export UV_CACHE_DIR

if [ -x "$ENV_PATH/bin/python" ]; then
    echo "Reusing existing environment at: $ENV_PATH"
elif [ -e "$ENV_PATH" ]; then
    echo "Error: ENV_PATH exists but is not a reusable Python environment: $ENV_PATH"
    echo "Expected to find: $ENV_PATH/bin/python"
    exit 1
else
    echo "Creating uv environment at: $ENV_PATH"
    uv venv --python "$PYTHON_VER" "$ENV_PATH"
fi

if [ ! -x "$ENV_PATH/bin/python" ]; then
    echo "Error: environment is missing python: $ENV_PATH/bin/python"
    exit 1
fi

echo "Using CycloneDDS from: $CYCLONEDDS_DIR"
echo "Using unitree_sdk2_python from: $UNITREE_SDK2_PYTHON_DIR"

export VIRTUAL_ENV="$ENV_PATH"
export PATH="$VIRTUAL_ENV/bin:$PATH"

echo "**************************************************"
echo "Generate certificate files..."
echo "Just keep pressing the Enter key."
echo "**************************************************"
openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout key.pem -out cert.pem
mkdir -p ~/.config/xr_teleoperate/
cp key.pem cert.pem ~/.config/xr_teleoperate/
rm key.pem cert.pem

echo "Installing PyTorch CUDA 12.8 packages..."
uv pip install -U torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0 --index-url "$TORCH_INDEX_URL"

echo "Installing IsaacLab 2.3.2.post1 with Isaac Sim 5.1..."
uv pip install "isaaclab[isaacsim,all]==2.3.2.post1" --extra-index-url "$NVIDIA_INDEX_URL"

echo "Building CycloneDDS..."
if [ ! -d "$CYCLONEDDS_INSTALL_DIR" ]; then
    mkdir -p "$CYCLONEDDS_DIR/build" "$CYCLONEDDS_INSTALL_DIR"
    cmake -S "$CYCLONEDDS_DIR" -B "$CYCLONEDDS_DIR/build" -DCMAKE_INSTALL_PREFIX="$CYCLONEDDS_INSTALL_DIR"
    cmake --build "$CYCLONEDDS_DIR/build" --target install
fi
export CYCLONEDDS_HOME="$CYCLONEDDS_INSTALL_DIR"

echo "Installing unitree_sdk2_python..."
uv pip install -e "$UNITREE_SDK2_PYTHON_DIR"

echo "Installing project requirements..."
uv pip install -r "$UNITREE_DIR/requirements.txt"

echo "Installing teleimager..."
uv pip install -e "$UNITREE_DIR/teleimager"

echo ""
echo "Environment is ready: $ENV_PATH"
echo "Activate with: source $ENV_PATH/bin/activate"
echo "CycloneDDS home: $CYCLONEDDS_HOME"
echo "Verify with: python -c 'import torch, isaacsim, isaaclab; print(torch.__version__)'"
