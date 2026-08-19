#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/setup_3parts.sh [options]

Options:
  --runtime cpu|gpu|none    Python runtime to install (default: cpu).
  --gpu-index-url URL       Jetson/CUDA wheel index used by GPU mode.
                            Can also be set with LUXI_HLOC_GPU_INDEX_URL.
  --skip-models             Do not pre-download HLoc model weights.
  --skip-submodules         Do not initialize or update Git submodules.
  --python PATH             Python interpreter to use (default: python3).
  -h, --help                Show this help.

Examples:
  scripts/setup_3parts.sh --runtime cpu
  scripts/setup_3parts.sh --runtime none --skip-models
  LUXI_HLOC_GPU_INDEX_URL=<URL> scripts/setup_3parts.sh --runtime gpu
EOF
}

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

log() {
  printf '\n==> %s\n' "$*"
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
RUNTIME="cpu"
GPU_INDEX_URL="${LUXI_HLOC_GPU_INDEX_URL:-}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
DOWNLOAD_MODELS=1
UPDATE_SUBMODULES=1

while (($#)); do
  case "$1" in
    --runtime)
      (($# >= 2)) || die "--runtime requires a value"
      RUNTIME="$2"
      shift 2
      ;;
    --gpu-index-url)
      (($# >= 2)) || die "--gpu-index-url requires a value"
      GPU_INDEX_URL="$2"
      shift 2
      ;;
    --skip-models)
      DOWNLOAD_MODELS=0
      shift
      ;;
    --skip-submodules)
      UPDATE_SUBMODULES=0
      shift
      ;;
    --python)
      (($# >= 2)) || die "--python requires a value"
      PYTHON_BIN="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown option: $1 (use --help)"
      ;;
  esac
done

case "${RUNTIME}" in
  cpu|gpu|none) ;;
  *) die "--runtime must be one of: cpu, gpu, none" ;;
esac

command -v git >/dev/null 2>&1 || die "git is required"
command -v "${PYTHON_BIN}" >/dev/null 2>&1 ||
  die "Python interpreter not found: ${PYTHON_BIN}"
[[ -f "${WORKSPACE}/.gitmodules" ]] ||
  die ".gitmodules not found under ${WORKSPACE}"

declare -a DECLARED_3PARTS_PATHS=()
while read -r _key path; do
  if [[ "${path}" == 3parts/* ]]; then
    DECLARED_3PARTS_PATHS+=("${path}")
  fi
done < <(
  git -C "${WORKSPACE}" config -f .gitmodules \
    --get-regexp '^submodule\..*\.path$'
)

((${#DECLARED_3PARTS_PATHS[@]} > 0)) ||
  die "no 3parts submodules are declared in .gitmodules"

if ((UPDATE_SUBMODULES)); then
  log "Validating 3parts submodule pointers"
  declare -a SUBMODULE_PATHS=()
  invalid=0
  for path in "${DECLARED_3PARTS_PATHS[@]}"; do
    mode="$(
      git -C "${WORKSPACE}" ls-files --stage -- "${path}" |
        awk 'NR == 1 {print $1}'
    )"
    if [[ "${mode}" == "160000" ]]; then
      SUBMODULE_PATHS+=("${path}")
    elif [[ "${mode}" == 100* && -d "${WORKSPACE}/${path}" ]]; then
      printf 'using vendored third-party source: %s\n' "${path}"
    else
      printf 'not a committed submodule: %s (mode: %s)\n' \
        "${path}" "${mode:-missing}" >&2
      invalid=1
    fi
  done
  if ((invalid)); then
    die "the main repository has not committed all 3parts submodule pointers"
  fi

  if ((${#SUBMODULE_PATHS[@]} > 0)); then
    log "Initializing pinned 3parts submodules"
    git -C "${WORKSPACE}" submodule sync --recursive -- "${SUBMODULE_PATHS[@]}"
    git -C "${WORKSPACE}" submodule update \
      --init --recursive -- "${SUBMODULE_PATHS[@]}"
  fi
fi

CPU_REQUIREMENTS="${WORKSPACE}/project/luxi_hloc/requirements-cpu.txt"
GPU_REQUIREMENTS="${WORKSPACE}/project/luxi_hloc/requirements-gpu.txt"
CPU_TARGET="${WORKSPACE}/3parts/hloc_python"
GPU_TARGET="${WORKSPACE}/3parts/hloc_gpu_python"
MODEL_TARGET="${WORKSPACE}/3parts/hloc_models"

if [[ "${RUNTIME}" != "none" ]]; then
  [[ -f "${CPU_REQUIREMENTS}" ]] ||
    die "missing requirements file: ${CPU_REQUIREMENTS}"

  log "Installing common HLoc dependencies into 3parts/hloc_python"
  mkdir -p -- "${CPU_TARGET}"
  "${PYTHON_BIN}" -m pip install \
    --upgrade \
    --target "${CPU_TARGET}" \
    --requirement "${CPU_REQUIREMENTS}"
fi

if [[ "${RUNTIME}" == "gpu" ]]; then
  [[ -f "${GPU_REQUIREMENTS}" ]] ||
    die "missing requirements file: ${GPU_REQUIREMENTS}"
  [[ -n "${GPU_INDEX_URL}" ]] ||
    die "GPU mode requires --gpu-index-url or LUXI_HLOC_GPU_INDEX_URL"

  log "Installing the CUDA PyTorch overlay into 3parts/hloc_gpu_python"
  mkdir -p -- "${GPU_TARGET}"
  "${PYTHON_BIN}" -m pip install \
    --upgrade \
    --no-deps \
    --target "${GPU_TARGET}" \
    --index-url "${GPU_INDEX_URL}" \
    --requirement "${GPU_REQUIREMENTS}"
fi

if ((DOWNLOAD_MODELS)); then
  [[ -d "${CPU_TARGET}" ]] ||
    die "3parts/hloc_python is missing; install a runtime or use --skip-models"
  [[ -d "${WORKSPACE}/3parts/hloc" ]] ||
    die "HLoc source is missing; initialize the submodules first"
  [[ -d "${WORKSPACE}/3parts/lightglue" ]] ||
    die "LightGlue source is missing; initialize the submodules first"
  command -v wget >/dev/null 2>&1 ||
    die "wget is required to download the NetVLAD model"

  log "Downloading HLoc model weights into 3parts/hloc_models"
  mkdir -p -- "${MODEL_TARGET}"
  env \
    PYTHONPATH="${CPU_TARGET}:${WORKSPACE}/3parts/hloc:${WORKSPACE}/3parts/lightglue:${WORKSPACE}/project/luxi_hloc" \
    TORCH_HOME="${MODEL_TARGET}" \
    "${PYTHON_BIN}" - <<'PY'
from luxi_hloc.inference import HlocFeatureBackend

HlocFeatureBackend(
    device="cpu",
    resize_max=640,
    max_keypoints=1024,
    cpu_threads=1,
)
print("HLoc model weights are ready.")
PY
fi

if [[ "${RUNTIME}" != "none" ]]; then
  log "Checking the installed HLoc runtime"
  check_args=()
  if [[ "${RUNTIME}" == "gpu" ]]; then
    check_args+=(--require-cuda)
  fi
  env \
    LUXI_HLOC_RUNTIME="${RUNTIME}" \
    "${PYTHON_BIN}" \
    "${WORKSPACE}/project/luxi_hloc/scripts/check_hloc_environment.py" \
    "${check_args[@]}"
fi

log "3parts setup completed"
