TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "${TOP_DIR}/scripts/apollo.bashrc"

export HOST_ARCH="$(uname -m)"
export HOST_OS="$(uname -s)"

GEO_REGISTRY=
function geo_specific_config() {
    local geo="$1"
    if [[ -z "${geo}" ]]; then
        info "Use default GeoLocation settings"
        return
    fi
    info "Setup geolocation specific configurations for ${geo}"
    if [[ "${geo}" == "cn" ]]; then
        info "GeoLocation settings for Mainland China"
        GEO_REGISTRY="registry.baidubce.com"
    else
        info "GeoLocation settings for ${geo} is not ready, fallback to default"
    fi
}