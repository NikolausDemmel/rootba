# to be sourced in .gitlab-ci.yml
mkdir -p ccache
export CCACHE_BASEDIR=${PWD}
export CCACHE_DIR=${PWD}/ccache
ccache -M 50G
ccache -s
