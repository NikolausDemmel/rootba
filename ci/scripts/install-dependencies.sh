# to be sourced in .gitlab-ci.yml

if [ "$UBUNTU_INSTALL_LATEST_TBB" = 1 ]; then
    wget -O - https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB | apt-key add -
    wget https://apt.repos.intel.com/setup/intelproducts.list -O /etc/apt/sources.list.d/intelproducts.list
    apt-get update
    apt-get install -y intel-tbb-2020.2-102
fi
