##
## BSD 3-Clause License
##
## This file is part of the RootBA project.
## https://github.com/NikolausDemmel/rootba
##
## Copyright (c) 2021, Nikolaus Demmel.
## All rights reserved.
##

# helper to detect the CPU type for our slurm machines
case `LC_ALL=C lscpu | grep 'Model name'` in
    *"Intel(R) Xeon(R) CPU E5-26"*)
        ARCH_SUFFIX=sbep
        ;;
    *"Intel(R) Xeon(R) Gold 6148"*)
        ARCH_SUFFIX=skyl
        ;;
    *"Intel(R) Xeon(R) Gold 6248"*)
        ARCH_SUFFIX=cl
        ;;
    *"Intel(R) Xeon(R) Gold 6254"*)
        ARCH_SUFFIX=cl
        ;;
    *)
        ARCH_SUFFIX=unknown
        ;;
esac

if [ "$ARCH_SUFFIX" == unknown ]; then
    echo "Could not detect CPU architecture. Are you running this on slurm?"
    exit 1
fi

