#!/usr/bin/env bash
##
## BSD 3-Clause License
##
## This file is part of the RootBA project.
## https://github.com/NikolausDemmel/rootba
##
## Copyright (c) 2021, Nikolaus Demmel.
## All rights reserved.
##

# Usage:
#    download-bal-problems.sh PRESET
#
# Downloads and unzips BAL problems from
# https://grail.cs.washington.edu/projects/bal/ into
# ../rootba_data/bal/.
#
# PRESET defines which problems to download. Possible values are:
#
# all: download all 97 problems
# tutorial: download the 10 ladybug problems used in the batch evaluation tutorial

# exit on error
set -o errexit -o pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

DATA_DIR="$SCRIPT_DIR/../../rootba_data/bal"

TUTORIAL_DATASETS=(
    ladybug-49-7776
    ladybug-73-11032
    ladybug-138-19878
    ladybug-318-41628
    ladybug-372-47423
    ladybug-412-52215
    ladybug-460-56811
    ladybug-539-65220
    ladybug-598-69218
    ladybug-646-73584
)

ALL_DATASETS=(
    ladybug-49-7776
    ladybug-73-11032
    ladybug-138-19878
    ladybug-318-41628
    ladybug-372-47423
    ladybug-412-52215
    ladybug-460-56811
    ladybug-539-65220
    ladybug-598-69218
    ladybug-646-73584
    ladybug-707-78455
    ladybug-783-84444
    ladybug-810-88814
    ladybug-856-93344
    ladybug-885-97473
    ladybug-931-102699
    ladybug-969-105826
    ladybug-1031-110968
    ladybug-1064-113655
    ladybug-1118-118384
    ladybug-1152-122269
    ladybug-1197-126327
    ladybug-1235-129634
    ladybug-1266-132593
    ladybug-1340-137079
    ladybug-1469-145199
    ladybug-1514-147317
    ladybug-1587-150845
    ladybug-1642-153820
    ladybug-1695-155710
    ladybug-1723-156502

    trafalgar-21-11315
    trafalgar-39-18060
    trafalgar-50-20431
    trafalgar-126-40037
    trafalgar-138-44033
    trafalgar-161-48126
    trafalgar-170-49267
    trafalgar-174-50489
    trafalgar-193-53101
    trafalgar-201-54427
    trafalgar-206-54562
    trafalgar-215-55910
    trafalgar-225-57665
    trafalgar-257-65132

    dubrovnik-16-22106
    dubrovnik-88-64298
    dubrovnik-135-90642
    dubrovnik-142-93602
    dubrovnik-150-95821
    dubrovnik-161-103832
    dubrovnik-173-111908
    dubrovnik-182-116770
    dubrovnik-202-132796
    dubrovnik-237-154414
    dubrovnik-253-163691
    dubrovnik-262-169354
    dubrovnik-273-176305
    dubrovnik-287-182023
    dubrovnik-308-195089
    dubrovnik-356-226730

    venice-52-64053
    venice-89-110973
    venice-245-198739
    venice-427-310384
    venice-744-543562
    venice-951-708276
    venice-1102-780462
    venice-1158-802917
    venice-1184-816583
    venice-1238-843534
    venice-1288-866452
    venice-1350-894716
    venice-1408-912229
    venice-1425-916895
    venice-1473-930345
    venice-1490-935273
    venice-1521-939551
    venice-1544-942409
    venice-1638-976803
    venice-1666-983911
    venice-1672-986962
    venice-1681-983415
    venice-1682-983268
    venice-1684-983269
    venice-1695-984689
    venice-1696-984816
    venice-1706-985529
    venice-1776-993909
    venice-1778-993923

    final-93-61203
    final-394-100368
    final-871-527480
    final-961-187103
    final-1936-649673
    final-3068-310854
    final-4585-1324582
    final-13682-4456117
)

if [[ $# -ne 1 ]]; then
    echo "Please pass 'all' or 'tutorial'"
    exit 1
fi

case $1 in
    all)
        DATASETS=("${ALL_DATASETS[@]}")
        ;;
    tutorial)
        DATASETS=("${TUTORIAL_DATASETS[@]}")
        ;;
    *)
        echo "Unknown argument $1"
        exit 1
        ;;
esac



read -p "Will download ${#DATASETS[@]} BAL datasets into $DATA_DIR. Continue [yN]? " -r
if ! [[ $REPLY =~ ^[Yy]$ ]]
then
    echo "Aborted..."
    exit 1
fi


echo "Ensuring '$DATA_DIR' exists..."
mkdir -p "$DATA_DIR"


for d in "${DATASETS[@]}"; do
    # split string on first "-" character
    type=${d%%-*}
    problem=${d#*-}

    OUTPUT_DIR="$DATA_DIR/$type/"
    mkdir -p "$OUTPUT_DIR"
    cd "$OUTPUT_DIR"
    
    filename="problem-${problem}-pre.txt"
    filename_zip="${filename}.bz2"

    if [ -f "$filename" ]; then
        echo "Skipping $type/$filename (exists)"
        continue
    fi

    echo "Downloading $type/$filename"

    URL="https://grail.cs.washington.edu/projects/bal/data/$type/$filename_zip"

    curl "$URL" -O

    bzip2 -d "$filename_zip" -f
done
