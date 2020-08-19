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
#    slurm-list-jobs.sh DIRNAME [DIRNAME ...] [-s|--short] [-o|--only STATUS]
#
# Lists all (slurm) batch jobs found in DIRNAME. If the optional argument
# STATUS is passed, only lists jobs with that status. Multiple
# statuses can be passed in a space-separated string.
#
# Possible status arguments: queued, running, completed, failed, unknown
# You can also use 'active' as a synonym for 'queued running unknown'
#
# 'unknown' tries to detect a current issue reading files from
# /storage/slurm via NFS.
#
# Examples:
# $ path/to/slurm-list-jobs.sh 06_rho_01_const_rho_noab/ "unkown running"
# 06_rho_01_const_rho_noab/20200220-021002/pen/pen_lmdup_cpu5_addobs2_filterobs_rho1e2_rhoscale1em2t_it100_noab_kitti00 : running (162294 R 4:24:48 node3 5 50G)
# 06_rho_01_const_rho_noab/20200220-021002/pen/pen_lmdup_cpu5_addobs2_filterobs_rho1e2_rhoscale1em2t_it100_noab_kitti02 : running (162297 R 4:23:24 node5 5 50G)
# 06_rho_01_const_rho_noab/20200220-021002/pen/pen_lmdup_cpu5_addobs2_filterobs_rho1e2_rhoscale1e1t_it100_noab_kitti08 : running (162296 R 4:23:30 node5 5 50G)
# 06_rho_01_const_rho_noab/20200220-021002/pen/pen_lmdup_cpu5_addobs2_filterobs_rho1e3_rhoscale1em1t_it100_noab_kitti02 : unknown (10 0x00 bytes)
# 06_rho_01_const_rho_noab/20200220-021002/pen/pen_lmdup_cpu5_addobs2_filterobs_rho1e3_rhoscale1em1t_it100_noab_kitti00 : running (161796 R 17:45:43 node4 5 50G)
# 06_rho_01_const_rho_noab/20200220-021002/pen/pen_lmdup_cpu5_addobs2_filterobs_rho1e3_rhoscale1em2t_it100_noab_kitti00 : running (161808 R 14:54:10 node4 5 50G)
# 06_rho_01_const_rho_noab/20200220-021002/pen/pen_lmdup_cpu5_addobs2_filterobs_rho1e2_rhoscale1em1t_it100_noab_kitti08 : running (162295 R 4:23:39 node5 5 50G)
# 06_rho_01_const_rho_noab/20200220-021002/pen/pen_lmdup_cpu5_addobs2_filterobs_rho1e3_rhoscale1em1t_it100_noab_kitti08 : unknown (10 0x00 bytes)
# 06_rho_01_const_rho_noab/20200220-021002/pen/pen_lmdup_cpu5_addobs2_filterobs_rho1e3_rhoscale1e1t_it100_noab_kitti00 : running (161784 R 19:41:01 node2 5 50G)


# exit on error
set -o errexit -o pipefail


# we need GNU getopt...
GETOPT=getopt
if [[ "$OSTYPE" == "darwin"* ]]; then
    if [ -f /usr/local/opt/gnu-getopt/bin/getopt ]; then
        GETOPT="/usr/local/opt/gnu-getopt/bin/getopt"
    fi
fi

# option parsing, see: https://stackoverflow.com/a/29754866/1813258
usage() { echo "Usage: `basename $0` DIRNAME [DIRNAME ...] [-s|--short] [-o|--only STATUS]" ; exit 1; }

# -allow a command to fail with !’s side effect on errexit
# -use return value from ${PIPESTATUS[0]}, because ! hosed $?
! "$GETOPT" --test > /dev/null
if [[ ${PIPESTATUS[0]} -ne 4 ]]; then
    echo 'I’m sorry, `getopt --test` failed in this environment.'
    exit 1
fi

OPTIONS=hsjo:
LONGOPTS=help,short,jobids,only:

# -regarding ! and PIPESTATUS see above
# -temporarily store output to be able to check for errors
# -activate quoting/enhanced mode (e.g. by writing out “--options”)
# -pass arguments only via   -- "$@"   to separate them correctly
! PARSED=$("$GETOPT" --options=$OPTIONS --longoptions=$LONGOPTS --name "`basename $0`" -- "$@")
if [[ ${PIPESTATUS[0]} -ne 0 ]]; then
    # e.g. return value is 1
    #  then getopt has complained about wrong arguments to stdout
    usage
fi
# read getopt’s output this way to handle the quoting right:
eval set -- "$PARSED"

SHORT=n
ONLY=""
JOBIDS=n
# now enjoy the options in order and nicely split until we see --
while true; do
    case "$1" in
        -h|--help) usage ;;
        -s|--short) SHORT=y; shift ;;
        -j|--jobids) JOBIDS=y; shift ;;
        -o|--only) ONLY="$2"; shift 2 ;;
        --) shift; break ;;
        *) echo "Programming error"; exit 3 ;;
    esac
done

# handle non-option arguments --> directories
if [[ $# -lt 1 ]]; then
    echo "Error: Pass at least one folder"
    usage
fi
DIRS=("$@")


# query slurm queue
RUNNING_JOBS=`squeue -u $USER -o "%.0i %.0t %.0M %R %.0C %.0m" -h`

# status aliases:
ONLY="${ONLY/active/queued running unknown}"
ONLY="${ONLY/notcompleted/queued running failed unknown}"

contains() {
    [[ $1 =~ (^| )$2($| ) ]] && return 0 || return 1
}

zero_bytes() {
    [ `hexdump -ve '1/1 " %.2x"' "$1" | grep -o 00 | wc -w` -gt 0 ] && return 0 || return 1
}

count_zero_bytes() {
   hexdump -ve '1/1 " %.2x"' "$1" | grep -o 00 | wc -w
}

display() {
    if [ -z "$ONLY" ] || contains "$ONLY" $2; then
        if [ $JOBIDS = y ]; then
            echo $JOBID
        elif [ $SHORT = y ]; then
            echo "$1"
        else
            echo -n "$1 : $2"
            if [ -n "$3" ]; then
                echo -n " - $3"
            fi
            if [ -n "$4" ]; then
                echo -n " ($4)"
            fi
            echo ""
        fi
    fi
}

for d in "${DIRS[@]}"; do
    for f in `find "$d" -name status.log | sort`; do
        DIR=`dirname "$f"`

        # ignore backup folder from "rerun" scripts
        if [[ `basename $DIR` = results-backup* ]]; then
            continue
        fi

        if [ -f "$DIR"/*.jobid ]; then
            JOBID=`ls "$DIR"/*.jobid`
            JOBID=`basename $JOBID`
            JOBID=${JOBID%.jobid}
            QUEUE=`echo "$RUNNING_JOBS" | grep $JOBID || true`
        fi

        if zero_bytes "$f"; then
            # There are 0 bytes in status file. This seems to be an
            # issue with NFS. Cannot tell the actual contents.
            display "$DIR" unknown "" "`count_zero_bytes \"$f\"` 0x00 bytes"
            continue
        fi

        if ! grep Started "$f" > /dev/null; then

            # if slurm job -> check queue, else not sure if queued or aborted
            if [ -n "$JOBID" ]; then
                if [ -n "$QUEUE" ]; then
                    display "$DIR" queued "" "$QUEUE"
                else
                    # Started, but not in slurm queue. E.g. cancelled during queuing.
                    display "$DIR" failed "not started"
                fi
            else
                display "$DIR" unknown "not started"
            fi

            continue
        fi

        # job has started:

        if grep Completed "$f" > /dev/null ; then
            display "$DIR" completed "" "$JOBID"
            continue
        fi

        # job has started, but not completed (cleanly)
        if [ -n "$QUEUE" ]; then
            display "$DIR" running "" "$QUEUE"
            continue
        fi

        # job is not running any more, but not completed -> failed/cancelled/...

        if [ -f "$DIR"/slurm-output.log ] && grep "CANCELLED AT" "$DIR"/slurm-output.log > /dev/null; then
            display "$DIR" failed cancelled "$JOBID"
            continue
        fi

        if [ -f "$DIR"/output.log ] && grep "Command terminated by signal" "$DIR"/output.log > /dev/null; then
            display "$DIR" failed killed "`grep -oP 'Command terminated by \Ksignal .+' "$DIR"/output.log`"
            continue
        fi

        # TODO: add other conditions like OOM, OOT, ...

        if [ -n "$JOBID" ]; then
            # slurm job: failed, but don't know why
            display "$DIR" failed ? "$JOBID"
        else
            # non slurm job: might be running or aborted
            display "$DIR" unknown started
        fi

    done
done
