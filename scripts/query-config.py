#!/usr/bin/env python3
#
# BSD 3-Clause License
#
# This file is part of the RootBA project.
# https://github.com/NikolausDemmel/rootba
#
# Copyright (c) 2021, Nikolaus Demmel.
# All rights reserved.
#

#
# Example usage:
#     $ ./query-config.py path/to/rooba_config.toml slurm.mem
#     10G

import toml
import argparse
import munch
import sys


def query_config(path, query, default_value=None, format_env=False):
    cfg = munch.munchify(toml.load(path))
    try:
        result = munch.unmunchify(eval("cfg.{}".format(query)))
    except:
        if default_value is None:
            result = ""
        else:
            result = default_value
    if isinstance(result, dict):
        if format_env:
            lines = []
            for k, v in result.items():
                # NOTE: assumes no special escaping is necessary
                lines.append("{}='{}'".format(k, v))
            return "\n".join(lines)
        else:
            result = toml.dumps(result)
    else:
        result = "{}".format(result)
    return result


def main():
    parser = argparse.ArgumentParser("Parse toml file and print content of query key.")
    parser.add_argument("config_path", help="path to toml file")
    parser.add_argument("query", help="query string")
    parser.add_argument("default_value", help="value printed if query is not successful", nargs='?')
    parser.add_argument(
        "--format-env",
        action="store_true",
        help="Expect dictionary as query result and output like environment variables, i.e. VAR='VALUE' lines.")
    args = parser.parse_args()

    res = query_config(args.config_path, args.query, default_value=args.default_value, format_env=args.format_env)
    print(res)


if __name__ == "__main__":
    main()
