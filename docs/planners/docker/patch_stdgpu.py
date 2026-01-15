#!/usr/bin/env python3
import pathlib
import re
import sys


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: patch_stdgpu.py <path-or-dir>", file=sys.stderr)
        return 2
    target = pathlib.Path(sys.argv[1])
    if target.is_dir():
        matches = list(target.rglob("memory_detail.h"))
        if not matches:
            print(f"no memory_detail.h under {target}", file=sys.stderr)
            return 3
        path = matches[0]
    else:
        path = target
    data = path.read_text()
    data = re.sub(r"(?<!::)\bdestroy_at\(", "stdgpu::destroy_at(", data)
    data = re.sub(r"(?<!::)\bconstruct_at\(", "stdgpu::construct_at(", data)
    data = re.sub(r"(?<!::)\bforward<", "stdgpu::forward<", data)
    path.write_text(data)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
