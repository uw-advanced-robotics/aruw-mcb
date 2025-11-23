# Copyright (c) 2023-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
#
# This file is part of aruw-mcb.
#
# aruw-mcb is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# aruw-mcb is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.



import argparse
import re
import sys
import pathlib
import fnmatch
from typing import List

SOURCE_FILE_EXTENSIONS = ['.cpp', '.hpp', '.h']
SRC_ROOT = pathlib.Path("aruw-mcb-project/src")

# regexes
NAMESPACE_OPEN_INLINE = re.compile(r'\bnamespace\s+([a-zA-Z_][a-zA-Z0-9_:]*)\s*\{')
NAMESPACE_ONLY = re.compile(r'\bnamespace\s+([a-zA-Z_][a-zA-Z0-9_:]*)\s*$')
BRACE_OPEN = re.compile(r'\{')
BRACE_CLOSE = re.compile(r'\}')

def parse_args():
    arg = argparse.ArgumentParser(
        description='Script that validates class namespaces have valid structure. A valid\
                namespace takes the form of the folder structure where :: delimitates a \
                subfolder and any - in the filename must be a _ ')
    arg.add_argument(
        '-i', '--ignored',
        nargs='*',
        help='files to ignore when validating namespaces',
        required=False,
        default=[])
    arg.add_argument(
        '-rn', '--removeNamespace',
        nargs = "*",
        help="list of namespaces that shouldn't be apart of the check",
        required=False,
        default=[])
    return arg.parse_args()


# helpers to strip comments and string literals
def strip_comments_and_strings(text: str) -> str:
    # This is a pragmatic approach (not a full lexer) but works well for our needs.
    # block comments
    text = re.sub(r'/\*.*?\*/', lambda m: ' ' * (m.end() - m.start()), text, flags=re.S)
    # string literals (double and single) - naive but OK
    text = re.sub(r'".*?(?<!\\)"', lambda m: ' ' * (m.end() - m.start()), text, flags=re.S)
    text = re.sub(r"'.*?(?<!\\)'", lambda m: ' ' * (m.end() - m.start()), text, flags=re.S)
    # line comments
    text = re.sub(r'//.*', lambda m: ' ' * (m.end() - m.start()), text)
    return text

def extract_primary_namespace(path: pathlib.Path) -> List[str]:
    """
    Return list of namespace components for the primary namespace that contains code,
    or [] if none found.
    """
    text = path.read_text(encoding='utf8')
    cleaned = strip_comments_and_strings(text)

    # We'll scan token by token (line-based is fine), but need to handle brace counting.
    ns_stack = []             # stack of namespace component-lists
    ns_blocks = []            # snapshots when we open a namespace (list of stacks)
    brace_count = 0

    lines = cleaned.splitlines()
    i = 0
    while i < len(lines):
        line = lines[i].strip()

        # try inline pattern: namespace foo::bar {
        m_inline = NAMESPACE_OPEN_INLINE.search(line)
        if m_inline:
            parts = m_inline.group(1).split("::")
            ns_stack.append(parts)
            ns_blocks.append([p for stack in ns_stack for p in stack])  # flatten snapshot
            # count braces on this line (might be more than one)
            brace_count += len(BRACE_OPEN.findall(line)) - len(BRACE_CLOSE.findall(line))
            i += 1
            continue

        # try 'namespace foo' with brace on next line
        m_only = NAMESPACE_ONLY.search(line)
        if m_only:
            # peek next non-empty line for a '{'
            j = i + 1
            while j < len(lines) and lines[j].strip() == "":
                j += 1
            if j < len(lines) and lines[j].find("{") != -1:
                parts = m_only.group(1).split("::")
                ns_stack.append(parts)
                ns_blocks.append([p for stack in ns_stack for p in stack])
                # increment brace count for that found brace(s)
                brace_count += len(BRACE_OPEN.findall(lines[j])) - len(BRACE_CLOSE.findall(lines[j]))
                # advance i to line j+1 because we consumed the next line's brace
                i = j + 1
                continue

        # track braces generally
        # (if there are closes and we have namespaces stacked, pop appropriately)
        opens = len(BRACE_OPEN.findall(line))
        closes = len(BRACE_CLOSE.findall(line))
        brace_count += opens - closes

        # If there are closing braces and we have ns_stack entries — try to pop.
        # We can't know which '}' corresponds to a namespace vs other block exactly,
        # but popping when we see closes keeps a reasonable balance for typical code.
        for _ in range(closes):
            if ns_stack:
                ns_stack.pop()

        i += 1

    if not ns_blocks:
        return []
    # ns_blocks holds flattened snapshots; the last one is the deepest + last opened namespace
    last_flat = ns_blocks[-1]
    return last_flat


def expected_namespace_from_path(path: pathlib.Path, exclude: List[str]):
    # compile regex patterns
    exclude_patterns = [re.compile(p) for p in exclude]

    try:
        rel = path.relative_to(SRC_ROOT)
    except ValueError:
        return []

    parts = rel.parts

    if "aruwsrc" not in parts:
        return []

    idx = parts.index("aruwsrc")

    # everything after aruwsrc/, except the filename
    ns_parts = list(parts[idx:-1])

    # replace '-' with '_'
    ns_parts = [p.replace("-", "_") for p in ns_parts]

    # remove any namespace parts matching exclude regex
    ns_parts = [
        p for p in ns_parts
        if not any(pat.fullmatch(p) for pat in exclude_patterns)
    ]

    return ns_parts




def check_file(path: pathlib.Path, exclude: List[str]):
    expected = expected_namespace_from_path(path, exclude)
    declared = extract_primary_namespace(path)

    # No namespace declared → allow (e.g. forward-declare-only headers)
    if not declared:
        return True

    if declared != expected:
        print(f"  \033[31m Namespace mismatch in {path} \033[39m")
        print(f"   Expected: {'::'.join(expected) or '<none>'}")
        print(f"   Found:    {'::'.join(declared) or '<none>'}")
        return False

    return True



def main():
    args = parse_args()
    
    if not SRC_ROOT.exists():
        print(f"ERROR: Not in source root: {SRC_ROOT}")
        sys.exit(1)
        
    ignored = set(args.ignored)
    
    def is_ignored(path):
        # match file name
        for pattern in ignored:
            if fnmatch.fnmatch(path.name, pattern):
                return True
        # match any folder name in parents
        for parent in path.parents:
            for pattern in ignored:
                if fnmatch.fnmatch(parent.name, pattern):
                    return True
        return False
    

    bad_files = 0

    for path in SRC_ROOT.rglob("*"):
        if is_ignored(path):
            continue

        if path.suffix in (".cpp", ".hpp", ".h", ".cc"):
            if path.name in ignored:
                continue
            if not check_file(path, args.removeNamespace):
                bad_files += 1

    if bad_files > 0:
        print(f"\nNamespace path check FAILED. {bad_files} file(s) have incorrect namespaces.")
        sys.exit(1)
    else:
        print("Namespace path check passed.")


if __name__ == "__main__":
    main()
