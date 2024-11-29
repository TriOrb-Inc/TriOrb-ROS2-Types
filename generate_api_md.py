#!/usr/bin/env python
# -*- coding: utf-8 -*-
# **
# * Copyright 2023 TriOrb Inc.
# *
# * Licensed under the Apache License, Version 2.0 (the "License");
# * you may not use this file except in compliance with the License.
# * You may obtain a copy of the License at
# *
# *     http://www.apache.org/licenses/LICENSE-2.0
# *
# * Unless required by applicable law or agreed to in writing, software
# * distributed under the License is distributed on an "AS IS" BASIS,
# * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# * See the License for the specific language governing permissions and
# * limitations under the License.
# **

import os
import glob
import re
import argparse
import subprocess

BLACK_LIST = ['rosbridge_suite', 'Version.srv']

parser = argparse.ArgumentParser(description='API Referenceを生成する')
parser.add_argument('-output', '-o', default="./README.md", help='生成するMarkdownファイルのパス')
parser.add_argument('-version', '-v', default="1.0.0", help='パッケージのバージョン')
parser.add_argument('-hash', default="", help='パッケージのバージョン(HASH)')
parser.add_argument('-date', default="", help='パッケージのバージョン(Date)')
#parser.add_argument('pkg_name', help='パッケージ名')
#parser.add_argument('dir', help='パッケージディレクトリ')
args = parser.parse_args()
if len(args.hash) <= 0:
    args.hash = subprocess.run("git show --format=%h --no-patch".split(), stdout=subprocess.PIPE).stdout.decode('utf-8').strip()
if len(args.date) <= 0:
    args.date = subprocess.run("date +%Y-%m-%d".split(), stdout=subprocess.PIPE).stdout.decode('utf-8').strip()

def generate_types_md():
    path_md = os.path.join(".".join(args.output.split(".")[:-1]) + "_types.md")
    repo_name = ""
    with open(path_md, 'w', encoding='utf-8') as f_md:
        f_md.write(f'# TriOrb-ROS2-Types {args.version} ({args.date})\n\n')
        for curDir, dirs, files in os.walk('./'):
            if len([black_word for black_word in BLACK_LIST if black_word in curDir]) > 0:
                continue
            files = [name for name in files if name.endswith('.msg') or name.endswith('.srv') or name.endswith('.action')]
            if len(files) > 0:
                _repo_name = ""
                if "\\" in curDir:
                    _repo_name = curDir.split("\\")[-2].split("/")[-1]
                elif "/" in curDir:
                    _repo_name = curDir.split("/")[-2].split("\\")[-1]
                if _repo_name != repo_name:
                    repo_name = _repo_name
                    f_md.write(f'# {repo_name} \n')
                package_name = "/".join(("/".join(curDir.split("\\")[-2:])).split("/")[-2:])
                f_md.write(f'## {package_name} \n')
            for file in files:
                file_path = os.path.join(curDir, file)
                interface_name = "/".join(("/".join(file_path.split("\\")[-3:])).split("/")[-3:])
                if len([black_word for black_word in BLACK_LIST if black_word in interface_name]) > 0:
                    continue
                try:
                    with open(file_path, 'r', encoding='utf-8') as f:
                        text_raw = f.read()
                        # '#**から#**'で囲まれた部分を削除する
                        remove_spans = []
                        MatchObjects = list(re.finditer(r"#\*\*.*\n", text_raw))
                        for i in range(0, len(MatchObjects), 2):
                            remove_spans.append((MatchObjects[i].start(), MatchObjects[i+1].end()))
                        for remove_span in remove_spans[::-1]:
                            text_raw = text_raw[:remove_span[0]] + text_raw[remove_span[1]:]
                        text_lines = text_raw.split('\n')
                        while len(text_lines[0]) <= 0:
                            text_lines.pop(0)
                        while len(text_lines[-1]) <= 0:
                            text_lines.pop(-1)
                        text_md = ("\n".join(text_lines))
                        text_md = "```bash\n" + text_md + "\n```"
                        text_md = "### " + interface_name + "\n" + text_md + "\n\n"
                        print(interface_name)
                        f_md.write(text_md)
                except Exception as e:
                    print(f'Failed to open {file_path}')
                    import traceback
                    traceback.print_exc()
                    raise e

if __name__ == '__main__':
    print('API Referenceを生成します...')
    print(args)
    
    generate_types_md()