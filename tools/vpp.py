import os
import subprocess


if not os.path.exists('vpp'):
    if os.path.exists('vex'):
        root = '.'
    else:
        root = '..'
else:
    root = '.'

def containsKeywords(line: str, keywords: list):
    for keyword in keywords:
        if keyword in line:
            return True
    return False

def combineLikeFiles(files: list):
    filenames = []
    allContent = ''
    for file in files:
        with open(file, 'r') as f:
            filenames.append(file.split('/')[-1])
            allContent += f'\n// {filenames[-1]}\n{f.read().strip('\n').replace('\n\n', '\n')}\n'

    includeCache = []
    def storeAndRemoveIncludes(line: str):
        if (line.startswith('#include')):
            if line not in includeCache and not containsKeywords(line, filenames):
                includeCache.append(line)
            return False
        elif (line.startswith('#pragma once')):
            return False
        return True

    codeOnly = list(filter(storeAndRemoveIncludes, allContent.split('\n')))

    output = '#pragma once\n' + ('\n'.join(includeCache) + '\n') + ('\n'.join(codeOnly))

    return output

def vppFiles(path: str):
    try:
        return list(filter(
            lambda x: x.endswith('.h') or x.endswith('.cpp'),
            subprocess.check_output(f'find {path} | grep vpp', shell=True).decode('utf-8').strip('\n').split('\n')
        ))
    except subprocess.CalledProcessError as e:
        return ''

headerFiles = vppFiles(f'{root}/include')
sourceFiles = vppFiles(f'{root}/src')

with open(f'{root}/tools/vpp.h', 'w') as f:
    f.write(combineLikeFiles(headerFiles))

if sourceFiles:
    with open(f'{root}/tools/vpp.cpp', 'w') as f:
        f.write(combineLikeFiles(sourceFiles))

os.system(f'clang-format -style=file:{root}/.clang-format -i {root}/tools/vpp.h')