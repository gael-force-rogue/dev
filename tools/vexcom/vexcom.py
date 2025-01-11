import os

if not os.path.exists('vpp'):
    if os.path.exists('vex'):
        root = '.'
    else:
        root = '..'
else:
    root = '.'

def fetch_vexcom(directory = "~/.vscode/extensions", platform_folder = "osx"):
    extensions_directory = os.path.expanduser(directory) 
    vexcom_binary_directory = extensions_directory + "/" + list(filter(lambda path: 'vexcode' in path, os.listdir(extensions_directory)))[0] + "/resources/tools/vexcom/" + platform_folder
    os.system(f"mkdir -p {root}/tools/bin")
    os.system(f"cp -r {vexcom_binary_directory} {root}/tools/bin")

fetch_vexcom()