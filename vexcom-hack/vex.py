import os

def fetch_vexcom(directory = "~/.vscode/extensions", platform_folder = "osx"):
    extensions_directory = os.path.expanduser(directory) 
    vexcom_binary_directory = extensions_directory + "/" + list(filter(lambda path: 'vexcode' in path, os.listdir(extensions_directory)))[0] + "/resources/tools/vexcom/" + platform_folder
    os.system(f"cp -r {vexcom_binary_directory} bin")

fetch_vexcom()