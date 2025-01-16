target = 'include/autons.h'

with open(target, 'r') as file:
    text = file.read()
    lines = text.split('\n')

autonNames = []
functionTextToReplace = ''
namesLineToReplace = ''
enumFound = False
for line in lines:
    # AUTONS
    if 'enum Auton' in line:
        enumFound = True
    elif enumFound and '}' in line:
        enumFound = False
    elif enumFound:
        autonNames.append(line.strip().split(' ')[0])

    # AUTON FUNCTIONS
    if 'void' in line:
        functionTextToReplace += line + '\n'

    # NAMES LINE
    if 'AUTON_NAMES' in line:
        namesLineToReplace = line

if functionTextToReplace == '':
    print('Auton functions not found!')
    exit(1)

newFunctionText = ''
for autonName in autonNames:
    newFunctionText += f'void {autonName.lower()}();\n'
newFunctionText = '\n'.join(sorted(newFunctionText.split('\n'))).strip() + '\n'

newNamesLine = 'const std::vector<std::string> AUTON_NAMES = {'
for autonName in autonNames:
    newNamesLine += f'"{autonName}", '
newNamesLine = newNamesLine[:-2] + '};'

with open(target, 'w') as file:
    file.write(text.replace(functionTextToReplace, newFunctionText).replace(namesLineToReplace, newNamesLine))