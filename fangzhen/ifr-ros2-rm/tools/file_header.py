import os
import re
import xml.etree.ElementTree as ET


def find_matching_folders():
    matching_folders = []
    current_folder = os.getcwd()
    folders = [folder for folder in os.listdir(current_folder) if os.path.isdir(folder)]

    for folder in folders:
        package_xml_path = os.path.join(folder, "package.xml")
        if os.path.exists(package_xml_path):
            tree = ET.parse(package_xml_path)
            root = tree.getroot()
            package_name = root.find("name")
            if package_name is not None and package_name.text == folder:
                matching_folders.append(folder)
            else:
                print(f"警告：文件夹 '{folder}' 的 包名 '{None if package_name is None else package_name.text}' 与文件夹名不匹配！")

    return matching_folders


def process_matching_folders(matching_folders):
    for folder in matching_folders:
        include_folder = os.path.join(folder, "include", folder)
        if os.path.exists(include_folder):
            for root, _, files in os.walk(include_folder):
                for file in files:
                    if file.endswith((".h", ".hpp")):
                        file_path = os.path.join(root, file)
                        if not handleHeadFile(folder, file_path, file):
                            return False
    return True


def get_fst_valid_line(getter):
    """
    获取第一个有效行
    """
    line = getter()
    in_commit = False
    while line.startswith("//") or in_commit or re.match(r'^\s*$', line) or line.startswith('/*'):
        if line.startswith('/*'):
            in_commit = True
        elif line.endswith('*/'):
            in_commit = False
        line = getter()
    return line


def get_lst_valid_line(file):
    """
    获取最后一个有效行
    """
    last_line = None
    readed_line = 0
    readed_tmp = 0
    in_commit = False
    for _line in file:
        line = _line.decode()
        if line.startswith("//") or re.match(r'^\s*$', line):
            readed_tmp += 1
        elif line.startswith('/*'):
            in_commit = True
            readed_tmp += 1
        elif in_commit and line.endswith('*/'):
            in_commit = False
            readed_tmp += 1
        elif not in_commit:
            readed_line += readed_tmp+1
            readed_tmp = 0
            last_line = line
    return last_line, readed_line


def modify_line(file_path, mode_lines):
    lines = []
    with open(file_path, 'r') as file:
        lines = file.readlines()

    for line_number, new_content in mode_lines:
        if line_number >= 1 and line_number <= len(lines):
            lines[line_number - 1] = new_content + '\n'

    with open(file_path, 'w') as file:
        file.writelines(lines)


def handleHeadFile(folder: str, file_path: str, rel_path):
    # 在这里编写处理头文件的代码
    rel_path = os.path.relpath(file_path, os.path.join(folder, 'include', folder))
    print(f"处理头文件：{folder} ~ {file_path} ~ {rel_path}")
    first_idx, second_idx, lst_ldx = 0, 0, 0
    with open(file_path, 'rb') as file:
        getter_idx = 0

        def getter():
            nonlocal getter_idx
            getter_idx += 1
            return file.readline().decode()

        first_line = get_fst_valid_line(getter)
        first_idx = getter_idx
        second_line = getter()
        second_idx = getter_idx
        last_line, lst_ldx = get_lst_valid_line(file)
        lst_ldx += getter_idx

    # 提取宏名称
    match = re.search(r'#ifndef\s+(\w+)', first_line)
    if match:
        macro_name = match.group(1)
        print("提取的宏名称：", macro_name)

        # 判断第二行和最后一行是否符合要求
        pattern_second = r'^\s*#define\s+' + re.escape(macro_name) + r'(\s+.*?)?\s*$'
        pattern_last = r'^\s*#endif\s*(\/\/\s*' + re.escape(macro_name) + r')?\s*$'
        if re.match(pattern_second, second_line) and re.match(pattern_last, last_line):
            print(f"符合要求 {first_idx} {second_idx} {lst_ldx}")
        else:
            print(" ===================== 不符合要求", '\n')
            return False
    else:
        print(" ===================== 未能提取到宏名称", '\n')
        return False
    macro_name = f"IFR_ROS2_CV__PACKAGE_{folder.upper()}__{rel_path.replace('.','__').replace('/','___').replace('-','_').upper()}"
    print("新的宏名称：", macro_name, '\n')
    modify_line(file_path, [
        (first_idx, f"#ifndef {macro_name}"),
        (second_idx, f"#define {macro_name}"),
        (lst_ldx, f"#endif// {macro_name}"),
    ])
    return True


matching_folders = find_matching_folders()

print("匹配的文件夹列表：")
for folder in matching_folders:
    print(folder)

process_matching_folders(matching_folders)
