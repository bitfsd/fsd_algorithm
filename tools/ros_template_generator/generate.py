import shutil,os

def generate_file(file,old_str,new_str):
    file_data = ''
    with open(file, "r", encoding="utf-8") as f:
        for line in f:
            if old_str in line:
                line = line.replace(old_str,new_str)
            file_data += line
    with open(file,"w",encoding="utf-8") as f:
        f.write(file_data)

if __name__ == "__main__":

    language = input("Select language: \n【1】C++\n【2】Python\n")
    if language == '1':

        package = input("Input package name: ")
        small = input("Input object name (lower case): ")
        big = input("Input class name (capital): ")
        shutil.copytree("roscpp-template",package)

        #rename
        shutil.move(package+"/config/template.yaml",package+"/config/"+small+".yaml")
        shutil.move(package+"/include/template.hpp",package+"/include/"+small+".hpp")
        shutil.move(package+"/include/template_handle.hpp",package+"/include/"+small+"_handle.hpp")
        shutil.move(package+"/launch/template.launch",package+"/launch/"+small+".launch")
        shutil.move(package+"/src/template.cpp",package+"/src/"+small+".cpp")
        shutil.move(package+"/src/template_handle.cpp",package+"/src/"+small+"_handle.cpp")

        #small
        generate_file(package+"/include/"+small+".hpp",'template',small)
        generate_file(package+"/include/"+small+"_handle.hpp",'template',small)
        generate_file(package+"/src/"+small+"_handle.cpp",'template',small)
        generate_file(package+"/src/"+small+".cpp",'template',small)
        generate_file(package+"/src/"+"main.cpp",'template',small)
        generate_file(package+"/CMakeLists.txt",'template',small)
        generate_file(package+"/package.xml",'template',small)
        generate_file(package+"/launch/"+small+".launch",'template',small)
        generate_file(package+"/config/"+small+".yaml",'template',small)

        #big
        generate_file(package+"/include/"+small+".hpp",'Template',big)
        generate_file(package+"/include/"+small+"_handle.hpp",'Template',big)
        generate_file(package+"/src/"+small+"_handle.cpp",'Template',big)
        generate_file(package+"/src/"+small+".cpp",'Template',big)
        generate_file(package+"/src/"+"main.cpp",'Template',big)

        #upper
        generate_file(package+"/include/"+small+"_handle.hpp",'TEMPLATE',small.upper())
        generate_file(package+"/include/"+small+".hpp",'TEMPLATE',small.upper())

    elif language == '2':

        package = input("Input package name: ")
        small = input("Input object name (lower case): ")
        big = input("Input class name (capital): ")
        shutil.copytree("rospy-template",package)

        #rename
        shutil.move(package+"/config/template.yaml",package+"/config/"+small+".yaml")
        shutil.move(package+"/bin/template_node",package+"/bin/"+small+"_node")
        shutil.move(package+"/launch/template.launch",package+"/launch/"+small+".launch")
        shutil.copytree(package+"/src/template",package+"/src/"+small)
        shutil.move(package+"/src/"+small+"/"+"template.py",package+"/src/"+small+"/"+small+".py")
        shutil.rmtree(package+"/src/template", ignore_errors=False, onerror=None)

        #small
        generate_file(package+"/bin/"+small+"_node",'template',small)
        generate_file(package+"/src/"+small+"/"+small+".py",'template',small)
        generate_file(package+"/CMakeLists.txt",'template',small)
        generate_file(package+"/package.xml",'template',small)
        generate_file(package+"/launch/"+small+".launch",'template',small)
        generate_file(package+"/config/"+small+".yaml",'template',small)
        generate_file(package+"/setup.py",'template',small)

        #big
        generate_file(package+"/bin/"+small+"_node",'Template',big)
        generate_file(package+"/src/"+small+"/"+small+".py",'Template',big)
        generate_file(package+"/CMakeLists.txt",'Template',big)
        generate_file(package+"/package.xml",'Template',big)
        generate_file(package+"/launch/"+small+".launch",'Template',big)
        generate_file(package+"/config/"+small+".yaml",'Template',big)
        generate_file(package+"/setup.py",'Template',big)


