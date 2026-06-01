#!/usr/bin/env python3
import os

# Ignore compiled/cache directories to keep the context clean
IGNORE_DIRS = {'build', 'install', 'log', '__pycache__', '.git'}
# We explicitly ignore the 'worlds' folder because the generated maze.sdf is massive 
# and will consume too much context memory in the next chat. The LLM only needs to 
# know the world exists, not its exact XML layout.

TARGET_EXTS = {'.py', '.cpp', '.xml', '.yaml', '.xacro', '.urdf', '.txt', '.md'}
TARGET_FILES = {'CMakeLists.txt', 'package.xml','commands.txt'}

def generate_tree(dir_path, prefix=""):
    """Generates a visual directory tree structure."""
    tree_str = ""
    try:
        items = sorted(os.listdir(dir_path))
    except PermissionError:
        return ""

    dirs = [d for d in items if os.path.isdir(os.path.join(dir_path, d)) and d not in IGNORE_DIRS]
    files = [f for f in items if os.path.isfile(os.path.join(dir_path, f)) and 
             (os.path.splitext(f)[1] in TARGET_EXTS or f in TARGET_FILES)]

    for i, d in enumerate(dirs):
        connector = "├── " if i < len(dirs) - 1 or len(files) > 0 else "└── "
        tree_str += f"{prefix}{connector}{d}/\n"
        extension = "│   " if i < len(dirs) - 1 or len(files) > 0 else "    "
        tree_str += generate_tree(os.path.join(dir_path, d), prefix + extension)

    for i, f in enumerate(files):
        connector = "├── " if i < len(files) - 1 else "└── "
        tree_str += f"{prefix}{connector}{f}\n"

    return tree_str

def export_context(root_dir, output_filename):
    with open(output_filename, 'w', encoding='utf-8') as outfile:
        # 1. Write the directory tree
        outfile.write("# Workspace Structure\n")
        outfile.write("```text\n")
        outfile.write("swerve_robot/\n")
        outfile.write(generate_tree(root_dir))
        outfile.write("```\n\n")

        # 2. Write the file contents
        outfile.write("# File Contents\n\n")
        
        for current_root, dirs, files in os.walk(root_dir):
            dirs[:] = [d for d in dirs if d not in IGNORE_DIRS]

            for file in sorted(files):
                ext = os.path.splitext(file)[1]
                if ext in TARGET_EXTS or file in TARGET_FILES:
                    filepath = os.path.join(current_root, file)
                    rel_path = os.path.relpath(filepath, root_dir)
                    
                    # Map extensions to markdown syntax highlighting
                    lang = ""
                    if file == 'CMakeLists.txt': lang = 'cmake'
                    elif ext == '.py': lang = 'python'
                    elif ext == '.cpp': lang = 'cpp'
                    elif ext in ['.yaml', '.yml']: lang = 'yaml'
                    elif ext in ['.xml', '.xacro', '.urdf']: lang = 'xml'

                    outfile.write(f"### File: {rel_path}\n")
                    outfile.write(f"```{lang}\n")
                    try:
                        with open(filepath, 'r', encoding='utf-8') as infile:
                            outfile.write(infile.read().strip())
                    except Exception as e:
                        outfile.write(f"// Error reading file: {e}")
                    outfile.write(f"\n```\n\n")

if __name__ == "__main__":
    output_file = "chat_context.txt"
    print(f"Scanning workspace and exporting to {output_file}...")
    export_context(".", output_file)
    print("Export complete. You can copy the contents of this file to your new chat.")