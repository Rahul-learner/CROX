import os
import shutil
import subprocess

def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    include_dir = os.path.join(base_dir, "include")
    src_dir = os.path.join(base_dir, "src")
    
    # 1. Move headers to src directory
    print("Moving headers from include/ to src/...")
    
    # Move root headers
    for item in os.listdir(include_dir):
        item_path = os.path.join(include_dir, item)
        if os.path.isfile(item_path) and item.endswith(".h"):
            shutil.move(item_path, os.path.join(src_dir, item))
            print(f"Moved {item} to src/")
            
    # Move core headers
    core_include = os.path.join(include_dir, "core")
    core_src = os.path.join(src_dir, "core")
    if os.path.exists(core_include):
        for item in os.listdir(core_include):
            item_path = os.path.join(core_include, item)
            if os.path.isfile(item_path) and item.endswith(".h"):
                shutil.move(item_path, os.path.join(core_src, item))
                print(f"Moved core/{item} to src/core/")
                
    # Try to remove include dir if empty
    try:
        os.rmdir(core_include)
        os.rmdir(include_dir)
        print("Removed empty include/ directory.")
    except OSError:
        print("Note: include/ directory not empty, left as is.")

    # 2. Add FatFS Submodule
    print("\nAdding no-OS-FatFS-SD-SPI-RPi-Pico submodule...")
    try:
        subprocess.run(["git", "submodule", "add", "https://github.com/carlk3/no-OS-FatFS-SD-SPI-RPi-Pico.git", "lib/no-OS-FatFS-SD-SPI-RPi-Pico"], check=True)
        print("Submodule added successfully.")
    except Exception as e:
        print(f"Failed to add submodule: {e}")
        print("Please run manually: git submodule add https://github.com/carlk3/no-OS-FatFS-SD-SPI-RPi-Pico.git lib/no-OS-FatFS-SD-SPI-RPi-Pico")

if __name__ == "__main__":
    main()
