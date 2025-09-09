import os
import glob
from run import process_data
import argparse

def process_db3_files(target_dir):
    for root, dirs, files in os.walk(target_dir):
        for file in files:
            if file.endswith('.db3'):
                dir_path = root
                png_files = glob.glob(os.path.join(dir_path, '*.png'))
                
                if png_files:
                    # print(f"Skipping {dir_path} (PNG file already exists).")
                    continue
                
                if 'ul' in os.path.basename(dir_path):
                    location = 'ul'
                elif 'byu' in os.path.basename(dir_path):
                    location = 'byu'
                else:
                    # print("Skipping Directory without byu or ul name")
                    continue
                
                # print(f"Processing {dir_path}...")
                
                process_data(folder=dir_path, maptype='sat', process='all', location=location, behavior=True)
                
                output_file = 'output.png'
                if os.path.exists(output_file):
                    # print(f"Moving {output_file} to {dir_path}...")
                    os.rename(output_file, os.path.join(dir_path, output_file))
                else:
                    print(f"Warning: {output_file} not found for {dir_path}.")

def main(dir):
    target_dir = dir
    
    if not os.path.isdir(target_dir):
        print(f"Error: Directory {target_dir} does not exist.")
        return 1
    
    process_db3_files(target_dir)
    
    return 0

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process some maps.')
    parser.add_argument('folder', help='Coug Workspace Directory, bag, vehicle folder', default='null')
    args = parser.parse_args()
    exit(main(args.folder))
