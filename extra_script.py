from os.path import isfile

Import("env") # type: ignore

# Path to your environment variable file
env_file_path = ".env" 

if isfile(env_file_path):
    try:
        with open(env_file_path, "r") as f:
            lines = f.readlines()
            build_flags_to_add = []
            for line in lines:
                line = line.strip()
                if line and not line.startswith("#"): # Ignore empty lines and comments
                    key, value = line.split("=", 1)
                    build_flags_to_add.append(f"-D{key}={value}")
            env.Append(BUILD_FLAGS=build_flags_to_add) # type: ignore
    except IOError:
        print(f"Error: File {env_file_path} not accessible.")
else:
    print(f"Warning: Environment file {env_file_path} not found.")
